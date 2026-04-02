#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-logger.h"
#include "DASPi-aperture-peer.h"
#include "DASPi-camera-config.h"
#include "DASPi-fisheye-camera-model.h"
#include "DASPi-fisheye-params.h"
#include "DASPi-image-rotation.h"
#include "DASPi-live-frame-buffer.h"
#include "DASPi-sphere-stitcher.h"
#include "DASPi-logicalstreamrole.h"
#include "DASPi-livecamerastate.h"

using namespace DASPi;

namespace {

constexpr int kFrameWidth = 1456;
constexpr int kFrameHeight = 1088;
constexpr int kExpectedPixels = kFrameWidth * kFrameHeight;
constexpr int kControlPortOffset = 1;

// n = overlap images per module
constexpr size_t kOverlapStreams = 4;

// total images per module = image 0 + overlaps 1..n
constexpr size_t kPeerStreams = kOverlapStreams + 1;
constexpr size_t kCamerasPerModule = kPeerStreams;

constexpr auto kRetryDelay = std::chrono::milliseconds(5);
constexpr auto kNoFrameDelay = std::chrono::milliseconds(20);
constexpr auto kStitchDelay = std::chrono::milliseconds(30);

struct ProgramOptions {
    int nApertureComputeModules{0};
    std::string usbBaseIp;
    int framePort{0};
};

struct NetworkAddressPlan {
    in_addr_t gateway{};
    in_addr_t client{};
    std::vector<in_addr_t> servers;
};

size_t GlobalCameraIndex(size_t moduleIndex, size_t localCameraIndex)
{
    return moduleIndex * kCamerasPerModule + localCameraIndex;
}

size_t TotalCameraCount(int nApertureComputeModules)
{
    return static_cast<size_t>(nApertureComputeModules) * kCamerasPerModule;
}

template <size_t N>
bool tryCopyLatestFrameFromPeer(AperturePeer<N>& peer,
                                size_t localCameraIndex,
                                std::vector<std::uint16_t>& outBayer)
{
    return peer.CopyBuffer(localCameraIndex, outBayer);
}

in_addr_t StringToInAddrT(const std::string& ipStr)
{
    struct in_addr addr{};
    if (inet_pton(AF_INET, ipStr.c_str(), &addr) != 1) {
        std::cerr << "Invalid IP address format: " << ipStr << '\n';
        return INADDR_NONE;
    }
    return addr.s_addr;
}

std::string InAddrTToString(in_addr_t addrNetOrder)
{
    char str[INET_ADDRSTRLEN]{};
    struct in_addr addr{};
    addr.s_addr = addrNetOrder;

    if (inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN) == nullptr) {
        std::cerr << "inet_ntop failed\n";
        return {};
    }

    return std::string(str);
}

bool ParseIpv4(const std::string& ipStr, std::array<int, 4>& octets)
{
    struct in_addr addr{};
    if (inet_pton(AF_INET, ipStr.c_str(), &addr) != 1) {
        return false;
    }

    const uint32_t ip = ntohl(addr.s_addr);
    octets[0] = static_cast<int>((ip >> 24) & 0xFF);
    octets[1] = static_cast<int>((ip >> 16) & 0xFF);
    octets[2] = static_cast<int>((ip >> 8) & 0xFF);
    octets[3] = static_cast<int>(ip & 0xFF);
    return true;
}

std::string MakeIpv4String(int a, int b, int c, int d)
{
    return std::to_string(a) + "." +
           std::to_string(b) + "." +
           std::to_string(c) + "." +
           std::to_string(d);
}

std::string IncrementIpv4Host(const std::string& ipStr, int delta)
{
    std::array<int, 4> octets{};
    if (!ParseIpv4(ipStr, octets)) {
        throw std::runtime_error("Invalid IPv4 address: " + ipStr);
    }

    const int host = octets[3] + delta;
    if (host < 0 || host > 254) {
        throw std::runtime_error("IPv4 host out of range for address: " + ipStr);
    }

    return MakeIpv4String(octets[0], octets[1], octets[2], host);
}

NetworkAddressPlan BuildNetworkAddressPlan(const std::string& usbBaseIp,
                                           int nApertureComputeModules)
{
    if (nApertureComputeModules <= 0) {
        throw std::runtime_error("nApertureComputeModules must be > 0");
    }

    NetworkAddressPlan plan;
    plan.gateway = StringToInAddrT(usbBaseIp);
    plan.client = StringToInAddrT(IncrementIpv4Host(usbBaseIp, 1));

    if (plan.gateway == INADDR_NONE || plan.client == INADDR_NONE) {
        throw std::runtime_error("Failed to construct gateway/client IP addresses");
    }

    plan.servers.reserve(static_cast<size_t>(nApertureComputeModules));
    for (int i = 0; i < nApertureComputeModules; ++i) {
        const std::string serverIp = IncrementIpv4Host(usbBaseIp, 2 + i);
        const in_addr_t serverAddr = StringToInAddrT(serverIp);
        if (serverAddr == INADDR_NONE) {
            throw std::runtime_error("Failed to construct server IP address: " + serverIp);
        }
        plan.servers.push_back(serverAddr);
    }

    return plan;
}

void PrintAddressPlan(const NetworkAddressPlan& plan)
{
    std::cout << "[main] Address plan\n";
    std::cout << "  gateway=" << InAddrTToString(plan.gateway) << '\n';
    std::cout << "  client=" << InAddrTToString(plan.client) << '\n';

    for (size_t i = 0; i < plan.servers.size(); ++i) {
        std::cout << "  server[" << i << "]="
                  << InAddrTToString(plan.servers[i]) << '\n';
    }
}

void PrintUsage(const char* programName)
{
    std::cerr
        << "Usage: " << programName
        << " [--verbose]"
        << " --nApertureComputeModules=<N>"
        << " --usbBaseIp=<base_ip>"
        << " --port=<frame_port>\n"
        << "Example: " << programName
        << " --nApertureComputeModules=2 --usbBaseIp=10.0.2.1 --port=5000\n";
}

ProgramOptions ParseArgs(int argc, char* argv[])
{
    ProgramOptions options;

    if (argc < 4) {
        PrintUsage(argv[0]);
        throw std::runtime_error("Insufficient arguments");
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};

        if (arg == "--verbose") {
            verbosity = Verbosity::Verbose;
            continue;
        }

        const std::string prefix0 = "--nApertureComputeModules=";
        if (arg.rfind(prefix0, 0) == 0) {
            options.nApertureComputeModules = std::stoi(arg.substr(prefix0.size()));
            continue;
        }

        const std::string prefix1 = "--usbBaseIp=";
        if (arg.rfind(prefix1, 0) == 0) {
            options.usbBaseIp = arg.substr(prefix1.size());
            continue;
        }

        const std::string prefix2 = "--port=";
        if (arg.rfind(prefix2, 0) == 0) {
            options.framePort = std::stoi(arg.substr(prefix2.size()));
            continue;
        }

        PrintUsage(argv[0]);
        throw std::runtime_error("Unknown argument: " + arg);
    }

    if (options.nApertureComputeModules <= 0) {
        throw std::runtime_error("No valid --nApertureComputeModules argument provided.");
    }
    if (options.usbBaseIp.empty()) {
        throw std::runtime_error("No valid --usbBaseIp argument provided.");
    }
    if (options.framePort <= 0) {
        throw std::runtime_error("No valid --port argument provided.");
    }

    return options;
}

void PrintProgramOptions(const ProgramOptions& options)
{
    std::cout << "[ARGS] nApertureComputeModules=" << options.nApertureComputeModules
              << " usbBaseIp='" << options.usbBaseIp
              << "' baseFramePort=" << options.framePort
              << '\n';
}

template <size_t N>
std::vector<std::unique_ptr<AperturePeer<N>>> CreateAperturePeers(
    const ProgramOptions& options,
    const NetworkAddressPlan& addressPlan)
{
    std::vector<std::unique_ptr<AperturePeer<N>>> aperturePeers(addressPlan.servers.size());

    std::cout << "[main] Setting up AperturePeers\n";
    for (size_t i = 0; i < aperturePeers.size(); ++i) {
        const int framePort = options.framePort + static_cast<int>(i) * 2;
        const int controlPort = framePort + kControlPortOffset;

        std::cout << "[main] Creating AperturePeer " << i
                  << ", framePort=" << framePort
                  << ", controlPort=" << controlPort
                  << ", clntAddr=" << InAddrTToString(addressPlan.client)
                  << ", srvAddr=" << InAddrTToString(addressPlan.servers[i])
                  << '\n';

        aperturePeers[i] = std::make_unique<AperturePeer<N>>(
            addressPlan.client,
            framePort,
            controlPort,
            addressPlan.servers[i]);
    }

    return aperturePeers;
}

cv::Mat applyImageRotation(const cv::Mat& image, ImageRotation rotation)
{
    cv::Mat out;

    switch (rotation) {
    case ImageRotation::None:
        out = image.clone();
        break;
    case ImageRotation::Rotate90CW:
        cv::rotate(image, out, cv::ROTATE_90_CLOCKWISE);
        break;
    case ImageRotation::Rotate90CCW:
        cv::rotate(image, out, cv::ROTATE_90_COUNTERCLOCKWISE);
        break;
    case ImageRotation::Rotate180:
        cv::rotate(image, out, cv::ROTATE_180);
        break;
    }

    return out;
}

cv::Size rotatedSize(const cv::Size& size, ImageRotation rotation)
{
    switch (rotation) {
    case ImageRotation::Rotate90CW:
    case ImageRotation::Rotate90CCW:
        return cv::Size(size.height, size.width);
    case ImageRotation::None:
    case ImageRotation::Rotate180:
    default:
        return size;
    }
}

std::shared_ptr<ICameraModel> makeFisheyeModelForRotation(ImageRotation rotation,
                                                          int width,
                                                          int height,
                                                          const FisheyeParams& params)
{
    const cv::Size size = rotatedSize(cv::Size(width, height), rotation);
    const double cx = static_cast<double>(size.width) / 2.0;
    const double cy = static_cast<double>(size.height) / 2.0;

    return std::make_shared<FisheyeCameraModel>(
        params.fx,
        params.fy,
        cx,
        cy,
        params.maxImageRadiusPx,
        size);
}

CameraView makeCameraView(const cv::Mat& image,
                          const cv::Mat& maskNonOverlap,
                          const cv::Mat& maskOverlap,
                          const std::shared_ptr<ICameraModel>& model,
                          const Eigen::Matrix3d& Rcw,
                          ImageRotation imageRotation)
{
    CameraView cam;
    cam.image = applyImageRotation(image, imageRotation);
    cam.maskNonOverlap = applyImageRotation(maskNonOverlap, imageRotation);
    cam.maskOverlap = applyImageRotation(maskOverlap, imageRotation);
    cam.model = model;
    cam.Rcw = Rcw;
    return cam;
}

cv::Mat decodeBayer16ToBgr8(const std::vector<uint16_t>& data)
{
    cv::Mat raw16(kFrameHeight, kFrameWidth, CV_16UC1,
                  const_cast<uint16_t*>(data.data()));
    cv::Mat rawCopy = raw16.clone();

    cv::Mat bgr16;
    cv::cvtColor(rawCopy, bgr16, cv::COLOR_BayerRG2BGR);

    cv::Mat bgr8;
    bgr16.convertTo(bgr8, CV_8UC3, 1.0 / 256.0);

    return bgr8;
}

void updateLatestFrame(LiveFrameBuffer& dst, const cv::Mat& frameBgr8)
{
    std::scoped_lock lock(dst.mutex);
    dst.latestBgr8 = frameBgr8.clone();
    dst.hasFrame = true;
}

bool tryGetLatestFrame(LiveFrameBuffer& src, cv::Mat& dst)
{
    std::scoped_lock lock(src.mutex);
    if (!src.hasFrame || src.latestBgr8.empty()) {
        return false;
    }

    dst = src.latestBgr8.clone();
    return true;
}

std::vector<CameraConfig> makeCameraConfigs(int nApertureComputeModules)
{
    if (nApertureComputeModules <= 0) {
        throw std::runtime_error("nApertureComputeModules must be > 0");
    }

    const Eigen::Matrix3d R_identity = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_right =
        Eigen::AngleAxisd(2.0 * 41.81 * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()).toRotationMatrix();

    std::vector<CameraConfig> configs;
    configs.reserve(TotalCameraCount(nApertureComputeModules));

    for (int module = 0; module < nApertureComputeModules; ++module) {
        for (size_t localCam = 0; localCam < kCamerasPerModule; ++localCam) {
            CameraConfig cfg{
                "module_" + std::to_string(module) + "_cam_" + std::to_string(localCam),
                "",
                ImageRotation::None,
                R_identity,
            };

            if (module == 0) {
                cfg.imageRotation = ImageRotation::Rotate90CCW;
                cfg.Rcw = R_identity;
            } else if (module == 1) {
                cfg.imageRotation = ImageRotation::Rotate90CW;
                cfg.Rcw = R_right;
            }

            configs.push_back(cfg);
        }
    }

    return configs;
}

std::vector<CameraView> CreateCameraViews(const std::vector<CameraConfig>& configs,
                                          const FisheyeParams& fisheyeParams)
{
    std::vector<CameraView> cameras;
    cameras.reserve(configs.size());

    for (const auto& cfg : configs) {
        cv::Mat emptyImage(kFrameHeight, kFrameWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat emptyMask(kFrameHeight, kFrameWidth, CV_8UC1, cv::Scalar(0));

        auto model = makeFisheyeModelForRotation(cfg.imageRotation,
                                                 kFrameWidth,
                                                 kFrameHeight,
                                                 fisheyeParams);

        cameras.push_back(makeCameraView(emptyImage,
                                         emptyMask,
                                         emptyMask,
                                         model,
                                         cfg.Rcw,
                                         cfg.imageRotation));
    }

    return cameras;
}

void updateCameraImages(std::vector<CameraView>& cameras,
                        const std::vector<CameraConfig>& configs,
                        std::vector<LiveCameraState>& liveCameras)
{
    if (cameras.size() != configs.size() || cameras.size() != liveCameras.size()) {
        throw std::runtime_error("Camera/config/state size mismatch");
    }

    for (size_t i = 0; i < cameras.size(); ++i) {
        cv::Mat latest;
        if (!tryGetLatestFrame(liveCameras[i].frame, latest)) {
            continue;
        }

        if (!liveCameras[i].hasMask) {
            throw std::runtime_error("Missing valid mask for logical camera " +
                                     std::to_string(i));
        }

        const ImageRotation rotation = configs[i].imageRotation;
        cameras[i].image = applyImageRotation(latest, rotation);

        const cv::Mat rotatedValidMask =
            applyImageRotation(liveCameras[i].validMask, rotation);

        if (liveCameras[i].role == LogicalStreamRole::NonOverlap) {
            cameras[i].maskNonOverlap = rotatedValidMask;
            cameras[i].maskOverlap =
                cv::Mat::zeros(rotatedValidMask.rows, rotatedValidMask.cols, CV_8UC1);
        } else {
            cameras[i].maskNonOverlap =
                cv::Mat::zeros(rotatedValidMask.rows, rotatedValidMask.cols, CV_8UC1);
            cameras[i].maskOverlap = rotatedValidMask;
        }
    }
}

bool haveAnyFrames(std::vector<LiveCameraState>& liveCameras)
{
    for (auto& camera : liveCameras) {
        cv::Mat tmp;
        if (tryGetLatestFrame(camera.frame, tmp)) {
            return true;
        }
    }
    return false;
}

template <size_t N>
void StartPeerThreads(std::vector<std::unique_ptr<AperturePeer<N>>>& aperturePeers,
                      std::vector<LiveCameraState>& liveCameras,
                      std::vector<std::jthread>& frameThreads,
                      std::vector<std::jthread>& controlThreads)
{
    std::cout << "[main] Starting per-peer frame/control threads\n";

    frameThreads.reserve(aperturePeers.size());
    controlThreads.reserve(aperturePeers.size());

    for (size_t moduleIndex = 0; moduleIndex < aperturePeers.size(); ++moduleIndex) {
        AperturePeer<N>* peer = aperturePeers[moduleIndex].get();

        frameThreads.emplace_back([peer, moduleIndex, &liveCameras]() {
            std::vector<uint16_t> raw;
            raw.reserve(kExpectedPixels);

            for (;;) {
                if (!peer->RunFrameLoop()) {
                    std::cerr << "[frame thread] RunFrameLoop failed for module "
                              << moduleIndex << '\n';
                    std::this_thread::sleep_for(kRetryDelay);
                    continue;
                }

                for (size_t localCameraIndex = 0; localCameraIndex < (N + 1); ++localCameraIndex) {
                    raw.clear();

                    if (!tryCopyLatestFrameFromPeer(*peer, localCameraIndex, raw)) {
                        continue;
                    }

                    if (static_cast<int>(raw.size()) != kExpectedPixels) {
                        std::cerr << "[frame thread] unexpected Bayer size for module "
                                  << moduleIndex
                                  << " localCameraIndex=" << localCameraIndex
                                  << " size=" << raw.size()
                                  << " expected=" << kExpectedPixels << '\n';
                        continue;
                    }

                    const size_t globalIndex =
                        GlobalCameraIndex(moduleIndex, localCameraIndex);

                    if (globalIndex >= liveCameras.size()) {
                        std::cerr << "[frame thread] globalIndex out of range: "
                                  << globalIndex
                                  << " liveCameras.size()=" << liveCameras.size() << '\n';
                        continue;
                    }

                    cv::Mat bgr = decodeBayer16ToBgr8(raw);
                    updateLatestFrame(liveCameras[globalIndex].frame, bgr);
                }
            }
        });

        controlThreads.emplace_back([peer, moduleIndex]() {
            for (;;) {
                if (!peer->RunControlLoop()) {
                    std::cerr << "[control thread] RunControlLoop failed for module "
                              << moduleIndex << '\n';
                    break;
                }
            }
        });
    }

    std::cout << "[main] Threads running\n";
}

void DrawPanoramaOverlay(cv::Mat& img,
                         uint64_t frameNumber,
                         const cv::Mat& validMask)
{
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = 0.8;
    const int thickness = 2;
    const cv::Scalar color(255, 255, 255);

    cv::putText(img,
                "Frame: " + std::to_string(frameNumber),
                cv::Point(20, 35),
                font, scale, color, thickness);

    cv::putText(img,
                "Size: " + std::to_string(img.cols) + "x" + std::to_string(img.rows),
                cv::Point(20, 70),
                font, scale, color, thickness);

    if (!validMask.empty()) {
        const int validPixels = cv::countNonZero(validMask);
        cv::putText(img,
                    "Valid pixels: " + std::to_string(validPixels),
                    cv::Point(20, 105),
                    font, scale, color, thickness);
    }

    cv::line(img,
             cv::Point(img.cols / 2 - 20, img.rows / 2),
             cv::Point(img.cols / 2 + 20, img.rows / 2),
             color, 1);

    cv::line(img,
             cv::Point(img.cols / 2, img.rows / 2 - 20),
             cv::Point(img.cols / 2, img.rows / 2 + 20),
             color, 1);
}

void RunStitchLoop(std::vector<CameraView>& cameras,
                   const std::vector<CameraConfig>& configs,
                   std::vector<LiveCameraState>& liveCameras)
{
    SphereStitchConfig stitchConfig;
    stitchConfig.outputWidth = 1456;
    stitchConfig.outputHeight = 1088;
    stitchConfig.blendPower = 4.0;

    uint64_t frameNumber = 0;
    const bool saveDebugImages = false;

    cv::namedWindow("Stitched Panorama", cv::WINDOW_NORMAL);
    cv::resizeWindow("Stitched Panorama", 1200, 800);

    for (;;) {
        if (!haveAnyFrames(liveCameras)) {
            std::this_thread::sleep_for(kNoFrameDelay);
            continue;
        }

        updateCameraImages(cameras, configs, liveCameras);

        SphereStitcher stitcher(cameras, stitchConfig);

        cv::Mat validMask;
        cv::Mat pano = stitcher.stitchFisheye(&validMask);

        if (pano.empty()) {
            std::cerr << "[RunStitchLoop] stitchFisheye returned empty pano\n";
            std::this_thread::sleep_for(kStitchDelay);
            continue;
        }

        if (saveDebugImages) {
            cv::imwrite("panorama.png", pano);
            if (!validMask.empty()) {
                cv::imwrite("valid_mask.png", validMask);
            }
        }

        cv::Mat display = pano.clone();
        DrawPanoramaOverlay(display, frameNumber++, validMask);

		std::cout << "[GUI] rows=" << display.rows
		          << " cols=" << display.cols
		          << " empty=" << display.empty()
		          << std::endl;
		          
        cv::imshow("Stitched Panorama", display);

        const int key = cv::waitKey(1);
        
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        std::this_thread::sleep_for(kStitchDelay);
    }

    cv::destroyWindow("Stitched Panorama");
}

template <size_t N>
void InitializeCameraMasks(std::vector<std::unique_ptr<AperturePeer<N>>>& aperturePeers,
                           std::vector<LiveCameraState>& liveCameras)
{
    for (size_t moduleIndex = 0; moduleIndex < aperturePeers.size(); ++moduleIndex) {
        AperturePeer<N>* peer = aperturePeers[moduleIndex].get();

        for (size_t localCameraIndex = 0; localCameraIndex < N + 1; ++localCameraIndex) {
            const size_t globalIndex = GlobalCameraIndex(moduleIndex, localCameraIndex);

            cv::Mat validMask;
            if (!peer->CopyValidMask(localCameraIndex, validMask)) {
                throw std::runtime_error("Failed to initialize valid mask for module " +
                                         std::to_string(moduleIndex) +
                                         ", localCameraIndex=" +
                                         std::to_string(localCameraIndex));
            }

            liveCameras[globalIndex].validMask = std::move(validMask);
            liveCameras[globalIndex].role =
                (localCameraIndex == 0) ? LogicalStreamRole::NonOverlap
                                        : LogicalStreamRole::Overlap;
            liveCameras[globalIndex].hasMask = true;
        }
    }
}

} // namespace

int main(int argc, char* argv[])
{
    try {
        std::cout << "Program - started\n";

        const ProgramOptions options = ParseArgs(argc, argv);
        PrintProgramOptions(options);

        const auto addressPlan =
            BuildNetworkAddressPlan(options.usbBaseIp, options.nApertureComputeModules);
        PrintAddressPlan(addressPlan);

        constexpr size_t n = kOverlapStreams;
        auto aperturePeers = CreateAperturePeers<n>(options, addressPlan);

        const FisheyeParams fisheyeParams{600.0, 600.0, 580.0};

        const size_t totalCameras =
	    TotalCameraCount(options.nApertureComputeModules);
	
		const std::vector<CameraConfig> configs =
		    makeCameraConfigs(options.nApertureComputeModules);
		
		auto cameras = CreateCameraViews(configs, fisheyeParams);
		std::vector<LiveCameraState> liveCameras(totalCameras);
		
		if (configs.size() != totalCameras || cameras.size() != totalCameras) {
		    throw std::runtime_error("Logical camera count mismatch during initialization");
		}
		
		InitializeCameraMasks(aperturePeers, liveCameras);
		
		std::cout << "[main] configs.size()=" << configs.size()
		          << " cameras.size()=" << cameras.size()
		          << " liveCameras.size()=" << liveCameras.size()
		          << '\n';
		
		std::vector<std::jthread> frameThreads;
		std::vector<std::jthread> controlThreads;
		
		StartPeerThreads<n>(aperturePeers, liveCameras, frameThreads, controlThreads);
		RunStitchLoop(cameras, configs, liveCameras);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
