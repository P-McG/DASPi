#include <array>
#include <chrono>
#include <cstdlib>
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

using namespace DASPi;

namespace {

constexpr int kFrameWidth = 1456;
constexpr int kFrameHeight = 1088;
constexpr int kExpectedPixels = kFrameWidth * kFrameHeight;
constexpr int kControlPortOffset = 1;
constexpr size_t kPeerStreams = 3;
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

template <size_t N>
bool tryCopyLatestFrameFromPeer(AperturePeer<N>& peer,
                                std::vector<std::uint16_t>& outBayer)
{
    return peer.CopyBuffer(0, outBayer);
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

cv::Mat fullMask(int width, int height)
{
    return cv::Mat(height, width, CV_8U, cv::Scalar(255));
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
                          const std::shared_ptr<ICameraModel>& model,
                          const Eigen::Matrix3d& Rcw,
                          ImageRotation imageRotation)
{
    CameraView cam;
    cam.image = applyImageRotation(image, imageRotation);

    cv::Mat maskNonOverlap = fullMask(image.cols, image.rows);
    cv::Mat maskOverlap = cv::Mat::zeros(image.rows, image.cols, CV_8U);

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

    struct CameraPreset {
        std::string name;
        ImageRotation imageRotation;
        Eigen::Matrix3d Rcw;
    };

    const Eigen::Matrix3d R_identity = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_right =
        Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

    const std::vector<CameraPreset> presets = {
        {"left", ImageRotation::Rotate90CCW, R_identity},
        {"right", ImageRotation::Rotate90CW, R_right},
    };

    std::vector<CameraConfig> configs;
    configs.reserve(static_cast<size_t>(nApertureComputeModules));

    for (int i = 0; i < nApertureComputeModules; ++i) {
        if (static_cast<size_t>(i) < presets.size()) {
            const auto& preset = presets[static_cast<size_t>(i)];
            configs.push_back(CameraConfig{
                preset.name,
                "",
                preset.imageRotation,
                preset.Rcw,
            });
        } else {
            configs.push_back(CameraConfig{
                "camera_" + std::to_string(i),
                "",
                ImageRotation::None,
                R_identity,
            });
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
        cv::Mat empty(kFrameHeight, kFrameWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        auto model = makeFisheyeModelForRotation(cfg.imageRotation,
                                                 kFrameWidth,
                                                 kFrameHeight,
                                                 fisheyeParams);
        cameras.push_back(makeCameraView(empty, model, cfg.Rcw, cfg.imageRotation));
    }

    return cameras;
}

void updateCameraImages(std::vector<CameraView>& cameras,
                        const std::vector<CameraConfig>& configs,
                        std::vector<LiveFrameBuffer>& liveFrames)
{
    for (size_t i = 0; i < cameras.size(); ++i) {
        cv::Mat latest;
        if (!tryGetLatestFrame(liveFrames[i], latest)) {
            continue;
        }

        const ImageRotation rotation = configs[i].imageRotation;
        cameras[i].image = applyImageRotation(latest, rotation);

        cv::Mat maskNonOverlap = fullMask(latest.cols, latest.rows);
        cv::Mat maskOverlap = cv::Mat::zeros(latest.rows, latest.cols, CV_8U);

        cameras[i].maskNonOverlap = applyImageRotation(maskNonOverlap, rotation);
        cameras[i].maskOverlap = applyImageRotation(maskOverlap, rotation);
    }
}

bool haveAnyFrames(std::vector<LiveFrameBuffer>& liveFrames)
{
    for (auto& frame : liveFrames) {
        cv::Mat tmp;
        if (tryGetLatestFrame(frame, tmp)) {
            return true;
        }
    }
    return false;
}

template <size_t N>
void StartPeerThreads(std::vector<std::unique_ptr<AperturePeer<N>>>& aperturePeers,
                      std::vector<LiveFrameBuffer>& liveFrames,
                      std::vector<std::jthread>& frameThreads,
                      std::vector<std::jthread>& controlThreads)
{
    std::cout << "[main] Starting per-peer frame/control threads\n";

    frameThreads.reserve(aperturePeers.size());
    controlThreads.reserve(aperturePeers.size());

    for (size_t i = 0; i < aperturePeers.size(); ++i) {
        AperturePeer<N>* peer = aperturePeers[i].get();
        LiveFrameBuffer* live = &liveFrames[i];

        frameThreads.emplace_back([peer, live]() {
            std::vector<uint16_t> raw;
            raw.reserve(kExpectedPixels);

            for (;;) {
                if (!peer->RunFrameLoop()) {
                    std::cerr << "[frame thread] RunFrameLoop failed\n";
                    std::this_thread::sleep_for(kRetryDelay);
                    continue;
                }

                if (!tryCopyLatestFrameFromPeer(*peer, raw)) {
                    std::cerr << "[frame thread] CopyBuffer failed\n";
                    std::this_thread::sleep_for(kRetryDelay);
                    continue;
                }

                if (static_cast<int>(raw.size()) != kExpectedPixels) {
                    std::cerr << "[frame thread] unexpected Bayer size: "
                              << raw.size() << " expected=" << kExpectedPixels << '\n';
                    continue;
                }

                cv::Mat bgr = decodeBayer16ToBgr8(raw);
                updateLatestFrame(*live, bgr);
            }
        });

        controlThreads.emplace_back([peer]() {
            for (;;) {
                if (!peer->RunControlLoop()) {
                    std::cerr << "[control thread] RunControlLoop failed\n";
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
                   std::vector<LiveFrameBuffer>& liveFrames)
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
        if (!haveAnyFrames(liveFrames)) {
            std::this_thread::sleep_for(kNoFrameDelay);
            continue;
        }

        updateCameraImages(cameras, configs, liveFrames);

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

        cv::imshow("Stitched Panorama", display);

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        std::this_thread::sleep_for(kStitchDelay);
    }

    cv::destroyWindow("Stitched Panorama");
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

        constexpr size_t n = kPeerStreams;
        auto aperturePeers = CreateAperturePeers<n>(options, addressPlan);

        const FisheyeParams fisheyeParams{600.0, 600.0, 580.0};
        const std::vector<CameraConfig> configs =
            makeCameraConfigs(options.nApertureComputeModules);

        auto cameras = CreateCameraViews(configs, fisheyeParams);

        std::vector<LiveFrameBuffer> liveFrames(
            static_cast<size_t>(options.nApertureComputeModules));

        std::vector<std::jthread> frameThreads;
        std::vector<std::jthread> controlThreads;

        StartPeerThreads<n>(aperturePeers, liveFrames, frameThreads, controlThreads);
        RunStitchLoop(cameras, configs, liveFrames);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
