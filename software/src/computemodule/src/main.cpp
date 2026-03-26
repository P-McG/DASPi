#include <chrono>
#include <cstdlib>
#include <cstring>
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

#include "DASPi-fisheye-camera-model.h"
#include "DASPi-sphere-stitcher.h"
#include "DASPi-image-rotation.h"
#include "DASPi-camera-config.h"
#include "DASPi-fisheye-params.h"
#include "DASPi-live-frame-buffer.h"
using namespace DASPi;

namespace {

constexpr int kFrameWidth = 1456;
constexpr int kFrameHeight = 1088;
constexpr int kExpectedPixels = kFrameWidth * kFrameHeight;

template <size_t N>
bool tryReceiveLatestFrameFromPeer(AperturePeer<N>& peer,
                                   std::vector<std::uint16_t>& outBayer)
{
    // use index 0 first; adjust if your desired camera stream is another index
    return peer.CopyBuffer(0, outBayer);
}

std::string IncrementIp(std::string& ipStr)
{
    struct in_addr addr{};
    if (inet_aton(ipStr.c_str(), &addr) == 0) {
        std::cerr << "Invalid IP address format\n";
        return ipStr;
    }

    uint32_t ip = ntohl(addr.s_addr);
    ++ip;
    addr.s_addr = htonl(ip);

    ipStr = inet_ntoa(addr);
    return ipStr;
}

std::string IncrementSubnet(std::string ipStr)
{
    struct in_addr addr{};
    if (inet_aton(ipStr.c_str(), &addr) == 0) {
        std::cerr << "Invalid IP address format\n";
        return ipStr;
    }

    uint32_t ip = ntohl(addr.s_addr);

    uint8_t octet1 = static_cast<uint8_t>((ip >> 24) & 0xFF);
    uint8_t octet2 = static_cast<uint8_t>((ip >> 16) & 0xFF);
    uint8_t octet3 = static_cast<uint8_t>((ip >> 8)  & 0xFF);
    uint8_t octet4 = static_cast<uint8_t>(ip & 0xFF);

    ++octet3;

    ip = (static_cast<uint32_t>(octet1) << 24) |
         (static_cast<uint32_t>(octet2) << 16) |
         (static_cast<uint32_t>(octet3) << 8)  |
         static_cast<uint32_t>(octet4);

    addr.s_addr = htonl(ip);
    return std::string(inet_ntoa(addr));
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

void PrintUsage(const char* programName)
{
    std::cerr
        << "Usage: " << programName
        << " [--verbose]"
        << " --nApertureComputeModules=<N>"
        << " --usbSubnets=<subnet>"
        << " --port=<frame_port>\n";
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
        size
    );
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

//template <size_t N>
//bool tryReceiveLatestFrameFromPeer(AperturePeer<N>& peer,
                                   //std::vector<uint16_t>& outBayer)
//{
    //// Assumes you add:
    //// bool CopyBuffer(size_t index, std::vector<uint16_t>& out) const;
    //return peer.CopyBuffer(0, outBayer);
//}

std::vector<CameraConfig> makeCameraConfigs()
{
    const Eigen::Matrix3d R_left = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_right;
    R_right = Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitY()).toRotationMatrix();

    return {
        {
            "left",
            "",
            ImageRotation::Rotate90CCW,
            R_left
        },
        {
            "right",
            "",
            ImageRotation::Rotate90CW,
            R_right
        }
    };
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

} // namespace

int main(int argc, char* argv[])
{
    try {
        std::cout << "Program - started\n";

        int nApertureComputeModules{0};
        std::string usbSubnet;
        int framePort{0};

        if (argc < 4) {
            PrintUsage(argv[0]);
            return EXIT_FAILURE;
        }

        for (int i = 1; i < argc; ++i) {
            const std::string arg{argv[i]};

            if (arg == "--verbose") {
                verbosity = Verbosity::Verbose;
                continue;
            }

            const std::string prefix0 = "--nApertureComputeModules=";
            if (arg.rfind(prefix0, 0) == 0) {
                nApertureComputeModules = std::stoi(arg.substr(prefix0.size()));
                continue;
            }

            const std::string prefix1 = "--usbSubnets=";
            if (arg.rfind(prefix1, 0) == 0) {
                usbSubnet = arg.substr(prefix1.size());
                continue;
            }

            const std::string prefix2 = "--port=";
            if (arg.rfind(prefix2, 0) == 0) {
                framePort = std::stoi(arg.substr(prefix2.size()));
                continue;
            }

            std::cerr << "Unknown argument: " << arg << '\n';
            return EXIT_FAILURE;
        }

        std::cout << "[ARGS] nApertureComputeModules=" << nApertureComputeModules
                  << " usbSubnet='" << usbSubnet
                  << "' framePort=" << framePort << '\n';

        if (nApertureComputeModules <= 0) {
            std::cerr << "No valid --nApertureComputeModules argument provided.\n";
            return EXIT_FAILURE;
        }

        if (usbSubnet.empty()) {
            std::cerr << "No valid --usbSubnets argument provided.\n";
            return EXIT_FAILURE;
        }

        if (framePort <= 0) {
            std::cerr << "No valid --port argument provided.\n";
            return EXIT_FAILURE;
        }

        constexpr int controlPortOffset = 1;
        const int controlPort = framePort + controlPortOffset;

        std::vector<in_addr_t> gatewayAddrs(static_cast<size_t>(nApertureComputeModules));
        std::vector<in_addr_t> clntAddrs(static_cast<size_t>(nApertureComputeModules));
        std::vector<in_addr_t> srvAddrs(static_cast<size_t>(nApertureComputeModules));

        std::string subnetBase = usbSubnet;

        std::string gatewayAddr = IncrementIp(subnetBase);
        gatewayAddrs[0] = StringToInAddrT(gatewayAddr);

        std::string clntAddr = IncrementIp(gatewayAddr);
        clntAddrs[0] = StringToInAddrT(clntAddr);

        std::string srvAddr = IncrementIp(clntAddr);
        srvAddrs[0] = StringToInAddrT(srvAddr);

        for (int i = 1; i < nApertureComputeModules; ++i) {
            gatewayAddrs[static_cast<size_t>(i)] =
                StringToInAddrT(IncrementSubnet(InAddrTToString(gatewayAddrs[static_cast<size_t>(i - 1)])));

            clntAddrs[static_cast<size_t>(i)] =
                StringToInAddrT(IncrementSubnet(InAddrTToString(clntAddrs[static_cast<size_t>(i - 1)])));

            srvAddrs[static_cast<size_t>(i)] =
                StringToInAddrT(IncrementSubnet(InAddrTToString(srvAddrs[static_cast<size_t>(i - 1)])));
        }

        std::cout << "[main] Address map\n";
        for (int i = 0; i < nApertureComputeModules; ++i) {
            struct in_addr gatewayIpAddr{};
            struct in_addr clntIpAddr{};
            struct in_addr srvIpAddr{};

            gatewayIpAddr.s_addr = gatewayAddrs[static_cast<size_t>(i)];
            clntIpAddr.s_addr = clntAddrs[static_cast<size_t>(i)];
            srvIpAddr.s_addr = srvAddrs[static_cast<size_t>(i)];

            std::cout << "  [" << i << "] "
                      << "gateway=" << inet_ntoa(gatewayIpAddr)
                      << " client=" << inet_ntoa(clntIpAddr)
                      << " server=" << inet_ntoa(srvIpAddr)
                      << '\n';
        }

        constexpr size_t n = 3;
        std::vector<std::unique_ptr<AperturePeer<n>>> aperturePeers(
            static_cast<size_t>(nApertureComputeModules));

        std::cout << "[main] Setting up AperturePeers\n";

        for (size_t i = 0; i < aperturePeers.size(); ++i) {
            struct in_addr srvIpAddr{};
            struct in_addr clntIpAddr{};
            srvIpAddr.s_addr = srvAddrs[i];
            clntIpAddr.s_addr = clntAddrs[i];

            std::cout << "[main] Creating AperturePeer " << i
                      << ", framePort=" << framePort
                      << ", controlPort=" << controlPort
                      << ", srvAddr=" << inet_ntoa(srvIpAddr)
                      << ", clntAddr=" << inet_ntoa(clntIpAddr)
                      << '\n';

            aperturePeers[i] = std::make_unique<AperturePeer<n>>(
                clntAddrs[i],
                framePort,
                controlPort,
                srvAddrs[i]
            );
        }

        const FisheyeParams fisheyeParams{600.0, 600.0, 580.0};
        const std::vector<CameraConfig> configs = makeCameraConfigs();

        if (static_cast<int>(configs.size()) != nApertureComputeModules) {
            std::cerr << "Camera config count does not match --nApertureComputeModules\n";
            return EXIT_FAILURE;
        }

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

        SphereStitchConfig stitchConfig;
        stitchConfig.outputWidth = 1456;
        stitchConfig.outputHeight = 1088;
        stitchConfig.blendPower = 4.0;

        std::vector<LiveFrameBuffer> liveFrames(static_cast<size_t>(nApertureComputeModules));

        std::cout << "[main] Starting per-peer frame/control threads\n";

        std::vector<std::jthread> frameThreads;
        std::vector<std::jthread> controlThreads;

        frameThreads.reserve(aperturePeers.size());
        controlThreads.reserve(aperturePeers.size());

        for (size_t i = 0; i < aperturePeers.size(); ++i) {
            AperturePeer<n>* peer = aperturePeers[i].get();
            LiveFrameBuffer* live = &liveFrames[i];

            frameThreads.emplace_back([peer, live]() {
                std::vector<uint16_t> raw;
                raw.reserve(kExpectedPixels);

                for (;;) {
                    if (!tryReceiveLatestFrameFromPeer(*peer, raw)) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        continue;
                    }

                    if (static_cast<int>(raw.size()) != kExpectedPixels) {
                        std::cerr << "[frame thread] unexpected Bayer size: "
                                  << raw.size() << '\n';
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

        for (;;) {
            if (!haveAnyFrames(liveFrames)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            updateCameraImages(cameras, configs, liveFrames);

            SphereStitcher stitcher(cameras, stitchConfig);

            cv::Mat validMask;
            cv::Mat pano = stitcher.stitchFisheye(&validMask);

            cv::imwrite("panorama.png", pano);
            cv::imwrite("valid_mask.png", validMask);

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
