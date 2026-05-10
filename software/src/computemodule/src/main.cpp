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
#include <algorithm>
#include <cmath>
#include <limits>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "DASPi-config.h"
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
#include "DASPi-rig-data.h"
#include "DASPi-rig-geometry.h"
#include "DASPi-mesh_topology.h"
#include "DASPi-icosahedron_topology.h"
#include "DASPi-camera_setup.h"
#include "DASPi-flat_triangle_map.h"

using namespace DASPi;
using namespace DASPi::spherical;

namespace {

constexpr int kFrameWidth = 1456;
constexpr int kFrameHeight = 1088;
constexpr int kExpectedPixels = kFrameWidth * kFrameHeight;
constexpr int kModulePortStride = 10;
constexpr int kControlPortOffset = 1;

// n = overlap images per module
constexpr size_t kOverlapStreams = NUM_SIDES;

// total images per module = image 0 + overlaps 1..n
constexpr size_t kPeerStreams = kOverlapStreams + 1;
constexpr size_t kCamerasPerModule = kPeerStreams;

constexpr auto kRetryDelay  = std::chrono::milliseconds(5);
constexpr auto kNoFrameDelay = std::chrono::milliseconds(20);
constexpr auto kStitchDelay  = std::chrono::milliseconds(30);
constexpr std::uint64_t kSignatureLogEvery = 10;

struct ProgramOptions {
    int nApertureComputeModules{0};
    std::string usbBaseIp;
    int framePort{0};
    bool reverseModuleOrder{false};
    std::vector<std::string> serverIps;
};

struct NetworkAddressPlan {
    in_addr_t client{};
    std::vector<in_addr_t> servers;
};



//Eigen::Matrix3d RotationFromImageRotation(ImageRotation rot)
//{
    //switch (rot) {
    //case ImageRotation::Rotate90CW:
        //return Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //case ImageRotation::Rotate90CCW:
        //return Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //case ImageRotation::Rotate180:
        //return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //case ImageRotation::None:
    //default:
        //return Eigen::Matrix3d::Identity();
    //}
//}
size_t GlobalCameraIndex(size_t moduleIndex, size_t localCameraIndex)
{
    return moduleIndex * kCamerasPerModule + localCameraIndex;
}

size_t PeerToModuleIndex(size_t peerIndex,
                         size_t moduleCount,
                         bool reverseModuleOrder)
{
    if (!reverseModuleOrder) {
        return peerIndex;
    }
    return (moduleCount - 1) - peerIndex;
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

NetworkAddressPlan BuildNetworkAddressPlan(const ProgramOptions& options)
{
    const std::string& usbBaseIp = options.usbBaseIp;
    const int nApertureComputeModules = options.nApertureComputeModules;

    if (nApertureComputeModules <= 0) {
        throw std::runtime_error("nApertureComputeModules must be > 0");
    }

    std::array<int, 4> octets{};
    if (!ParseIpv4(usbBaseIp, octets)) {
        throw std::runtime_error("Invalid --usbBaseIp: " + usbBaseIp);
    }

    const int clientHost = octets[3];
    if (clientHost < 1 || clientHost > 252) {
        throw std::runtime_error("--usbBaseIp host octet must be in [1,252]");
    }

    if (options.serverIps.empty()) {
        const int highestServerHost = clientHost + 3 + (nApertureComputeModules - 1);
        if (highestServerHost > 254) {
            throw std::runtime_error(
                "Default server host allocation exceeds IPv4 host range; "
                "use --serverIps for explicit addressing");
        }
    }

    NetworkAddressPlan plan;
    plan.client = StringToInAddrT(usbBaseIp);
    if (plan.client == INADDR_NONE) {
        throw std::runtime_error("Failed to construct client IP address: " + usbBaseIp);
    }
    plan.servers.reserve(static_cast<size_t>(nApertureComputeModules));

    for (int i = 0; i < nApertureComputeModules; ++i) {
        std::string serverIp;
        if (!options.serverIps.empty()) {
            serverIp = options.serverIps[static_cast<size_t>(i)];
        } else {
            serverIp = IncrementIpv4Host(usbBaseIp, 3 + i);
        }

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
        << " --usbBaseIp=<client_ip>"
        << " --port=<frame_port>"
        << " [--serverIps=<ip0,ip1,...>]"
        << " [--reverseModuleOrder]\n"
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

        if (arg == "--reverseModuleOrder") {
            options.reverseModuleOrder = true;
            continue;
        }

        const std::string prefix3 = "--serverIps=";
        if (arg.rfind(prefix3, 0) == 0) {
            const std::string csv = arg.substr(prefix3.size());
            std::size_t start = 0;
            while (start < csv.size()) {
                const std::size_t comma = csv.find(',', start);
                const std::string ip = csv.substr(start, comma - start);
                if (!ip.empty()) {
                    options.serverIps.push_back(ip);
                }
                if (comma == std::string::npos) {
                    break;
                }
                start = comma + 1;
            }
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
    if (!options.serverIps.empty() &&
        static_cast<int>(options.serverIps.size()) != options.nApertureComputeModules) {
        throw std::runtime_error("--serverIps count must match --nApertureComputeModules");
    }

    return options;
}

void PrintProgramOptions(const ProgramOptions& options)
{
    std::cout << "[ARGS] nApertureComputeModules=" << options.nApertureComputeModules
              << " usbBaseIp='" << options.usbBaseIp
              << "' baseFramePort=" << options.framePort
              << " reverseModuleOrder=" << (options.reverseModuleOrder ? "true" : "false")
              << '\n';

    if (!options.serverIps.empty()) {
        std::cout << "[ARGS] serverIps=";
        for (size_t i = 0; i < options.serverIps.size(); ++i) {
            if (i != 0) std::cout << ",";
            std::cout << options.serverIps[i];
        }
        std::cout << '\n';
    }
}

template <size_t N>
std::vector<std::unique_ptr<AperturePeer<N>>> CreateAperturePeers(
    const ProgramOptions& options,
    const NetworkAddressPlan& addressPlan)
{
    std::vector<std::unique_ptr<AperturePeer<N>>> aperturePeers(addressPlan.servers.size());

    std::cout << "[main] Setting up AperturePeers\n";
    for (size_t i = 0; i < aperturePeers.size(); ++i) {
        const int clntFramePort = options.framePort + static_cast<int>(i) * kModulePortStride;
        const int clntControlPort = clntFramePort + 1;
		const int srvFramePort = clntFramePort;
		const int srvControlPort = clntControlPort;

        std::cout << "[main] Creating AperturePeer " << i
                  << ", clntFramePort=" << clntFramePort
                  << ", clntControlPort=" << clntControlPort
                  << ", srvFramePort=" << srvFramePort
                  << ", srvControlPort=" << srvControlPort
                  << ", clntAddr=" << InAddrTToString(addressPlan.client)
                  << ", srvAddr=" << InAddrTToString(addressPlan.servers[i])
                  << '\n';

        aperturePeers[i] = std::make_unique<AperturePeer<N>>(
            addressPlan.client,
            clntFramePort,
            clntControlPort,
            addressPlan.servers[i],
            srvFramePort,
            srvControlPort);
    }

    return aperturePeers;
}

//cv::Mat applyImageRotation(const cv::Mat& image, ImageRotation rotation)
//{
    //cv::Mat out;

    //switch (rotation) {
    //case ImageRotation::None:
        //out = image.clone();
        //break;
    //case ImageRotation::Rotate90CW:
        //cv::rotate(image, out, cv::ROTATE_90_CLOCKWISE);
        //break;
    //case ImageRotation::Rotate90CCW:
        //cv::rotate(image, out, cv::ROTATE_90_COUNTERCLOCKWISE);
        //break;
    //case ImageRotation::Rotate180:
        //cv::rotate(image, out, cv::ROTATE_180);
        //break;
    //}

    //return out;
//}

//cv::Size rotatedSize(const cv::Size& size, ImageRotation rotation)
//{
    //switch (rotation) {
    //case ImageRotation::Rotate90CW:
    //case ImageRotation::Rotate90CCW:
        //return cv::Size(size.height, size.width);
    //case ImageRotation::None:
    //case ImageRotation::Rotate180:
    //default:
        //return size;
    //}
//}

std::shared_ptr<ICameraModel> makeFisheyeModelForRotation(ImageRotation /*rotation*/,
                                                          int width,
                                                          int height,
                                                          const FisheyeParams& params)
{
    const cv::Size size(width, height);

    return std::make_shared<FisheyeCameraModel>(
        params.fx,
        params.fy,
        params.cx,
        params.cy,
        params.maxImageRadiusPx,
        size);
}

//Eigen::Matrix3d ApplyLocalCameraRoll(const Eigen::Matrix3d& Rcw,
                                     //double rollDeg)
//{
    //const double rollRad = rollDeg * M_PI / 180.0;

    //const Eigen::Matrix3d Rlocal =
        //Eigen::AngleAxisd(rollRad, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //// Post-multiply = rotate around camera's own local axis.
    //return Rcw * Rlocal;
//}

int CountMaskNonZero(const cv::Mat& mask)
{
    return mask.empty() ? 0 : cv::countNonZero(mask);
}

CameraView makeCameraView(const cv::Mat& image,
                          const cv::Mat& maskNonOverlap,
                          const cv::Mat& maskOverlap,
                          const std::shared_ptr<ICameraModel>& model,
                          const Eigen::Matrix3d& Rcw,
                          ImageRotation imageRotation)
{
	(void)imageRotation;
	
    CameraView cam;

    // ✅ NO ROTATION
    cam.image = image;
    cam.maskNonOverlap = maskNonOverlap;
    cam.maskOverlap = maskOverlap;

    cam.model = model;
    cam.Rcw = Rcw;
    
	std::cout << "[makeCameraView] "
	          << "image=" << image.cols << "x" << image.rows
	          << " nonOverlap=" << CountMaskNonZero(maskNonOverlap)
	          << " overlap=" << CountMaskNonZero(maskOverlap)
	          << '\n';

    return cam;
}

cv::Mat decodeBayer16ToBgr8(const std::vector<uint16_t>& data)
{
    constexpr bool kWriteRawDebugImage = false;
    constexpr bool kVerboseDecode = false;

    const size_t expected =
        static_cast<size_t>(kFrameWidth) * static_cast<size_t>(kFrameHeight);

    if (data.size() != expected) {
        std::cerr << "[decodeBayer16ToBgr8] bad input size: data.size()="
                  << data.size()
                  << " expected=" << expected
                  << std::endl;
        return {};
    }

    cv::Mat raw16(kFrameHeight,
                  kFrameWidth,
                  CV_16UC1,
                  const_cast<uint16_t*>(data.data()));

    if constexpr (kVerboseDecode || kWriteRawDebugImage) {
        cv::Mat test8;
        raw16.convertTo(test8, CV_8UC1, 1.0 / 256.0);

        if constexpr (kVerboseDecode) {
            std::cout << "[decodeBayer16ToBgr8] data.size()=" << data.size()
                      << " expected=" << expected
                      << " raw16.empty()=" << raw16.empty()
                      << " test8.empty()=" << test8.empty()
                      << " test8.type()=" << test8.type()
                      << std::endl;
        }

        if constexpr (kWriteRawDebugImage) {
            cv::imwrite("/tmp/raw_debug.png", test8);
        }
    }

    cv::Mat bgr16;
    cv::cvtColor(raw16, bgr16, cv::COLOR_BayerRG2BGR);

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

template <std::size_t N>
RigData<N> BuildRigDataFromTopology(const MeshTopology<N>& topo)
{
    static_assert(N >= 3, "Faces must have at least 3 vertices");

    RigData<N> rig;

    rig.vertices.reserve(topo.vertices.size());
    for (const auto& v : topo.vertices) {
        rig.vertices.push_back(v.normalized());
    }

    rig.faces.reserve(topo.faces.size());

    for (const auto& f : topo.faces) {
        const Eigen::Vector3d& a = rig.vertices[f[0]];

        Eigen::Vector3d normal(0.0, 0.0, 0.0);
        for (std::size_t i = 1; i + 1 < N; ++i) {
            const Eigen::Vector3d& b = rig.vertices[f[i]];
            const Eigen::Vector3d& c = rig.vertices[f[i + 1]];
            normal += (b - a).cross(c - a);
        }

        const double normalNorm = normal.norm();
        if (normalNorm < 1e-12) {
            throw std::runtime_error("Degenerate face encountered in BuildRigDataFromTopology");
        }
        normal /= normalNorm;

        // Use the average vertex direction only to determine outwardness.
        Eigen::Vector3d center(0.0, 0.0, 0.0);
        for (std::size_t i = 0; i < N; ++i) {
            center += rig.vertices[f[i]];
        }

        const double centerNorm = center.norm();
        if (centerNorm < 1e-12) {
            throw std::runtime_error("Degenerate face center encountered in BuildRigDataFromTopology");
        }
        center /= centerNorm;

        // Ensure outward-facing normal.
        if (normal.dot(center) < 0.0) {
            normal = -normal;
        }

        RigFace<N> face;
        face.indices = f;
        face.normal = normal;
        face.lookDir = normal;   // important: use geometric normal for ownership/orientation

        rig.faces.push_back(std::move(face));
    }

    return rig;
}

template <std::size_t N>
RigData<N> MakeRigData(const MeshTopology<N>& topo)
{
    return BuildRigDataFromTopology(topo);
}

template <std::size_t N>
bool FacesAreNeighbors(const RigFace<N>& a, const RigFace<N>& b)
{
    std::size_t shared = 0;

    for (std::size_t va : a.indices) {
        for (std::size_t vb : b.indices) {
            if (va == vb) {
                ++shared;
                if (shared >= 2) {
                    return true; // early exit (faster)
                }
            }
        }
    }

    return false;
}

template<std::size_t N>
std::vector<std::vector<int>> BuildFaceNeighborGraph(const std::vector<RigFace<N>>& faces)
{
    std::vector<std::vector<int>> graph(faces.size());

    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (std::size_t j = i + 1; j < faces.size(); ++j) {
            if (FacesAreNeighbors(faces[i], faces[j])) {
                graph[i].push_back(static_cast<int>(j));
                graph[j].push_back(static_cast<int>(i));
            }
        }
    }

    return graph;
}

bool CameraProjectsToValidUv(const CameraView& cam,
                             const Eigen::Vector3d& ray_world,
                             ProjectionResult& projOut)
{
    const Eigen::Vector3d ray_cam = cam.Rcw.transpose() * ray_world;

    if (ray_cam.z() <= 0.0) {
        return false;
    }

    projOut = cam.model->project(ray_cam);
    if (!projOut.valid) {
        return false;
    }

    return true;
}

//bool IsPointNearSegment(const cv::Point2d& p,
                        //const cv::Point2d& a,
                        //const cv::Point2d& b,
                        //double pxTol)
//{
    //const cv::Point2d ab = b - a;
    //const cv::Point2d ap = p - a;

    //const double ab2 = ab.dot(ab);
    //if (ab2 < 1e-12) {
        //return cv::norm(p - a) <= pxTol;
    //}

    //const double t = std::clamp(ap.dot(ab) / ab2, 0.0, 1.0);
    //const cv::Point2d proj = a + t * ab;
    //return cv::norm(p - proj) <= pxTol;
//}

cv::Point2d WorldRayToEquirectUv(const Eigen::Vector3d& ray,
                                 int width,
                                 int height)
{
    const Eigen::Vector3d r = ray.normalized();

    const double theta = std::atan2(r.x(), r.z());   // longitude
    const double phi   = std::asin(std::clamp(r.y(), -1.0, 1.0)); // latitude

    const double u = (theta / (2.0 * M_PI) + 0.5) * static_cast<double>(width);
    const double v = (0.5 - phi / M_PI) * static_cast<double>(height);

    return cv::Point2d(u, v);
}

template<std::size_t N>
std::array<std::size_t, 2> FindSharedEdgeVertices(const RigFace<N>& a,
                                                  const RigFace<N>& b)
{
    static_assert(N >= 2, "Faces must have at least 2 vertices");

    std::array<std::size_t, 2> shared{};
    std::size_t count = 0;

    for (std::size_t va : a.indices) {
        for (std::size_t vb : b.indices) {
            if (va == vb) {
                if (count >= 2) {
                    throw std::runtime_error("Faces share more than one edge");
                }
                shared[count++] = va;
            }
        }
    }

    if (count != 2) {
        throw std::runtime_error("Faces do not share exactly one edge");
    }

    return shared;
}

template<std::size_t N>
cv::Mat RenderSharedFaceSeamDebug(const RigFace<N>& faceA,
                                  const RigFace<N>& faceB,
                                  const std::vector<Eigen::Vector3d>& vertices,
                                  int width,
                                  int height,
                                  double pxTol = 1.5)
{
    const auto shared = FindSharedEdgeVertices(faceA, faceB);

    const Eigen::Vector3d a =
        vertices[static_cast<size_t>(shared[0])].normalized();
    const Eigen::Vector3d b =
        vertices[static_cast<size_t>(shared[1])].normalized();

    cv::Mat out(height, width, CV_8UC1, cv::Scalar(0));

    // Sample the great-circle arc between a and b.
    constexpr int kSamples = 2048;
    std::vector<cv::Point2d> seamPts;
    seamPts.reserve(kSamples);

    const double dotAB = std::clamp(a.dot(b), -1.0, 1.0);
    const double omega = std::acos(dotAB);

    if (omega < 1e-12) {
        const cv::Point2d uv = WorldRayToEquirectUv(a, width, height);
        cv::circle(out, uv, 2, cv::Scalar(255), cv::FILLED);
        return out;
    }

    for (int i = 0; i < kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);

        const double s0 = std::sin((1.0 - t) * omega) / std::sin(omega);
        const double s1 = std::sin(t * omega) / std::sin(omega);

        const Eigen::Vector3d p = (s0 * a + s1 * b).normalized();
        seamPts.push_back(WorldRayToEquirectUv(p, width, height));
    }

    for (size_t i = 1; i < seamPts.size(); ++i) {
        const cv::Point2d& p0 = seamPts[i - 1];
        const cv::Point2d& p1 = seamPts[i];

        // Handle wraparound across left/right pano edge.
        if (std::abs(p1.x - p0.x) > width * 0.5) {
            continue;
        }

        cv::line(out, p0, p1, cv::Scalar(255), 2, cv::LINE_AA);
    }

    return out;
}

template <std::size_t N>
void DebugPrintRigFaces(const std::vector<RigFace<N>>& faces)
{
    std::cout << "[rig faces]\n";
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const auto& f = faces[i];

        std::cout << "  face " << i << " indices=(";
        for (std::size_t j = 0; j < N; ++j) {
            std::cout << f.indices[j];
            if (j + 1 < N) {
                std::cout << ", ";
            }
        }
        std::cout << ")"
                  << " normal=(" << f.normal.transpose() << ")"
                  << " lookDir=(" << f.lookDir.transpose() << ")"
                  << '\n';
    }
}

cv::Mat RenderCameraFootprintDebug(const CameraView& cam,
                                   int width,
                                   int height)
{
    cv::Mat out(height, width, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Eigen::Vector3d ray_world =
                spherical::EquirectPixelToRay(x, y, width, height);

            ProjectionResult proj;
            if (!CameraProjectsToValidUv(cam, ray_world, proj)) {
                continue;
            }

            out.at<std::uint8_t>(y, x) = 255;
        }
    }

    return out;
}

[[maybe_unused]]
cv::Mat RenderCameraMaskDebug(const CameraView& cam,
                              int width,
                              int height)
{
    cv::Mat out(height, width, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Eigen::Vector3d ray_world =
                spherical::EquirectPixelToRay(x, y, width, height);

            ProjectionResult proj;
            if (!CameraProjectsToValidUv(cam, ray_world, proj)) {
                continue;
            }

            const bool inNonOverlap =
                SphereStitcher::IsInsideMask(cam.maskNonOverlap, proj.uv);

            const bool inOverlap =
                SphereStitcher::IsInsideMask(cam.maskOverlap, proj.uv);

            if (inNonOverlap || inOverlap) {
                out.at<std::uint8_t>(y, x) = 255;
            }
        }
    }

    return out;
}

[[maybe_unused]]
bool Barycentric2D(const cv::Point2d& p,
                   const cv::Point2d& a,
                   const cv::Point2d& b,
                   const cv::Point2d& c,
                   Eigen::Vector3d& bary)
{
    const cv::Point2d v0 = b - a;
    const cv::Point2d v1 = c - a;
    const cv::Point2d v2 = p - a;

    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

    const double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) <= 1e-12) {
        return false;
    }

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    bary = Eigen::Vector3d(u, v, w);
    return true;
}

bool BarycentricOnFace3D(const Eigen::Vector3d& rayWorld,
                         const Eigen::Vector3d& a,
                         const Eigen::Vector3d& b,
                         const Eigen::Vector3d& c,
                         Eigen::Vector3d& bary)
{
    const Eigen::Vector3d n = (b - a).cross(c - a);
    const double denom = n.dot(rayWorld);

    if (std::abs(denom) <= 1e-12) {
        return false;
    }

    // Intersect ray from origin with the face plane.
    const double t = n.dot(a) / denom;
    if (t <= 0.0) {
        return false;
    }

    const Eigen::Vector3d p = t * rayWorld;

    const Eigen::Vector3d v0 = b - a;
    const Eigen::Vector3d v1 = c - a;
    const Eigen::Vector3d v2 = p - a;

    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

    const double denom2 = d00 * d11 - d01 * d01;
    if (std::abs(denom2) <= 1e-12) {
        return false;
    }

    const double v = (d11 * d20 - d01 * d21) / denom2;
    const double w = (d00 * d21 - d01 * d20) / denom2;
    const double u = 1.0 - v - w;

    bary = Eigen::Vector3d(u, v, w);
    return true;
}

FlatTriangleMap ExtractFlatTriangleMapFromMask(const cv::Mat& nonOverlapFlatMask)
{
    if (nonOverlapFlatMask.empty()) {
        throw std::runtime_error("ExtractFlatTriangleMapFromMask: empty mask");
    }

    if (nonOverlapFlatMask.type() != CV_8UC1) {
        throw std::runtime_error("ExtractFlatTriangleMapFromMask: mask must be CV_8UC1");
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(nonOverlapFlatMask.clone(),
                     contours,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        throw std::runtime_error("ExtractFlatTriangleMapFromMask: no contours found");
    }

    auto largestIt = std::max_element(
        contours.begin(),
        contours.end(),
        [](const auto& a, const auto& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    std::vector<cv::Point> approx;
    const double eps = 0.01 * cv::arcLength(*largestIt, true);
    cv::approxPolyDP(*largestIt, approx, eps, true);

    if (approx.size() != 3) {
        throw std::runtime_error(
            "ExtractFlatTriangleMapFromMask: expected 3 vertices, got " +
            std::to_string(approx.size()));
    }

    std::array<cv::Point2d, 3> pts{};
    for (std::size_t i = 0; i < 3; ++i) {
        pts[i] = cv::Point2d(approx[i].x, approx[i].y);
    }

    // Current flat atlas shape:
    //   one right/apex point
    //   two left-side points: top-left and bottom-left
    auto rightIt = std::max_element(
        pts.begin(),
        pts.end(),
        [](const cv::Point2d& a, const cv::Point2d& b) {
            return a.x < b.x;
        });

    const cv::Point2d rightApex = *rightIt;

    std::vector<cv::Point2d> leftPts;
    leftPts.reserve(2);

    for (const cv::Point2d& p : pts) {
        if (p != rightApex) {
            leftPts.push_back(p);
        }
    }

    if (leftPts.size() != 2) {
        throw std::runtime_error("ExtractFlatTriangleMapFromMask: failed to split vertices");
    }

    const cv::Point2d leftTop =
        (leftPts[0].y < leftPts[1].y) ? leftPts[0] : leftPts[1];

    const cv::Point2d leftBottom =
        (leftPts[0].y < leftPts[1].y) ? leftPts[1] : leftPts[0];

    FlatTriangleMap map;

    // Try this ordering first.
    //
    // This means:
    //   bary.x -> leftTop
    //   bary.y -> rightApex
    //   bary.z -> leftBottom
    map.p[0] = leftTop;
    map.p[1] = rightApex;
    map.p[2] = leftBottom;

    std::cout << "[flat map ordered] "
              << "p0=(" << map.p[0].x << "," << map.p[0].y << ") "
              << "p1=(" << map.p[1].x << "," << map.p[1].y << ") "
              << "p2=(" << map.p[2].x << "," << map.p[2].y << ")"
              << '\n';

    return map;
}

[[maybe_unused]]
cv::Mat WarpFlatMaskToCameraMaskForward(const cv::Mat& flatMask,
                                        const CameraView& cam,
                                        const RigFace<3>& face,
                                        const std::vector<Eigen::Vector3d>& vertices,
                                        int cameraWidth,
                                        int cameraHeight)
{
    if (flatMask.empty()) {
        return {};
    }

    if (flatMask.type() != CV_8UC1) {
        throw std::runtime_error("WarpFlatMaskToCameraMaskForward: flatMask must be CV_8UC1");
    }

    cv::Mat cameraMask(cameraHeight, cameraWidth, CV_8UC1, cv::Scalar(0));

    const Eigen::Vector3d a =
        vertices[static_cast<std::size_t>(face.indices[0])].normalized();
    const Eigen::Vector3d b =
        vertices[static_cast<std::size_t>(face.indices[1])].normalized();
    const Eigen::Vector3d c =
        vertices[static_cast<std::size_t>(face.indices[2])].normalized();

    for (int y = 0; y < flatMask.rows; ++y) {
        const auto* row = flatMask.ptr<std::uint8_t>(y);

        for (int x = 0; x < flatMask.cols; ++x) {
            if (row[x] == 0) {
                continue;
            }

            const double u =
                static_cast<double>(x) /
                static_cast<double>(std::max(1, flatMask.cols - 1));

            const double v =
                static_cast<double>(y) /
                static_cast<double>(std::max(1, flatMask.rows - 1));

            const Eigen::Vector3d worldPoint =
                (1.0 - u - v) * a + u * b + v * c;

            const double n = worldPoint.norm();
            if (n <= 1e-12) {
                continue;
            }

            const Eigen::Vector3d rayWorld = worldPoint / n;
            const Eigen::Vector3d rayCam = cam.Rcw.transpose() * rayWorld;

            if (rayCam.z() <= 0.0) {
                continue;
            }

            const ProjectionResult proj = cam.model->project(rayCam);
            if (!proj.valid) {
                continue;
            }

            const int cx = static_cast<int>(std::round(proj.uv.x));
            const int cy = static_cast<int>(std::round(proj.uv.y));

            if (cx < 0 || cy < 0 ||
                cx >= cameraMask.cols || cy >= cameraMask.rows) {
                continue;
            }

            cameraMask.at<std::uint8_t>(cy, cx) = 255;
        }
    }

    cv::Mat dilated;
    constexpr int kDilatePx = 2;
    const int k = 2 * kDilatePx + 1;

    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

    cv::dilate(cameraMask, dilated, kernel);

    return dilated;
}

[[maybe_unused]]
cv::Mat WarpFlatMaskToCameraMaskInverse(const cv::Mat& flatMask,
                                        const CameraView& cam,
                                        const RigFace<3>& face,
                                        const std::vector<Eigen::Vector3d>& vertices,
                                        const FlatTriangleMap& flatMap,
                                        int cameraWidth,
                                        int cameraHeight)
{
    if (flatMask.empty()) {
        return {};
    }

    if (flatMask.type() != CV_8UC1) {
        throw std::runtime_error("WarpFlatMaskToCameraMaskInverse: flatMask must be CV_8UC1");
    }

    cv::Mat cameraMask(cameraHeight, cameraWidth, CV_8UC1, cv::Scalar(0));

    const Eigen::Vector3d a =
        vertices[static_cast<std::size_t>(face.indices[0])].normalized();
    const Eigen::Vector3d b =
        vertices[static_cast<std::size_t>(face.indices[1])].normalized();
    const Eigen::Vector3d c =
        vertices[static_cast<std::size_t>(face.indices[2])].normalized();

    int unprojectFail = 0;
    int baryFail = 0;
    int outsideFlat = 0;
    int flatMaskHit = 0;

    for (int cy = 0; cy < cameraHeight; ++cy) {
        for (int cx = 0; cx < cameraWidth; ++cx) {
            Eigen::Vector3d rayCam;
            if (!cam.model->unproject(cv::Point2d(cx, cy), rayCam)) {
                ++unprojectFail;
                continue;
            }

            if (rayCam.norm() <= 1e-12) {
                ++unprojectFail;
                continue;
            }

            rayCam.normalize();

            // Camera ray -> world ray.
            const Eigen::Vector3d rayWorld =
                (cam.Rcw * rayCam).normalized();

            Eigen::Vector3d bary;
            if (!BarycentricOnFace3D(rayWorld, a, b, c, bary)) {
                ++baryFail;
                continue;
            }

            // Optional tolerance lets edge pixels survive numerical noise.
            //constexpr double kBaryTol = 1e-4;
            //if (bary.x() < -kBaryTol ||
                //bary.y() < -kBaryTol ||
                //bary.z() < -kBaryTol) {
                //++outsideFlat;
                //continue;
            //}
            constexpr double kMaxBaryAbs = 4.0;
			if (std::abs(bary.x()) > kMaxBaryAbs ||
			    std::abs(bary.y()) > kMaxBaryAbs ||
			    std::abs(bary.z()) > kMaxBaryAbs) {
			    ++outsideFlat;
			    continue;
			}

            const cv::Point2d flatUv =
                bary.x() * flatMap.p[0] +
                bary.y() * flatMap.p[1] +
                bary.z() * flatMap.p[2];

            const int fx = static_cast<int>(std::round(flatUv.x));
            const int fy = static_cast<int>(std::round(flatUv.y));

            if (fx < 0 || fy < 0 ||
                fx >= flatMask.cols || fy >= flatMask.rows) {
                ++outsideFlat;
                continue;
            }

            if (flatMask.at<std::uint8_t>(fy, fx) == 0) {
                continue;
            }

            cameraMask.at<std::uint8_t>(cy, cx) = 255;
            ++flatMaskHit;
        }
    }

    // Small dilation only to cover sampling/rounding gaps.
    cv::Mat dilated;
    constexpr int kDilatePx = 2;
    const int k = 2 * kDilatePx + 1;

    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

    cv::dilate(cameraMask, dilated, kernel);

    std::cout << "[WarpFlatMaskToCameraMaskInverse] "
              << "flatNZ=" << CountMaskNonZero(flatMask)
              << " unprojectFail=" << unprojectFail
              << " baryFail=" << baryFail
              << " outsideFlat=" << outsideFlat
              << " flatMaskHit=" << flatMaskHit
              << " cameraMaskNZ=" << CountMaskNonZero(cameraMask)
              << " dilatedNZ=" << CountMaskNonZero(dilated)
              << '\n';

    return dilated;
}


[[maybe_unused]]
cv::Mat OverlayMaskOnImage(const cv::Mat& image, const cv::Mat& mask)
{
    if (image.empty()) {
        return {};
    }

    if (mask.empty()) {
        return image.clone();
    }

    if (mask.rows != image.rows || mask.cols != image.cols || mask.type() != CV_8UC1) {
        throw std::runtime_error("OverlayMaskOnImage: mask must be CV_8UC1 and same size as image");
    }

    cv::Mat out = image.clone();

    for (int y = 0; y < out.rows; ++y) {
        for (int x = 0; x < out.cols; ++x) {
            if (mask.at<std::uint8_t>(y, x) != 0) {
                cv::Vec3b& px = out.at<cv::Vec3b>(y, x);
                px = cv::Vec3b(
                    static_cast<std::uint8_t>(0.5 * px[0] + 0.5 * 0),
                    static_cast<std::uint8_t>(0.5 * px[1] + 0.5 * 255),
                    static_cast<std::uint8_t>(0.5 * px[2] + 0.5 * 255));
            }
        }
    }

    return out;
}

//cv::Mat RenderCameraIntersectionDebug(const CameraView& cam,
                                      //int width,
                                      //int height)
//{
    ////return RenderCameraMaskDebug(cam, width, height);
//}


template<std::size_t N>
void DebugPrintRigAdjacency(const std::vector<RigFace<N>>& faces)
{
    const auto graph = BuildFaceNeighborGraph(faces);

    std::cout << "\n[DebugPrintRigAdjacency]\n";

    for (size_t i = 0; i < graph.size(); ++i) {
        std::cout << "codeFace[" << i << "] neighbors = ";
        for (int n : graph[i]) {
            std::cout << n << ' ';
        }
        std::cout << '\n';
    }

    std::cout << std::endl;
}

void DebugCanonicalProjection(const ICameraModel& model)
{
    const std::vector<std::pair<std::string, Eigen::Vector3d>> rays = {
        {"forward", {0, 0, 1}},
        {"right",   {1, 0, 0}},
        {"left",    {-1, 0, 0}},
        {"up",      {0, 1, 0}},
        {"down",    {0, -1, 0}},
        {"fr",      Eigen::Vector3d(0.4, 0.0, 0.9).normalized()},
        {"fl",      Eigen::Vector3d(-0.4, 0.0, 0.9).normalized()},
        {"fu",      Eigen::Vector3d(0.0, 0.4, 0.9).normalized()},
        {"fd",      Eigen::Vector3d(0.0, -0.4, 0.9).normalized()},
    };

    for (const auto& [name, ray] : rays) {
        const ProjectionResult p = model.project(ray);
        std::cout << name
                  << " valid=" << p.valid
                  << " uv=(" << p.uv.x << ", " << p.uv.y << ")\n";
    }
}

void DebugProjectUnprojectConsistency(const ICameraModel& model)
{
    const std::vector<Eigen::Vector3d> rays = {
        Eigen::Vector3d(0.0,  0.0,  1.0).normalized(),
        Eigen::Vector3d(0.2,  0.0,  0.98).normalized(),
        Eigen::Vector3d(-0.2, 0.0,  0.98).normalized(),
        Eigen::Vector3d(0.0,  0.2,  0.98).normalized(),
        Eigen::Vector3d(0.0, -0.2,  0.98).normalized(),
        Eigen::Vector3d(0.3,  0.2,  0.93).normalized(),
        Eigen::Vector3d(-0.3, 0.2,  0.93).normalized(),
        Eigen::Vector3d(0.3, -0.2,  0.93).normalized(),
        Eigen::Vector3d(-0.3,-0.2,  0.93).normalized(),
    };

    std::cout << "\n[DebugProjectUnprojectConsistency]\n";

    for (size_t i = 0; i < rays.size(); ++i) {
        const Eigen::Vector3d ray0 = rays[i];
        const ProjectionResult proj = model.project(ray0);

        std::cout << "ray[" << i << "]=" << ray0.transpose();

        if (!proj.valid) {
            std::cout << " -> project invalid uv=("
                      << proj.uv.x << ", " << proj.uv.y << ")\n";
            continue;
        }

        Eigen::Vector3d ray1;
        const bool ok = model.unproject(proj.uv, ray1);
        if (!ok) {
            std::cout << " -> unproject failed uv=("
                      << proj.uv.x << ", " << proj.uv.y << ")\n";
            continue;
        }

        const double dot = std::clamp(ray0.dot(ray1.normalized()), -1.0, 1.0);
        const double angleDeg = std::acos(dot) * 180.0 / M_PI;

        std::cout << " -> uv=(" << proj.uv.x << ", " << proj.uv.y << ")"
                  << " ray1=(" << ray1.transpose() << ")"
                  << " dot=" << dot
                  << " angleDeg=" << angleDeg
                  << '\n';
    }

    std::cout << std::endl;
}

void DebugCameraPose(const CameraView& cam, const std::string& name)
{
    const Eigen::Vector3d right   = cam.Rcw.col(0);
    const Eigen::Vector3d up      = cam.Rcw.col(1);
    const Eigen::Vector3d forward = cam.Rcw.col(2);

    std::cout << "\n[DebugCameraPose] " << name << '\n';
    std::cout << "  right   = (" << right.transpose() << ")\n";
    std::cout << "  up      = (" << up.transpose() << ")\n";
    std::cout << "  forward = (" << forward.transpose() << ")\n";
}

std::vector<CameraView> CreateCameraViews(const std::vector<CameraConfig>& configs,
                                          const FisheyeParams& fisheyeParams,
                                          const RigData<3>& rig)
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

        CameraView cam = makeCameraView(emptyImage,
                                        emptyMask,
                                        emptyMask,
                                        model,
                                        cfg.Rcw,
                                        cfg.imageRotation);

        cam.faceIndex = cfg.faceIndex;
        //cam.sensorValidMask = cfg.sensorValidMask.clone();
        cam.sensorValidMask = cv::Mat();

        cam.localStreamIndex  = cfg.localStreamIndex;
        cam.localEdgeIndex    = cfg.localEdgeIndex;
        cam.neighborFaceIndex = cfg.neighborFaceIndex;
        cam.edgeIndex         = cfg.edgeIndex;
        cam.moduleIndex       = cfg.moduleIndex;

        if (cam.faceIndex < 0 ||
            cam.faceIndex >= static_cast<int>(rig.faces.size())) {
            throw std::runtime_error("CreateCameraViews: invalid faceIndex");
        }

        cameras.push_back(std::move(cam));
    }

    return cameras;
}

//int PickBestNeighborFace(const std::vector<RigFace>& faces,
                         //int anchorFaceIndex,
                         //const std::vector<std::vector<int>>& graph)
//{
    //if (anchorFaceIndex < 0 ||
        //static_cast<size_t>(anchorFaceIndex) >= faces.size()) {
        //throw std::runtime_error("anchorFaceIndex out of range");
    //}

    //const auto& neighbors = graph[static_cast<size_t>(anchorFaceIndex)];
    //if (neighbors.empty()) {
        //throw std::runtime_error("Anchor face has no neighbors");
    //}

    //const Eigen::Vector3d forward =
        //faces[static_cast<size_t>(anchorFaceIndex)].lookDir.normalized();

    //Eigen::Vector3d worldUp(0.0, 1.0, 0.0);
    //if (std::abs(forward.dot(worldUp)) > 0.95) {
        //worldUp = Eigen::Vector3d(1.0, 0.0, 0.0);
    //}

    //const Eigen::Vector3d right = worldUp.cross(forward).normalized();
    //const Eigen::Vector3d up = forward.cross(right).normalized();

    //double bestScore = -std::numeric_limits<double>::infinity();
    //int bestIndex = -1;

    //for (int candidate : neighbors) {
        //const Eigen::Vector3d candDir =
            //faces[static_cast<size_t>(candidate)].lookDir.normalized();

        //// Difference projected into the local tangent frame at the anchor face
        //const Eigen::Vector3d delta = candDir - forward;

        //const double side = delta.dot(right); // +right, -left
        //const double elev = delta.dot(up);    // +up, -down

        //// We want: strongly left, minimally up/down
        //const double score = (-side * 10.0) - std::abs(elev);

        //std::cout << "[neighbor pick] candidate=" << candidate
                  //<< " side=" << side
                  //<< " elev=" << elev
                  //<< " score=" << score
                  //<< '\n';

        //if (score > bestScore) {
            //bestScore = score;
            //bestIndex = candidate;
        //}
    //}

    //if (bestIndex < 0) {
        //throw std::runtime_error("Failed to pick neighboring face");
    //}

    //return bestIndex;
//}

//int PickFaceClosestToDirection(const std::vector<RigFace>& faces,
                               //const Eigen::Vector3d& direction)
//{
    //if (faces.empty()) {
        //throw std::runtime_error("No rig faces available");
    //}

    //const Eigen::Vector3d dir = direction.normalized();

    //double bestDot = -std::numeric_limits<double>::infinity();
    //int bestIndex = -1;

    //for (size_t i = 0; i < faces.size(); ++i) {
        //const double dot = faces[i].lookDir.dot(dir);
        //if (dot > bestDot) {
            //bestDot = dot;
            //bestIndex = static_cast<int>(i);
        //}
    //}

    //if (bestIndex < 0) {
        //throw std::runtime_error("Failed to pick face closest to direction");
    //}

    //return bestIndex;
//}

Eigen::Matrix3d RotationAligningAToB(const Eigen::Vector3d& a,
                                     const Eigen::Vector3d& b)
{
    const Eigen::Vector3d an = a.normalized();
    const Eigen::Vector3d bn = b.normalized();

    const double dot = std::clamp(an.dot(bn), -1.0, 1.0);

    if (dot > 1.0 - 1e-12) {
        return Eigen::Matrix3d::Identity();
    }

    if (dot < -1.0 + 1e-12) {
        Eigen::Vector3d axis = Eigen::Vector3d::UnitY().cross(an);
        if (axis.norm() < 1e-12) {
            axis = Eigen::Vector3d::UnitX().cross(an);
        }
        axis.normalize();
        return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
    }

    const Eigen::Vector3d axis = an.cross(bn).normalized();
    const double angle = std::acos(dot);
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

//int PickNeighborByAlignedDirection(const std::vector<RigFace>& faces,
                                   //const std::vector<std::vector<int>>& graph,
                                   //int anchorFace,
                                   //const Eigen::Matrix3d& Rrig)
//{
    //const auto& neighbors = graph[static_cast<size_t>(anchorFace)];

    //int bestFace = -1;
    //double bestY = -std::numeric_limits<double>::infinity();

    //for (int f : neighbors) {
        //const Eigen::Vector3d aligned = Rrig * faces[static_cast<size_t>(f)].lookDir;

        //std::cout << "[aligned neighbor] face=" << f
                  //<< " aligned=(" << aligned.transpose() << ")\n";

        //if (aligned.y() > bestY) {
            //bestY = aligned.y();
            //bestFace = f;
        //}
    //}

    //if (bestFace < 0) {
        //throw std::runtime_error("Failed to pick aligned neighbor face");
    //}

    //return bestFace;
//}

template<std::size_t N>
std::vector<int> BuildModuleFaceIndices(const std::vector<RigFace<N>>& faces,
                                        int moduleCount)
{
    if (moduleCount <= 0) {
        throw std::runtime_error("moduleCount must be > 0");
    }
    if (moduleCount > 2) {
        throw std::runtime_error("Current explicit mapping only handles 2 modules");
    }

    const auto graph = BuildFaceNeighborGraph(faces);

    // Keep face 6 as your anchor for now.
    const int anchorFace = 0;

    // Align anchor to pano center.
    //const Eigen::Matrix3d Rrig =
        //RotationAligningAToB(faces[static_cast<size_t>(anchorFace)].lookDir,
                             //Eigen::Vector3d(0.0, 0.0, 1.0));

    std::vector<int> out;
    out.push_back(anchorFace);

    if (moduleCount >= 2) {
        const int secondFace = 1;//anchorFace+6;
            //PickNeighborByAlignedDirection(faces, graph, anchorFace, Rrig);
        out.push_back(secondFace);
    }

    std::cout << "[face mapping] module0=" << out[0];
    if (out.size() > 1) {
        std::cout << " module1=" << out[1];
    }
    std::cout << '\n';

    return out;
}

//Eigen::Matrix3d MakeCameraRcwFromLookDirection(const Eigen::Vector3d& lookDir)
//{
    //const Eigen::Vector3d forward = lookDir.normalized();

    //Eigen::Vector3d worldUp(0.0, 1.0, 0.0);
    //if (std::abs(forward.dot(worldUp)) > 0.95) {
        //worldUp = Eigen::Vector3d(1.0, 0.0, 0.0);
    //}

    //const Eigen::Vector3d right = worldUp.cross(forward).normalized();
    //const Eigen::Vector3d up = forward.cross(right).normalized();

    //Eigen::Matrix3d Rcw;
    //Rcw.col(0) = right;
    //Rcw.col(1) = up;
    //Rcw.col(2) = forward;

    //return Rcw;
//}

//ImageRotation ImageRotationForModule(int moduleIndex)
//{
    //if (moduleIndex == 0) {
        //return ImageRotation::Rotate90CCW;
    //}
    //if (moduleIndex == 1) {
        //return ImageRotation::Rotate90CW;
    //}
    //return ImageRotation::None;
//}

template<std::size_t N>
void PrintRigAssignment(const std::vector<RigFace<N>>& faces,
                        const std::vector<int>& moduleFaceIndices)
{
    std::cout << "[rig] module assignments\n";

    for (size_t module = 0; module < moduleFaceIndices.size(); ++module) {
        const int faceIndex = moduleFaceIndices[module];
        const auto& face = faces[static_cast<size_t>(faceIndex)];

        std::cout << "  module[" << module << "] -> face[" << faceIndex
                  << "] dir=(" << face.lookDir.transpose() << ")\n";
    }

    for (size_t i = 1; i < moduleFaceIndices.size(); ++i) {
        const auto& a = faces[static_cast<size_t>(moduleFaceIndices[i - 1])];
        const auto& b = faces[static_cast<size_t>(moduleFaceIndices[i])];

        const double angleDeg =
            std::acos(std::clamp(a.lookDir.dot(b.lookDir), -1.0, 1.0)) * 180.0 / M_PI;

        std::cout << "  angle(module[" << (i - 1) << "], module[" << i
                  << "]) = " << angleDeg << " deg\n";
    }
}

Eigen::Matrix3d CameraModelToRigAlignment()
{
    // Start with Identity.
    // If the image is upside down and mirrored, the first thing to try is
    // a 180-degree rotation about camera Z.
     // return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Other candidates to try if needed:
   return Eigen::Matrix3d::Identity();
	// return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
	// return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

template<std::size_t N>
Eigen::Matrix3d MakeCameraRcwFromFace(const RigFace<N>& face,
                                      const std::vector<Eigen::Vector3d>& vertices)
{
    const Eigen::Vector3d v0 = vertices[face.indices[0]].normalized();
    const Eigen::Vector3d v1 = vertices[face.indices[1]].normalized();
    const Eigen::Vector3d v2 = vertices[face.indices[2]].normalized();

    const Eigen::Vector3d forward = face.lookDir.normalized();

    // Use one face edge to define a stable in-face reference direction.
    Eigen::Vector3d edge = (v1 - v0);
    edge -= forward * edge.dot(forward);
    if (edge.norm() < 1e-12) {
        edge = (v2 - v0);
        edge -= forward * edge.dot(forward);
    }
    if (edge.norm() < 1e-12) {
        throw std::runtime_error("Failed to construct face edge direction");
    }
    edge.normalize();

    // Build a right-handed camera frame:
    // camera +Z = forward
    // camera +Y = projected face-edge reference
    // camera +X = +Y x +Z
    Eigen::Vector3d up = edge;
    Eigen::Vector3d right = up.cross(forward).normalized();
    up = forward.cross(right).normalized();

    Eigen::Matrix3d Rcw;
    Rcw.col(0) = right;
    Rcw.col(1) = up;
    Rcw.col(2) = forward;
    return Rcw;
}

//std::array<int, 2> FindSharedEdgeVertices(const RigFace& a, const RigFace& b)
//{
    //std::vector<int> shared;
    //shared.reserve(2);

    //for (int va : a.vi) {
        //for (int vb : b.vi) {
            //if (va == vb) {
                //shared.push_back(va);
            //}
        //}
    //}

    //if (shared.size() != 2) {
        //throw std::runtime_error("Faces do not share exactly one edge");
    //}

    //return {shared[0], shared[1]};
//}

template<std::size_t N>
Eigen::Vector3d MakeFaceSeamTangent(const RigFace<N>& face,
                                    const std::vector<Eigen::Vector3d>& vertices,
                                    std::size_t vi0,
                                    std::size_t vi1)
{
    if (vi0 >= vertices.size() || vi1 >= vertices.size()) {
        throw std::runtime_error("Shared edge vertex index out of range");
    }

    Eigen::Vector3d edgeWorld =
        (vertices[vi1] - vertices[vi0]).normalized();

    const Eigen::Vector3d forward = face.lookDir.normalized();

    // Project the shared edge into the face tangent plane.
    edgeWorld -= forward * forward.dot(edgeWorld);

    const double norm = edgeWorld.norm();
    if (norm < 1e-12) {
        throw std::runtime_error("Degenerate seam tangent");
    }

    return edgeWorld / norm;
}

Eigen::Matrix3d MakeCameraRcwFromForwardAndUpHint(const Eigen::Vector3d& forward,
                                                  const Eigen::Vector3d& upHint)
{
    const Eigen::Vector3d z = forward.normalized();

    Eigen::Vector3d y = upHint - z * z.dot(upHint);
    if (y.norm() < 1e-12) {
        throw std::runtime_error("Degenerate up hint");
    }
    y.normalize();

    Eigen::Vector3d x = y.cross(z).normalized();
    y = z.cross(x).normalized();

    Eigen::Matrix3d Rcw;
    Rcw.col(0) = x;
    Rcw.col(1) = y;
    Rcw.col(2) = z;
    return Rcw;
}


Eigen::Matrix3d CameraRollDeg(double deg)
{
    const double rad = deg * M_PI / 180.0;
    return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double CameraLocalRollDegForModule(int module)
{
    switch (module) {
    case 0: return 0.0;
    case 1: return 180.0;
    case 2: return 0.0;
    case 3: return 0.0;
    default: return 0.0;
    }
}

template <std::size_t N>
bool IsLocalEdgeReversedRelativeToTopology(const MeshTopology<N>& topo,
                                           int faceIndex,
                                           int localEdgeIndex,
                                           int edgeIndex)
{
    if (faceIndex < 0 ||
        faceIndex >= static_cast<int>(topo.faces.size())) {
        throw std::runtime_error("faceIndex out of range");
    }

    if (localEdgeIndex < 0 ||
        localEdgeIndex >= static_cast<int>(N)) {
        throw std::runtime_error("localEdgeIndex out of range");
    }

    if (edgeIndex < 0 ||
        edgeIndex >= static_cast<int>(topo.edges.size())) {
        throw std::runtime_error("edgeIndex out of range");
    }

    const auto& faceVerts =
        topo.faces[static_cast<std::size_t>(faceIndex)];

    const std::size_t localV0 =
        faceVerts[static_cast<std::size_t>(localEdgeIndex)];

    const std::size_t localV1 =
        faceVerts[(static_cast<std::size_t>(localEdgeIndex) + 1) % N];

    const auto& edge =
        topo.edges[static_cast<std::size_t>(edgeIndex)];

    const std::size_t edgeV0 = edge.vertices[0];
    const std::size_t edgeV1 = edge.vertices[1];

    if (localV0 == edgeV0 && localV1 == edgeV1) {
        return false;
    }

    if (localV0 == edgeV1 && localV1 == edgeV0) {
        return true;
    }

    throw std::runtime_error("Local edge vertices do not match topology edge");
}

[[maybe_unused]]
double SeamTForCamera(const CameraConfig& cfg, double t)
{
    return cfg.seamDirectionReversed ? (1.0 - t) : t;
}


template<std::size_t N>
CameraSetup<N> makeCameraConfigs(int nApertureComputeModules)
{
    if (nApertureComputeModules <= 0) {
        throw std::runtime_error("nApertureComputeModules must be > 0");
    }

    constexpr RigGeometry kRigGeometry = RigGeometry::Icosahedron;

    MeshTopology<N> topo;
    std::vector<int> moduleFaceIndices;

    switch (kRigGeometry) {
    case RigGeometry::Icosahedron: {
        if constexpr (N != 3) {
            throw std::runtime_error("Icosahedron topology requires N=3");
        } else {
            IcosahedronTopology topologyBuilder;
            topo = topologyBuilder.Make();
            moduleFaceIndices =
                topologyBuilder.DefaultModuleOwningFaces(nApertureComputeModules);
        }
        break;
    }
    default:
        throw std::runtime_error("Unsupported rig geometry");
    }

    const RigData<N> rig = BuildRigDataFromTopology(topo);
    const auto& faces = rig.faces;
    const auto& vertices = rig.vertices;

    static bool printedFaces = false;
    if (!printedFaces) {
        DebugPrintRigFaces(faces);
        DebugPrintRigAdjacency(faces);
        printedFaces = true;
    }

    if (moduleFaceIndices.empty()) {
        throw std::runtime_error("No module face indices were generated");
    }

    if (static_cast<int>(moduleFaceIndices.size()) != nApertureComputeModules) {
        throw std::runtime_error("moduleFaceIndices size mismatch");
    }

    for (int faceIndex : moduleFaceIndices) {
        if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size())) {
            throw std::runtime_error("module face index out of range");
        }
    }

    PrintRigAssignment(faces, moduleFaceIndices);

    const RigFace<N>& anchorFace =
        faces[static_cast<std::size_t>(moduleFaceIndices[0])];

    const Eigen::Matrix3d Rrig =
        RotationAligningAToB(anchorFace.lookDir, Eigen::Vector3d(0.0, 0.0, 1.0));

    const Eigen::Matrix3d Ralign = CameraModelToRigAlignment();

    std::vector<Eigen::Matrix3d> moduleFaceRotations(
        static_cast<std::size_t>(nApertureComputeModules),
        Eigen::Matrix3d::Identity());

    if (nApertureComputeModules == 1) {
        const int faceIndex = moduleFaceIndices[0];
        const RigFace<N>& face = faces[static_cast<std::size_t>(faceIndex)];

        moduleFaceRotations[0] = MakeCameraRcwFromFace<N>(face, vertices);
    } else if (nApertureComputeModules == 2) {
        const int faceIndex0 = moduleFaceIndices[0];
        const int faceIndex1 = moduleFaceIndices[1];

        const RigFace<N>& face0 = faces[static_cast<std::size_t>(faceIndex0)];
        const RigFace<N>& face1 = faces[static_cast<std::size_t>(faceIndex1)];

        const auto sharedEdge = FindSharedEdgeVertices(face0, face1);

        Eigen::Vector3d seam0 =
            MakeFaceSeamTangent(face0, vertices, sharedEdge[0], sharedEdge[1]);
        Eigen::Vector3d seam1 =
            MakeFaceSeamTangent(face1, vertices, sharedEdge[0], sharedEdge[1]);

        if (seam0.dot(seam1) < 0.0) {
            seam1 = -seam1;
        }

        moduleFaceRotations[0] =
            MakeCameraRcwFromForwardAndUpHint(face0.lookDir, seam0);
        moduleFaceRotations[1] =
            MakeCameraRcwFromForwardAndUpHint(face1.lookDir, seam1);
    } else {
        for (int module = 0; module < nApertureComputeModules; ++module) {
            const int faceIndex = moduleFaceIndices[static_cast<std::size_t>(module)];
            const RigFace<N>& face = faces[static_cast<std::size_t>(faceIndex)];

            moduleFaceRotations[static_cast<std::size_t>(module)] =
                MakeCameraRcwFromFace(face, vertices);
        }
    }

    std::vector<CameraConfig> configs;
    configs.reserve(TotalCameraCount(nApertureComputeModules));

    constexpr std::size_t kFaceEdgeCount = N;
    static_assert(kCamerasPerModule == kFaceEdgeCount + 1,
                  "Expected one non-overlap stream plus one overlap stream per face edge");

    for (int module = 0; module < nApertureComputeModules; ++module) {
        const int faceIndex = moduleFaceIndices[static_cast<std::size_t>(module)];
        const RigFace<N>& face = faces[static_cast<std::size_t>(faceIndex)];

        const Eigen::Matrix3d& Rface =
            moduleFaceRotations[static_cast<std::size_t>(module)];

        const ImageRotation imageRotation = ImageRotation::None;
        //Eigen::Matrix3d Rimg = Eigen::Matrix3d::Identity();

		//const ImageRotation imageRotation = ImageRotation::None;
		
		const double cameraRollDeg = CameraLocalRollDegForModule(module);
		const Eigen::Matrix3d Rimg = CameraRollDeg(cameraRollDeg);
		
		const Eigen::Matrix3d Rfinal = Rrig * Rface * Ralign * Rimg;

        const Eigen::Vector3d camRightWorld   = Rfinal.col(0);
        const Eigen::Vector3d camUpWorld      = Rfinal.col(1);
        const Eigen::Vector3d camForwardWorld = Rfinal.col(2);

        std::cout << "[camera config] module=" << module
                  << " faceIndex=" << faceIndex
                  << " lookDir=(" << face.lookDir.transpose() << ")"
                  << " camRightWorld=(" << camRightWorld.transpose() << ")"
                  << " camUpWorld=(" << camUpWorld.transpose() << ")"
                  << " camForwardWorld=(" << camForwardWorld.transpose() << ")"
                  << '\n';

        if (nApertureComputeModules == 2) {
            const int otherModule = (module == 0) ? 1 : 0;
            const int otherFaceIndex =
                moduleFaceIndices[static_cast<std::size_t>(otherModule)];
            const RigFace<N>& otherFace =
                faces[static_cast<std::size_t>(otherFaceIndex)];

            const auto sharedEdge = FindSharedEdgeVertices(face, otherFace);
            Eigen::Vector3d seam =
                MakeFaceSeamTangent(face, vertices, sharedEdge[0], sharedEdge[1]);

            std::cout << "  seam=(" << seam.transpose() << ")\n";
        }

        // Stream 0 = non-overlap interior of owning face
        {
            CameraConfig cfg;
            cfg.name = "module_" + std::to_string(module) + "_cam_0";
            cfg.device = "";
            cfg.sourceName = "";
            cfg.imageRotation = imageRotation;
            cfg.Rcw = Rfinal;
            cfg.moduleIndex = module;
            cfg.sensorValidMask = cv::Mat();
            cfg.faceIndex = faceIndex;

            cfg.localStreamIndex = 0;
            cfg.localEdgeIndex = -1;
            cfg.neighborFaceIndex = -1;
            cfg.edgeIndex = -1;
            cfg.seamDirectionReversed = false;

            configs.push_back(std::move(cfg));
        }

		// Streams 1..N = overlap regions, one per local face edge
		for (std::size_t localEdge = 0; localEdge < kFaceEdgeCount; ++localEdge) {
		    constexpr int kOverlapLocalEdgeOffset = 0; // try +1, then -1 if wrong

			const int maskSlot = static_cast<int>(localEdge);
			const int edgeCount = static_cast<int>(kFaceEdgeCount);
			
			const int localEdgeIndex =
			    (maskSlot + kOverlapLocalEdgeOffset + edgeCount) % edgeCount;
		
		    const int neighborFaceIndex =
		        topo.faceNeighborIndices[static_cast<std::size_t>(faceIndex)]
		                                [static_cast<std::size_t>(localEdgeIndex)];
		
		    const int edgeIndex =
		        topo.faceEdgeIndices[static_cast<std::size_t>(faceIndex)]
		                            [static_cast<std::size_t>(localEdgeIndex)];
		
		    if (neighborFaceIndex < 0) {
		        throw std::runtime_error(
		            "Missing neighborFaceIndex for face " + std::to_string(faceIndex) +
		            " localEdge " + std::to_string(localEdgeIndex));
		    }
		
		    if (edgeIndex < 0) {
		        throw std::runtime_error(
		            "Missing edgeIndex for face " + std::to_string(faceIndex) +
		            " localEdge " + std::to_string(localEdgeIndex));
		    }
		
		    const bool seamDirectionReversed =
		        IsLocalEdgeReversedRelativeToTopology<N>(
		            topo,
		            faceIndex,
		            localEdgeIndex,
		            edgeIndex);
		
		    const auto& faceVerts =
		        topo.faces[static_cast<std::size_t>(faceIndex)];
		
		    const std::size_t localV0 =
		        faceVerts[static_cast<std::size_t>(localEdgeIndex)];
		
		    const std::size_t localV1 =
		        faceVerts[(static_cast<std::size_t>(localEdgeIndex) + 1) % N];
		
		    const auto& topoEdge =
		        topo.edges[static_cast<std::size_t>(edgeIndex)];
		
		    CameraConfig cfg;
		    cfg.name = "module_" + std::to_string(module) + "_cam_" +
		               std::to_string(static_cast<int>(localEdge) + 1);
		
		    cfg.device = "";
		    cfg.sourceName = "";
		    cfg.imageRotation = imageRotation;
		    cfg.Rcw = Rfinal;
		    cfg.moduleIndex = module;
		    cfg.sensorValidMask = cv::Mat();
		    cfg.faceIndex = faceIndex;
		
			cfg.localStreamIndex = maskSlot + 1;
			cfg.localEdgeIndex = localEdgeIndex;
			cfg.neighborFaceIndex = neighborFaceIndex;
			cfg.edgeIndex = edgeIndex;
		    cfg.seamDirectionReversed = seamDirectionReversed;
		
			std::cout << "[camera config overlap] "
			          << "cfg.name=" << cfg.name
			          << " module=" << cfg.moduleIndex
			          << " faceIndex=" << cfg.faceIndex
			          << " localStreamIndex=" << cfg.localStreamIndex
			          << " maskSlot=" << maskSlot
			          << " edgeOffset=" << kOverlapLocalEdgeOffset
			          << " mappedLocalEdgeIndex=" << cfg.localEdgeIndex
			          << " localEdge=(" << localV0 << " -> " << localV1 << ")"
			          << " topologyEdge=(" << topoEdge.vertices[0]
			          << " -> " << topoEdge.vertices[1] << ")"
			          << " seamDirectionReversed="
			          << (cfg.seamDirectionReversed ? "true" : "false")
			          << " neighborFaceIndex=" << cfg.neighborFaceIndex
			          << " edgeIndex=" << cfg.edgeIndex
			          << '\n';
		
		    configs.push_back(std::move(cfg));
		}
    }

    CameraSetup<N> setup;
	setup.configs = std::move(configs);
	setup.rig = rig;
	return setup;
}

//std::vector<CameraConfig> makeCameraConfigs(int nApertureComputeModules)
//{
    //if (nApertureComputeModules <= 0) {
        //throw std::runtime_error("nApertureComputeModules must be > 0");
    //}

    //constexpr RigGeometry kRigGeometry = RigGeometry::Icosahedron;

    //const auto faces = MakeRigFaceSet(kRigGeometry);
    //const auto moduleFaceIndices = BuildModuleFaceIndices(faces, nApertureComputeModules);

    //PrintRigAssignment(faces, moduleFaceIndices);

    //std::vector<Eigen::Matrix3d> moduleRotations;
    //moduleRotations.reserve(static_cast<size_t>(nApertureComputeModules));

    //for (int module = 0; module < nApertureComputeModules; ++module) {
        //const int faceIndex = moduleFaceIndices[static_cast<size_t>(module)];
        //moduleRotations.push_back(
            //MakeCameraRcwFromLookDirection(faces[static_cast<size_t>(faceIndex)].lookDir));
    //}

    //std::vector<CameraConfig> configs;
    //configs.reserve(TotalCameraCount(nApertureComputeModules));

    //for (int module = 0; module < nApertureComputeModules; ++module) {
        //for (size_t localCam = 0; localCam < kCamerasPerModule; ++localCam) {
            //CameraConfig cfg{
                //"module_" + std::to_string(module) + "_cam_" + std::to_string(localCam),
                //"",
                //ImageRotationForModule(module),
                //moduleRotations[static_cast<size_t>(module)],
            //};

            //configs.push_back(cfg);
        //}
    //}

    //return configs;
//}





//Eigen::Matrix3d CameraRollDeg(double deg)
//{
    //const double rad = deg * M_PI / 180.0;
    //return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//}

//std::vector<CameraConfig> makeCameraConfigs(int nApertureComputeModules)
//{
    //if (nApertureComputeModules <= 0) {
        //throw std::runtime_error("nApertureComputeModules must be > 0");
    //}

    //constexpr RigGeometry kRigGeometry = RigGeometry::Icosahedron;

    //const RigData rig = MakeRigData(kRigGeometry);
    //const auto& faces = rig.faces;
    //const auto& vertices = rig.vertices;

    //static bool printedFaces = false;
    //if (!printedFaces) {
        //DebugPrintRigFaces(faces);
        //DebugPrintRigAdjacency(faces);
        //printedFaces = true;
    //}

    //const std::vector<int> moduleFaceIndices =
        //BuildModuleFaceIndices(faces, nApertureComputeModules);

    //if (moduleFaceIndices.empty()) {
        //throw std::runtime_error("No module face indices were generated");
    //}

    //for (int faceIndex : moduleFaceIndices) {
        //if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size())) {
            //throw std::runtime_error("module face index out of range");
        //}
    //}

    //PrintRigAssignment(faces, moduleFaceIndices);

    //// Align the anchor module face to pano center (+Z).
    //const RigFace& anchorFace =
        //faces[static_cast<size_t>(moduleFaceIndices[0])];

    //const Eigen::Matrix3d Rrig =
        //RotationAligningAToB(anchorFace.lookDir, Eigen::Vector3d(0.0, 0.0, 1.0));

    //const Eigen::Matrix3d Ralign = CameraModelToRigAlignment();

    //std::vector<CameraConfig> configs;
    //configs.reserve(TotalCameraCount(nApertureComputeModules));

    //for (int module = 0; module < nApertureComputeModules; ++module) {
        //const int faceIndex = moduleFaceIndices[static_cast<size_t>(module)];
        //const RigFace& face = faces[static_cast<size_t>(faceIndex)];

        //// Full face-based orientation, including roll from triangle edge.
        //const Eigen::Matrix3d Rface =
            //MakeCameraRcwFromFace(face, vertices);

        //// Keep debug path simple: no image-space rotation.
        //const ImageRotation imageRotation = ImageRotation::None;
        //const Eigen::Matrix3d Rimg = Eigen::Matrix3d::Identity();

        //const Eigen::Matrix3d Rfinal = Rrig * Rface * Ralign * Rimg;

        //const Eigen::Vector3d camRightWorld   = Rfinal.col(0);
        //const Eigen::Vector3d camUpWorld      = Rfinal.col(1);
        //const Eigen::Vector3d camForwardWorld = Rfinal.col(2);

        //std::cout << "[camera config] module=" << module
                  //<< " faceIndex=" << faceIndex
                  //<< " lookDir=(" << face.lookDir.transpose() << ")"
                  //<< " camRightWorld=(" << camRightWorld.transpose() << ")"
                  //<< " camUpWorld=(" << camUpWorld.transpose() << ")"
                  //<< " camForwardWorld=(" << camForwardWorld.transpose() << ")"
                  //<< '\n';

        //for (size_t localCam = 0; localCam < kCamerasPerModule; ++localCam) {
            //configs.push_back(CameraConfig{
                //"module_" + std::to_string(module) + "_cam_" + std::to_string(localCam),
                //"",
                //imageRotation,
                //Rfinal,
            //});
        //}
    //}

    //return configs;
//}



//void updateCameraImages(std::vector<CameraView>& cameras,
                        //const std::vector<CameraConfig>& configs,
                        //std::vector<LiveCameraState>& liveCameras)
//{
    //if (cameras.size() != configs.size() || cameras.size() != liveCameras.size()) {
        //throw std::runtime_error("Camera/config/state size mismatch");
    //}

    //for (size_t i = 0; i < cameras.size(); ++i) {
        //cv::Mat latest;
        //if (!tryGetLatestFrame(liveCameras[i].frame, latest)) {
            //continue;
        //}

        //if (!liveCameras[i].hasMask) {
            //throw std::runtime_error("Missing valid mask for logical camera " +
                                     //std::to_string(i));
        //}

        //const ImageRotation rotation = configs[i].imageRotation;
        //cameras[i].image = applyImageRotation(latest, rotation);

        //const cv::Mat rotatedValidMask =
            //applyImageRotation(liveCameras[i].validMask, rotation);

        //if (liveCameras[i].role == LogicalStreamRole::NonOverlap) {
            //cameras[i].maskNonOverlap = rotatedValidMask;
            //cameras[i].maskOverlap =
                //cv::Mat::zeros(rotatedValidMask.rows, rotatedValidMask.cols, CV_8UC1);
        //} else {
            //cameras[i].maskNonOverlap =
                //cv::Mat::zeros(rotatedValidMask.rows, rotatedValidMask.cols, CV_8UC1);
            //cameras[i].maskOverlap = rotatedValidMask;
        //}
    //}
//}

[[maybe_unused]]
int CountNonZeroSafe(const cv::Mat& mask)
{
    return mask.empty() ? 0 : cv::countNonZero(mask);
}

void updateCameraImages(std::vector<CameraView>& cameras,
                        const std::vector<CameraConfig>& configs,
                        std::vector<LiveCameraState>& liveCameras,
                        const RigData<3>& rig)
{
    if (cameras.size() != configs.size() ||
        cameras.size() != liveCameras.size()) {
        throw std::runtime_error("Camera/config/state size mismatch");
    }

    static std::vector<bool> warpedMaskReady;
    static std::vector<bool> savedRawFlatMask;
    static std::vector<bool> flatMapReady;
    static std::vector<FlatTriangleMap> flatMapsByModule;

    if (warpedMaskReady.size() != cameras.size()) {
        warpedMaskReady.assign(cameras.size(), false);
    }

    if (savedRawFlatMask.size() != cameras.size()) {
        savedRawFlatMask.assign(cameras.size(), false);
    }

    std::size_t moduleCount = 0;
    for (const CameraConfig& cfg : configs) {
        if (cfg.moduleIndex >= 0) {
            moduleCount =
                std::max(moduleCount,
                         static_cast<std::size_t>(cfg.moduleIndex + 1));
        }
    }

    if (flatMapReady.size() != moduleCount) {
        flatMapReady.assign(moduleCount, false);
        flatMapsByModule.resize(moduleCount);
    }

    // First pass: update images and build flat triangle maps from stream 0.
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        CameraView& cam = cameras[i];
        const CameraConfig& cfg = configs[i];
        LiveCameraState& live = liveCameras[i];

        cv::Mat latest;
        const bool haveFrame = tryGetLatestFrame(live.frame, latest);

        if (haveFrame) {
            cam.image = latest;
        } else {
            std::cout << "[WARN] Missing frame for cam=" << i
                      << " module=" << cam.moduleIndex
                      << " stream=" << cfg.localStreamIndex
                      << '\n';
        }

        if (cfg.localStreamIndex == 0 &&
            cfg.moduleIndex >= 0 &&
            !live.validMask.empty()) {
            const std::size_t moduleIndex =
                static_cast<std::size_t>(cfg.moduleIndex);

            if (!flatMapReady[moduleIndex]) {
                flatMapsByModule[moduleIndex] =
                    ExtractFlatTriangleMapFromMask(live.validMask);

                flatMapReady[moduleIndex] = true;

                std::cout << "[flat map] module=" << cfg.moduleIndex
                          << " p0=(" << flatMapsByModule[moduleIndex].p[0].x
                          << "," << flatMapsByModule[moduleIndex].p[0].y << ")"
                          << " p1=(" << flatMapsByModule[moduleIndex].p[1].x
                          << "," << flatMapsByModule[moduleIndex].p[1].y << ")"
                          << " p2=(" << flatMapsByModule[moduleIndex].p[2].x
                          << "," << flatMapsByModule[moduleIndex].p[2].y << ")"
                          << '\n';
            }
        }
    }

    // Second pass: warp masks once after flat maps are available.
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        if (warpedMaskReady[i]) {
            continue;
        }

        CameraView& cam = cameras[i];
        const CameraConfig& cfg = configs[i];
        LiveCameraState& live = liveCameras[i];

        if (cfg.moduleIndex < 0 ||
            static_cast<std::size_t>(cfg.moduleIndex) >= flatMapsByModule.size() ||
            !flatMapReady[static_cast<std::size_t>(cfg.moduleIndex)]) {
            continue;
        }

        if (live.validMask.empty()) {
            continue;
        }

        if (!savedRawFlatMask[i]) {
            cv::imwrite("/tmp/raw_flat_" + cfg.name + ".png", live.validMask);
            savedRawFlatMask[i] = true;

            std::cout << "[raw flat mask] "
                      << cfg.name
                      << " stream=" << cfg.localStreamIndex
                      << " nz=" << CountMaskNonZero(live.validMask)
                      << '\n';
        }

        if (cfg.faceIndex < 0 ||
            cfg.faceIndex >= static_cast<int>(rig.faces.size())) {
            throw std::runtime_error(
                "updateCameraImages: invalid faceIndex for " + cfg.name);
        }

        const RigFace<3>& face =
            rig.faces[static_cast<std::size_t>(cfg.faceIndex)];

        const FlatTriangleMap& flatMap =
            flatMapsByModule[static_cast<std::size_t>(cfg.moduleIndex)];

        cv::Mat warped =
            WarpFlatMaskToCameraMaskInverse(live.validMask,
                                            cam,
                                            face,
                                            rig.vertices,
                                            flatMap,
                                            kFrameWidth,
                                            kFrameHeight);

        cv::Mat zeroMask(kFrameHeight, kFrameWidth, CV_8UC1, cv::Scalar(0));

        if (cfg.localStreamIndex == 0) {
            cam.maskNonOverlap = std::move(warped);
            cam.maskOverlap = zeroMask;
        } else {
            cam.maskNonOverlap = zeroMask;
            cam.maskOverlap = std::move(warped);
        }

        // live.validMask is flat-space, so do not use it as sensorValidMask.
        cam.sensorValidMask = cv::Mat();

        warpedMaskReady[i] = true;

        std::cout << "[mask warp inverse] "
                  << cfg.name
                  << " stream=" << cfg.localStreamIndex
                  << " flatNZ=" << CountMaskNonZero(live.validMask)
                  << " nonOverlapNZ=" << CountMaskNonZero(cam.maskNonOverlap)
                  << " overlapNZ=" << CountMaskNonZero(cam.maskOverlap)
                  << '\n';
    }
}

//void updateCameraImages(std::vector<CameraView>& cameras,
                        //const std::vector<CameraConfig>& configs,
                        //std::vector<LiveCameraState>& liveCameras,
                        //const std::vector<cv::Mat>& moduleFaceMasks)
//{
    //if (cameras.size() != configs.size() || cameras.size() != liveCameras.size()) {
        //throw std::runtime_error("Camera/config/state size mismatch");
    //}

    //if (moduleFaceMasks.empty()) {
        //throw std::runtime_error("moduleFaceMasks is empty");
    //}

    //constexpr bool kUseFullDebugMasks = false;
    //constexpr bool kSaveDebugMasks = false;

    //static int debugSaveCount = 0;

    //auto makeFullMask = [](int rows, int cols) {
        //return cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
    //};

    //auto saveDebugMaskSet = [&](size_t i,
                                //const cv::Mat& effectiveMask,
                                //const cv::Mat& nonOverlapMask,
                                //const cv::Mat& overlapMask) {
        //if (!(kSaveDebugMasks && debugSaveCount < 5)) {
            //return;
        //}

        //cv::imwrite("debug_mask_effective_" + std::to_string(i) + ".png",
                    //effectiveMask);

        //cv::imwrite("debug_mask_nonoverlap_" + std::to_string(i) + ".png",
                    //nonOverlapMask);

        //cv::imwrite("debug_mask_overlap_" + std::to_string(i) + ".png",
                    //overlapMask);
    //};

    //for (size_t i = 0; i < cameras.size(); ++i) {
        //cv::Mat latest;
        //if (!tryGetLatestFrame(liveCameras[i].frame, latest)) {
            //continue;
        //}

        //if (!liveCameras[i].hasMask) {
            //throw std::runtime_error("Missing valid mask for logical camera " +
                                     //std::to_string(i));
        //}

        //const cv::Mat& validMask = liveCameras[i].validMask;
        //if (validMask.empty()) {
            //throw std::runtime_error("Valid mask is empty for logical camera " +
                                     //std::to_string(i));
        //}

        //const size_t module = i / kCamerasPerModule;
        //if (module >= moduleFaceMasks.size()) {
            //throw std::runtime_error("moduleFaceMasks index out of range for logical camera " +
                                     //std::to_string(i));
        //}

        //const cv::Mat& sphericalFaceMask = moduleFaceMasks[module];
        //if (sphericalFaceMask.empty()) {
            //throw std::runtime_error("Spherical face mask is empty for module " +
                                     //std::to_string(module));
        //}

        //if (validMask.size() != sphericalFaceMask.size()) {
            //throw std::runtime_error("Mask size mismatch for logical camera " +
                                     //std::to_string(i) +
                                     //": validMask=" +
                                     //std::to_string(validMask.cols) + "x" +
                                     //std::to_string(validMask.rows) +
                                     //" sphericalFaceMask=" +
                                     //std::to_string(sphericalFaceMask.cols) + "x" +
                                     //std::to_string(sphericalFaceMask.rows));
        //}

        //if (latest.cols != validMask.cols || latest.rows != validMask.rows) {
            //throw std::runtime_error("Image/mask size mismatch for logical camera " +
                                     //std::to_string(i) +
                                     //": image=" +
                                     //std::to_string(latest.cols) + "x" +
                                     //std::to_string(latest.rows) +
                                     //" mask=" +
                                     //std::to_string(validMask.cols) + "x" +
                                     //std::to_string(validMask.rows));
        //}

        //// Keep source pixels in native sensor orientation.
        //cameras[i].image = latest;

        //cv::Mat effectiveMask;
        //if (kUseFullDebugMasks) {
            //effectiveMask = makeFullMask(validMask.rows, validMask.cols);
        //} else {
            //cv::bitwise_and(validMask, sphericalFaceMask, effectiveMask);
        //}

        //const cv::Mat zeroMask =
            //cv::Mat::zeros(effectiveMask.rows, effectiveMask.cols, CV_8UC1);

        //if (liveCameras[i].role == LogicalStreamRole::NonOverlap) {
            //cameras[i].maskNonOverlap = effectiveMask;
            //cameras[i].maskOverlap = zeroMask;
        //} else {
            //cameras[i].maskNonOverlap = zeroMask;
            //cameras[i].maskOverlap = effectiveMask;
        //}

        //saveDebugMaskSet(i,
                         //effectiveMask,
                         //cameras[i].maskNonOverlap,
                         //cameras[i].maskOverlap);

        //std::cout << "[mask] camera " << i
                  //<< " module=" << module
                  //<< " role=" << static_cast<int>(liveCameras[i].role)
                  //<< " nonzero=" << cv::countNonZero(effectiveMask)
                  //<< " rows=" << effectiveMask.rows
                  //<< " cols=" << effectiveMask.cols
                  //<< '\n';
    //}

    //if (kSaveDebugMasks && debugSaveCount < 5) {
        //++debugSaveCount;
    //}
//}


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
                      std::vector<std::jthread>& controlThreads,
                      bool reverseModuleOrder)
{
	log_verbose("[StartPeerThreads]");
    std::cout << "[main] Starting per-peer frame/control threads\n";

    frameThreads.reserve(aperturePeers.size());
    controlThreads.reserve(aperturePeers.size());

    for (size_t peerIndex = 0; peerIndex < aperturePeers.size(); ++peerIndex) {
        const size_t moduleIndex =
            PeerToModuleIndex(peerIndex, aperturePeers.size(), reverseModuleOrder);

        AperturePeer<N>* peer = aperturePeers[peerIndex].get();

        frameThreads.emplace_back([peer, peerIndex, moduleIndex, &liveCameras]() {
            std::vector<uint16_t> raw;
            raw.reserve(kExpectedPixels);
            //std::uint64_t frameCounter = 0;
            std::uint64_t noFrameLoopCount = 0;
            std::uint64_t runFrameLoopFailCount = 0;

            for (;;) {
                if (!peer->RunFrameLoop()) {
                    ++runFrameLoopFailCount;
                    std::cerr << "[frame thread] RunFrameLoop failed for peer "
                              << peerIndex << " (module " << moduleIndex << ")\n";
                    if ((runFrameLoopFailCount % kSignatureLogEvery) == 0) {
                        std::cout << "[frame warning] peer=" << peerIndex
                                  << " module=" << moduleIndex
                                  << " RunFrameLoop failed for "
                                  << runFrameLoopFailCount
                                  << " consecutive attempts\n";
                    }
                    std::this_thread::sleep_for(kRetryDelay);
                    continue;
                }
                runFrameLoopFailCount = 0;

                bool gotAnyFrameThisLoop = false;
                bool gotStream0ThisLoop = false;
                bool gotStream1ThisLoop = false;

                for (size_t localCameraIndex = 0; localCameraIndex < (N + 1); ++localCameraIndex) {
                    raw.clear();

                    if (!tryCopyLatestFrameFromPeer(*peer, localCameraIndex, raw)) {
                        continue;
                    }
                    gotAnyFrameThisLoop = true;
                    if (localCameraIndex == 0) {
                        gotStream0ThisLoop = true;
                    }
                    if (localCameraIndex == 1) {
                        gotStream1ThisLoop = true;
                    }

                    if (static_cast<int>(raw.size()) != kExpectedPixels) {
                        std::cerr << "[frame thread] unexpected Bayer size for peer "
                                  << peerIndex << " (module " << moduleIndex << ")"
                                  << " localCameraIndex=" << localCameraIndex
                                  << " size=" << raw.size()
                                  << " expected=" << kExpectedPixels << '\n';
                        continue;
                    }

                    const size_t globalIndex =
                        GlobalCameraIndex(moduleIndex, localCameraIndex);

                    if (globalIndex >= liveCameras.size()) {
                        std::cerr << "[frame thread] globalIndex out of range (peer "
                                  << peerIndex << ", module " << moduleIndex << "): "
                                  << globalIndex
                                  << " liveCameras.size()=" << liveCameras.size() << '\n';
                        continue;
                    }

                    cv::Mat bgr = decodeBayer16ToBgr8(raw);
                    updateLatestFrame(liveCameras[globalIndex].frame, bgr);
                    gotAnyFrameThisLoop = true;
                    if (localCameraIndex == 0) {
                        gotStream0ThisLoop = true;
                    }
                    if (localCameraIndex == 1) {
                        gotStream1ThisLoop = true;
                    }

                    //const bool shouldLogSignature =
                        //((++frameCounter % kSignatureLogEvery) == 0) &&
                        //(localCameraIndex == 0 || localCameraIndex == 1);

                    //if (shouldLogSignature) {
                        //const cv::Scalar meanBgr = cv::mean(bgr);
                        //std::cout << "[frame signature] peer=" << peerIndex
                                  //<< " module=" << moduleIndex
                                  //<< " global=" << globalIndex
                                  //<< " local=" << localCameraIndex
                                  //<< " meanBGR=("
                                  //<< meanBgr[0] << ","
                                  //<< meanBgr[1] << ","
                                  //<< meanBgr[2] << ")\n";
                    //}
                }

                if (!gotAnyFrameThisLoop) {
                    ++noFrameLoopCount;
                    if ((noFrameLoopCount % kSignatureLogEvery) == 0) {
                        std::cout << "[frame warning] peer=" << peerIndex
                                  << " module=" << moduleIndex
                                  << " has not published any frames for "
                                  << noFrameLoopCount
                                  << " RunFrameLoop cycles\n";
                    }
                } else if (!gotStream0ThisLoop || !gotStream1ThisLoop) {
                    ++noFrameLoopCount;
                    if ((noFrameLoopCount % kSignatureLogEvery) == 0) {
                        std::cout << "[frame warning] peer=" << peerIndex
                                  << " module=" << moduleIndex
                                  << " missing stream(s):"
                                  << (gotStream0ThisLoop ? "" : " 0")
                                  << (gotStream1ThisLoop ? "" : " 1")
                                  << " for "
                                  << noFrameLoopCount
                                  << " RunFrameLoop cycles\n";
                    }
                } else {
                    noFrameLoopCount = 0;
                }
            }
        });

        controlThreads.emplace_back([peer, peerIndex, moduleIndex]() {
            for (;;) {
                if (!peer->RunControlLoop()) {
                    std::cerr << "[control thread] RunControlLoop failed for peer "
                              << peerIndex << " (module " << moduleIndex << ")\n";
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

template<std::size_t N>
void RunStitchLoop(std::vector<CameraView>& cameras,
                   const std::vector<CameraConfig>& configs,
                   std::vector<LiveCameraState>& liveCameras,
                   const std::vector<cv::Mat>& moduleFaceMasks,
                   const RigData<N>& rig)
{
    if (cameras.size() != configs.size() || cameras.size() != liveCameras.size()) {
        throw std::runtime_error("RunStitchLoop: camera/config/state size mismatch");
    }

    if (moduleFaceMasks.empty()) {
        throw std::runtime_error("RunStitchLoop: moduleFaceMasks is empty");
    }

    SphereStitchConfig stitchConfig;
    stitchConfig.outputWidth = 1456;
    stitchConfig.outputHeight = 1088;
    stitchConfig.blendPower = 4.0;

    // Benchmark mode: projection only.
    //stitchConfig.mode = StitchMode::ProjectionOnly;

    // Full spherical stitching mode:
     stitchConfig.mode = StitchMode::Blend;

    constexpr bool kVerboseStitchTiming = true;
    constexpr bool kSaveDebugImagesEveryFrame = false;
    constexpr bool kSaveOneShotDebugImages = true;
    constexpr bool kSleepBetweenFrames = false;

    using Clock = std::chrono::steady_clock;

    uint64_t frameNumber = 0;

    cv::namedWindow("Stitched Panorama", cv::WINDOW_NORMAL);
    cv::resizeWindow("Stitched Panorama", 1200, 800);

    auto saveOneShotDebugImages = [&](const cv::Mat& sourceImage) {
        static bool savedSource = false;

        if (!savedSource && !sourceImage.empty()) {
            cv::imwrite("/tmp/debug_camera0_source.png", sourceImage);
            savedSource = true;
        }
    };

    auto ms = [](const auto a, const auto b) -> double {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };

    struct StitchTimingStats {
        uint64_t frames = 0;
        uint64_t emptyPanoCount = 0;

        double waitMs = 0.0;
        double updateMs = 0.0;
        double oneShotDebugMs = 0.0;
        double setCamerasMs = 0.0;
        double stitchMs = 0.0;
        double debugSaveMs = 0.0;
        double overlayMs = 0.0;
        double displayMs = 0.0;
        double totalActiveMs = 0.0;

        Clock::time_point lastPrint = Clock::now();

        void reset(Clock::time_point now)
        {
            frames = 0;
            emptyPanoCount = 0;

            waitMs = 0.0;
            updateMs = 0.0;
            oneShotDebugMs = 0.0;
            setCamerasMs = 0.0;
            stitchMs = 0.0;
            debugSaveMs = 0.0;
            overlayMs = 0.0;
            displayMs = 0.0;
            totalActiveMs = 0.0;

            lastPrint = now;
        }
    };

    StitchTimingStats stats{};

    auto maybePrintTiming = [&]() {
        if constexpr (!kVerboseStitchTiming) {
            return;
        }

        const auto now = Clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - stats.lastPrint).count();

        if (elapsed < 1.0 || stats.frames == 0) {
            return;
        }

        std::cout << "[Stitch timing]"
                  << " frames=" << stats.frames
                  << " fps=" << static_cast<double>(stats.frames) / elapsed
                  << " empty_pano=" << stats.emptyPanoCount
                  << " wait_ms=" << stats.waitMs / stats.frames
                  << " update_ms=" << stats.updateMs / stats.frames
                  << " one_shot_debug_ms=" << stats.oneShotDebugMs / stats.frames
                  << " set_cameras_ms=" << stats.setCamerasMs / stats.frames
                  << " stitch_ms=" << stats.stitchMs / stats.frames
                  << " debug_save_ms=" << stats.debugSaveMs / stats.frames
                  << " overlay_ms=" << stats.overlayMs / stats.frames
                  << " display_ms=" << stats.displayMs / stats.frames
                  << " total_active_ms=" << stats.totalActiveMs / stats.frames
                  << '\n';

        stats.reset(now);
    };

    /*
     * Construct once.
     *
     * This precomputes world rays and face mappings once, then each loop
     * updates the copied CameraView images with setCameras().
     */
    SphereStitcher stitcher(cameras, stitchConfig, rig);

    for (;;) {
        const auto tLoopStart = Clock::now();

        if (!haveAnyFrames(liveCameras)) {
            std::this_thread::sleep_for(kNoFrameDelay);
            maybePrintTiming();
            continue;
        }

        const auto tHaveFrames = Clock::now();

        updateCameraImages(cameras, configs, liveCameras, rig);

        const auto tUpdateDone = Clock::now();

        if constexpr (kSaveOneShotDebugImages) {
            saveOneShotDebugImages(cameras[0].image);
        }

        const auto tOneShotDebugDone = Clock::now();

        stitcher.setCameras(cameras);

        const auto tSetCamerasDone = Clock::now();

		cv::Mat validMask;
		cv::Mat pano;
		
		if (stitchConfig.mode == StitchMode::ProjectionOnly) {
		    pano = stitcher.stitchProjectionOnlyFast(&validMask);
		} else {
		    pano = stitcher.stitch(&validMask);
		}
		
		const auto tStitchDone = Clock::now();

        ++stats.frames;
        stats.waitMs += ms(tLoopStart, tHaveFrames);
        stats.updateMs += ms(tHaveFrames, tUpdateDone);
        stats.oneShotDebugMs += ms(tUpdateDone, tOneShotDebugDone);
        stats.setCamerasMs += ms(tOneShotDebugDone, tSetCamerasDone);
        stats.stitchMs += ms(tSetCamerasDone, tStitchDone);

        if (pano.empty()) {
            ++stats.emptyPanoCount;

            stats.debugSaveMs += 0.0;
            stats.overlayMs += 0.0;
            stats.displayMs += 0.0;
            stats.totalActiveMs += ms(tHaveFrames, tStitchDone);

            maybePrintTiming();

            std::cerr << "[RunStitchLoop] stitch returned empty pano\n";

            if constexpr (kSleepBetweenFrames) {
                std::this_thread::sleep_for(kStitchDelay);
            }

            continue;
        }

        if constexpr (kSaveDebugImagesEveryFrame) {
            cv::imwrite("panorama.png", pano);
            if (!validMask.empty()) {
                cv::imwrite("valid_mask.png", validMask);
            }
        }

        const auto tDebugSaveDone = Clock::now();

        cv::Mat display = pano.clone();
        DrawPanoramaOverlay(display, frameNumber++, validMask);

        const auto tOverlayDone = Clock::now();

        cv::imshow("Stitched Panorama", display);
        const int key = cv::waitKey(1);

        const auto tDisplayDone = Clock::now();

        stats.debugSaveMs += ms(tStitchDone, tDebugSaveDone);
        stats.overlayMs += ms(tDebugSaveDone, tOverlayDone);
        stats.displayMs += ms(tOverlayDone, tDisplayDone);
        stats.totalActiveMs += ms(tHaveFrames, tDisplayDone);

        maybePrintTiming();

        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if constexpr (kSleepBetweenFrames) {
            std::this_thread::sleep_for(kStitchDelay);
        }
    }

    cv::destroyWindow("Stitched Panorama");
}

template <size_t N>
void InitializeCameraMasks(std::vector<std::unique_ptr<AperturePeer<N>>>& aperturePeers,
                           std::vector<LiveCameraState>& liveCameras,
                           bool reverseModuleOrder)
{

	
    for (size_t peerIndex = 0; peerIndex < aperturePeers.size(); ++peerIndex) {
        const size_t moduleIndex =
            PeerToModuleIndex(peerIndex, aperturePeers.size(), reverseModuleOrder);
        AperturePeer<N>* peer = aperturePeers[peerIndex].get();
          
        for (size_t localCameraIndex = 0; localCameraIndex < N + 1; ++localCameraIndex) {
            const size_t globalIndex = GlobalCameraIndex(moduleIndex, localCameraIndex);

            cv::Mat validMask;
            if (!peer->CopyValidMask(localCameraIndex, validMask)) {
                throw std::runtime_error("Failed to initialize valid mask for module " +
                                         std::to_string(moduleIndex) +
                                         ", localCameraIndex=" +
                                         std::to_string(localCameraIndex));
            }
            
            std::cout << "[InitializeCameraMasks] peer=" << peerIndex
              << " module=" << moduleIndex
	          << " local=" << localCameraIndex
	          << " global=" << globalIndex
	          << " copyOk=" << (peer->CopyValidMask(localCameraIndex, validMask) ? 1 : 0)
	          << " nz=" << (validMask.empty() ? -1 : cv::countNonZero(validMask))
	          << '\n';

            liveCameras[globalIndex].validMask = std::move(validMask);
            liveCameras[globalIndex].role =
                (localCameraIndex == 0) ? LogicalStreamRole::NonOverlap
                                        : LogicalStreamRole::Overlap;
            liveCameras[globalIndex].hasMask = true;
        }
    }
}

cv::Point FindMaskCentroid(const cv::Mat& mask)
{
    cv::Moments m = cv::moments(mask, true);
    if (m.m00 == 0.0) {
        return cv::Point(-1, -1);
    }
    return cv::Point(static_cast<int>(m.m10 / m.m00),
                     static_cast<int>(m.m01 / m.m00));
}


template<std::size_t N>
cv::Mat BuildFaceMaskForCamera(const ICameraModel& model,
                               const Eigen::Matrix3d& Rcw,
                               const RigFace<N>& face,
                               const std::vector<Eigen::Vector3d>& vertices,
                               int width,
                               int height)
{
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Eigen::Vector3d rayCam;
            if (!model.unproject(cv::Point2d(x + 0.5, y + 0.5), rayCam)) {
                continue;
            }

            const Eigen::Vector3d rayWorld = (Rcw * rayCam).normalized();

            if (spherical::IsRayInsideSphericalFace(rayWorld, face, vertices)) {
                mask.at<std::uint8_t>(y, x) = 255;
            }
        }
    }

    return mask;
}


} // namespace

int main(int argc, char* argv[])
{
    try {
        std::cout << "Program - started\n";

        const ProgramOptions options = ParseArgs(argc, argv);
        PrintProgramOptions(options);

        const NetworkAddressPlan addressPlan = BuildNetworkAddressPlan(options);
        PrintAddressPlan(addressPlan);

        constexpr size_t n = kOverlapStreams;
        auto aperturePeers = CreateAperturePeers<n>(options, addressPlan);

        const FisheyeParams fisheyeParams{
            .fx = 600.0,
            .fy = 600.0,
            .cx = 728.0,
            .cy = 544.0,
            .maxImageRadiusPx = 728.0
        };

        const size_t totalCameras =
            TotalCameraCount(options.nApertureComputeModules);

        // Build topology once and use it as the single source of truth
        // for both rig geometry and module face assignment.
        IcosahedronTopology topologyBuilder;
        const MeshTopology<3> topo = topologyBuilder.Make();

 		CameraSetup<3> cameraSetup =
		    makeCameraConfigs<3>(options.nApertureComputeModules);
		
		const std::vector<CameraConfig>& configs = cameraSetup.configs;
		const RigData<3>& rig = cameraSetup.rig;
		
		auto cameras = CreateCameraViews(configs, fisheyeParams, rig);
       //const RigData<3> rig = MakeRigData<3>(topo);
        const auto& faces = rig.faces;
        const auto& vertices = rig.vertices;

        const std::vector<int> moduleFaceIndices =
            topologyBuilder.DefaultModuleOwningFaces(options.nApertureComputeModules);

        if (moduleFaceIndices.empty()) {
            throw std::runtime_error("moduleFaceIndices is empty");
        }

        for (int faceIndex : moduleFaceIndices) {
            if (faceIndex < 0 || faceIndex >= static_cast<int>(faces.size())) {
                throw std::runtime_error("moduleFaceIndices contains out-of-range face");
            }
        }

        PrintRigAssignment(faces, moduleFaceIndices);



        if (configs.size() != totalCameras || cameras.size() != totalCameras) {
            throw std::runtime_error("Logical camera count mismatch during initialization");
        }

        std::vector<cv::Mat> moduleFaceMasks;
        moduleFaceMasks.reserve(static_cast<size_t>(options.nApertureComputeModules));

        for (int module = 0; module < options.nApertureComputeModules; ++module) {
            const int faceIndex = moduleFaceIndices[static_cast<size_t>(module)];
            const RigFace<3>& face = faces[static_cast<size_t>(faceIndex)];

            const size_t logicalIndex =
                static_cast<size_t>(module) * kCamerasPerModule;

            if (logicalIndex >= cameras.size()) {
                throw std::runtime_error(
                    "logicalIndex out of range while building moduleFaceMasks");
            }

            const CameraView& cam = cameras[logicalIndex];

            if (cam.image.empty()) {
                throw std::runtime_error(
                    "Camera image is empty while building moduleFaceMasks");
            }
            if (!cam.model) {
                throw std::runtime_error(
                    "Camera model is null while building moduleFaceMasks");
            }

            cv::Mat faceMask = BuildFaceMaskForCamera(
                *cam.model,
                cam.Rcw,
                face,
                vertices,
                cam.image.cols,
                cam.image.rows);

            moduleFaceMasks.push_back(faceMask);

            std::cout << "[moduleFaceMask] module=" << module
                      << " faceIndex=" << faceIndex
                      << " nonzero=" << cv::countNonZero(faceMask)
                      << " rows=" << faceMask.rows
                      << " cols=" << faceMask.cols
                      << '\n';
        }

        for (size_t i = 0; i < cameras.size(); ++i) {
            DebugCameraPose(cameras[i], "camera[" + std::to_string(i) + "]");
        }

        if (!cameras.empty() && cameras[0].model) {
            DebugCanonicalProjection(*cameras[0].model);
            DebugProjectUnprojectConsistency(*cameras[0].model);
        }

        if (!cameras.empty()) {
            cv::Mat footprint =
                RenderCameraFootprintDebug(cameras[0], 1456, 1088);

            std::cout << "[DEBUG] Saved camera footprint\n";

            const cv::Point c = FindMaskCentroid(footprint);
            std::cout << "[DEBUG] footprint centroid = ("
                      << c.x << ", " << c.y << ")\n";
        }

        std::vector<LiveCameraState> liveCameras(totalCameras);

        InitializeCameraMasks(aperturePeers, liveCameras, options.reverseModuleOrder);

        std::cout << "[main] configs.size()=" << configs.size()
                  << " cameras.size()=" << cameras.size()
                  << " liveCameras.size()=" << liveCameras.size()
                  << '\n';

        if (moduleFaceIndices.size() >= 2) {
            const RigFace<3>& face0 =
                faces[static_cast<size_t>(moduleFaceIndices[0])];
            const RigFace<3>& face1 =
                faces[static_cast<size_t>(moduleFaceIndices[1])];

            cv::Mat seam =
                RenderSharedFaceSeamDebug(face0, face1, vertices, 1456, 1088);

            std::cout << "[DEBUG] saved /tmp/debug_shared_seam.png\n";
        }

        std::vector<std::jthread> frameThreads;
        std::vector<std::jthread> controlThreads;

		
        StartPeerThreads<n>(aperturePeers,
                            liveCameras,
                            frameThreads,
                            controlThreads,
                            options.reverseModuleOrder);

        RunStitchLoop(cameras,
                      configs,
                      liveCameras,
                      moduleFaceMasks,
                      rig);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
