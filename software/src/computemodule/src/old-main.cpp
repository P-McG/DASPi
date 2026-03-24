#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>

#include "DASPi-logger.h"
#include "DASPi-aperture-peer.h"

using namespace DASPi;

namespace {

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

} // namespace

int main(int argc, char* argv[])
{
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

    std::cout << "Number of Aperture Compute Modules: " << nApertureComputeModules << '\n';
    std::cout << "USB Subnet: " << usbSubnet << '\n';
    std::cout << "Frame Port: " << framePort << '\n';
    std::cout << "Control Port: " << controlPort << '\n';

    std::vector<in_addr_t> gatewayAddrs(static_cast<size_t>(nApertureComputeModules));
    std::vector<in_addr_t> clntAddrs(static_cast<size_t>(nApertureComputeModules));
    std::vector<in_addr_t> srvAddrs(static_cast<size_t>(nApertureComputeModules));

    std::string subnetBase = usbSubnet;

    std::string gatewayAddr = IncrementIp(subnetBase);
    gatewayAddrs[0] = StringToInAddrT(gatewayAddr);   // 10.0.x.1

    std::string clntAddr = IncrementIp(gatewayAddr);
    clntAddrs[0] = StringToInAddrT(clntAddr);         // 10.0.x.2 (computemodule)

    std::string srvAddr = IncrementIp(clntAddr);
    srvAddrs[0] = StringToInAddrT(srvAddr);           // 10.0.x.3 (aperturecomputemodule)

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

    std::cout << "[main] Setting up AperturePeers\n";

    constexpr size_t n = 3;
    std::vector<std::unique_ptr<AperturePeer<n>>> aperturePeers(
        static_cast<size_t>(nApertureComputeModules));

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

    std::cout << "[main] Starting per-peer frame/control threads\n";

    std::vector<std::jthread> frameThreads;
    std::vector<std::jthread> controlThreads;

    frameThreads.reserve(aperturePeers.size());
    controlThreads.reserve(aperturePeers.size());

    for (auto& peerPtr : aperturePeers) {
        AperturePeer<n>* peer = peerPtr.get();

        frameThreads.emplace_back([peer]() {
            for (;;) {
                if (!peer->RunFrameLoop()) {
                    std::cerr << "[frame thread] RunFrameLoop failed\n";
                    break;
                }
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
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }

    return EXIT_SUCCESS;
}
