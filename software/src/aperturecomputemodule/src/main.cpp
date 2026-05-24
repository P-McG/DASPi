// Server side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <chrono>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <sys/mman.h>
#include <unistd.h>
#include <string>
#include <netinet/in.h>
#include <pthread.h>

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>

#include "event_loop.h"
#include "DASPi-logger.h"
#include "DASPi-udp-srv.h"
#include "DASPi-aperture.h"
#include "DASPi-config.h"

using namespace DASPi;

template<std::size_t ModuleFaceIndex>
int RunAperture(const std::string& clientIp, int port)
{
    std::cout << "ModuleFaceIndex: " << ModuleFaceIndex << std::endl;

    DASPi::Aperture<static_cast<unsigned int>(ModuleFaceIndex)> aperture(
        clientIp,
        static_cast<std::size_t>(port)
    );

    std::cout << ">>> BEFORE ContinuousCapture <<<" << std::endl;
    aperture.ContinuousCapture(0);
    std::cout << ">>> AFTER ContinuousCapture <<<" << std::endl;

    return 0;
}

template<class IndexType, IndexType... ModuleFaceIndices>
int RunApertureForModuleFaceIndexImpl(
    std::size_t requestedModuleFaceIndex,
    const std::string& clientIp,
    int port,
    std::integer_sequence<IndexType, ModuleFaceIndices...>)
{
    using RunFunction = int (*)(const std::string&, int);

    static constexpr std::array<
        std::pair<std::size_t, RunFunction>,
        sizeof...(ModuleFaceIndices)
    > dispatchTable{{
        {
            static_cast<std::size_t>(ModuleFaceIndices),
            &RunAperture<static_cast<std::size_t>(ModuleFaceIndices)>
        }...
    }};

    for (const auto& [moduleFaceIndex, run] : dispatchTable) {
        if (requestedModuleFaceIndex == moduleFaceIndex) {
            return run(clientIp, port);
        }
    }

    std::cerr << "Invalid --moduleFaceIndex="
              << requestedModuleFaceIndex
              << ". This binary was built for module face indices: ";

    ((std::cerr << static_cast<std::size_t>(ModuleFaceIndices) << " "), ...);

    std::cerr << std::endl;

    return 1;
}

int RunApertureForModuleFaceIndex(
    std::size_t requestedModuleFaceIndex,
    const std::string& clientIp,
    int port)
{
    using SphereSpaceType = typename DASPi::tpgy_t::Space_t;

    using ModuleFaceIndexSequence =
        typename SphereSpaceType::module_face_index_sequence_t;

    return RunApertureForModuleFaceIndexImpl(
        requestedModuleFaceIndex,
        clientIp,
        port,
        ModuleFaceIndexSequence{}
    );
}

int main(int argc, char* argv[])
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    pthread_setname_np(pthread_self(), "MainThread");

    std::cout << "Program - started" << std::endl;

    std::string clientIp;
    int port{5000};
    std::size_t moduleFaceIndex{};
    bool hasModuleFaceIndex{false};

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--verbose") {
            verbosity = Verbosity::Verbose;
        }
        else if (arg.rfind("--clientIp=", 0) == 0) {
            clientIp = arg.substr(std::string("--clientIp=").size());
        }
        else if (arg.rfind("--port=", 0) == 0) {
            try {
                port = std::stoi(arg.substr(std::string("--port=").size()));
            } catch (const std::exception& e) {
                std::cerr << "Invalid port number: " << e.what() << std::endl;
                return 1;
            }
        }
        else if (arg.rfind("--moduleFaceIndex=", 0) == 0) {
            try {
                moduleFaceIndex =
                    static_cast<std::size_t>(
                        std::stoul(arg.substr(std::string("--moduleFaceIndex=").size()))
                    );

                hasModuleFaceIndex = true;
            } catch (const std::exception& e) {
                std::cerr << "Invalid module face index: " << e.what() << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            std::cerr << "Usage: " << argv[0]
                      << " [--verbose] --clientIp=<clientIp> --port=<port> --moduleFaceIndex=<0-19>\n";
            return 1;
        }
    }

    if (clientIp.empty()) {
        std::cerr << "No valid --clientIp argument provided." << std::endl;
        return 1;
    }

    if (port == 0) {
        std::cerr << "No valid --port argument provided." << std::endl;
        return 1;
    }

    if (!hasModuleFaceIndex) {
        std::cerr << "No valid --moduleFaceIndex argument provided." << std::endl;
        return 1;
    }

    std::cout << "Client Ip: " << clientIp << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "ModuleFaceIndex: " << moduleFaceIndex << std::endl;

    return RunApertureForModuleFaceIndex(moduleFaceIndex, clientIp, port);
}

