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

using namespace DASPi;

int main(int argc, char* argv[])
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);  // Pin to CPU core 2
    //pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    pthread_setname_np(pthread_self(), "MainThread");

    std::cout << "Program - started" << std::endl;

    std::string clientIp{""};    
    int port{5000}; // Default port


    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << "[--verbose] --clientIp=<clientIp> --port=<port>\n";
        return 1;
    }
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        // Check if argument is "--verbose"
        if (arg.rfind("--verbose", 0) == 0) {
            verbosity = Verbosity::Verbose;
        }
        
        // Check if argument starts with "--clientIp="
        if (arg.rfind("--clientIp=", 0) == 0) {
            try {
                clientIp = arg.substr(11); // Extract and convert port number
            } catch (const std::exception& e) {
                std::cerr << "Invalid client Ip: " << e.what() << std::endl;
                return 1; // Exit with error
            }
        }

        // Check if argument starts with "--port="
        if (arg.rfind("--port=", 0) == 0) {
            try {
                port = std::stoi(arg.substr(7)); // Extract and convert port number
            } catch (const std::exception& e) {
                std::cerr << "Invalid port number: " << e.what() << std::endl;
                return 1; // Exit with error
            }
        }
    }
   
    if (clientIp == "") {
        std::cerr << "No valid --clientIp argument provided." << std::endl;
        return 1;
    }
    
    if (port == 0) {
        std::cerr << "No valid --port argument provided." << std::endl;
        return 1;
    }

    // Output results
    std::cout << "Client Ip: " << clientIp << std::endl;
    std::cout << "Port: " << port << std::endl;
    
    Aperture aperture(clientIp, port);
    
    aperture.ContinuousCapture();
    
    return 0;
}

