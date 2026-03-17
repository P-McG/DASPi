#include <chrono>
#include <thread>
#include <execution>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "DASPi-logger.h"
#include "DASPi-aperture-client.h"
#include <sys/epoll.h>

//#define MAX_EVENTS 10
#define VERBATIUM_COUT

using namespace DASPi;

std::string IncrementIp(std::string& ipStr) {
    struct in_addr addr;
    
    if (inet_aton(ipStr.c_str(), &addr) == 0) {
        std::cerr << "Invalid IP address format!\n";
        return ipStr;
    }

    // Convert to host byte order, increment, and convert back to network byte order
    uint32_t ip = ntohl(addr.s_addr);
    ip++;  // Increment IP address
    addr.s_addr = htonl(ip);

    // Convert back to string
    ipStr = inet_ntoa(addr);
    
    return ipStr;
}

std::string IncrementSubnet(std::string ipStr) {
    struct in_addr addr;

    if (inet_aton(ipStr.c_str(), &addr) == 0) {
        std::cerr << "Invalid IP address format!\n";
        return ipStr;
    }

    uint32_t ip = ntohl(addr.s_addr);

    // Extract each octet
    uint8_t octet1 = (ip >> 24) & 0xFF;
    uint8_t octet2 = (ip >> 16) & 0xFF;
    uint8_t octet3 = (ip >> 8) & 0xFF;
    uint8_t octet4 = ip & 0xFF;

    // Increment the 3rd octet (subnet)
    octet3++;

    // Reconstruct the IP
    ip = (octet1 << 24) | (octet2 << 16) | (octet3 << 8) | octet4;
    addr.s_addr = htonl(ip);

    return std::string(inet_ntoa(addr));
}

in_addr_t stringToInAddrT(const std::string& ipStr) {
    struct in_addr addr;
    
    if (inet_pton(AF_INET, ipStr.c_str(), &addr) != 1) {
        std::cerr << "Invalid IP address format: " << ipStr << std::endl;
        return INADDR_NONE;  // Return 0xFFFFFFFF if conversion fails
    }

    return addr.s_addr;  // Return in network byte order
}

std::string inAddrTToString(in_addr_t addrNetOrder) {
    char str[INET_ADDRSTRLEN];
    struct in_addr addr;
    addr.s_addr = addrNetOrder;

    if (inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN) == nullptr) {
        std::cerr << "inet_ntop failed!" << std::endl;
        return "";
    }

    return std::string(str);
}


//void log(LogLevel level, const std::string& msg) {
    //if (static_cast<int>(level) <= static_cast<int>(logLevel)) {
        //std::cerr << msg << std::endl;
    //}
//}

/* main()
 * 
 * Arguments:
 * N Aperture Compute Modules
 * USB Subnet
 * Port Number Start
 * 
 * The following assumes the network is setup in the following
 * way:
 * e.g.
 *    10.0.2.1 - gateway
 *    10.0.2.2 - Compute Module
 *    10.0.2.3 - Aperture Compute Module
 *       ...
 *    10.0.2.(n+3) - n-Aperture Compute Module starting at ApertureComputeModule000;
 */
int main(int argc, char* argv[]){
    std::cout << "Program - started" << std::endl;
    
    // Parse input arguments
    int nApertureComputeModules{0};  // Number of Aperture Compute Modules
    std::string usbSubnet{""};
    int port{0}; // Port number
    
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << "[--verbose] <nApertureComputeModules> <usbSubnet> <portStart>\n";
        return 1;
    }
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg.rfind("--verbose", 0) == 0) {
            verbosity = Verbosity::Verbose;
        }

        // Check if argument starts with "--nApertureComputeModule="
        if (arg.rfind("--nApertureComputeModules=", 0) == 0) {
            try {                

                nApertureComputeModules = std::stoi(arg.substr(26)); // Extract and convert port number
            } catch (const std::exception& e) {
                std::cerr << "Invalid nApertureComputeModules number: " << e.what() << std::endl;
                return 1; // Exit with error
            }
        }

        // Check if argument starts with "--usbSubnet="
        if (arg.rfind("--usbSubnets=", 0) == 0) {
            try {
                usbSubnet = arg.substr(13);
            } catch (const std::exception& e) {
                std::cerr << "Invalid usbSubnets number: " << e.what() << std::endl;
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

    if (nApertureComputeModules == 0) {
        std::cerr << "No valid --nApertureComputeModules argument provided." << std::endl;
        return 1;
    }
   
    if (usbSubnet == "") {
        std::cerr << "No valid --usbSubnet argument provided." << std::endl;
        return 1;
    }
    
    if (port == 0) {
        std::cerr << "No valid --port argument provided." << std::endl;
        return 1;
    }

    // Output results
    std::cout << "Number of Aperture Compute Modules: " << nApertureComputeModules << std::endl;
    std::cout << "USB Subnet: " << usbSubnet << std::endl;
    std::cout << "Port Number : " << port << std::endl;
  
    //Setting up the IP addresses
    std::string subnetBase = usbSubnet;
      
    std::vector<in_addr_t> gatewayAddrs(nApertureComputeModules);
    std::vector<in_addr_t> srvAddrs(nApertureComputeModules);
    std::vector<in_addr_t> clntAddrs(nApertureComputeModules);
    
    // Increment the ip's 4th octet (host)
    std::string gatewayAddr = IncrementIp(subnetBase);
    gatewayAddrs[0] = stringToInAddrT(gatewayAddr); // Gateway 10.0.x.1
    std::string clntAddr = IncrementIp(gatewayAddr);
    clntAddrs[0] = stringToInAddrT(clntAddr); // ComputeModule 10.0.x.2
    std::string srvAddr = IncrementIp(clntAddr);
    srvAddrs[0] = stringToInAddrT(srvAddr); // ApertureComputeModule 10.0.x.3
    
    // Increment the ip's 3rd octet (subnet)
    for(int i = 1; i < nApertureComputeModules; i++) {
        gatewayAddrs[i] = stringToInAddrT(IncrementSubnet(inAddrTToString(gatewayAddrs[i-1])));
        clntAddrs[i] = stringToInAddrT(IncrementSubnet(inAddrTToString(clntAddrs[i-1]))); 
        srvAddrs[i] = stringToInAddrT(IncrementSubnet(inAddrTToString(srvAddrs[i-1])));
    }
    
    std::cout << "[main] Setting up ApertureClients" << std::endl;
    const size_t n=3;
    std::vector<std::unique_ptr<ApertureClient<n>>> apertureClients(nApertureComputeModules);
    for (size_t i = 0; i < static_cast<size_t>(nApertureComputeModules); ++i) {
        struct in_addr srv_ip_addr, clnt_ip_addr;
        srv_ip_addr.s_addr = srvAddrs[i];
        clnt_ip_addr.s_addr = clntAddrs[i];

        std::cout << "[main] Creating ApertureClient " << i
                  << ", port=" << (port/* + i*/) 
                  << ", srvAddr=" << ((i < srvAddrs.size()) ? inet_ntoa(srv_ip_addr) : "INVALID")
                  << ", clntAddr=" << ((i < srvAddrs.size()) ? inet_ntoa(clnt_ip_addr) : "INVALID")
                  << std::endl;
        apertureClients[i] = std::make_unique<ApertureClient<n>>(clntAddrs[i], port, srvAddrs[i]);

    }
    
    std::cout << "[main] Setting up ReceiveApertureCapture" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    for_each(std::execution::par, apertureClients.begin(), apertureClients.end(),
    [](const std::unique_ptr<ApertureClient<n>>& client){
        if(!client->ReceiveApertureCapture()){
            std::cerr << "ReceiveApertureCapture - FAILED" << std:: endl;
            exit(1);
        }
    });
	auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " microseconds\n";
    
    std::cout << "Program - finished" << std::endl;
    return 0; 
}
