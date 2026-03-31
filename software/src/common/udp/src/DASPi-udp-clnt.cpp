// Client side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ifaddrs.h> 
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include "DASPi-udp-clnt.h"
#include "DASPi-framepacket.h"
#include "DASPi-rx-frame-assembly.h"

#define VERBATIUM_COUT

using namespace DASPi;

// Constructor
UDPClnt::UDPClnt(const in_addr_t clntAddr, const int port, const in_addr_t srvAddr)
    :clntAddr_(FillingClientInformation(clntAddr, port)),
    port_(port)
{
#ifdef VERBATIUM_COUT
   std::cout << "[UDPClnt]" << std::endl;
#endif

    connect(sockfd_, (struct sockaddr*)&srvAddr_, sizeof(srvAddr_));

    FillingServerInformation(srvAddr);
    CreatingSocketFileDescriptor();
    BindSocketWithClientAddress();
#ifdef VERBATIUM_COUT
   std::cout << "[UDPClnt] - finished" << std::endl;
#endif
}

// Destructor
UDPClnt::~UDPClnt(){
#ifdef VERBATIUM_COUT
    std::cout << "[~UDPClnt]" << std::endl;
#endif
    close(sockfd_);
}

size_t UDPClnt::GetMaximumTransmittableUnits(){
    return maxUdpPayloadBytes_;
}

std::string UDPClnt::GetHostIp() {
#ifdef VERBATIUM_COUT
   std::cout << "[UDPClnt::GetHostIp]" << std::endl;
#endif
    struct ifaddrs* ifaddr;
    struct ifaddrs* iface;
    char ip[INET_ADDRSTRLEN];

    // Get list of network interfaces
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return "";
    }

    std::string host_ip;

    // Iterate through interfaces
    for (iface = ifaddr; iface != nullptr; iface = iface->ifa_next) {
        if (iface->ifa_addr == nullptr) continue; // Skip invalid entries

        // Check for IPv4 addresses
        if (iface->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in* addr = (struct sockaddr_in*)iface->ifa_addr;
            inet_ntop(AF_INET, &addr->sin_addr, ip, INET_ADDRSTRLEN);

            // Skip loopback addresses (127.x.x.x)
            if (strcmp(ip, "127.0.0.1") != 0) {
                host_ip = ip;
                break; // Use the first non-loopback address
            }
        }
    }

    freeifaddrs(ifaddr); // Free the memory allocated by getifaddrs

    return host_ip;
}

// FillingClientInformation
sockaddr_in UDPClnt::FillingClientInformation(const in_addr_t clntAddr, const int port){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::FillingClientInformation]" << std::endl;
#endif

    memset(&clntAddr_, 0, sizeof(clntAddr_));

    clntAddr_.sin_family = AF_INET; // IPv4

    clntAddr_.sin_addr.s_addr = clntAddr;
    
    clntAddr_.sin_port = htons(port);
    
    return clntAddr_;
};
	
//CreatingSocketFileDescriptor
/*
 */
int UDPClnt::CreatingSocketFileDescriptor(){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::CreatingSocketFileDescriptor]" << std::endl;
#endif
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    return 0;
}



// Filling server information
/*
 */ 
void UDPClnt::FillingServerInformation(const in_addr_t &addr) {
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::FillingServerInformation]" << std::endl;
#endif    
    memset(&srvAddr_, 0, sizeof(srvAddr_));

    srvAddr_.sin_family = AF_INET;
    srvAddr_.sin_port = htons(port_);
    srvAddr_.sin_addr.s_addr = addr;
}

//// TransmissionRequest
///*
 //*/
//bool UDPClnt::TransmissionRequest(bool flag){

    //const bool transmissionRequest{flag};
      
    //if(sendto(sockfd_, (const char *)&transmissionRequest, sizeof(transmissionRequest), 
        //MSG_CONFIRM, (const struct sockaddr *) &srvAddr_,  
            //sizeof(srvAddr_)) <= 0){
        //std::cout << "Transmission request failed." <<  std::endl;
        //return 1;
    //}

    //return 0;
//}


bool UDPClnt::WaitForValidHeader(FrameHeader &headerOut, sockaddr_in &senderOut)
{
#ifdef VERBATIUM_COUT
     std::cout << "[UDPClnt::WaitForValidHeader]" << std::endl;
#endif
    sockaddr_in sender {};
    socklen_t senderLen = sizeof(sender);
    FrameHeader headerBuffer;

    while (true) {
        ssize_t bytes = recvfrom(sockfd_, &headerBuffer, sizeof(headerBuffer), 0,
                                 reinterpret_cast<sockaddr*>(&sender), &senderLen);

        if (bytes != sizeof(headerBuffer)) {
            // Ignore incomplete packets
            continue;
        }

        // If using a magic number, validate it
        if (headerBuffer.magic_ != MAGIC_NUMBER) {
            // Reject anything not starting with proper header
            continue;
        }

        // Optionally: validate payloadSize sanity
        if (headerBuffer.payloadSize_ == 0 || headerBuffer.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_) {
            continue;
        }

        // Valid header received
        headerOut = headerBuffer;
        senderOut = sender;
        return true;
    }
   // unreachable in normal loop
    return false;
}

bool UDPClnt::isEqual(const sockaddr_in &a, const sockaddr_in &b){
    return (a.sin_family == b.sin_family) &&
           (a.sin_port == b.sin_port) &&
           (a.sin_addr.s_addr == b.sin_addr.s_addr);
}

void UDPClnt::PrintSockaddr_in(const struct sockaddr_in *addr) {
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::PrintSocksaddr_in]" << std::endl;
#endif
    char ip_str[INET_ADDRSTRLEN];  // Buffer to store IP address string

    // Convert the IP to a readable format
    inet_ntop(AF_INET, &(addr->sin_addr), ip_str, sizeof(ip_str));

    // Convert port number from network byte order to host byte order
    int port = ntohs(addr->sin_port);

    printf("IP: %s, Port: %d\n", ip_str, port);
}

//int UDPClnt::BindSocketWithClientAddress(){
////#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt] Bind socket with client address" << std::endl;
////#endif
    
    //if ( bind(sockfd_, (const struct sockaddr *)&clntAddr_,
            //sizeof(clntAddr_)) < 0 )
    //{
        //perror("bind failed");
        //exit(EXIT_FAILURE);
    //}
    //return 0;
//};

int UDPClnt::BindSocketWithClientAddress(){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::BindSocketWithClientAddress]" << std::endl;
#endif

    if (bind(sockfd_, (const struct sockaddr *)&clntAddr_, sizeof(clntAddr_)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    return 0;
}


//bool UDPClnt::ReceiveAndReassembleFramePacket( std::vector<uint16_t> &outPayload, FrameHeader& outHeader)
//{
//#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::ReceiveAndReassembleFramePacket]" << std::endl;
//#endif
    //std::vector<uint8_t> buffer;
    //buffer.reserve(FramePacket::MAX_ALLOWED_FRAME_SIZE_ + sizeof(FrameHeader));

    //bool headerReceived = false;
    //size_t totalBytesExpected = 0;
    //size_t totalBytesReceived = 0;

    //sockaddr_in sender {};
    //socklen_t senderLen = sizeof(sender);
    
    //std::vector<uint8_t> packet(maximumTransmittableUnits_);

    //while (true) {
        //ssize_t received = recvfrom(sockfd_, packet.data(), packet.size(), 0,
                                    //reinterpret_cast<sockaddr*>(&sender), &senderLen);

        //if (received < 0) {
            //if (errno == EAGAIN || errno == EWOULDBLOCK) {
                ////std::cerr << /*"recvfrom: no data yet, waiting..." << std::endl*/ ;
                //continue;
            //} else {
                //perror("recvfrom failed");
                //return false;
            //}
        //}
        //if (received > static_cast<ssize_t>(packet.size())) {
            //std::cerr << "Received more bytes than allocated packet size!" << std::endl;
            //exit(1); // or handle error
        //}
        
        //// If this is the first packet, extract the header
        //if (!headerReceived) {
            //if (received < static_cast<ssize_t>(sizeof(FrameHeader))) {
                //std::cerr << "Packet too small for header" << std::endl;
                //continue;
            //}

            //std::memcpy(&outHeader, packet.data(), sizeof(FrameHeader));
            
            //outHeader.magic_ = ntohl(outHeader.magic_);
            //outHeader.payloadSize_ = ntohl(outHeader.payloadSize_);
            //outHeader.checksum_ = ntohl(outHeader.checksum_);
            

            //if (outHeader.magic_ != MAGIC_NUMBER ||
                //outHeader.payloadSize_ == 0 ||
                //outHeader.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_) {
                //std::cerr << "Invalid header received" << std::endl;
                //std::cout << "outHeader.magic_: " << outHeader.magic_ << " MAGIC_NUMBER: " << MAGIC_NUMBER << std::endl;
                //std::cout << "outHeader.payloadSize_: " << outHeader.payloadSize_ << std::endl;
                //continue;
            //}
            
            //totalBytesExpected = outHeader.payloadSize_;
            //buffer.resize(totalBytesExpected);

            //// Copy the payload part of this first packet
            //size_t payloadInThisPacket = received - sizeof(FrameHeader);
////            buffer.insert(buffer.end(), packet.begin() + sizeof(FrameHeader), packet.begin() + received);
             //std::memcpy(buffer.data(), packet.data() + sizeof(FrameHeader), payloadInThisPacket);

            //totalBytesReceived += payloadInThisPacket;
            //headerReceived = true;
        //}
        //else {
            //// Copy payload of this subsequent packet
////            buffer.insert(buffer.end(), packet.begin(), packet.begin() + received);
            //std::memcpy(buffer.data() + totalBytesReceived, packet.data(), received);
            //totalBytesReceived += received;
        //}

        //if (totalBytesReceived >= totalBytesExpected) {
            ////std::cout << "totalBytesReceived >= totalBytesExpected: " << ((totalBytesReceived >= totalBytesExpected)?"true":"false") << std::endl;
            //break;
        //}
    //}

    //uint32_t computed = SimpleChecksum(buffer.data(), buffer.size());
    
    //std::cout << "Header Checksums: " << outHeader.checksum_ << std::endl;
    //std::cout << "Header payload size: " << outHeader.payloadSize_ << std::endl;
    //std::cout << "buffer Checksums: " << computed << std::endl;
    //std::cout << "buffer size: " << buffer.size() << std::endl;
    
    //if (computed != outHeader.checksum_) {
        //std::cerr << "Checksum mismatch! Packet may be corrupted." << std::endl;
        //return false;
    //}

    //outPayload = std::move(buffer);
    //return true;
//}

bool UDPClnt::ReceiveAndReassembleFramePacket(std::vector<uint16_t>& outPayload,
                                              FrameHeader& outHeader)
{
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::ReceiveAndReassembleFramePacket]\n";
#endif

    std::unordered_map<uint32_t, RxFrameAssembly> assemblies;
    std::vector<uint8_t> packet(maxUdpPayloadBytes_);

    while (true) {
        sockaddr_in sender{};
        socklen_t senderLen = sizeof(sender);

        const ssize_t received = recvfrom(sockfd_,
                                          packet.data(),
                                          packet.size(),
                                          0,
                                          reinterpret_cast<sockaddr*>(&sender),
                                          &senderLen);

        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            perror("recvfrom failed");
            return false;
        }

        const size_t got = static_cast<size_t>(received);
        if (got < sizeof(UdpChunkHeader)) {
            continue;
        }

        UdpChunkHeader wire{};
        std::memcpy(&wire, packet.data(), sizeof(wire));

        wire.magic_ = ntohl(wire.magic_);
        wire.frameId_ = ntohl(wire.frameId_);
        wire.chunkId_ = ntohs(wire.chunkId_);
        wire.chunkCount_ = ntohs(wire.chunkCount_);
        wire.payloadBytes_ = ntohs(wire.payloadBytes_);

        if (wire.magic_ != MAGIC_NUMBER) {
            continue;
        }

        if (wire.chunkCount_ == 0 || wire.chunkId_ >= wire.chunkCount_) {
            continue;
        }

        if (sizeof(UdpChunkHeader) + wire.payloadBytes_ > got) {
            continue;
        }

        auto& a = assemblies[wire.frameId_];

        if (a.frameId == 0) {
            a.frameId = wire.frameId_;
            a.chunkCount = wire.chunkCount_;
            a.chunkSeen.assign(wire.chunkCount_, 0);
        } else if (a.chunkCount != wire.chunkCount_) {
            assemblies.erase(wire.frameId_);
            continue;
        }

        if (wire.chunkId_ == 0 && !a.headerSeen) {
            a.frameHeader = wire.frameHeader_;
            a.frameHeader.magic_ = ntohl(a.frameHeader.magic_);
            a.frameHeader.payloadSize_ = ntohl(a.frameHeader.payloadSize_);
            a.frameHeader.checksum_ = ntohl(a.frameHeader.checksum_);

            for (auto& s : a.frameHeader.regionSizes_) {
                s = ntohl(s);
            }

            if (a.frameHeader.magic_ != MAGIC_NUMBER) {
                assemblies.erase(wire.frameId_);
                continue;
            }

            if (a.frameHeader.payloadSize_ == 0 ||
                a.frameHeader.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_ ||
                (a.frameHeader.payloadSize_ % sizeof(uint16_t)) != 0) {
                assemblies.erase(wire.frameId_);
                continue;
            }

            size_t totalElems = 0;
            for (const auto s : a.frameHeader.regionSizes_) {
                totalElems += s;
            }

            if (totalElems * sizeof(uint16_t) != a.frameHeader.payloadSize_) {
                assemblies.erase(wire.frameId_);
                continue;
            }

            a.totalBytesExpected = a.frameHeader.payloadSize_;
            a.payload.assign(a.totalBytesExpected / sizeof(uint16_t), 0);
            a.headerSeen = true;
        }

        if (!a.headerSeen) {
            continue;
        }

        if (a.chunkSeen[wire.chunkId_]) {
            continue;
        }

        const size_t maxChunkPayload =
            maxUdpPayloadBytes_ - sizeof(UdpChunkHeader);
        const size_t dstOffset = static_cast<size_t>(wire.chunkId_) * maxChunkPayload;

        if (dstOffset + wire.payloadBytes_ > a.totalBytesExpected) {
            assemblies.erase(wire.frameId_);
            continue;
        }

        auto dstBytes = std::as_writable_bytes(std::span(a.payload));
        std::memcpy(dstBytes.data() + dstOffset,
                    packet.data() + sizeof(UdpChunkHeader),
                    wire.payloadBytes_);

        a.chunkSeen[wire.chunkId_] = 1;
        a.totalBytesReceived += wire.payloadBytes_;

        if (a.totalBytesReceived < a.totalBytesExpected) {
            continue;
        }

        const auto usedBytes =
            std::as_bytes(std::span(a.payload)).first(a.totalBytesExpected);
        const uint32_t computed = SimpleChecksum(usedBytes);

        if (computed != a.frameHeader.checksum_) {
            assemblies.erase(wire.frameId_);
            std::cerr << "Checksum mismatch! Packet may be corrupted.\n";
            return false;
        }

        outHeader = a.frameHeader;
        outPayload = std::move(a.payload);
        assemblies.erase(wire.frameId_);
        return true;
    }
}

//bool UDPClnt::ReceiveAndReassembleFramePacket(std::vector<uint16_t>& outPayload, FrameHeader& outHeader)
//{
//#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::ReceiveAndReassembleFramePacket]\n";
//#endif
    //bool headerReceived = false;
    //size_t totalBytesExpected = 0;   // payload size in BYTES
    //size_t totalBytesReceived = 0;

    //sockaddr_in sender{};
    //socklen_t senderLen = sizeof(sender);
    //std::vector<uint8_t> packet(maximumTransmittableUnits_);

    //while (true) {
        //ssize_t received = recvfrom(sockfd_, packet.data(), packet.size(), 0,
                                    //reinterpret_cast<sockaddr*>(&sender), &senderLen);
        //if (received < 0) {
            //if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            //perror("recvfrom failed");
            //return false;
        //}
        //size_t got = static_cast<size_t>(received);

        //if (!headerReceived) {
            //if (got < sizeof(FrameHeader)) {
                //std::cerr << "Packet too small for header\n";
                //continue;
            //}

            //std::memcpy(&outHeader, packet.data(), sizeof(FrameHeader));
            //outHeader.magic_       = ntohl(outHeader.magic_);
            //outHeader.payloadSize_ = ntohl(outHeader.payloadSize_);
            //outHeader.checksum_    = ntohl(outHeader.checksum_);
            
            //for (auto& s : outHeader.regionSizes_) {
                //s = ntohl(s);
            //}
            //if (outHeader.magic_ != MAGIC_NUMBER ||
                //outHeader.payloadSize_ == 0 ||
                //outHeader.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_) {
                //std::cerr << "Invalid header received\n";
                //continue;
            //}

            //totalBytesExpected = outHeader.payloadSize_;
            //if (totalBytesExpected % sizeof(uint16_t) != 0) {
                //std::cerr << "Payload size not u16-aligned\n";
                //return false;
            //}

            //// Allocate the u16 buffer we ultimately want
            //outPayload.assign(totalBytesExpected / sizeof(uint16_t), 0);

            //// Copy first packet's payload section into the u16 storage (as bytes)
            //auto dstBytes = std::as_writable_bytes(std::span(outPayload));
            //size_t payloadInThisPacket = got - sizeof(FrameHeader);
            //std::memcpy(dstBytes.data(),
                        //packet.data() + sizeof(FrameHeader),
                        //payloadInThisPacket);

            //totalBytesReceived += payloadInThisPacket;
            //headerReceived = true;
        //} else {
            //// Copy subsequent payload chunks
            //size_t remaining = totalBytesExpected - totalBytesReceived;
            //size_t toCopy = std::min(remaining, got);
            //auto dstBytes = std::as_writable_bytes(std::span(outPayload));
            //std::memcpy(dstBytes.data() + totalBytesReceived,
                        //packet.data(),
                        //toCopy);
            //totalBytesReceived += toCopy;
        //}

        //if (totalBytesReceived >= totalBytesExpected) break;
    //}
//}

//bool UDPClnt::ReceiveAndReassembleFramePacket(std::vector<uint16_t>& outPayload, FrameHeader& outHeader)
//{
//#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::ReceiveAndReassembleFramePacket]\n";
//#endif
    //bool headerReceived = false;
    //size_t totalBytesExpected = 0;
    //size_t totalBytesReceived = 0;

    //sockaddr_in sender{};
    //socklen_t senderLen = sizeof(sender);
    //std::vector<uint8_t> packet(maximumTransmittableUnits_);

    //while (true) {
        //ssize_t received = recvfrom(sockfd_, packet.data(), packet.size(), 0,
                                    //reinterpret_cast<sockaddr*>(&sender), &senderLen);
        //if (received < 0) {
            //if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            //perror("recvfrom failed");
            //return false;
        //}

        //size_t got = static_cast<size_t>(received);

        //if (!headerReceived) {
            //if (got < sizeof(FrameHeader)) {
                //std::cerr << "Packet too small for header\n";
                //continue;
            //}

            //std::memcpy(&outHeader, packet.data(), sizeof(FrameHeader));
            //outHeader.magic_       = ntohl(outHeader.magic_);
            //outHeader.payloadSize_ = ntohl(outHeader.payloadSize_);
            //outHeader.checksum_    = ntohl(outHeader.checksum_);

            //for (auto& s : outHeader.regionSizes_) {
                //s = ntohl(s);
            //}

            //if (outHeader.magic_ != MAGIC_NUMBER ||
                //outHeader.payloadSize_ == 0 ||
                //outHeader.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_) {
                //std::cerr << "Invalid header received\n";
                //continue;
            //}

            //size_t totalElems = 0;
            //for (auto s : outHeader.regionSizes_) {
                //totalElems += s;
            //}

            //if (totalElems * sizeof(uint16_t) != outHeader.payloadSize_) {
                //std::cerr << "[ERROR] payload mismatch: "
                          //<< "expected=" << outHeader.payloadSize_
                          //<< " actual=" << totalElems * sizeof(uint16_t)
                          //<< std::endl;
                //return false;
            //}

            //totalBytesExpected = outHeader.payloadSize_;
            //if (totalBytesExpected % sizeof(uint16_t) != 0) {
                //std::cerr << "Payload size not u16-aligned\n";
                //return false;
            //}

            //outPayload.assign(totalBytesExpected / sizeof(uint16_t), 0);

            //auto dstBytes = std::as_writable_bytes(std::span(outPayload));
            //size_t payloadInThisPacket = got - sizeof(FrameHeader);
            //std::memcpy(dstBytes.data(),
                        //packet.data() + sizeof(FrameHeader),
                        //payloadInThisPacket);

            //totalBytesReceived += payloadInThisPacket;
            //headerReceived = true;
        //} else {
            //size_t remaining = totalBytesExpected - totalBytesReceived;
            //size_t toCopy = std::min(remaining, got);

            //auto dstBytes = std::as_writable_bytes(std::span(outPayload));
            //std::memcpy(dstBytes.data() + totalBytesReceived,
                        //packet.data(),
                        //toCopy);

            //totalBytesReceived += toCopy;
        //}

        //if (totalBytesReceived >= totalBytesExpected) {
            //break;
        //}
    //}

    //auto usedBytes = std::as_bytes(std::span(outPayload)).first(totalBytesExpected);
    //uint32_t computed = SimpleChecksum(usedBytes);

    //std::cout << "Header checksum: "   << outHeader.checksum_    << "\n"
              //<< "Payload size: "      << outHeader.payloadSize_ << "\n"
              //<< "Computed checksum: " << computed               << "\n";

    //if (computed != outHeader.checksum_) {
        //std::cerr << "Checksum mismatch! Packet may be corrupted.\n";
        //return false;
    //}

    //size_t offset = 0;
    //for (size_t i = 0; i < outHeader.regionSizes_.size(); ++i) {
        //const size_t count = outHeader.regionSizes_[i];

        //std::span<const uint16_t> region(
            //outPayload.data() + offset,
            //count
        //);

        //std::cout << "[RX] region " << i
                  //<< " size=" << count << std::endl;

        //offset += count;
    //}

    //return true;
//}

//uint32_t UDPClnt::SimpleChecksum(const uint8_t* data, size_t size) {
//#ifdef VERBATIUM_COUT
   //std::cout << "[UDPClnt::SimpleChecksum]" << std::endl;
//#endif
    //uint32_t sum = 0;
    //for (size_t i = 0; i < size; ++i) {
        //sum += static_cast<uint8_t>(data[i]);
    //}
    //return sum;
//}

uint32_t UDPClnt::SimpleChecksum(std::span<const std::byte> bytes) {
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::SimpleChecksum]" << std::endl;
#endif
    uint32_t sum = 0;
    for (auto b : bytes) {
        sum += static_cast<uint8_t>(b);
    }
    return sum;
}



bool UDPClnt::InitEpollForSrvUDPPackets(/*std::vector<char> &buffer,*/ EpollData &epollData){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::InitEpollForSrvUDPPackets]" << std::endl;
#endif

    epollData.maxEvents_ =1;

    // Create epoll instance
    epollData.epoll_fd_ = epoll_create1(0);
    if (epollData.epoll_fd_ < 0) {
        perror("epoll_create1");
        return false;
    }

    // Set socket to non-blocking
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("fcntl");
        close(epollData.epoll_fd_);
        return false;
    }

    epoll_event ev {};
    epollData.events_=std::make_unique<epoll_event[]>(epollData.maxEvents_);
    ev.events = EPOLLIN;
    ev.data.fd = sockfd_;

    if (epoll_ctl(epollData.epoll_fd_, EPOLL_CTL_ADD, sockfd_, &ev) < 0) {
        perror("epoll_ctl");
        close(epollData.epoll_fd_);
        return false;
    }

    // Optional: connect socket to fixed server address once (if needed)
    // If you plan to use recv() instead of recvfrom(), uncomment this:
    /*
    if (connect(sockfd_, (struct sockaddr *)&srvAddr_, sizeof(srvAddr_)) < 0) {
        perror("connect failed");
        close(epollData.epoll_fd_);
        return -1;
    }
    */
    
    PrintSockaddr_in(&srvAddr_);

//        sockaddr_in headerSender;//retrieved UDP header from

    FlushSocket();

    return true;
}

bool UDPClnt::FinalizeEpollForSrvUDPPackets(const EpollData &epollData){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::FinalizeEpollForSrvUDPPackets]" << std::endl;
#endif

    close(epollData.epoll_fd_);
    return true;
}

bool UDPClnt::ReadDataFromSrvUDPPackets(const EpollData &epollData, std::vector<uint16_t> &buffer){
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::ReadDataFromSrvUDPPackets]" << std::endl;
#endif
    
//    for(unsigned int frameCount = 0; frameCount < 2; frameCount++){
//        std::cout << "Frame: " << frameCount << std::endl;
        
        auto t0 = std::chrono::high_resolution_clock::now();
        
        //auto bufferBytes = std::as_writable_bytes(std::span(buffer));
        
        //std::vector<char> outPayload;
        FrameHeader outHeader;
                         
        std::cout << "epoll_wait" << std::endl;
        int nfds = epoll_wait(epollData.epoll_fd_, epollData.events_.get(), epollData.maxEvents_, -1);
        if (nfds < 0) {
            perror("epoll_wait");
            return false;
        }
    
        for (int i = 0; i < nfds; ++i) {
            if (epollData.events_[i].events & EPOLLIN) {
                 if(!ReceiveAndReassembleFramePacket(buffer, outHeader)){
                     perror("Receive and Reassembly of Frame Packet failed\n");
                     return false;
                 }
                 else{
                    uint32_t computed = SimpleChecksum(std::as_bytes(std::span(buffer)));
                    
                    std::cout << "Header Checksums: " << outHeader.checksum_ << std::endl;
                    std::cout << "Header payload size: " << outHeader.payloadSize_ << std::endl;
                    std::cout << "buffer Checksums: " << computed << std::endl;
                    std::cout << "buffer size: " << buffer.size() << std::endl;
                    
                    if (computed != outHeader.checksum_) {
                        std::cerr << "Checksum mismatch! Packet may be corrupted." << std::endl;
                        return false;
                    }
                    auto t1 = std::chrono::high_resolution_clock::now();
                    double udpFps = 1.0 / std::chrono::duration<double>(t1 - t0).count();
                    std::cout << "udp fps: " << udpFps << std::endl;
                
                    goto end;
                 }
             }
        }
end:
 //   }
   // close(epollData.epoll_fd_);
    return true;
}

void UDPClnt::FlushSocket()
{
#ifdef VERBATIUM_COUT
    std::cout << "[UDPClnt::FlushSocket]" << std::endl;
#endif

    char discardBuffer[2048]; // Large enough for a single packet
    sockaddr_in sender {};
    socklen_t senderLen = sizeof(sender);

    int flags = MSG_DONTWAIT; // Non-blocking read

    while (true) {
        ssize_t received = recvfrom(sockfd_, discardBuffer, sizeof(discardBuffer), flags,
                                    reinterpret_cast<sockaddr*>(&sender), &senderLen);
        if (received <= 0) {
            // Nothing more to read
            break;
        }
#ifdef VERBATIUM_COUT
        std::cout << "Flushed " << received << " bytes from socket" << std::endl;
#endif
    }
}

int UDPClnt::SetNonBlocking(bool enabled)
{
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0) {
        perror("fcntl(F_GETFL) failed");
        return -1;
    }

    if (enabled) {
        flags |= O_NONBLOCK;
    } else {
        flags &= ~O_NONBLOCK;
    }

    if (fcntl(sockfd_, F_SETFL, flags) < 0) {
        perror("fcntl(F_SETFL) failed");
        return -1;
    }

    return 0;
}

bool UDPClnt::SendFramePackets(const std::vector<std::vector<uint8_t>>& packets)
{
    constexpr size_t kBatchSize = 32;
    size_t sentTotal = 0;

    while (sentTotal < packets.size()) {
        const size_t batchCount = std::min(kBatchSize, packets.size() - sentTotal);

        std::array<mmsghdr, kBatchSize> msgs{};
        std::array<iovec, kBatchSize> iovs{};

        for (size_t i = 0; i < batchCount; ++i) {
            const auto& pkt = packets[sentTotal + i];

            iovs[i].iov_base = const_cast<uint8_t*>(pkt.data());
            iovs[i].iov_len = pkt.size();

            msgs[i].msg_hdr.msg_name = &srvAddr_;
            msgs[i].msg_hdr.msg_namelen = sizeof(srvAddr_);
            msgs[i].msg_hdr.msg_iov = &iovs[i];
            msgs[i].msg_hdr.msg_iovlen = 1;
        }

        const int rc = sendmmsg(sockfd_,
                                msgs.data(),
                                static_cast<unsigned int>(batchCount),
                                0);

        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("sendmmsg");
            return false;
        }

        if (rc == 0) {
            std::cerr << "sendmmsg sent 0 packets\n";
            return false;
        }

        sentTotal += static_cast<size_t>(rc);
    }

    return true;
}
std::vector<std::vector<uint8_t>>
UDPClnt::BuildPackets(const FrameHeader& header,
             const uint8_t* data,
             size_t bytes,
             size_t mtu)
{
    const size_t headerSize = sizeof(FrameHeader);
    const size_t payloadPerPacket = mtu;

    std::vector<std::vector<uint8_t>> packets;

    size_t offset = 0;
    bool first = true;

    while (offset < bytes) {
        size_t chunk = std::min(payloadPerPacket, bytes - offset);

        std::vector<uint8_t> pkt;

        if (first) {
            pkt.resize(headerSize + chunk);
            std::memcpy(pkt.data(), &header, headerSize);
            std::memcpy(pkt.data() + headerSize, data + offset, chunk);
            first = false;
        } else {
            pkt.resize(chunk);
            std::memcpy(pkt.data(), data + offset, chunk);
        }

        offset += chunk;
        packets.push_back(std::move(pkt));
    }

    return packets;
}


