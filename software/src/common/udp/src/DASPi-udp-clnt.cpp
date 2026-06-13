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
#include <unordered_map>
#include <span>
#include <bit>
#include <cstdint>
#include <arpa/inet.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include "DASPi-udp-clnt.h"
#include "DASPi-framepacket.h"
#include "DASPi-rx-frame-assembly.h"
#include "DASPi-udp-chunk-header.h"
#include "DASPi-wire-utils.h"
#include "DASPi-raw10-pack.h"

#define VERBATIUM_COUT

using namespace DASPi;

// Constructor
UDPClnt::UDPClnt(const in_addr_t clntAddr,
                 const int clntPort,
                 const in_addr_t srvAddr,
                 const int srvPort)
    :clntAddr_(FillingClientInformation(clntAddr, clntPort)),
    clntPort_(clntPort),
    srvPort_(srvPort)
{
#ifdef VERBATIUM_COUT
   std::cout << "[UDPClnt]" << std::endl;
#endif

    std::cout << "[RX sizeof]\n";
    std::cout << "sizeof(MessageHeader)=" << sizeof(MessageHeader) << '\n';
    std::cout << "sizeof(GainMsg)=" << sizeof(GainMsg) << '\n';
    std::cout << "sizeof(FrameHeader)=" << sizeof(FrameHeader) << '\n';
    std::cout << "sizeof(UdpChunkHeader)=" << sizeof(UdpChunkHeader) << '\n';

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

namespace {

void ConfigureUdpReceiveBuffer(int sockfd, int requestedBytes)
{
    if (setsockopt(sockfd,
                   SOL_SOCKET,
                   SO_RCVBUF,
                   &requestedBytes,
                   sizeof(requestedBytes)) < 0) {
        std::cerr << "[UDPClnt] SO_RCVBUF failed requested="
                  << requestedBytes
                  << " errno=" << errno
                  << " " << std::strerror(errno)
                  << '\n';
        return;
    }

    int actualBytes = 0;
    socklen_t actualLen = sizeof(actualBytes);

    if (getsockopt(sockfd,
                   SOL_SOCKET,
                   SO_RCVBUF,
                   &actualBytes,
                   &actualLen) < 0) {
        std::cerr << "[UDPClnt] SO_RCVBUF getsockopt failed errno="
                  << errno
                  << " " << std::strerror(errno)
                  << '\n';
        return;
    }

    std::cout << "[UDPClnt] SO_RCVBUF requested="
              << requestedBytes
              << " actual="
              << actualBytes
              << '\n';
}

} // namespace	

int UDPClnt::CreatingSocketFileDescriptor()
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd_ < 0) {
        perror("socket creation failed");
        std::exit(EXIT_FAILURE);
    }

    constexpr int requestedReceiveBufferBytes = 64 * 1024 * 1024;
    ConfigureUdpReceiveBuffer(sockfd_, requestedReceiveBufferBytes);

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
    srvAddr_.sin_port = htons(srvPort_);
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

        if (sender.sin_addr.s_addr != srvAddr_.sin_addr.s_addr ||
            sender.sin_port != srvAddr_.sin_port) {
            continue;
        }

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
//#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::PrintSocksaddr_in]" << std::endl;
//#endif
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

    timeval receiveTimeout{};
    receiveTimeout.tv_sec = 0;
    receiveTimeout.tv_usec = 200000;
    if (setsockopt(sockfd_,
                   SOL_SOCKET,
                   SO_RCVTIMEO,
                   &receiveTimeout,
                   sizeof(receiveTimeout)) < 0) {
        perror("setsockopt(SO_RCVTIMEO) failed");
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
    using Clock = std::chrono::steady_clock;

    constexpr auto MAX_IDLE_WAIT = std::chrono::milliseconds(250);
    constexpr auto IDLE_SLEEP    = std::chrono::milliseconds(1);

    constexpr bool kVerboseRxFrame  = false;
    constexpr bool kVerboseRxDrops  = false;
    constexpr bool kVerboseRxIgnore = false;

    /*
     * Enable while diagnosing receive_ms.
     * Disable once wait_first_ms/drain_ms are understood.
     */
    constexpr bool kVerboseRxTiming = true;

    enum class RecvResult {
        Packet,
        Timeout,
        Fatal
    };

    enum class FinishResult {
        Incomplete,
        Complete,
        Corrupt
    };

    struct RxTiming {
        Clock::time_point enter{Clock::now()};
        Clock::time_point firstAccepted{};
        Clock::time_point payloadComplete{};
        Clock::time_point checksumStart{};
        Clock::time_point checksumDone{};

        bool sawFirstAccepted{false};

        std::uint64_t recvCalls{0};
        std::uint64_t eagainCount{0};
        std::uint64_t acceptedPackets{0};
        std::uint64_t ignoredSender{0};
        std::uint64_t invalidPackets{0};
        std::uint64_t ignoredMissingHeader{0};
        std::uint64_t duplicatePackets{0};
        std::uint64_t droppedOldFrames{0};
    };

    RxTiming timing{};
    auto lastProgress = Clock::now();

    std::vector<uint8_t> packet(maxUdpPayloadBytes_);

    static std::mutex rxLogMutex;

    auto elapsedMs = [](const Clock::time_point a,
                        const Clock::time_point b) -> double {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };

    auto senderToString = [](const sockaddr_in& addr) {
        char ip[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));

        std::ostringstream oss;
        oss << ip << ":" << ntohs(addr.sin_port);
        return oss.str();
    };

    auto log = [&](const std::string& msg) {
        std::lock_guard<std::mutex> lock(rxLogMutex);
        std::cout << msg << '\n';
    };

    auto fail = [&](const std::string& reason) -> bool {
        if constexpr (kVerboseRxDrops) {
            std::ostringstream oss;
            oss << "[RXF FAIL] this=" << this
                << " clntPort=" << clntPort_
                << " expectedSrv=" << senderToString(srvAddr_)
                << " reason=" << reason;
            log(oss.str());
        }
        return false;
    };

    auto dropFrame = [&](const uint32_t frameId,
                         const std::string& reason) {
        rxAssemblies_.erase(frameId);

        if constexpr (kVerboseRxDrops) {
            std::ostringstream oss;
            oss << "[RXF DROP] frameId=" << frameId
                << " reason=" << reason;
            log(oss.str());
        }
    };

    auto isExpectedSender = [&](const sockaddr_in& sender) -> bool {
        return sender.sin_addr.s_addr == srvAddr_.sin_addr.s_addr &&
               sender.sin_port == srvAddr_.sin_port;
    };

    auto markAcceptedPacket = [&]() {
        if (!timing.sawFirstAccepted) {
            timing.sawFirstAccepted = true;
            timing.firstAccepted = Clock::now();
        }

        ++timing.acceptedPackets;
    };

    auto logTiming = [&](const uint32_t frameId,
                         const RxFrameAssembly& assembly) {
        if constexpr (!kVerboseRxTiming) {
            return;
        }

        const double waitFirstMs =
            timing.sawFirstAccepted
                ? elapsedMs(timing.enter, timing.firstAccepted)
                : -1.0;

        const double drainMs =
            timing.sawFirstAccepted
                ? elapsedMs(timing.firstAccepted, timing.payloadComplete)
                : -1.0;

        const double checksumMs =
            elapsedMs(timing.checksumStart, timing.checksumDone);

        const double totalMs =
            elapsedMs(timing.enter, timing.checksumDone);

        std::ostringstream oss;
        oss << "[RX detailed]"
            << " frame_id=" << frameId
            << " chunks=" << assembly.chunkCount
            << " payload_bytes=" << assembly.totalBytesExpected
            << " recv_calls=" << timing.recvCalls
            << " accepted_packets=" << timing.acceptedPackets
            << " eagain=" << timing.eagainCount
            << " ignored_sender=" << timing.ignoredSender
            << " invalid_packets=" << timing.invalidPackets
            << " missing_header=" << timing.ignoredMissingHeader
            << " duplicate=" << timing.duplicatePackets
            << " dropped_old=" << timing.droppedOldFrames
            << " wait_first_ms=" << waitFirstMs
            << " drain_ms=" << drainMs
            << " checksum_ms=" << checksumMs
            << " total_receive_call_ms=" << totalMs;

        log(oss.str());
    };

    auto receivePacket = [&](sockaddr_in& sender,
                             size_t& got) -> RecvResult {
        sender = {};
        got = 0;

        socklen_t senderLen = sizeof(sender);

        const ssize_t received = recvfrom(sockfd_,
                                          packet.data(),
                                          packet.size(),
                                          0,
                                          reinterpret_cast<sockaddr*>(&sender),
                                          &senderLen);

        ++timing.recvCalls;

        if (received >= 0) {
            got = static_cast<size_t>(received);
            lastProgress = Clock::now();
            return RecvResult::Packet;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++timing.eagainCount;

            const auto now = Clock::now();

            if (now - lastProgress > MAX_IDLE_WAIT) {
                ++rxCounters_.timeout;
                rxAssemblies_.clear();
                return RecvResult::Timeout;
            }

            std::this_thread::sleep_for(IDLE_SLEEP);
            return RecvResult::Packet;
        }

        return RecvResult::Fatal;
    };

    auto decodeWireChunkHeader = [&](const size_t got,
                                     UdpChunkHeader& wire) -> bool {
        if (got < sizeof(UdpChunkHeader)) {
            ++timing.invalidPackets;

            if constexpr (kVerboseRxDrops) {
                log("[RXF DROP] packet too small for UdpChunkHeader");
            }

            return false;
        }

        uint32_t magic = 0;
        std::memcpy(&magic, packet.data(), sizeof(magic));
        magic = ntohl(magic);

        if (magic != MAGIC_NUMBER) {
            ++timing.invalidPackets;

            if constexpr (kVerboseRxDrops) {
                log("[RXF DROP] bad packet magic");
            }

            return false;
        }

        std::memcpy(&wire, packet.data(), sizeof(wire));

        wire.magic_        = ntohl(wire.magic_);
        wire.frameId_      = ntohl(wire.frameId_);
        wire.chunkId_      = ntohs(wire.chunkId_);
        wire.chunkCount_   = ntohs(wire.chunkCount_);
        wire.payloadBytes_ = ntohs(wire.payloadBytes_);

        if (wire.magic_ != MAGIC_NUMBER) {
            ++timing.invalidPackets;

            if constexpr (kVerboseRxDrops) {
                log("[RXF DROP] decoded magic mismatch");
            }

            return false;
        }

        if (wire.chunkCount_ == 0 || wire.chunkId_ >= wire.chunkCount_) {
            ++timing.invalidPackets;

            if constexpr (kVerboseRxDrops) {
                std::ostringstream oss;
                oss << "[RXF DROP] invalid chunk fields frameId="
                    << wire.frameId_
                    << " chunkId=" << wire.chunkId_
                    << " chunkCount=" << wire.chunkCount_;
                log(oss.str());
            }

            return false;
        }

        if (sizeof(UdpChunkHeader) + wire.payloadBytes_ > got) {
            ++timing.invalidPackets;

            if constexpr (kVerboseRxDrops) {
                std::ostringstream oss;
                oss << "[RXF DROP] payload exceeds datagram frameId="
                    << wire.frameId_
                    << " got=" << got
                    << " payloadBytes=" << wire.payloadBytes_
                    << " headerSize=" << sizeof(UdpChunkHeader);
                log(oss.str());
            }

            return false;
        }

        return true;
    };

     auto resetReceiveTimingForNewFrame = [&]() {
        const auto droppedOldFrames = timing.droppedOldFrames;
    
        timing.enter = Clock::now();
        timing.firstAccepted = {};
        timing.payloadComplete = {};
        timing.checksumStart = {};
        timing.checksumDone = {};
        timing.sawFirstAccepted = false;
    
        timing.recvCalls = 0;
        timing.eagainCount = 0;
        timing.acceptedPackets = 0;
        timing.ignoredSender = 0;
        timing.invalidPackets = 0;
        timing.ignoredMissingHeader = 0;
        timing.duplicatePackets = 0;
    
        /*
         * Preserve this across the reset so the completed-frame log still shows
         * that an old partial frame was dropped before this frame completed.
         */
        timing.droppedOldFrames = droppedOldFrames;
    };
    
    auto logDropOldFrame = [&](const UdpChunkHeader& wire) {
        if constexpr (kVerboseRxDrops) {
            std::ostringstream oss;
            oss << "[RXF DROP OLD] newFrameId=" << wire.frameId_
                << " reason=new frame started before old frame completed";
            log(oss.str());
        }
    };
    
    auto isCurrentFrame = [&](const UdpChunkHeader& wire) -> bool {
        return rxAssemblies_.find(wire.frameId_) != rxAssemblies_.end();
    };
    
    auto canStartReplacementFrame = [](const UdpChunkHeader& wire) -> bool {
        /*
         * Only chunk 0 can initialize the frame header.
         * A mid-frame packet for a different frame must be ignored.
         */
        return wire.chunkId_ == 0;
    };
    
    auto dropOldFrameAndStartNewTiming = [&](const UdpChunkHeader& wire) {
        rxAssemblies_.clear();
    
        ++rxCounters_.dropOld;
        ++timing.droppedOldFrames;
    
        logDropOldFrame(wire);
        resetReceiveTimingForNewFrame();
    };
    
    auto acceptFrameIdOrDropOld = [&](const UdpChunkHeader& wire) -> bool {
        if (rxAssemblies_.empty()) {
            return true;
        }
    
        if (isCurrentFrame(wire)) {
            return true;
        }
    
        if (!canStartReplacementFrame(wire)) {
            return false;
        }
    
        dropOldFrameAndStartNewTiming(wire);
        return true;
    };

    auto getOrCreateAssembly = [&](const UdpChunkHeader& wire)
        -> RxFrameAssembly* {
        auto [it, inserted] = rxAssemblies_.try_emplace(wire.frameId_);
        auto& assembly = it->second;

        if (inserted) {
            assembly = RxFrameAssembly{};
            assembly.frameId = wire.frameId_;
            assembly.chunkCount = wire.chunkCount_;
            assembly.chunkSeen.assign(wire.chunkCount_, 0);
            return &assembly;
        }

        if (assembly.chunkCount != wire.chunkCount_) {
            dropFrame(wire.frameId_, "chunkCount mismatch");
            return nullptr;
        }

        return &assembly;
    };

    auto initializeHeaderIfPresent = [&](RxFrameAssembly& assembly,
                                         const UdpChunkHeader& wire) -> bool {
        if (assembly.headerSeen) {
            return true;
        }
    
        if (wire.chunkId_ != 0) {
            ++timing.ignoredMissingHeader;
            return false;
        }
    
        assembly.frameHeader = FromWireFrameHeader(wire.frameHeader_);
    
        if (assembly.frameHeader.magic_ != MAGIC_NUMBER) {
            dropFrame(wire.frameId_, "frameHeader magic mismatch");
            return false;
        }
    
        if (assembly.frameHeader.payloadSize_ == 0 ||
            assembly.frameHeader.payloadSize_ > FramePacket::MAX_ALLOWED_FRAME_SIZE_) {
            std::ostringstream oss;
            oss << "invalid frameHeader payloadSize="
                << assembly.frameHeader.payloadSize_;
            dropFrame(wire.frameId_, oss.str());
            return false;
        }
    
        const WirePixelFormat wirePixelFormat =
            ToWirePixelFormat(assembly.frameHeader.wirePixelFormat_);
    
        if (wirePixelFormat != WirePixelFormat::Uint16LE &&
            wirePixelFormat != WirePixelFormat::BayerRaw10Packed) {
            std::ostringstream oss;
            oss << "unsupported wirePixelFormat="
                << assembly.frameHeader.wirePixelFormat_;
            dropFrame(wire.frameId_, oss.str());
            return false;
        }
    
        size_t totalPixels = 0;
        size_t totalPayloadBytesFromRegions = 0;
    
        for (size_t region = 0; region < NUM_REGIONS; ++region) {
            const size_t pixelCount =
                static_cast<size_t>(assembly.frameHeader.regionSizes_[region]);
    
            const size_t regionPayloadBytes =
                static_cast<size_t>(assembly.frameHeader.regionPayloadBytes_[region]);
    
            if (pixelCount > std::numeric_limits<size_t>::max() - totalPixels) {
                dropFrame(wire.frameId_, "regionSizes pixel count overflow");
                return false;
            }
    
            totalPixels += pixelCount;
    
            if (regionPayloadBytes >
                std::numeric_limits<size_t>::max() - totalPayloadBytesFromRegions) {
                dropFrame(wire.frameId_, "regionPayloadBytes sum overflow");
                return false;
            }
    
            totalPayloadBytesFromRegions += regionPayloadBytes;
    
            size_t expectedRegionPayloadBytes = 0;
    
            if (wirePixelFormat == WirePixelFormat::Uint16LE) {
                if (pixelCount >
                    std::numeric_limits<size_t>::max() / sizeof(uint16_t)) {
                    dropFrame(wire.frameId_, "Uint16LE region byte count overflow");
                    return false;
                }
    
                expectedRegionPayloadBytes =
                    pixelCount * sizeof(uint16_t);
            } else {
                expectedRegionPayloadBytes =
                    Raw10PackedByteCount(pixelCount);
            }
    
            if (regionPayloadBytes != expectedRegionPayloadBytes) {
                std::ostringstream oss;
                oss << "region payload byte mismatch"
                    << " region=" << region
                    << " wirePixelFormat=" << assembly.frameHeader.wirePixelFormat_
                    << " pixelCount=" << pixelCount
                    << " regionPayloadBytes=" << regionPayloadBytes
                    << " expectedRegionPayloadBytes=" << expectedRegionPayloadBytes;
    
                dropFrame(wire.frameId_, oss.str());
                return false;
            }
        }
    
        if (totalPayloadBytesFromRegions != assembly.frameHeader.payloadSize_) {
            std::ostringstream oss;
            oss << "regionPayloadBytes mismatch"
                << " totalPixels=" << totalPixels
                << " totalPayloadBytesFromRegions=" << totalPayloadBytesFromRegions
                << " payloadSize=" << assembly.frameHeader.payloadSize_;
    
            dropFrame(wire.frameId_, oss.str());
            return false;
        }
    
        assembly.totalBytesExpected = assembly.frameHeader.payloadSize_;
        assembly.totalBytesReceived = 0;
    
        /*
         * Store the encoded wire payload bytes.
         *
         * For Uint16LE:
         *   payload bytes are copied as raw uint16_t bytes and decoded later.
         *
         * For BayerRaw10Packed:
         *   payload bytes are copied as packed RAW10 and unpacked later.
         */
        assembly.payload.assign(assembly.totalBytesExpected, 0);
    
        assembly.headerSeen = true;
    
        return true;
    };

    auto copyChunkPayload = [&](RxFrameAssembly& assembly,
                                const UdpChunkHeader& wire) -> bool {
        if (!assembly.headerSeen) {
            ++timing.ignoredMissingHeader;
            return false;
        }

        if (wire.chunkId_ >= assembly.chunkSeen.size()) {
            dropFrame(wire.frameId_, "chunkId outside chunkSeen size");
            return false;
        }

        if (assembly.chunkSeen[wire.chunkId_]) {
            ++timing.duplicatePackets;
            return false;
        }

        const size_t maxChunkPayload =
            maxUdpPayloadBytes_ - sizeof(UdpChunkHeader);

        const size_t dstOffset =
            static_cast<size_t>(wire.chunkId_) * maxChunkPayload;

        if (dstOffset + wire.payloadBytes_ > assembly.totalBytesExpected) {
            std::ostringstream oss;
            oss << "chunk write exceeds payload dstOffset=" << dstOffset
                << " payloadBytes=" << wire.payloadBytes_
                << " expected=" << assembly.totalBytesExpected;
            dropFrame(wire.frameId_, oss.str());
            return false;
        }

        std::memcpy(assembly.payload.data() + dstOffset,
                    packet.data() + sizeof(UdpChunkHeader),
                    wire.payloadBytes_);

        assembly.chunkSeen[wire.chunkId_] = 1;
        assembly.totalBytesReceived += wire.payloadBytes_;

        return true;
    };

    auto finishFrameIfComplete = [&](RxFrameAssembly& assembly,
                                     const UdpChunkHeader& wire) -> FinishResult {
        if (assembly.totalBytesReceived < assembly.totalBytesExpected) {
            return FinishResult::Incomplete;
        }

        timing.payloadComplete = Clock::now();

        const auto usedBytes =
            std::as_bytes(std::span(assembly.payload)).first(
                assembly.totalBytesExpected
            );

        timing.checksumStart = Clock::now();
        const uint32_t computed = SimpleChecksum(usedBytes);
        timing.checksumDone = Clock::now();

        if (computed != assembly.frameHeader.checksum_) {
            std::ostringstream oss;
            oss << "checksum mismatch computed=" << computed
                << " expected=" << assembly.frameHeader.checksum_;
            dropFrame(wire.frameId_, oss.str());
            return FinishResult::Corrupt;
        }

        logTiming(wire.frameId_, assembly);

        outHeader = assembly.frameHeader;
        outPayload.clear();
        
        const WirePixelFormat wirePixelFormat =
            ToWirePixelFormat(assembly.frameHeader.wirePixelFormat_);
        
        std::size_t totalPixels = 0;
        for (const auto count : assembly.frameHeader.regionSizes_) {
            totalPixels += count;
        }
        
        if (wirePixelFormat == WirePixelFormat::Uint16LE) {
            if ((assembly.payload.size() % sizeof(std::uint16_t)) != 0) {
                dropFrame(wire.frameId_, "Uint16LE payload not u16 aligned");
                return FinishResult::Corrupt;
            }
        
            outPayload.resize(assembly.payload.size() / sizeof(std::uint16_t));
        
            if (!assembly.payload.empty()) {
                std::memcpy(outPayload.data(),
                            assembly.payload.data(),
                            assembly.payload.size());
            }
        } else if (wirePixelFormat == WirePixelFormat::BayerRaw10Packed) {
            outPayload.reserve(totalPixels);
        
            std::size_t srcOffset = 0;
        
            for (std::size_t region = 0; region < NUM_REGIONS; ++region) {
                const std::size_t pixels =
                    assembly.frameHeader.regionSizes_[region];
        
                const std::size_t bytes =
                    assembly.frameHeader.regionPayloadBytes_[region];
        
                if (srcOffset + bytes > assembly.payload.size()) {
                    dropFrame(wire.frameId_, "RAW10 region read overflow");
                    return FinishResult::Corrupt;
                }
        
                const bool ok =
                    AppendRaw10UnpackedExpanded16(
                        std::span<const std::uint8_t>(
                            assembly.payload.data() + srcOffset,
                            bytes
                        ),
                        pixels,
                        outPayload
                    );
        
                if (!ok) {
                    dropFrame(wire.frameId_, "RAW10 unpack failed");
                    return FinishResult::Corrupt;
                }
        
                srcOffset += bytes;
            }
        } else {
            dropFrame(wire.frameId_, "unsupported wire pixel format at finish");
            return FinishResult::Corrupt;
        }
        
        if (outPayload.size() != totalPixels) {
            dropFrame(wire.frameId_, "decoded pixel count mismatch");
            return FinishResult::Corrupt;
        }
        
        rxAssemblies_.erase(wire.frameId_);
        return FinishResult::Complete;
    };

    if constexpr (kVerboseRxFrame) {
        std::ostringstream oss;
        oss << "[RXF ENTER] this=" << this
            << " clntPort=" << clntPort_
            << " expectedSrv=" << senderToString(srvAddr_);
        log(oss.str());
    }

    while (true) {
        sockaddr_in sender{};
        size_t got = 0;

        const RecvResult recvResult = receivePacket(sender, got);

        if (recvResult == RecvResult::Timeout) {
            return fail("idle timeout waiting for frame chunks");
        }

        if (recvResult == RecvResult::Fatal) {
            std::ostringstream oss;
            oss << "recvfrom failed errno=" << errno
                << " " << std::strerror(errno);
            return fail(oss.str());
        }

        /*
         * EAGAIN/EWOULDBLOCK returns RecvResult::Packet after sleeping,
         * but got stays 0. Continue the loop and try again.
         */
        if (got == 0) {
            continue;
        }

        if (!isExpectedSender(sender)) {
            ++timing.ignoredSender;

            if constexpr (kVerboseRxIgnore) {
                std::ostringstream oss;
                oss << "[RXF IGNORE] unexpected sender got="
                    << senderToString(sender)
                    << " expected=" << senderToString(srvAddr_);
                log(oss.str());
            }

            continue;
        }

        UdpChunkHeader wire{};

        if (!decodeWireChunkHeader(got, wire)) {
            continue;
        }

        markAcceptedPacket();

        if (!acceptFrameIdOrDropOld(wire)) {
            continue;
        }

        RxFrameAssembly* assembly = getOrCreateAssembly(wire);

        if (assembly == nullptr) {
            continue;
        }

        if (!initializeHeaderIfPresent(*assembly, wire)) {
            continue;
        }

        if (!copyChunkPayload(*assembly, wire)) {
            continue;
        }

        const FinishResult finishResult = finishFrameIfComplete(*assembly, wire);

        if (finishResult == FinishResult::Complete) {
            return true;
        }

        if (finishResult == FinishResult::Corrupt) {
            return false;
        }
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
    //std::cout << "[UDPClnt::SimpleChecksum]" << std::endl;
#endif
    uint32_t sum = 0;
    for (auto b : bytes) {
        sum += static_cast<uint8_t>(b);
    }
    return sum;
}



bool UDPClnt::InitEpollForSrvUDPPackets(/*std::vector<char> &buffer,*/ EpollData &epollData){
#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::InitEpollForSrvUDPPackets]" << std::endl;
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
    
    //constexpr bool kVerboseEpoll = false;

    //if constexpr (kVerboseEpoll) {
        ////PrintSocksaddr_in(...);
    //}

//        sockaddr_in headerSender;//retrieved UDP header from

    FlushSocket();

    return true;
}

bool UDPClnt::FinalizeEpollForSrvUDPPackets(const EpollData &epollData){
#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::FinalizeEpollForSrvUDPPackets]" << std::endl;
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
//#ifdef VERBATIUM_COUT
    //std::cout << "[UDPClnt::FlushSocket]" << std::endl;
//#endif

    //char discardBuffer[2048]; // Large enough for a single packet
    //sockaddr_in sender {};
    //socklen_t senderLen = sizeof(sender);

    //int flags = MSG_DONTWAIT; // Non-blocking read

    //while (true) {
        //ssize_t received = recvfrom(sockfd_, discardBuffer, sizeof(discardBuffer), flags,
                                    //reinterpret_cast<sockaddr*>(&sender), &senderLen);
        //if (received <= 0) {
            //// Nothing more to read
            //break;
        //}
//#ifdef VERBATIUM_COUT
        //std::cout << "Flushed " << received << " bytes from socket" << std::endl;
//#endif
//    }
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

UDPClntRxCounters UDPClnt::ConsumeRxCounters()
{
    UDPClntRxCounters out = rxCounters_;
    rxCounters_ = {};
    return out;
}
