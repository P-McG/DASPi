// Server side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <netdb.h>

#include "DASPi-logger.h"
#include "DASPi-udp-srv.h"

#define VERBATIUM_COUT

using namespace DASPi;

// Constructor
UDPSrv::UDPSrv(const std::string& clientIp, const size_t port)
:cliaddr_(InitSockaddrFromIP(clientIp, port)),
port_(port){
  CreatingSocketFileDescriptor();
  FillingServerInformation();
  BindSocketWithServerAddress();
};

// Destructor
UDPSrv::~UDPSrv(){
};

size_t UDPSrv::GetMaximumTransmittableUnits(){
    std::lock_guard<std::mutex> lock(maximumTransmittableUnitsMtx_);
    return maximumTransmittableUnits_;
};

const sockaddr_in UDPSrv::GetServerAddress(){
    std::lock_guard<std::mutex> lock(servaddrMtx_);
    return servaddr_;
}

void UDPSrv::SetServerAddress( const sockaddr_in addr ){
    std::lock_guard<std::mutex> lock(servaddrMtx_);
    servaddr_ = addr;
}

const sockaddr_in UDPSrv::GetClientAddress(){
    std::lock_guard<std::mutex> lock(cliaddrMtx_);
    return cliaddr_;
}

// CreatingSocketFileDescriptor
int UDPSrv::CreatingSocketFileDescriptor(){
#ifdef VERBATIUM_COUT
    std::cout << "Creating socket file descriptor" << std::endl;
#endif
    if ( (sockfd_= socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    return 0;
};

sockaddr_in UDPSrv::InitSockaddrFromIP(const std::string& clientIp, const size_t port){

    sockaddr_in cliaddr;
    //memset(&cliaddr_, 0, sizeof(cliaddr_));  // Zero out the struct

    cliaddr.sin_family = AF_INET;           // IPv4
    cliaddr.sin_port = htons(port);         // Convert port to network byte order

    // Convert IP string to binary form
    if (inet_pton(AF_INET, clientIp.c_str(), &cliaddr.sin_addr) <= 0) {
        perror("inet_pton failed");
        exit(1);
    }

    return cliaddr;
}

// FillingServerInformation
sockaddr_in UDPSrv::FillingServerInformation(){
#ifdef VERBATIUM_COUT
    std::cout << "Filling server information" << std::endl;
#endif

    std::lock_guard<std::mutex> lock(servaddrMtx_);

    memset(&servaddr_, 0, sizeof(servaddr_));

    servaddr_.sin_family = AF_INET; // IPv4

    std::cout << "host addr: " << get_host_ip().c_str() << std::endl;

    servaddr_.sin_addr.s_addr = inet_addr(get_host_ip().c_str());
    
    servaddr_.sin_port = htons(port_);
    
    return servaddr_;
};

int UDPSrv::BindSocketWithServerAddress(){
#ifdef VERBATIUM_COUT
    std::cout << "Bind socket with server address" << std::endl;
#endif
    std::lock_guard<std::mutex> lock(servaddrMtx_);
    
    if ( bind(sockfd_, (const struct sockaddr *)&servaddr_,
            sizeof(servaddr_)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    return 0;
};

std::string UDPSrv::get_host_ip() {
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

void UDPSrv::printSockAddr(const sockaddr_in &addr) {
    // Convert IP address to a string
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);

    // Get port in host byte order
    uint16_t port = ntohs(addr.sin_port);

    // Output IP and port
    std::cout << "IP Address: " << ip << "\nPort: " << port << std::endl;
}

//uint32_t UDPSrv::SimpleChecksum(const uint8_t* data, size_t size) {
    //uint32_t sum = 0;
    //for (size_t i = 0; i < size; ++i) {
        //sum += static_cast<uint8_t>(data[i]);
    //}
    //return sum;
//}

uint32_t UDPSrv::SimpleChecksum(std::span<const std::byte> bytes) {
#ifdef VERBATIUM_COUT
    std::cout << "[UDPSrv::SimpleChecksum]" << std::endl;
#endif
    uint32_t sum = 0;
    for (auto b : bytes) {
        sum += static_cast<uint8_t>(b);
    }
    return sum;
}

//void UDPSrv::SendFramePacketToClient(const FramePacket &framePacket) {
    //log_verbose("[UDPSrv::SendUDPPacketToClient]");

    //// FPS tracking block
    //static int count = 0;
    //static auto last = std::chrono::steady_clock::now();
    
    //count++;
    //auto now = std::chrono::steady_clock::now();
    //if (std::chrono::duration_cast<std::chrono::seconds>(now - last).count() >= 1) {
        //std::cout << "[Output FPS] " << count << " frames/sec" << std::endl;
        //count = 0;
        //last = now;
    //}


    //const size_t mtu = GetMaximumTransmittableUnits();
    //size_t bytesSent = 0;

    //// --- First Packet (header + partial payload) ---
    //size_t firstPayloadBytes = std::min(mtu - sizeof(FrameHeader), framePacket.payload_.size());

    //// Use stack buffer for first packet only
    //uint8_t firstChunk[1500]; // fits common MTU

    //std::memcpy(firstChunk, &framePacket.header_, sizeof(FrameHeader));
    //std::memcpy(firstChunk + sizeof(FrameHeader), framePacket.payload_.data(), firstPayloadBytes);

    //size_t firstPacketSize = sizeof(FrameHeader) + firstPayloadBytes;

    //int sendResult = SendUDPPacketToClient(firstChunk, firstPacketSize);
    //if (sendResult < 0) {
        //std::cerr << "Send to client failed at offset: 0" << std::endl;
        //exit(1);
    //}

    //bytesSent += firstPayloadBytes;

    //// --- Remaining Payload Packets ---
    //while (bytesSent < framePacket.payload_.size()) {
        //size_t chunkSize = std::min(mtu, framePacket.payload_.size() - bytesSent);

//#ifdef VERBATIUM_COUT
        //std::cout << "Sending chunk of size " << chunkSize << " at offset " << bytesSent << std::endl;
//#endif

        //const uint8_t* chunkPtr = framePacket.payload_.data() + bytesSent;
        //int result = SendUDPPacketToClient(chunkPtr, chunkSize);
        //if (result < 0) {
            //std::cerr << "Send to client failed at offset: " << bytesSent << std::endl;
            //exit(1);
        //}

        //bytesSent += chunkSize;
    //}

//#ifdef VERBATIUM_COUT
    //std::cout << "Finished sending frame packet. Total bytes sent: " << bytesSent + sizeof(FrameHeader) << std::endl;
//#endif
//}

void UDPSrv::SendFramePacketToClient(const FramePacket &framePacket) {
    log_verbose("[UDPSrv::SendUDPPacketToClient]");

    // FPS tracking
    static int count = 0;
    static auto last = std::chrono::steady_clock::now();
    if (++count, std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - last).count() >= 1) {
        std::cout << "[Output FPS] " << count << " frames/sec" << std::endl;
        count = 0;
        last = std::chrono::steady_clock::now();
    }

    const size_t mtu = GetMaximumTransmittableUnits();
    auto payloadBytes = framePacket.payload_as_bytes(); // already a byte span
    const size_t payloadSize = payloadBytes.size();

    // --- First Packet (header + partial payload) ---
    const size_t firstPayloadBytes = std::min(mtu - sizeof(FrameHeader), payloadSize);

    std::vector<uint8_t> firstChunk(sizeof(FrameHeader) + firstPayloadBytes);
    std::memcpy(firstChunk.data(), &framePacket.header_, sizeof(FrameHeader));
    std::memcpy(firstChunk.data() + sizeof(FrameHeader),
                payloadBytes.data(), firstPayloadBytes);

    if (SendUDPPacketToClient(firstChunk.data(), firstChunk.size()) < 0) {
        std::cerr << "Send to client failed at offset: 0" << std::endl;
        std::exit(1);
    }

    // --- Remaining Payload Packets ---
    size_t bytesSent = firstPayloadBytes;
    const uint8_t* payloadPtr = reinterpret_cast<const uint8_t*>(payloadBytes.data());

    while (bytesSent < payloadSize) {
        size_t chunkSize = std::min(mtu, payloadSize - bytesSent);

#ifdef VERBATIUM_COUT
        std::cout << "Sending chunk of size " << chunkSize
                  << " at offset " << bytesSent << std::endl;
#endif
        if (SendUDPPacketToClient(payloadPtr + bytesSent, chunkSize) < 0) {
            std::cerr << "Send to client failed at offset: " << bytesSent << std::endl;
            std::exit(1);
        }
        bytesSent += chunkSize;
    }

#ifdef VERBATIUM_COUT
    std::cout << "Finished sending frame packet. Total bytes sent: "
              << (bytesSent + sizeof(FrameHeader)) << std::endl;
#endif
}


void UDPSrv::TransmitFrame(const FramePacket &&data) {
  log_verbose("[UDPSrv::TransmitFrame]");
    SendFramePacketToClient(std::move(data));
}

void UDPSrv::SubmitFrameOutput(uint64_t frameNumber, FramePacket &&framePacket) {

    log_verbose("[UDPSrv::SubmitFrameOutput]");
    
    static size_t submitted = 0;
    std::cout << "[SubmitFrameOutput] Frame #" << frameNumber << " submitted (" << ++submitted << " total)" << std::endl;


    static uint64_t lastFrameNumber = 0;
    static std::map<int64_t, size_t> disorderHistogram;  // offset -> count
    
    int64_t diff = 0;
    if (lastFrameNumber != 0) {
        diff = static_cast<int64_t>(frameNumber) - static_cast<int64_t>(lastFrameNumber);
        if (diff != 1) {
            std::cout << "[FrameOutOfOrder] Current: " << frameNumber
                      << ", Last: " << lastFrameNumber
                      << ", Delta: " << diff << std::endl;
            disorderHistogram[diff]++;
        }
    }
    lastFrameNumber = frameNumber;
    
    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[DisorderHistogram] Last 100 frames:" << std::endl;
        for (const auto& [diff, freq] : disorderHistogram) {
            std::cout << "Delta" << diff << ": " << freq << std::endl;
        }
        disorderHistogram.clear();
    }
    
    static int64_t maxDisorder = 0;
    maxDisorder = std::max(maxDisorder, std::abs(diff));

    //std::vector<uint8_t> payload(data, data + size);

     {
        std::lock_guard<std::mutex> lock(outputMutex_);
    
        // Check for frame rollback or unexpected reset
        if (nextFrameInitialized_ && frameNumber < nextFrameToSend_) {
            std::cout << "[WARN] Detected frame rollback from " << nextFrameToSend_
                      << " to " << frameNumber << ". Resetting state." << std::endl;
    
            // Reset internal state
            orderedFrames_.clear();
            nextFrameToSend_ = frameNumber;
            nextFrameInitialized_ = true;  // immediately reinitialize
        }
    
        // Normal one-time init
        if (!nextFrameInitialized_) {
            nextFrameToSend_ = frameNumber;
            nextFrameInitialized_ = true;
            std::cout << "[Init] nextFrameToSend_ = " << nextFrameToSend_ << std::endl;
        }
    
        orderedFrames_[frameNumber] = std::move(framePacket);
        
        std::cout << "[SubmitFrameOutput] Stored frame " << frameNumber
                  << ", queue size = " << orderedFrames_.size() << std::endl;
        
        if (orderedFrames_.size() > 100)
            orderedFrames_.erase(orderedFrames_.begin());
    }
    {
        std::lock_guard<std::mutex> lock(outputMutex_);
        TrySendFramesInOrder();
    }
}

bool UDPSrv::TrySendFramesInOrder() {
    log_verbose("[USPSrv::TrySendFramesInOrder]");

    bool didSend = false;

    // Initialize frame counter if needed
    if (nextFrameToSend_ == std::numeric_limits<uint64_t>::max() && !orderedFrames_.empty()) {
        nextFrameToSend_ = orderedFrames_.begin()->first;
        std::cout << "[TSFO] Resynced nextFrameToSend_ = " << nextFrameToSend_ << std::endl;
    }

    if (orderedFrames_.empty()) {
        std::cout << "[TSFO] orderedFrames_ is empty — nothing to send" << std::endl;
        return false;
    }

    std::cout << "[TrySendFramesInOrder] nextFrameToSend_ = " << nextFrameToSend_ << std::endl;
    std::cout << "[TrySendFramesInOrder] orderedFrames_ keys: ";
    for (const auto& kv : orderedFrames_)
        std::cout << kv.first << " ";
    std::cout << std::endl;

    constexpr size_t maxPerLoop = 100;
    constexpr size_t maxSkips = 1;
    size_t skips = 0;
    size_t sent = 0;

    while (true) {
        if (++sent > maxPerLoop) {
            std::cout << "[TSFO] Reached max batch size (" << maxPerLoop << "), breaking." << std::endl;
            break;
        }

        auto it = orderedFrames_.find(nextFrameToSend_);
        if (it == orderedFrames_.end()) {
            std::cout << "[TSFO] Missing frame " << nextFrameToSend_
                      << ", current map size: " << orderedFrames_.size() << std::endl;

            if (++skips >= maxSkips) {
                std::cout << "[TSFO] Skipping missing frame " << nextFrameToSend_ << std::endl;

                if (!orderedFrames_.empty() &&
                    orderedFrames_.begin()->first > nextFrameToSend_ + 100) {
                    std::cout << "[TSFO] Large gap detected — fast-forwarding to "
                              << orderedFrames_.begin()->first << std::endl;
                    nextFrameToSend_ = orderedFrames_.begin()->first;
                } else {
                    ++nextFrameToSend_;
                }

                skips = 0;
                continue;
            }

            break;  // allow frame to appear on next cycle
        }

        // Frame is ready → transmit
        std::cout << "[TSFO] Sending frame with metadata" << nextFrameToSend_ << std::endl;
        TransmitFrame(std::move(it->second));
        orderedFrames_.erase(it);

        ++nextFrameToSend_;
        skips = 0;
        didSend = true;
    }

    if (didSend)
        std::cout << "[TSFO] Finished sending " << sent << " frames this batch." << std::endl;

    return didSend;
}

//void UDPSrv::SendToServer(const uint8_t* buffer, size_t bufferLength) {
	//socklen_t len = sizeof(servaddr_);
	//ssize_t sendN = sendto(sockfd_, buffer, bufferLength, MSG_CONFIRM,
	                       //reinterpret_cast<const struct sockaddr*>(&servaddr_),
	                       //len);
	//if (sendN < 0) {
		//perror("UDP send failed");
    //}
 //#ifdef VERBATIUM_COUT
   ////else {
		////std::cout << "Sent UDP packet: " << sendN << " bytes" << std::endl;
	////}
//#endif
//}

//FramePacket UDPSrv::CreateFramePacket(const GainMsg &gainMsg, std::vector<uint16_t>&& buffer) {
//#ifdef VERBATIUM_COUT
    //std::cout << "CreateFramePacket" << std::endl;
//#endif
    //FramePacket framePacket;

    //framePacket.payload_ = std::move(buffer);

    //// Fill header
    //framePacket.header_.magic_ = htonl(MAGIC_NUMBER);
    
    //framePacket.header_.gainMsg = gainMsg;

    //uint32_t payloadSize = static_cast<uint32_t>(framePacket.payload_.size());
    //framePacket.header_.payloadSize_ = htonl(payloadSize);

    ////std::cout << "First 16 bytes of payload: ";
    ////for (size_t i = 0; i < std::min<size_t>(16, payloadSize); ++i) {
        ////std::cout << static_cast<int>(framePacket.payload_[i]) << " ";
    ////}
    ////std::cout << std::endl;


    //uint32_t checksum = SimpleChecksum(framePacket.payload_as_bytes());
    //framePacket.header_.checksum_ = htonl(checksum);

//#ifdef VERBATIUM_COUT
    //std::cout << "finish filling header - checksum, payloadSize:" << checksum << ", " << payloadSize << std::endl;
//#endif

    //return framePacket;
//}

FramePacket UDPSrv::CreateFramePacket(
    const GainMsg& gainMsg,
    const std::array<uint32_t, NUM_REGIONS>& regionSizes,
    std::vector<uint16_t>&& buffer)
{
    FramePacket pkt{};

    pkt.payload = std::move(buffer);

    pkt.header.magic_ = htonl(MAGIC_NUMBER);
    pkt.header.gainMsg = gainMsg;

    pkt.header.payloadSize_ =
        htonl(static_cast<uint32_t>(pkt.payload.size() * sizeof(uint16_t)));

    for (size_t i = 0; i < NUM_REGIONS; ++i) {
        pkt.header.regionSizes_[i] = htonl(regionSizes[i]);
    }

    pkt.header.checksum_ =
        htonl(SimpleChecksum(pkt.payload_as_bytes()));

    return pkt;
}

