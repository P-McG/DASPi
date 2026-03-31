// Client side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <span>
#include <vector>
#include <array>
#include <cstring>
#include <type_traits>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <algorithm>
#include <limits>
#include <atomic>

#include "DASPi-framepacket.h"
#include "DASPi-udp-chunk-header.h"

//#define VERBATIUM_COUT

namespace DASPi{

    template<typename T>
    ssize_t UDPClnt::SendToServer(T* data, size_t count)
    {
        if (data == nullptr || count == 0) {
            return 0;
        }
    
        static_assert(std::is_trivially_copyable_v<T>,
                      "SendToServer requires trivially copyable payload types");
    
        const auto* src = reinterpret_cast<const uint8_t*>(data);
        const size_t totalBytes = count * sizeof(T);
    
        if (maxUdpPayloadBytes_ <= sizeof(UdpChunkHeader)) {
            std::cerr << "[TX] maxUdpPayloadBytes_ too small for UdpChunkHeader\n";
            return -1;
        }
    
        const size_t maxChunkPayload = maxUdpPayloadBytes_ - sizeof(UdpChunkHeader);
        const size_t chunkCount =
            (totalBytes + maxChunkPayload - 1) / maxChunkPayload;
    
        if (chunkCount == 0) {
            return 0;
        }
    
        if (chunkCount > std::numeric_limits<uint16_t>::max()) {
            std::cerr << "[TX] Too many chunks for one frame: " << chunkCount << '\n';
            return -1;
        }
    
        static std::atomic<uint32_t> nextFrameId{1};
        const uint32_t frameId = nextFrameId.fetch_add(1, std::memory_order_relaxed);
    
        FrameHeader frameHeader{};
        frameHeader.magic_ = MAGIC_NUMBER;
        frameHeader.payloadSize_ = static_cast<uint32_t>(totalBytes);
        frameHeader.checksum_ =
            SimpleChecksum(std::as_bytes(std::span<const T>(data, count)));
    
        // TODO: populate these correctly for your actual frame stream.
        // frameHeader.gainMsg = ...;
        // frameHeader.regionSizes_ = ...;
    
        std::vector<std::vector<uint8_t>> packets;
        packets.reserve(chunkCount);
    
        size_t offset = 0;
        for (size_t chunkId = 0; chunkId < chunkCount; ++chunkId) {
            const size_t payloadBytes = std::min(maxChunkPayload, totalBytes - offset);
    
            UdpChunkHeader hdr{};
            hdr.magic_        = htonl(MAGIC_NUMBER);
            hdr.frameId_      = htonl(frameId);
            hdr.chunkId_      = htons(static_cast<uint16_t>(chunkId));
            hdr.chunkCount_   = htons(static_cast<uint16_t>(chunkCount));
            hdr.payloadBytes_ = htons(static_cast<uint16_t>(payloadBytes));
            hdr.reserved_     = 0;
    
            if (chunkId == 0) {
                hdr.frameHeader_ = ToWireFrameHeader(frameHeader);
            }
    
            std::vector<uint8_t> pkt(sizeof(UdpChunkHeader) + payloadBytes);
            std::memcpy(pkt.data(), &hdr, sizeof(UdpChunkHeader));
            std::memcpy(pkt.data() + sizeof(UdpChunkHeader), src + offset, payloadBytes);
    
            std::cout << std::dec
                      << "[TX] frameId=" << frameId
                      << " chunkId=" << chunkId
                      << "/" << chunkCount
                      << " payloadBytes=" << payloadBytes
                      << " packetBytes=" << pkt.size()
                      << '\n';
    
            packets.push_back(std::move(pkt));
            offset += payloadBytes;
        }
    
        if (!SendFramePackets(packets)) {
            std::cerr << "[TX] SendFramePackets failed for frameId=" << frameId << '\n';
            return -1;
        }
    
        return static_cast<ssize_t>(totalBytes);
    }    
    template<class T>
    ssize_t UDPClnt::ReceiveFromServer(T& buffer)
    {
        static_assert(std::is_standard_layout_v<T>);
        static_assert(std::is_trivially_copyable_v<T>);
    
        sockaddr_in sender{};
        socklen_t senderLen = sizeof(sender);
    
        const ssize_t received = recvfrom(
            sockfd_,
            &buffer,
            sizeof(T),
            0,
            reinterpret_cast<sockaddr*>(&sender),
            &senderLen
        );
    
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0; // no data available right now
            }
            perror("recvfrom failed");
            return -1;
        }
    
        return received;
    }
    
    //template<class T>
    //ssize_t UDPClnt::ReceiveFromServer(T &buffer){
//#ifdef VERBATIUM_COUT
////    std::cout << "Receive from server" << std::endl;
//#endif
    
        //static_assert(std::is_standard_layout_v<T>, "T must be standard layout");
        //static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    
        //sockaddr_in sender{};
        //socklen_t senderLen = sizeof(sender);
    
        //return recvfrom(sockfd_,
                        //&buffer,
                        //sizeof(T),
                        //0,
                        //reinterpret_cast<sockaddr*>(&sender),
                        //&senderLen);
    //}

};//ending namespace DASPi
