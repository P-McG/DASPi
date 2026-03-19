#pragma once
// Server side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <mutex>
#include "DASPi-framepacket.h"

//#define VERBATIUM_COUT

struct sockaddr_in;

namespace DASPi{

    class UDPSrv{
    public:
        inline static size_t fnCallCount{0};
    //private:
        inline static size_t maximumTransmittableUnits_{1024};
        inline static std::mutex maximumTransmittableUnitsMtx_;
        const sockaddr_in cliaddr_;
        std::mutex cliaddrMtx_;
        const size_t port_;
        int sockfd_ {0};
        sockaddr_in servaddr_;
        std::mutex servaddrMtx_;
    
        ssize_t recvN_ {0};
        ssize_t sendN_ {0};
            
        //std::map<uint64_t, std::vector<uint8_t>> orderedFrames_; // frameNumber -> payload
        std::map<uint64_t, FramePacket> orderedFrames_; // frameNumber -> framePacket
        std::mutex outputMutex_;
        bool nextFrameInitialized_ = false;
        uint64_t nextFrameToSend_ = std::numeric_limits<uint64_t>::max();
    
    
      public:
        UDPSrv(const std::string& clientIp, const size_t port);
        ~UDPSrv();
    
        size_t GetMaximumTransmittableUnits();
        static sockaddr_in InitSockaddrFromIP(const std::string& clientIp, const size_t port);
        const sockaddr_in GetServerAddress();
        void SetServerAddress(const sockaddr_in addr);
        const sockaddr_in GetClientAddress();
        int CreatingSocketFileDescriptor();
        sockaddr_in FillingServerInformation();
        sockaddr_in FillingClientInformation(const in_addr_t cli);
        int BindSocketWithServerAddress();
        std::string get_host_ip();
        void printSockAddr(const sockaddr_in &addr);
        //uint32_t SimpleChecksum(const uint8_t* data, size_t size);
        uint32_t SimpleChecksum(std::span<const std::byte> bytes);
        void SendFramePacketToClient(const FramePacket &framePacket);
        void TransmitFrame(const DASPi::FramePacket&&);
//        void SubmitFrameOutput(uint64_t frameNumber, std::vector<uint8_t>&& payload);
        void SubmitFrameOutput(uint64_t, FramePacket&&);
        bool TrySendFramesInOrder();
        void SendToServer(const uint8_t* buffer, size_t bufferLength);
        FramePacket CreateFramePacket(const GainMsg &gainMsg, std::vector<uint16_t>&& buffer);
        //template<typename T0>
        //FramePacket CreateFramePacket(T0&& buffer);
        
        //FramePacket CreateFramePacket(std::vector<uint8_t>&& buffer);
        //void FrameBufferToUDP(std::vector<uint8_t> &&buffer);
    
        template <typename T > ssize_t SendUDPPacketToClient(const T *buffer, const size_t bufferLength=maximumTransmittableUnits_ );
        template<class T> bool Receive(T& msg);
    };
}//end namespace DASPi

#include "DASPi-udp-srv.tpp"
