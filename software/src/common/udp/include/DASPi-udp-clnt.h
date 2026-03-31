#pragma once
// Client side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <array>
#include <sys/epoll.h>
#include <span>
#include <vector>
#include <array>
#include <unordered_map>
#include "DASPi-rx-frame-assembly.h"
#include "DASPi-frameheader.h"
#include "DASPi-udp-chunk-header.h"


struct sockaddr_in;

namespace DASPi{
	
	class UDPClnt{
		public:
		
		//Event Polling Data
		struct EpollData{
			int epoll_fd_;
		    int maxEvents_;
			std::unique_ptr<epoll_event[]> events_;

		};
		
		const size_t maxUdpPayloadBytes_{1400};
	    sockaddr_in clntAddr_;
		const int port_;
	
		int sockfd_{0};
		sockaddr_in srvAddr_;
		
		std::unordered_map<uint32_t, RxFrameAssembly> rxAssemblies_;
		
	    public:
	    //sockaddr_in clntAddr_ = { INADDR_NONE, 0, { INADDR_NONE } };

	    UDPClnt():UDPClnt(INADDR_NONE, 0000, INADDR_NONE){};
		UDPClnt(const in_addr_t clntAddr, const int port, const in_addr_t srvAddr);
		~UDPClnt();
		
		size_t GetMaximumTransmittableUnits();
		int CreatingSocketFileDescriptor();
		std::string GetHostIp();
		sockaddr_in FillingClientInformation(const in_addr_t clntAddr, const int port );
	    void FillingServerInformation(const in_addr_t &addr);
		    
		template<class T>
		ssize_t SendToServer(T *buffer, const size_t bufferLength );
		
		template<class T>
		ssize_t ReceiveFromServer(T &buffer);
		
		bool WaitForValidHeader(FrameHeader &headerOut, sockaddr_in &senderOut);
		
		bool ReceiveAndReassembleFramePacket(std::vector<uint16_t> &outPayload, FrameHeader &outHeader);
		//uint32_t SimpleChecksum(const uint16_t* data, size_t size);
		uint32_t SimpleChecksum(std::span<const std::byte> bytes);
		bool InitEpollForSrvUDPPackets(/*std::vector<char> &buffer,*/ EpollData &epollData);
		bool FinalizeEpollForSrvUDPPackets(const EpollData &epollData);
		bool ReadDataFromSrvUDPPackets(const EpollData &epollData, std::vector<uint16_t> &buffer);
		
		bool isEqual(const sockaddr_in &a, const sockaddr_in &b);
		void PrintSockaddr_in(const struct sockaddr_in *addr);
		int BindSocketWithClientAddress();
		void FlushSocket();
		int SetNonBlocking(bool enabled);
		bool SendFramePackets(const std::vector<std::vector<uint8_t>>& packets);
		std::vector<std::vector<uint8_t>> BuildPackets(const FrameHeader& header, const uint8_t* data, size_t bytes, size_t mtu);
	private:
private:
    static float HostToWireFloat(float v);
    static float WireToHostFloat(float v);

    static MessageHeader ToWireMessageHeader(MessageHeader h);
    static MessageHeader FromWireMessageHeader(MessageHeader h);

    static GainMsg ToWireGainMsg(GainMsg g);
    static GainMsg FromWireGainMsg(GainMsg g);

    static FrameHeader ToWireFrameHeader(FrameHeader h);
    static FrameHeader FromWireFrameHeader(FrameHeader h);
	};
};//ending namespace DASPi
#include "DASPi-udp-clnt.tpp"
