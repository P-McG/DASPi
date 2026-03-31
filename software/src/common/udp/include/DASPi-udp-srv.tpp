// Server side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "DASPi-frameheader.h"

using namespace DASPi;


//template< typename T0>
//FramePacket UDPSrv::CreateFramePacket(const T0 &buffer){
//#ifdef VERBATIUM_COUT
    //std::cout << "CreateFramePacket" << std::endl;
//#endif

    ////// Setup FramePacket
    ////FramePacket framePacket;
    ////framePacket.payload_.resize(buffer.size());
    ////std::memcpy(framePacket.payload.data(), reinterpret_cast<const char*>(buffer.data()), buffer.size());

	////// Fill FrameHeader
	////framePacket.header.magic_ = htonl(MAGIC_NUMBER);
	
	////uint32_t payloadSize = static_cast<uint32_t>(framePacket.payload.size());
	////framePacket.header.payloadSize_ = htonl(payloadSize);
	
	////uint32_t checksum = SimpleChecksum(framePacket.payload.data(), payloadSize);
	////framePacket.header.checksum_ = htonl(checksum);
	
	////std::cout << "finish filling header" << std::endl;
	
	
    ////return framePacket;
	
	//FramePacket framePacket;

	//framePacket.payload_ = std::move(buffer);  // move, not copy
	
	//// Fill FrameHeader
	//framePacket.header.magic_ = htonl(MAGIC_NUMBER);
	
	//uint32_t payloadSize = static_cast<uint32_t>(framePacket.payload_.size());
	//framePacket.header.payloadSize_ = htonl(payloadSize);
	
	//uint32_t checksum = SimpleChecksum(framePacket.payload_.data(), payloadSize);
	//framePacket.header.checksum_ = htonl(checksum);
	
	//#ifdef VERBATIUM_COUT
	//std::cout << "finish filling header" << std::endl;
	//#endif
	
	//return framePacket;
//}

//template<typename T0>
//FramePacket UDPSrv::CreateFramePacket(T0&& buffer) {
    //FramePacket framePacket;

    //if constexpr (std::is_rvalue_reference_v<decltype(buffer)>) {
        //framePacket.payload_ = std::move(buffer);
    //} else {
        //framePacket.payload_.resize(buffer.size());
        //std::memcpy(framePacket.payload_.data(), reinterpret_cast<const char*>(buffer.data()), buffer.size());
    //}

    //// Fill header
    //framePacket.header.magic_ = htonl(MAGIC_NUMBER);
    //uint32_t payloadSize = static_cast<uint32_t>(framePacket.payload_.size());
    //framePacket.header.payloadSize_ = htonl(payloadSize);
    //uint32_t checksum = SimpleChecksum(framePacket.payload_.data(), payloadSize);
    //framePacket.header.checksum_ = htonl(checksum);

//#ifdef VERBATIUM_COUT
    //std::cout << "finish filling header" << std::endl;
//#endif

    //return framePacket;
//}


template <typename T>
ssize_t UDPSrv::SendRawDatagramToClient(const T* buffer, const size_t byteCount)
{
#ifdef VERBATIUM_COUT
    std::cout << "SendRawDatagramToClient" << std::endl;
    char ipstr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(cliaddr_.sin_addr), ipstr, sizeof(ipstr));
    std::cout << "Sending to " << ipstr << ":" << ntohs(cliaddr_.sin_port) << std::endl;
#endif

    socklen_t len{sizeof(cliaddr_)};
    return sendto(sockfd_,
                  reinterpret_cast<const char*>(buffer),
                  byteCount,
                  MSG_CONFIRM,
                  reinterpret_cast<const struct sockaddr*>(&cliaddr_),
                  len);
}



//template <typename T >
//ssize_t UDPSrv::SendUDPPacketToClient(const T *buffer, const size_t bufferLength){
//#ifdef VERBATIUM_COUT
    //std::cout << "SendUDPPacketToClient" << std::endl;
	//char ipstr[INET_ADDRSTRLEN];
	//inet_ntop(AF_INET, &(cliaddr_.sin_addr), ipstr, sizeof(ipstr));
	//std::cout << "Sending to " << ipstr << ":" << ntohs(cliaddr_.sin_port) << std::endl;
	
//#endif

	//socklen_t len{sizeof(cliaddr_)};
	//ssize_t sendN = sendto(sockfd_, (const char *)buffer, bufferLength,
		//MSG_CONFIRM, (const struct sockaddr *) &cliaddr_,
			//len);
	   //return sendN;
//}

template<class T>
bool UDPSrv::Receive(T& msg)
{
    static_assert(std::is_standard_layout_v<T>);
    static_assert(std::is_trivially_copyable_v<T>);

    sockaddr_in sender{};
    socklen_t len = sizeof(sender);

    const ssize_t received = recvfrom(
        sockfd_,
        &msg,
        sizeof(T),
        0,
        reinterpret_cast<sockaddr*>(&sender),
        &len
    );

    if (received < 0) {
        perror("recvfrom failed");
        return false;
    }

    if (received != static_cast<ssize_t>(sizeof(T))) {
        std::cerr << "Receive: wrong size, got " << received
                  << ", expected " << sizeof(T) << std::endl;
        return false;
    }

    return true;
}
