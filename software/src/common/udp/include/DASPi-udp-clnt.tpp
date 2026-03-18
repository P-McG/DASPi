// Client side implementation of UDP client-server model
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <fcntl.h>

#include "DASPi-framepacket.h"

//#define VERBATIUM_COUT

namespace DASPi{

     template<class T>
     ssize_t UDPClnt::SendToServer(T *buffer, const size_t bufferLength ){
         
     static_assert(std::is_standard_layout_v<T>, "T must be standard layout");
     static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");

#ifdef VERBATIUM_COUT
        std::cout << "Send to server" << std::endl;
#endif
        return sendto(sockfd_,
                      buffer,
                      bufferLength,
                      MSG_CONFIRM,
                      (const struct sockaddr*)&srvAddr_,
                      sizeof(srvAddr_));
    }
    
    template<class T>
    ssize_t UDPClnt::ReceiveFromServer(T &buffer){
#ifdef VERBATIUM_COUT
//    std::cout << "Receive from server" << std::endl;
#endif
    
        static_assert(std::is_standard_layout_v<T>, "T must be standard layout");
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    
        sockaddr_in sender{};
        socklen_t senderLen = sizeof(sender);
    
        return recvfrom(sockfd_,
                        &buffer,
                        sizeof(T),
                        0,
                        reinterpret_cast<sockaddr*>(&sender),
                        &senderLen);
    }

};//ending namespace DASPi
