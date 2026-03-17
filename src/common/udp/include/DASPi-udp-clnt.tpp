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

     //template<class T>
     //ssize_t UDPClnt::SendToServer(T *buffer, const size_t bufferLength ){
//#ifdef VERBATIUM_COUT
        //std::cout << "Send to server" << std::endl;
//#endif
        //ssize_t sendN = sendto(sockfd_, buffer, bufferLength,
            //MSG_CONFIRM, (const struct sockaddr *) &srvAddr_,
            //sizeof(srvAddr_));
        //return sendN;
    //}
    
    template<class T>
    ssize_t UDPClnt::ReceiveFromServer(T &buffer){
#ifdef VERBATIUM_COUT
//    std::cout << "Receive from server" << std::endl;
#endif
   // connect to server 
        if(connect(sockfd_, (struct sockaddr *)&srvAddr_, sizeof(srvAddr_)) < 0) 
        { 
            printf("\n Error : Connect Failed \n"); 
            exit(0); 
        } 
    
        socklen_t len{sizeof(srvAddr_)};
        ssize_t recvN = recvfrom(sockfd_, (char *)buffer.data(), buffer.size()*sizeof(char),
                MSG_WAITALL, (struct sockaddr *) &srvAddr_,
                &len);
        return recvN;
    }

};//ending namespace DASPi
