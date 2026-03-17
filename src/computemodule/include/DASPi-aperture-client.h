#pragma once

#include <chrono>
#include <thread>
#include <execution>
#include "DASPi-udp-clnt.h"
#include "DASPi-overlapshapefunction.h"
#include "DASPi-shapefunctiondatapacket.h"

//#define VERBATIUM_COUT

namespace DASPi{

template<size_t n>
class ApertureClient{
    
        using sf_t = OverlapShapeFunction<
                    n,
                    { static_cast<size_t>(0.5*sensorWidthValue_), 
                      static_cast<size_t>(0.5*sensorHeightValue_)
                    },
                    { -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)), 
                      -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
                    },
                    0.999//0.75
                >;
        using sfdp_t = ShapeFunctionDataPacket<
                    n,
                    { static_cast<size_t>(0.5*sensorWidthValue_), 
                      static_cast<size_t>(0.5*sensorHeightValue_)
                    },
                    { -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)),
                      -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
                    },
                    0.999//0.75
                >;
    
        const int processingThreads_{4};
        const int chunkThreads_{4};
        static constexpr size_t n_ = n;
        //static constexpr unsigned int width_{1536};
        //static constexpr unsigned int height_{864};
        std::array<std::vector<uint16_t>, n+1> buffer_;
        std::array<std::unique_ptr<std::ofstream>, n+1 > files_;
        sf_t sf_;
        sfdp_t sfdp_;
        UDPClnt udpClnt_;

    public:
        ApertureClient(const in_addr_t clntAddr, const int port, const in_addr_t servaddr);
        ~ApertureClient();
        
        std::string inAddrTToString(in_addr_t ip);
        
        bool BufferToFile();        
        bool ReceiveApertureCapture();
        int GetPort();
        
        std::vector<uint16_t> GenerateStripedBayerBGGRImage(int width, int height, int stripeWidth);
        inline void ApplyWhiteBalanceToMosaic_BGGR(
            std::span<uint16_t> data,
            int width, int height,
            double rGain, double gGain, double bGain
        );
        //void BrightenImageInplace(std::span<uint8_t> buf, size_t shift);
        void BrightenImageInplace(std::span<uint16_t> buf, size_t shift);
        //auto BrightenImage(std::vector<uint8_t> &&input);
        void BrightenImageChunked2_NEON(uint16_t *buffer, size_t start, size_t end, size_t shift) ;
        
    };
}//end namespace DASPi
#include "DASPi-aperture-client.tpp"
