#pragma once

#include <array>
#include <memory>
#include <vector>
#include <fstream>
#include <mutex>
#include <span>
#include <cmath>
#include <numbers>

#include "DASPi-shape-config.h"
#include "DASPi-udp-clnt.h"
#include "DASPi-messages.h"
#include "DASPi-overlapshapefunction.h"
#include "DASPi-shapefunctiondatapacket.h"
#include "DASPi-fps-counter.h"


namespace DASPi {

template<size_t n>
class AperturePeer {
    
    using sf_t   = DASPi::sf_t<n>;
    using sfdp_t = DASPi::sfdp_t<n>;

    static constexpr size_t n_ = n;

    std::array<std::vector<uint16_t>, n+1> buffer_;
    std::array<std::unique_ptr<std::ofstream>, n+1> files_;

    sf_t sf_;
    sfdp_t sfdp_;

    // 🔥 Dual sockets
    UDPClnt frameClnt_;
    UDPClnt controlClnt_;

    // 🔒 Shared state (future-safe)
    std::mutex gainMutex_;
    float latestRGainApply_{1.0f};
    float latestBGainApply_{1.0f};
    uint32_t latestFrameId_{0};
    bool gainValid_{false};
    
    FPSCounter fpsReceived_{"computemodule received"};
    FPSCounter fpsPublished_{"computemodule published"};
    FPSCounter fpsRunFrameLoop_{"computemodule total"};
    
public:
    AperturePeer(in_addr_t clntAddr,
                 int framePort,
                 int controlPort,
                 in_addr_t srvAddr);

    bool RunFrameLoop();
    bool RunControlLoop();
    
    std::string inAddrTToString(in_addr_t ip);
    bool BufferToFile();
    int GetFramePort();
    int GetControlPort();
    std::vector<uint16_t> GenerateStripedBayerBGGRImage(int width, int height, int stripeWidth);
	inline void ApplyWhiteBalanceToMosaic_BGGR(std::span<uint16_t> data, int width, int height, double rGain, double gGain, double bGain);
	void BrightenImageInplace(std::span<uint16_t> buf, size_t shift);
    void BrightenImageChunked2(uint16_t* buffer, size_t start, size_t end, size_t shift);
#if defined(__ARM_NEON) || defined(__aarch64__)
    void BrightenImageChunked2_NEON(uint16_t* buffer, size_t start, size_t end, size_t shift);
#endif
    bool ReceiveApertureCapture();
    bool CopyBuffer(size_t index, std::vector<uint16_t>& out) const;
    //template<size_t N>bool receivePeerFrame(AperturePeer<N>& peer, std::vector<std::uint16_t>& outBayer);
    bool StitchWithPeer(AperturePeer<n>& other,
                        size_t sharedSideThis,
                        size_t sharedSideOther,
                        std::vector<uint16_t>& out,
                        bool reverseOther = false);
    cv::Mat BuildValidMask(size_t regionIndex);
    bool CopyValidMask(size_t regionIndex, cv::Mat& out);
    
private:
    const int processingThreads_{4};

    void HandleGainMsg(const GainMsg& msg);
    bool SendGainReply(const GainReply& reply);
    float ComputeRequestedGain(const GainMsg& msg);
    mutable std::mutex bufferMutex_;
};

} // namespace DASPi

#include "DASPi-aperture-peer.tpp"
