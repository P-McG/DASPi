#pragma once

#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <sys/mman.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <array>
#include <vector>
#include <cstdlib>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>

#include "DASPi-shape-config.h"
#include "event_loop.h"
#include "DASPi-logger.h"
#include "DASPi-udp-srv.h"
#include "DASPi-udp-clnt.h"
#include "DASPi-udpframe.h"
#include "DASPi-topologydatapacket.h"
#include "DASPi-overlaptopology.h"
#include "DASPi-postprocessitem.h"
#include "DASPi-fps-counter.h"
#include "DASPi-module-spherical-map.h"

namespace DASPi{

               
    template<unsigned int FacetIndex, std::size_t ModuleIndex>
    class Aperture {
        using tpgy_t   = DASPi::tpgy_t;
        using tpgydp_t = DASPi::tpgydp_t<FacetIndex>;

        using SphereSpaceType = typename tpgy_t::Space_t;
        using ModuleSphericalMapType = ModuleSphericalMap<SphereSpaceType, ModuleIndex, FacetIndex>;  
        using FacetTopologyType = typename tpgy_t::template FacetTopology_t<FacetIndex>;
        using OverlapTopologyType = typename tpgy_t::template OverlapTopology_t<FacetIndex>;
        using NonOverlapFacetTopologyType = typename OverlapTopologyType::NonOverlapFacetTopology_t;
        using GlobalLinearTopologyType = typename OverlapTopologyType::GlobalLinearTopology_t; 

        //Check DASPi-config.h if triggered
        static_assert(
            NUM_REGIONS == tpgydp_t::NumberOfRegions(),
            "NUM_REGIONS must match TopologyDataPacket regions"
        );
        
        tpgy_t tpgy_;
        tpgydp_t tpgydp_;
        
        static constexpr unsigned int facetIndex_ = FacetIndex;
        static constexpr std::size_t moduleIndex_ = ModuleIndex;
                
        ModuleSphericalMapType sphericalMap_;  
        static constexpr std::uint16_t kBlendWeightOneQ12_{4096};
        std::array< std::vector<std::uint16_t>, tpgydp_t::NumberOfRegions() > blendWeightsQ12_;
        bool blendWeightsReady_{false};

        const int processingThreads_{4};
        const int chunkThreads_{4};
        const int numBuffers_{4};
        std::atomic<uint64_t> globalFrameCounter_ = 0;
        std::string boardSerial_;
        uint32_t cameraId_;//<-need to come from the camera
        std::string cameraUniqueId_;
        GainMsg gainMsg_;
    
        inline static std::unique_ptr<libcamera::CameraManager> cm_;
        inline static std::shared_ptr<libcamera::Camera> camera_;
        std::unique_ptr<libcamera::CameraConfiguration> config_;
        std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
        inline static std::vector<std::unique_ptr<libcamera::Request>> requests_;
        inline static int timeout_{600};//seconds
        std::atomic<size_t> pendingRequests_{0};
        inline static EventLoop& loop_ = GetEventLoop();
        inline static std::unique_ptr<std::ofstream> outputFile_;
        inline static unsigned int frameNumber_{0};
        //constexpr inline static unsigned int width_{1536};
        //constexpr inline static unsigned int height_{864};
        libcamera::PixelFormat codec_{libcamera::formats::SRGGB16};
        inline static std::atomic<bool> isStartCaptureThreadDone_{false}; // Atomic flag
        inline static std::atomic<bool> isStartCaptureRunning_{false}; // Atomic flag
        tpgy_t sf_;
        std::chrono::time_point<std::chrono::high_resolution_clock> fpsTimer_;
        
        // Thread pool
        std::vector<std::thread> workerThreads_;
        std::queue<libcamera::Request*> requestQueue_;
        std::mutex queueMutex_;
        std::condition_variable queueCondVar_;
        std::atomic<bool> stopWorkers_{false};
        std::unordered_map<libcamera::Request*, libcamera::FrameBuffer*> bufferMap_;
        
        std::mutex udpQueueMutex_;
        std::condition_variable udpQueueCV_;
        std::thread udpSenderThread_;
        bool udpSending_ = false;
        
        // Thread coordination for start/finish
        std::mutex mutex_;
        std::condition_variable condition_;
        
        // Post-processing queue
        std::queue<PostProcessItem> postProcessingQueue_;
        std::mutex postProcessingQueueMutex_;
        std::condition_variable postProcessingQueueCV_;
        
        std::vector<std::thread> postProcessingThreads_;
        
        bool running_ = true;
        bool postProcessingRunning_ = false;
        std::atomic<bool> stopPostProcessing_{false};

        static constexpr std::size_t n_ = tpgy_t::verticesPerFaceN_;
        static constexpr std::size_t regionCount_ = tpgydp_t::NumberOfRegions();

        static_assert(regionCount_ == n_ + 1);
        static_assert(NUM_REGIONS == regionCount_);
      
    public:
        UDPSrv frameSrv_;
        UDPClnt controlClnt_;
        
        FPSCounter fpsSent_{"aperturecomputemodule sent"};
    
    public:
        Aperture(
            const std::string clientIp,
            const size_t port,
            const std::string cameraCalibrationPath = {},
            const std::string cameraIntrinsicsPrefix = {}
        );
        ~Aperture();

        auto OverlapTopologyRef() -> OverlapTopologyType&;
        auto OverlapTopologyRef() const -> const OverlapTopologyType&;
        auto NonOverlapTopologyRef() -> NonOverlapFacetTopologyType&;
        auto NonOverlapTopologyRef() const -> const NonOverlapFacetTopologyType&;

        //template<size_t n> OverlapTopologyType& OverlapTopologyRef();
        //template<size_t n> const OverlapTopologyType& OverlapTopologyRef() const;
        //template<size_t n> NonOverlapFacetTopologyType& NonOverlapTopologyRef();
        //template<size_t n> const NonOverlapFacetTopologyType& NonOverlapTopologyRef() const;
        void PrintThreadInfo();
        void CleanupCamera();
        void CreateCameraManager();
        void AquireCamera(unsigned int index = 0);
        void Stream();
        void CameraConfiguration();
        void StreamConfiguration(unsigned int index = 0);
        void StreamConfigurationValidation(unsigned int index = 0);
        void StreamConfigurationSet();
        void BufferAllocation();
        std::unique_ptr<libcamera::Request> CreateRequestWithControls();
        void FrameCapture(unsigned int index = 0);
        void SignalAndSlots();
        void StartCapture();
        void RunEventLoop(int timeout);
        void QueueCameraRequests(bool isRequests = true);
        void StartRequestProcessingThreads(int numThreads);
        void StartPostProcessingThreads(int numThreads); 
        void StopRequestProcessingThreads();
        void ProcessRequestImpl(libcamera::Request *request);
        void PostProcessingThread();
        void RequestProcessingThread();
        void ProcessRequest(libcamera::Request *request);
        void RequestComplete(libcamera::Request *request);
        void RequestReuse(libcamera::Request::ReuseFlag Flags = libcamera::Request::ReuseFlag::ReuseBuffers);
        std::string CameraName(libcamera::Camera *camera);
        template<typename input_t>
            void FrameBufferTransformation(input_t&& input, const GainMsg &gainMsg, tpgydp_t& output, const size_t numThreads);
        
        void FrameBufferToUDP(const uint64_t frameNumber,
                      const libcamera::FrameBuffer *buffer,
                      const GainMsg& gainMsg);
        void* StartCaptureThread(void* arg);
        bool ContinuousCapture(unsigned int index=0);
        void CaptureLoop(unsigned int index);
        void FreeStream(unsigned int indexCount=1);
        void neon_copy_and_brighten_u16(uint16_t* dest, const uint16_t* src, size_t count);
        void StartUDPSender();
        void StopUDPSender();
    
        void StartPostProcessingThreads();
        void StopPostProcessingThreads();
        inline void ApplyWhiteBalanceToMosaic_BGGR(size_t region, std::span<uint16_t> data, const GainMsg& gainMsg);
        std::string GetBoardSerial();
        std::string ExtractCameraBusAddr(const std::string& libcameraId);
        uint32_t ParseBusAndAddressCameraId(const std::string& libcameraId);
        void SendSphereMap();
        


    private:
        std::thread controlThread_;
        std::string cameraCalibrationPath_;    
        std::string cameraIntrinsicsPrefix_;
            
        mutable std::mutex gainMutex_;
        float latestBrightnessGainApply_{1.0f};
        float latestRGainApply_{1.0f};
        float latestBGainApply_{1.0f};
        uint32_t latestGainFrameId_{0};
        bool gainValid_{false};

        
        void ApplyCameraCalibrationBeforeSphereMap();
        void BuildBlendWeightMapsQ12();
        void ApplyBlendWeightsQ12(tpgydp_t& output);        
        bool ReceiveGainReply();
        void HandleGainReply(const GainReply& reply);
        bool RunControlLoop();
        
    };

};//end namespace DASPi
#include "DASPi-aperture.tpp"
