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

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>

#include "event_loop.h"
#include "DASPi-logger.h"
#include "DASPi-udp-srv.h"
#include "DASPi-udpframe.h"
#include "DASPi-shapefunctiondatapacket.h"
#include "DASPi-overlapshapefunction.h"
#include "DASPi-postprocessitem.h"

namespace DASPi{
    class Aperture {
        
        using sf_t = OverlapShapeFunction<
                            3,
                            { static_cast<size_t>(0.5*sensorWidthValue_), 
                              static_cast<size_t>(0.5*sensorHeightValue_)
                            },
                            { -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)),
                              -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
                            },
                            0.75
                        >;
        using sfdp_t = ShapeFunctionDataPacket<
                    3,
                    { static_cast<size_t>(0.5*sensorWidthValue_), 
                      static_cast<size_t>(0.5*sensorHeightValue_)
                    },
                    { -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*sin(2.0 * std::numbers::pi * 0.0 / 3)), 
                      -1 * static_cast<long>((1.0/2.0)*sensorHeightValue_*cos(2.0 * std::numbers::pi * 0.0 / 3))
                    },
                    0.75
                >;
    
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
        inline static EventLoop loop_;
        inline static std::unique_ptr<std::ofstream> outputFile_;
        inline static unsigned int frameNumber_{0};
        //constexpr inline static unsigned int width_{1536};
        //constexpr inline static unsigned int height_{864};
        libcamera::PixelFormat codec_{libcamera::formats::SBGGR10};
        inline static std::atomic<bool> isStartCaptureThreadDone_{false}; // Atomic flag
        inline static std::atomic<bool> isStartCaptureRunning_{false}; // Atomic flag
        sf_t sf_;
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
        
        
        static constexpr size_t n_ = sf_t::n_;
        static constexpr double nonOverlapScale_ = sf_t::nonOverlapScale_;
      
    public:
        UDPSrv udpSrv_;
    
    public:
        Aperture(const std::string clientIp, const size_t port);
        ~Aperture();
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
        //template<typename input_t>
            //void constexpr FrameBufferTransformation(input_t&& input, std::array<std::vector<uint16_t>, n_ + 1>& output, const size_t numThreads);
        template<typename input_t>
            void constexpr FrameBufferTransformation(input_t&& input, const GainMsg &gainMsg, sfdp_t& output, const size_t numThreads);
        //template<typename T0> void constexpr FrameBufferTransformation2(T0&& input, std::vector<uint16_t>& output, const size_t numThreads);
        
        void FrameBufferToUDP(const uint64_t frameNumber, const libcamera::FrameBuffer *buffer);
        void* StartCaptureThread(void* arg);
        bool ContinuousCapture(unsigned int index=0);
        void CaptureLoop(unsigned int index);
        void FreeStream(unsigned int indexCount=1);
        void neon_copy_and_brighten_u16(uint16_t* dest, const uint16_t* src, size_t count);
        void StartUDPSender();
        void StopUDPSender();
    
        void StartPostProcessingThreads();
        void StopPostProcessingThreads();
        inline void ApplyWhiteBalanceToMosaic_BGGR(size_t region, const sf_t &sf, std::span<uint16_t> data, const GainMsg& gainMsg);
        std::string GetBoardSerial();
        //uint32_t ParseCameraAddressId(const std::string& libcameraId);
        std::string ExtractCameraBusAddr(const std::string& libcameraId);
        uint32_t ParseBusAndAddressCameraId(const std::string& libcameraId);

        
    };

};//end namespace DASPi
#include "DASPi-aperture.tpp"
