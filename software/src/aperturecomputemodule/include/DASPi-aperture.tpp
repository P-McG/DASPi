#include <omp.h> // Include OpenMP
#include "scopedtimer.h"
#include <thread>
#include <pthread.h>
#include <atomic>
#include <span>
#include <vector>
#include <arm_neon.h>
#include <sys/mman.h>
#include <chrono>
#include <iostream>
#include <sched.h>      // for sched_getcpu()
#include <arm_neon.h>
#include <iomanip>
#include <algorithm>

#include <libcamera/controls.h>
#include <libcamera/control_ids.h>  // Required for controls::FrameDurationLimits

#include "DASPi-framepacket.h"
#include "DASPi-udp-srv.h"
#include "DASPi-aperture.h"
#include "DASPi-postprocessitem.h"
#include "scopedtimer.h"

using namespace std::chrono;
using namespace libcamera;

namespace DASPi{


// Constructor
template<size_t n>
Aperture<n>::Aperture(const std::string clientIp, const size_t port)
    : frameSrv_{clientIp, port},
      controlClnt_{INADDR_ANY,
             static_cast<int>(port + 1),
             inet_addr(clientIp.c_str()),
             static_cast<int>(port + 1)}
{
	
	std::cout << "[TX sizeof]\n";
    std::cout << "sizeof(MessageHeader)=" << sizeof(MessageHeader) << '\n';
    std::cout << "sizeof(GainMsg)=" << sizeof(GainMsg) << '\n';
    std::cout << "sizeof(FrameHeader)=" << sizeof(FrameHeader) << '\n';
    std::cout << "sizeof(UdpChunkHeader)=" << sizeof(UdpChunkHeader) << '\n';

	
    log_verbose("[Aperture::Aperture]");
    CreateCameraManager();
    AquireCamera();
    Stream();
    fpsTimer_ = std::chrono::high_resolution_clock::now();
}

// Destructor
/*
 * --------------------------------------------------------------------
 * Clean Up
 * Stop the Camera, release resources and stop the CameraManager.
 * libcamera has now released all resources it owned.
*/
template<size_t n>
Aperture<n>::~Aperture(){
    log_verbose("[Aperture::~Aperture]");
	
    running_ = false;
	if (controlThread_.joinable())
		controlThread_.join();
	
    CleanupCamera();
	StopUDPSender();
	StopPostProcessingThreads();
	StopRequestProcessingThreads();

}

template<size_t n>
void Aperture<n>::PrintThreadInfo() {
    std::cout << " (std::thread ID: " << std::this_thread::get_id() << ")"
              << " is running on CPU " << sched_getcpu() << std::endl;
}

template<size_t n>
void Aperture<n>::CleanupCamera() {
    log_verbose("[Aperture::CleanupCamera]");
    if (camera_->stop() != 0) {
        std::cerr << "Failed to stop the camera." << std::endl;
    }
    
    FreeStream(1);
    requests_.clear();
    if (camera_->release() != 0) {
        std::cerr << "Failed to release the camera." << std::endl;
    }

    camera_.reset();
    std::cout << "Camera fully cleaned up." << std::endl;
    
    //cm_->stop();
}


// Create a Camera Manager.
/*
 * --------------------------------------------------------------------
 *
 * The Camera Manager is responsible for enumerating all the Camera
 * in the system, by associating Pipeline Handlers with media entities
 * registered in the system.
 *
 * The CameraManager provides a list of available Cameras that
 * applications can operate on.
 *
 * When the CameraManager is no longer to be used, it should be deleted.
 * We use a unique_ptr here to manage the lifetime automatically during
 * the scope of this function.
 *
 * There can only be a single CameraManager constructed within any
 * process space.
 */
template<size_t n>
void Aperture<n>::CreateCameraManager(){

	log_verbose("[Aperture::CreateCameraManager]");
	 
	cm_ = std::make_unique<libcamera::CameraManager>();
	cm_->start();

	/*
	 * Just as a test, generate names of the Cameras registered in the
	 * system, and list them.
	 */
//	for (auto const &camera : cm->cameras())
//		std::cout << " - " << cameraName(camera.get()) << std::endl;
}

// AquireCamera
/*
 * --------------------------------------------------------------------
 *
 * Camera are entities created by pipeline handlers, inspecting the
 * entities registered in the system and reported to applications
 * by the CameraManager.
 *
 * In general terms, a Camera corresponds to a single image source
 * available in the system, such as an image sensor.
 *
 * Application lock usage of Camera by 'acquiring' them.
 * Once done with it, application shall similarly 'release' the Camera.
 *
 * As an example, use the first available camera in the system after
 * making sure that at least one camera is available.
 *
 * Cameras can be obtained by their ID or their index, to demonstrate
 * this, the following code gets the ID of the first camera; then gets
 * the camera associated with that ID (which is of course the same as
 * cm->cameras()[0]).
 */
template<size_t n>
void Aperture<n>::AquireCamera(unsigned int index){

	log_verbose("[Aperture::AquireCamera]");
	if (cm_->cameras().empty()) {
		std::cout << "No cameras were identified on the system."
			  << std::endl;
		cm_->stop();
		exit(EXIT_FAILURE);
	}

	std::string cameraId = cm_->cameras()[index]->id();
	camera_ = cm_->get(cameraId);
	camera_->acquire();
}

// Stream
/*
 *
 * Each Camera supports a variable number of Stream. A Stream is
 * produced by processing data produced by an image source, usually
 * by an ISP.
 *
 *   +-------------------------------------------------------+
 *   | Camera                                                |
 *   |                +-----------+                          |
 *   | +--------+     |           |------> [  Main output  ] |
 *   | | Image  |     |           |                          |
 *   | |        |---->|    ISP    |------> [   Viewfinder  ] |
 *   | | Source |     |           |                          |
 *   | +--------+     |           |------> [ Still Capture ] |
 *   |                +-----------+                          |
 *   +-------------------------------------------------------+
 *
 * The number and capabilities of the Stream in a Camera are
 * a platform dependent property, and it's the pipeline handler
 * implementation that has the responsibility of correctly
 * report them.
 */
template<size_t n>
void Aperture<n>::Stream(){
	log_verbose("[Aperture::Stream]");
         CameraConfiguration();
         StreamConfiguration();
         StreamConfigurationValidation();
         StreamConfigurationSet();
}

//Camera Configuration.
/*
 * --------------------------------------------------------------------
 *
 * Camera configuration is tricky! It boils down to assign resources
 * of the system (such as DMA engines, scalers, format converters) to
 * the different image streams an application has requested.
 *
 * Depending on the system characteristics, some combinations of
 * sizes, formats and stream usages might or might not be possible.
 *
 * A Camera produces a CameraConfigration based on a set of intended
 * roles for each Stream the application requires.
 */
template<size_t n>
void Aperture<n>::CameraConfiguration() {
    log_verbose("[Aperture::CameraConfiguration]");

    config_ = camera_->generateConfiguration({ libcamera::StreamRole::Raw });
    if (!config_ || config_->empty()) {
        throw std::runtime_error("camera_->generateConfiguration returned null or empty");
    }

    auto &cfg = config_->at(0);

    // Force requested raw format here
    cfg.pixelFormat = libcamera::formats::SBGGR16;
    cfg.size = {1456, 1088};

    std::cout << "[RequestedConfig] pixelFormat="
              << cfg.pixelFormat.toString()
              << " width=" << cfg.size.width
              << " height=" << cfg.size.height
              << std::endl;

    const libcamera::CameraConfiguration::Status status = config_->validate();

    std::cout << "[ValidatedConfig] pixelFormat="
              << config_->at(0).pixelFormat.toString()
              << " width=" << config_->at(0).size.width
              << " height=" << config_->at(0).size.height
              << " stride=" << config_->at(0).stride
              << " frameSize=" << config_->at(0).frameSize
              << " status=" << static_cast<int>(status)
              << std::endl;

    boardSerial_ = GetBoardSerial();

    const std::string libcameraId = camera_->id();
    const std::string cameraBusAddr = ExtractCameraBusAddr(libcameraId);
    cameraId_ = ParseBusAndAddressCameraId(libcameraId);
    cameraUniqueId_ = boardSerial_ + "_" + cameraBusAddr;

    std::cout << "Board serial : " << boardSerial_ << "\n";
    std::cout << "Camera ID    : " << cameraId_ << "\n";
    std::cout << "Unique ID    : " << cameraUniqueId_ << "\n";
}

template<size_t n>
void Aperture<n>::StreamConfiguration(unsigned int index) {
    log_verbose("[Aperture::StreamConfiguration] enter");
    std::cout << "[StreamConfiguration] index=" << index << std::endl;

    if (!config_) {
        throw std::runtime_error("config_ is null in StreamConfiguration");
    }
    if (index >= config_->size()) {
        throw std::runtime_error("StreamConfiguration index out of range");
    }

    libcamera::StreamConfiguration &streamConfig = config_->at(index);
    streamConfig.size.width = sensorWidthValue_;
    streamConfig.size.height = sensorHeightValue_;
    streamConfig.pixelFormat = codec_;
    streamConfig.bufferCount = numBuffers_;
}


// StreamConfigurationValidation
/*
 * The CameraConfiguration contains a StreamConfiguration instance
 * for each StreamRole requested by the application, provided
 * the Camera can support all of them.
 *
 * Each StreamConfiguration has default size and format, assigned
 * by the Camera depending on the Role the application has requested.
 */
template<size_t n>
void Aperture<n>::StreamConfigurationValidation(unsigned int index){
    log_verbose("[Aperture::StreamConfigurationValidation]");
	/*
	 * Each StreamConfiguration parameter which is part of a
	 * CameraConfiguration can be independently modified by the
	 * application.
	 *
	 * In order to validate the modified parameter, the CameraConfiguration
	 * should be validated -before- the CameraConfiguration gets applied
	 * to the Camera.
	 *
	 * The CameraConfiguration validation process adjusts each
	 * StreamConfiguration to a valid value.
	 */

	/*
	 * The Camera configuration procedure fails with invalid parameters.
	 */
#if 0
	 streamConfig.size.width = sensorWidthValue_;
	 streamConfig.size.height = sensorHeightValue_;
	 streamConfig.pixelFormat = codec_;
	 streamConfig.bufferCount = numBuffers_;

	int ret = camera_->configure(config_.get());
	if (ret) {
		std::cout << "CONFIGURATION FAILED!" << std::endl;
		exit(EXIT_FAILURE);
	}
#endif

	/*
	 * Validating a CameraConfiguration -before- applying it will adjust it
	 * to a valid configuration which is as close as possible to the one
	 * requested.
	 */
	config_->validate();
    
    libcamera::StreamConfiguration &streamConfig = config_->at(index);
    std::cout
        << "[ValidatedConfig] pixelFormat=" << streamConfig.pixelFormat.toString()
        << " width=" << streamConfig.size.width
        << " height=" << streamConfig.size.height
        << " stride=" << streamConfig.stride
        << " frameSize=" << streamConfig.frameSize
        << std::endl;
}

template<size_t n>
void Aperture<n>::StreamConfigurationSet(){
	/*
	 * Once we have a validated configuration, we can apply it to the
	 * Camera.
	 */
	log_verbose("[StreamConfigurationSet]");
	camera_->configure(config_.get());
}

//Buffer Allocation
/*
 * --------------------------------------------------------------------
 *
 * Now that a camera has been configured, it knows all about its
 * Streams sizes and formats. The captured images need to be stored in
 * framebuffers which can either be provided by the application to the
 * library, or allocated in the Camera and exposed to the application
 * by libcamera.
 *
 * An application may decide to allocate framebuffers from elsewhere,
 * for example in memory allocated by the display driver that will
 * render the captured frames. The application will provide them to
 * libcamera by constructing FrameBuffer instances to capture images
 * directly into.
 *
 * Alternatively libcamera can help the application by exporting
 * buffers allocated in the Camera using a FrameBufferAllocator
 * instance and referencing a configured Camera to determine the
 * appropriate buffer size and types to create.
 */
 template<size_t n>
void Aperture<n>::BufferAllocation(){

	log_verbose("[Aperture::BufferAllocation]");
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

	 for (libcamera::StreamConfiguration &cfg : *config_) {
		int ret = allocator_->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			exit(EXIT_FAILURE);
		}
	}
}

// Frame Capture
/*
 * --------------------------------------------------------------------
 *
 * libcamera frames capture model is based on the 'Request' concept.
 * For each frame a Request has to be queued to the Camera.
 *
 * A Request refers to (at least one) Stream for which a Buffer that
 * will be filled with image data shall be added to the Request.
 *
 * A Request is associated with a list of Controls, which are tunable
 * parameters (similar to v4l2_controls) that have to be applied to
 * the image.
 *
 * Once a request completes, all its buffers will contain image data
 * that applications can access and for each of them a list of metadata
 * properties that reports the capture parameters applied to the image.
 */
template<size_t n>
std::unique_ptr<libcamera::Request> Aperture<n>::CreateRequestWithControls(){
	log_verbose("[Aperture::CreateRequestWithControls]");
    auto request = camera_->createRequest();
    if (!request) return nullptr;

	auto &ctrls = request->controls();
    //request->controls().set(libcamera::controls::FrameDurationLimits, {16666, 16666});
	const std::array<int64_t, 2> duration = {33333, 33333}; // min = max = 1/30 s
	ctrls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>(duration));
	//ctrls.set(libcamera::controls::ExposureTime, 20000); // 20 ms
	ctrls.set(libcamera::controls::ExposureTime, 33333); // MAX
	ctrls.set(libcamera::controls::AnalogueGain, 1.5f);  // gain ×1.5; all colors
	ctrls.set(libcamera::controls::AeEnable, false);
	// Change the exposure
	//SetAeCompensationOnRequest(request.get(), +2.0f);

    return request;
}

template<size_t n>
void Aperture<n>::FrameCapture(unsigned int index)
{
    log_verbose("[FrameCapture]");

    if (!camera_) {
        std::cerr << "[FrameCapture] camera_ is null" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (!config_) {
        std::cerr << "[FrameCapture] config_ is null" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (!allocator_) {
        std::cerr << "[FrameCapture] allocator_ is null" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (index >= config_->size()) {
        std::cerr << "[FrameCapture] invalid stream index: " << index
                  << " config_->size()=" << config_->size() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    auto *stream = config_->at(index).stream();
    if (!stream) {
        std::cerr << "[FrameCapture] stream is null for index " << index << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const auto &buffers = allocator_->buffers(stream);
    std::cout << "[FrameCapture] buffers.size()=" << buffers.size() << std::endl;

    if (buffers.empty()) {
        std::cerr << "[FrameCapture] no buffers allocated for stream" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    requests_.clear();
    bufferMap_.clear();
    requests_.reserve(buffers.size());

    for (const auto &buffer : buffers) {
        if (!buffer) {
            std::cerr << "[FrameCapture] encountered null buffer" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::cout << "[FrameCapture] Creating Request..." << std::endl;

        auto request = camera_->createRequest();
        if (!request) {
            std::cerr << "[FrameCapture] createRequest() failed" << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::cout << "[FrameCapture] created request=" << request.get() << std::endl;

        const int addRet = request->addBuffer(stream, buffer.get());
        std::cout << "[FrameCapture] addBuffer req=" << request.get()
                  << " stream=" << stream
                  << " buffer=" << buffer.get()
                  << " ret=" << addRet
                  << std::endl;

        if (addRet < 0) {
            std::cerr << "[FrameCapture] addBuffer failed for request=" << request.get()
                      << " ret=" << addRet << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::cout << "[FrameCapture] request->buffers().size()="
                  << request->buffers().size() << std::endl;

        bufferMap_[request.get()] = buffer.get();
        requests_.push_back(std::move(request));
    }

    std::cout << "[FrameCapture] Stored request pointers in requests_:" << std::endl;
    for (const auto &req : requests_) {
        std::cout << "  Request at: " << req.get()
                  << " buffers=" << req->buffers().size()
                  << std::endl;
    }
}

//Not used
template<size_t n>
void Aperture<n>::QueueCameraRequests(bool isRequests){
	log_verbose("[Aperture::QueueCameraRequests]");
    if(isRequests){
        for (std::unique_ptr<libcamera::Request> &request : requests_){
            camera_->queueRequest(request.get());
        }
    }
}

// RequestComplete
/*
 * --------------------------------------------------------------------
 * For each Camera::requestCompleted Signal emitted from the Camera the
 * connected Slot is invoked.
 *
 * The Slot is invoked in the CameraManager's thread, hence one should avoid
 * any heavy processing here. The processing of the request shall be re-directed
 * to the application's thread instead, so as not to block the CameraManager's
 * thread for large amount of time.
 *
 * The Slot receives the Request as a parameter.
 */
 template<size_t n>
 void Aperture<n>::RequestComplete(libcamera::Request *request){
    std::cout << "[RequestComplete] req=" << request
              << " status=" << static_cast<int>(request->status())
              << " seq=" << request->sequence() << std::endl;
			  
	std::cout << ">>> REQUEST COMPLETE FIRED <<<" << std::endl;
	
    if (request->status() == libcamera::Request::RequestCancelled)
        return;

    ProcessRequestImpl(request);
}
//void Aperture::RequestComplete(libcamera::Request *request){
    //std::cout << "[RequestComplete] req=" << request
              //<< " status=" << static_cast<int>(request->status())
              //<< " seq=" << request->sequence() << std::endl;

    //if (request->status() == libcamera::Request::RequestCancelled)
        //return;

    //loop_.callLater(std::bind([this, request]() { this->ProcessRequest(request); }));
//}

template<size_t n>
void Aperture<n>::StartRequestProcessingThreads(int numThreads) {
	log_verbose("[Aperture::StartRequestProcessingThreads]");
    stopWorkers_ = false;
    for (int i = 0; i < numThreads; ++i) {
        workerThreads_.emplace_back([this, i]() {
			cpu_set_t cpuset;
			CPU_ZERO(&cpuset);
			// Not setting affinity give the highest FPS
			CPU_SET(2, &cpuset);  // Pin to CPU core 2
			CPU_SET(1 + (i % 3), &cpuset);  // Spread across cores 1–3
			//pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
			pthread_setname_np(pthread_self(), "Request");
            while (true) {
                libcamera::Request* req = nullptr;

                {
                    std::unique_lock<std::mutex> lock(queueMutex_);
                    queueCondVar_.wait(lock, [this] {
                        return !requestQueue_.empty() || stopWorkers_;
					});

                    if (stopWorkers_ && requestQueue_.empty())
                        break;

                    req = requestQueue_.front();
                    requestQueue_.pop();
                }
                ProcessRequestImpl(req);
            }
        });
    }
}

template<size_t n>
void Aperture<n>::StopRequestProcessingThreads() {
	log_verbose("[Aperture::StopRequestProcessingThreads]");
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        stopWorkers_ = true;
    }
    queueCondVar_.notify_all(); // wake up all waiting threads

    for (auto &thread : workerThreads_) {
        if (thread.joinable())
            thread.join();
    }

    workerThreads_.clear();
}


template<size_t n>
void Aperture<n>::RequestProcessingThread() {
	log_verbose("[Aperture::RequestProcessingThread]");
    while (!stopWorkers_) {
        libcamera::Request* req = nullptr;

        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCondVar_.wait(lock, [&] { return !requestQueue_.empty() || stopWorkers_; });
            if (stopWorkers_) break;
            req = requestQueue_.front();
            requestQueue_.pop();
        }

        ProcessRequestImpl(req);
    }
}

template<size_t n>
void Aperture<n>::ProcessRequestImpl(libcamera::Request *request)
{
    log_verbose("[Aperture::ProcessRequestImpl]");

    if (!request) {
        std::cerr << "ProcessRequestImpl: null request\n";
        return;
    }
	

    const uint64_t completedFrameId = request->sequence();

    GainMsg gainMsg{};
    gainMsg.camera_id = cameraId_;
    gainMsg.frame_id = completedFrameId;
    gainMsg.r_gain = 1.0f;
    gainMsg.b_gain = 1.0f;
    gainMsg.exposure_us = 0.0f;
    gainMsg.analogue_gain = 1.0f;

    {
        const libcamera::ControlList &meta = request->metadata();

        if (const auto exposure = meta.get(libcamera::controls::ExposureTime))
            gainMsg.exposure_us = static_cast<float>(*exposure);

        if (const auto analogue = meta.get(libcamera::controls::AnalogueGain))
            gainMsg.analogue_gain = *analogue;

        if (const auto colourGains = meta.get(libcamera::controls::ColourGains)) {
            gainMsg.r_gain = (*colourGains)[0];
            gainMsg.b_gain = (*colourGains)[1];
        }
    }

    const auto completedBuffers = request->buffers();

    {
        std::lock_guard<std::mutex> lock(postProcessingQueueMutex_);

        constexpr size_t MAX_QUEUE_SIZE = 256;
        const size_t needed = completedBuffers.size();

		 if (postProcessingQueue_.size() + needed <= MAX_QUEUE_SIZE) {
			for (const auto &[stream, buffer] : completedBuffers) {
				std::cout << "[PostQueuePush] frame=" << completedFrameId
						  << " buffers=" << completedBuffers.size()
						  << " queue_size_before=" << postProcessingQueue_.size()
						  << " buffer=" << buffer
						  << std::endl;
		
				postProcessingQueue_.push(PostProcessItem{
					.frameNumber = completedFrameId,
					.buffer = buffer,
					.gainMsg = gainMsg
				});
			}
			postProcessingQueueCV_.notify_all();
		} else {
			std::cerr << "[ProcessRequestImpl] postProcessingQueue full, dropping frame "
					  << completedFrameId
					  << " queue_size=" << postProcessingQueue_.size()
					  << " needed=" << needed << std::endl;
		}
	}
    request->reuse();

    auto it = bufferMap_.find(request);
    if (it != bufferMap_.end()) {
        request->addBuffer(config_->at(0).stream(), it->second);
    } else {
        std::cerr << "No buffer mapped to request: " << request << '\n';
        std::lock_guard<std::mutex> lock(queueMutex_);
        pendingRequests_--;
        return;
    }

    const int ret = camera_->queueRequest(request);
    if (ret < 0) {
        std::cerr << "Failed to re-queue request for completed frame "
                  << completedFrameId << " (ret=" << ret << ")\n";
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        pendingRequests_--;
    }
}

template<size_t n>
void Aperture<n>::PostProcessingThread()
{
	log_verbose("[Aperture::PostProcessingThread]");
    while (true) {
        PostProcessItem item;

        {
            std::unique_lock<std::mutex> lock(postProcessingQueueMutex_);

            postProcessingQueueCV_.wait(lock, [this] {
                return stopPostProcessing_ || !postProcessingQueue_.empty();
            });

            if (stopPostProcessing_ && postProcessingQueue_.empty()) {
                return;
            }

            item = std::move(postProcessingQueue_.front());
            postProcessingQueue_.pop();
			
			std::cout << "[PostProcessingThread] frame=" << item.frameNumber
					  << " buffer=" << item.buffer << std::endl;			
        }

        FrameBufferToUDP(item.frameNumber, item.buffer, item.gainMsg);
    }
}

template<size_t n>
void Aperture<n>::ProcessRequest(libcamera::Request* request) {
    std::cout << "[ProcessRequest] req=" << request
              << " seq=" << request->sequence() << std::endl;

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        requestQueue_.push(request);
    }
    queueCondVar_.notify_one();
}


// Camera Naming.
/*
 * ----------------------------------------------------------------------------
 *
 * Applications are responsible for deciding how to name cameras, and present
 * that information to the users. Every camera has a unique identifier, though
 * this string is not designed to be friendly for a human reader.
 *
 * To support human consumable names, libcamera provides camera properties
 * that allow an application to determine a naming scheme based on its needs.
 *
 * In this example, we focus on the location property, but also detail the
 * model string for external cameras, as this is more likely to be visible
 * information to the user of an externally connected device.
 *
 * The unique camera ID is appended for informative purposes.
 */
template<size_t n>
std::string Aperture<n>::CameraName(libcamera::Camera *camera)
{
	log_verbose("[Aperture::CameraName]");

	const libcamera::ControlList &props = camera->properties();
	std::string name;

	const auto &location = props.get(libcamera::properties::Location);
	if (location) {
		switch (*location) {
		case libcamera::properties::CameraLocationFront:
			name = "Internal front camera";
			break;
		case libcamera::properties::CameraLocationBack:
			name = "Internal back camera";
			break;
		case libcamera::properties::CameraLocationExternal:
			name = "External camera";
			const auto &model = props.get(libcamera::properties::Model);
			if (model)
				name = " '" + std::string(*model) + "'";
			break;
		}
	}

	name += " (" + camera->id() + ")";

	return name;
}

// Signal&Slots
/*
 * --------------------------------------------------------------------

 *
 * libcamera uses a Signal&Slot based system to connect events to
 * callback operations meant to handle them, inspired by the QT graphic
 * toolkit.
 *
 * Signals are events 'emitted' by a class instance.
 * Slots are callbacks that can be 'connected' to a Signal.
 *
 * A Camera exposes Signals, to report the completion of a Request and
 * the completion of a Buffer part of a Request to support partial
 * Request completions.
 *
 * In order to receive the notification for request completions,
 * applications shall connect a Slot to the Camera 'requestCompleted'
 * Signal before the camera is started.
 */
template<size_t n>
void Aperture<n>::SignalAndSlots(){
	log_verbose("[Aperture::SignalAndSlots]");
	camera_->requestCompleted.connect(
	    this, &DASPi::Aperture<n>::RequestComplete
	);
}

//Start Capture
/*
 * --------------------------------------------------------------------
 *
 * In order to capture frames the Camera has to be started and
 * Request queued to it. Enough Request to fill the Camera pipeline
 * depth have to be queued before the Camera start delivering frames.
 *
 * For each delivered frame, the Slot connected to the
 * Camera::requestCompleted Signal is called.
 */
template<size_t n>
void Aperture<n>::StartCapture() {
    log_verbose("[Aperture::StartCapture]");
    if (camera_->start() != 0) {
        std::cerr << "Camera failed to start" << std::endl;
        return;  // Early exit on failure
    }
      
    isStartCaptureRunning_ = true;

    // Run the event loop, ensure it's blocking or properly handled
    RunEventLoop(timeout_);

    isStartCaptureRunning_ = false;
       
    if (camera_->stop() != 0) {
        std::cerr << "Camera failed to stop" << std::endl;
    }
}


// RunEventLoop
/*
 * --------------------------------------------------------------------
 * Run an EventLoop
 *
 * In order to dispatch events received from the video devices, such
 * as buffer completions, an event loop has to be run.
 */
template<size_t n>
void Aperture<n>::RunEventLoop(int timeout){
	std::cout << "[EventLoop] ENTER" << std::endl;
	log_verbose("[Aperture::RunEventLoop]");

	loop_.timeout(timeout);
	loop_.exec();
	
}

template<size_t n>
void Aperture<n>::FrameBufferToUDP(const uint64_t frameNumber,
                                   const libcamera::FrameBuffer *buffer,
                                   const GainMsg& gainMsg)
{
    log_verbose("[Aperture::FrameBufferToUDP]");

    std::cout << "[FrameBufferToUDP] frame=" << frameNumber
              << " buffer=" << buffer << std::endl;

    static int producedFrames = 0;
    ++producedFrames;
    if ((producedFrames % 10) == 0) {
        std::cout << "[Produced] " << producedFrames
                  << " frames buffered for UDP" << std::endl;
    }

    const auto planes = buffer->planes();
    if (planes.empty()) {
        std::cerr << "Frame buffer has no planes" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const auto &plane = planes.back();
    if (!plane.fd.isValid()) {
        std::cerr << "Invalid file descriptor for plane" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const libcamera::StreamConfiguration &cfg = config_->at(0);
    const libcamera::PixelFormat pixelFormat = cfg.pixelFormat;
    const size_t strideBytes = cfg.stride;

    constexpr size_t activeWidth = sensorWidthValue_;
    constexpr size_t activeHeight = sensorHeightValue_;
    constexpr size_t expectedElems = activeWidth * activeHeight;

    std::cout << "[FrameBufferToUDP] pixelFormat=" << pixelFormat.toString()
              << " plane.length=" << plane.length
              << " strideBytes=" << strideBytes
              << " activeWidth=" << activeWidth
              << " activeHeight=" << activeHeight
              << std::endl;

    sfdp_t sfdp(sf_);

    if (expectedElems != sf_.sf_t::GlobalLinearShapeFunction_t::size()) {
        std::cerr << "Shape function size mismatch: expected activeElems="
                  << expectedElems
                  << " sf size=" << sf_.sf_t::GlobalLinearShapeFunction_t::size()
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const bool isSBGGR16 =
        (pixelFormat == libcamera::formats::SBGGR16);

    const bool isSBGGR10Packed =
        (pixelFormat == libcamera::formats::SBGGR10_CSI2P);

    if (!isSBGGR16 && !isSBGGR10Packed) {
        std::cerr << "Unsupported validated pixel format for current UDP path: "
                  << pixelFormat.toString() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    const size_t requiredBytes = strideBytes * activeHeight;
    if (plane.length < requiredBytes) {
        std::cerr << "Plane too small for configured stride/height: plane.length="
                  << plane.length
                  << " requiredBytes=" << requiredBytes
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    void *mapped = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mapped == MAP_FAILED) {
        perror("Failed to mmap plane data");
        std::exit(EXIT_FAILURE);
    }

    auto unmapPlane = [&]() {
        if (mapped != MAP_FAILED && munmap(mapped, plane.length) != 0) {
            perror("Failed to unmap plane data");
        }
    };

    std::vector<uint16_t> activeFrame(expectedElems);

    if (isSBGGR16) {
        if ((strideBytes % sizeof(uint16_t)) != 0) {
            std::cerr << "Stride is not 16-bit aligned for SBGGR16: strideBytes="
                      << strideBytes << std::endl;
            unmapPlane();
            std::exit(EXIT_FAILURE);
        }

        const size_t strideElems = strideBytes / sizeof(uint16_t);
        if (strideElems < activeWidth) {
            std::cerr << "Stride smaller than active width: strideElems="
                      << strideElems
                      << " activeWidth=" << activeWidth
                      << std::endl;
            unmapPlane();
            std::exit(EXIT_FAILURE);
        }

        const auto *mappedData = static_cast<const uint16_t *>(mapped);

        std::cout << "[FrameBufferToUDP] SBGGR16 strideElems=" << strideElems
                  << " expectedElems=" << expectedElems
                  << " requiredBytes=" << requiredBytes
                  << std::endl;

        for (size_t y = 0; y < activeHeight; ++y) {
            const uint16_t *srcRow = mappedData + y * strideElems;
            uint16_t *dstRow = activeFrame.data() + y * activeWidth;
            std::copy_n(srcRow, activeWidth, dstRow);
        }
    } else {
        const auto *mappedData = static_cast<const uint8_t *>(mapped);
        const size_t packedRowBytes = (activeWidth * 10 + 7) / 8;

        if (strideBytes < packedRowBytes) {
            std::cerr << "Stride smaller than packed RAW10 row size: strideBytes="
                      << strideBytes
                      << " packedRowBytes=" << packedRowBytes
                      << std::endl;
            unmapPlane();
            std::exit(EXIT_FAILURE);
        }

        std::cout << "[FrameBufferToUDP] SBGGR10_CSI2P packedRowBytes="
                  << packedRowBytes
                  << " expectedElems=" << expectedElems
                  << " requiredBytes=" << requiredBytes
                  << std::endl;

        for (size_t y = 0; y < activeHeight; ++y) {
            const uint8_t *srcRow = mappedData + y * strideBytes;
            uint16_t *dstRow = activeFrame.data() + y * activeWidth;

            size_t x = 0;
            size_t byteIdx = 0;

            for (; x + 3 < activeWidth; x += 4, byteIdx += 5) {
                const uint8_t b0 = srcRow[byteIdx + 0];
                const uint8_t b1 = srcRow[byteIdx + 1];
                const uint8_t b2 = srcRow[byteIdx + 2];
                const uint8_t b3 = srcRow[byteIdx + 3];
                const uint8_t b4 = srcRow[byteIdx + 4];

                dstRow[x + 0] = static_cast<uint16_t>(
                    (static_cast<uint16_t>(b0) << 2) | ((b4 >> 0) & 0x03));
                dstRow[x + 1] = static_cast<uint16_t>(
                    (static_cast<uint16_t>(b1) << 2) | ((b4 >> 2) & 0x03));
                dstRow[x + 2] = static_cast<uint16_t>(
                    (static_cast<uint16_t>(b2) << 2) | ((b4 >> 4) & 0x03));
                dstRow[x + 3] = static_cast<uint16_t>(
                    (static_cast<uint16_t>(b3) << 2) | ((b4 >> 6) & 0x03));
            }

            if (x != activeWidth) {
                std::cerr << "Active width must currently be divisible by 4 for RAW10 unpack: "
                          << activeWidth << std::endl;
                unmapPlane();
                std::exit(EXIT_FAILURE);
            }
        }
    }

    {
        auto [minIt, maxIt] = std::minmax_element(activeFrame.begin(), activeFrame.end());
        std::cout << "[activeFrame] min=" << *minIt
                  << " max=" << *maxIt << std::endl;
    
        cv::Mat raw16(static_cast<int>(activeHeight),
                      static_cast<int>(activeWidth),
                      CV_16UC1,
                      activeFrame.data());
    
        cv::Mat bgr_bg_16, bgr_gb_16, bgr_rg_16, bgr_gr_16;
        cv::Mat bgr_bg_8,  bgr_gb_8,  bgr_rg_8,  bgr_gr_8;
    
        cv::cvtColor(raw16, bgr_bg_16, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(raw16, bgr_gb_16, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(raw16, bgr_rg_16, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(raw16, bgr_gr_16, cv::COLOR_BayerGR2BGR);
    
        bgr_bg_16.convertTo(bgr_bg_8, CV_8UC3, 1.0 / 256.0);
        bgr_gb_16.convertTo(bgr_gb_8, CV_8UC3, 1.0 / 256.0);
        bgr_rg_16.convertTo(bgr_rg_8, CV_8UC3, 1.0 / 256.0);
        bgr_gr_16.convertTo(bgr_gr_8, CV_8UC3, 1.0 / 256.0);
    
        cv::imwrite("/tmp/bayer_bg.png", bgr_bg_8);
        cv::imwrite("/tmp/bayer_gb.png", bgr_gb_8);
        cv::imwrite("/tmp/bayer_rg.png", bgr_rg_8);
        cv::imwrite("/tmp/bayer_gr.png", bgr_gr_8);
    }

    std::span<uint16_t> mappedDataSpan(activeFrame.data(), activeFrame.size());

    GainMsg appliedGainMsg = gainMsg;
    appliedGainMsg.r_gain_apply = appliedGainMsg.r_gain;
    appliedGainMsg.b_gain_apply = appliedGainMsg.b_gain;

    {
        std::lock_guard<std::mutex> lock(gainMutex_);
        if (gainValid_) {
            appliedGainMsg.r_gain_apply = latestRGainApply_;
            appliedGainMsg.b_gain_apply = latestBGainApply_;
        }
    }

    FrameBufferTransformation(mappedDataSpan, appliedGainMsg, sfdp, chunkThreads_);

    auto data = sfdp.TakeContiguousMemory();

    std::array<uint32_t, NUM_REGIONS> regionSizes{};
    size_t totalElems = 0;
    for (size_t i = 0; i < NUM_REGIONS; ++i) {
        regionSizes[i] = static_cast<uint32_t>(sfdp.RegionValidSize(i));

        const auto valid = sfdp.RegionValidSize(i);
        std::cout << "[TX] region " << i
                  << " valid=" << valid
                  << " capacity=" << sfdp[i].size()
                  << std::endl;
        totalElems += valid;
    }

    std::cout << "[TX] packed payload elems=" << data.size()
              << " bytes=" << data.size() * sizeof(uint16_t)
              << std::endl;

    if (data.size() != totalElems) {
        std::cerr << "[TX ERROR] packed element count mismatch: "
                  << "sum(validSizes)=" << totalElems
                  << " packed=" << data.size()
                  << std::endl;
        unmapPlane();
        std::exit(EXIT_FAILURE);
    }

    frameSrv_.SubmitFrameOutput(
        frameNumber,
        frameSrv_.CreateFramePacket(
            appliedGainMsg,
            regionSizes,
            std::move(data)
        )
    );

    unmapPlane();

    std::cout << "[GainApply] frame=" << frameNumber
              << " r_gain=" << appliedGainMsg.r_gain
              << " b_gain=" << appliedGainMsg.b_gain
              << " r_gain_apply=" << appliedGainMsg.r_gain_apply
              << " b_gain_apply=" << appliedGainMsg.b_gain_apply
              << std::endl;

    fpsSent_.Tick();
}

template<size_t n>
void* Aperture<n>::StartCaptureThread(void* arg){
	log_verbose("[Aperture::StartCaptureThread]");
   StartCapture();
   {
		std::lock_guard<std::mutex> lock(mutex_);
		isStartCaptureThreadDone_ = true;
	}
	condition_.notify_all();
	
   
   return nullptr;
}

template<size_t n>
bool Aperture<n>::ContinuousCapture(unsigned int index){
	std::cout << ">>> ENTER ContinuousCapture <<<" << std::endl;
    log_verbose("[Aperture::ContinuousCapture]");

    requests_.clear();

    SignalAndSlots();
    BufferAllocation();

    StartUDPSender();
    StartRequestProcessingThreads(processingThreads_);
    StartPostProcessingThreads(processingThreads_);

    FrameCapture(index);

	std::cout << "[DEBUG] Before start()" << std::endl;
	
	if (camera_->start() != 0) {
		std::cerr << "Camera failed to start" << std::endl;
		return false;
	}
	
	std::cout << "[DEBUG] After start()" << std::endl;

    std::cout << "Camera started OK" << std::endl;
    std::cout << "Queuing " << requests_.size() << " requests..." << std::endl;

    for (auto &request : requests_) {
        int ret = camera_->queueRequest(request.get());

        std::cout << "[queueRequest] req=" << request.get()
                  << " ret=" << ret << std::endl;

        if (ret < 0) {
            std::cerr << "Failed to queue request" << std::endl;
        } else {
            pendingRequests_++;
        }
    }

    RunEventLoop(timeout_);

    if (camera_->stop() != 0) {
        std::cerr << "Camera failed to stop" << std::endl;
    }

    return true;
}

template<size_t n>
void Aperture<n>::CaptureLoop(unsigned int index) {
    log_verbose("[Aperture::CaptureLoop]");
    FrameCapture(index);

    std::cout << "Queuing " << requests_.size() << " requests..." << std::endl;
    for (auto &request : requests_) {
        int ret = camera_->queueRequest(request.get());

        std::cout << "[queueRequest] req=" << request.get()
                  << " ret=" << ret
                  << " pending=" << pendingRequests_.load()
                  << std::endl;

        if (ret >= 0) pendingRequests_++;
        else std::cerr << "Failed to queue request" << std::endl;
    }
}

template<size_t n>
void Aperture<n>::FreeStream(unsigned int indexCount){
	log_verbose("[Aperture::FreeStream]");
    for(unsigned int i = 0; i<indexCount; i++)
         allocator_->free(config_->at(i).stream());
}

template<size_t n>
void Aperture<n>::RequestReuse(libcamera::Request::ReuseFlag Flags){
	log_verbose("[Aperture::RequestReuse]");
    for( auto& request : requests_)
        request->reuse(Flags);
}

template<size_t n>
void Aperture<n>::StartUDPSender() {
	log_verbose("[Aperture::StartUDPSender]");
    udpSending_ = true;

    udpSenderThread_ = std::thread([this]() {
        // Set thread affinity and name
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(0, &cpuset);  // Pin to CPU core 0
        //pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        pthread_setname_np(pthread_self(), "UDP");

        while (udpSending_) {
			 bool sent = false;
			{
				std::lock_guard<std::mutex> lock(frameSrv_.outputMutex_);
				sent = frameSrv_.TrySendFramesInOrder();
			}
			if (!sent)
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			
        }
    });
}

template<size_t n>
void Aperture<n>::StopUDPSender() {
	log_verbose("[Aperture::StopUDPSender]");
	{
		std::lock_guard<std::mutex> lock(udpQueueMutex_);
		udpSending_ = false;
	}
	udpQueueCV_.notify_all();
	if (udpSenderThread_.joinable())
		udpSenderThread_.join();
}

template<size_t n>
void Aperture<n>::StartPostProcessingThreads(int numThreads)
{
    stopPostProcessing_ = false;

    for (int i = 0; i < numThreads; ++i) {
        postProcessingThreads_.emplace_back([this]
        {
            while (true) {
                PostProcessItem item;

                {
                    std::unique_lock<std::mutex> lock(postProcessingQueueMutex_);

                    postProcessingQueueCV_.wait(lock, [this] {
                        return stopPostProcessing_ || !postProcessingQueue_.empty();
                    });

                    if (stopPostProcessing_ && postProcessingQueue_.empty()) {
                        return;
                    }

                    item = std::move(postProcessingQueue_.front());
                    postProcessingQueue_.pop();
                }

                FrameBufferToUDP(item.frameNumber, item.buffer, item.gainMsg);
            }
        });
    }
}

template<size_t n>
void Aperture<n>::StopPostProcessingThreads()
{
    stopPostProcessing_ = true;
    postProcessingQueueCV_.notify_all();

    for (auto& t : postProcessingThreads_) {
        if (t.joinable()) {
            t.join();
        }
    }

    postProcessingThreads_.clear();
}

/// Apply white balance gains to a RAW16 Bayer BGGR mosaic in-place.
/// Layout BGGR:
///   row 0: B G B G ...
///   row 1: G R G R ...
///   row 2: B G B G ...
///   row 3: G R G R ...
///
/// data.size() must be width*height.
template<size_t n>
inline void Aperture<n>::ApplyWhiteBalanceToMosaic_BGGR(
    size_t region,
    const sf_t& sf,
	std::span<uint16_t> data,
	const GainMsg& gainMsg
){
	log_verbose("[Aperture::ApplyWhiteBalanceToMosaic_BGGR]");
	
	// Validate sizes (elements == pixels)
	const size_t elems = (region==0) ? sf_.sf_t::nonOverlapFacet_t::size() : sf.size(region-1);
	if ( data.size() != elems) {
		std::cerr << "[WB] size mismatch: data=" << data.size()
				  << " expected=" << elems <<"\n";
		exit(1); // or throw
	}
	//std::cout << "Check-Point 1" << std::endl;
	// Convert gains to fixed point (Q10: 1.0 == 1024)
	auto toQ10 = [](double g) -> uint32_t {
		if (g < 0.0) g = 0.0;
		return static_cast<uint32_t>(std::lround(g * 1024.0));
	};
	const uint32_t rQ = toQ10(gainMsg.r_gain_apply);
	const uint32_t gQ = toQ10(1.0);
	const uint32_t bQ = toQ10(gainMsg.b_gain_apply);

	// Saturating multiply: (v * gainQ + 512) >> 10, clamped to 65535
	auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
		uint64_t t = static_cast<uint64_t>(v) * gainQ + 512; // +0.5 for rounding
		t >>= 10;
		if (t > 65535u) t = 65535u;
		return static_cast<uint16_t>(t);
	};
	//std::cout << "Check-Point 2" << std::endl;

	size_t maskedSize{(region==0) ? sf_.sf_t::nonOverlapFacet_t::size() : sf.indexLinearMaxs_[region-1]->size()};
	//std::cout << "maskedSize: " << maskedSize << std::endl;
	
	for (size_t i = 0; i < maskedSize; ++i) {
		typename sf_t::GlobalLinearShapeFunction_t::Index globalIndex{
			(region==0)	? (*sf_.sf_t::nonOverlapFacet_t::indexLinearMax_)[i].value() : (*sf.indexLinearMaxs_[region-1])[i].value()};		
		//std::cout << "globalIndex.value():" << globalIndex.value() << std::endl;
		typename sf_t::GlobalLinearShapeFunction_t::Point globalPt{sf_t::GlobalLinearShapeFunction_t::Transform(globalIndex)};
		//std::cout << "globalPt:(" << globalPt.x() << "," << globalPt.y() << ") " << std::endl;

		//sf_t::GlobalLinearShapeFunction_t::Point pt{sf_t::Transform((*sf.indexLinearMaxs_[region])[i])};
		const bool evenRow = (globalPt.y_ & 1) == 0;
		const bool evenCol = (globalPt.x_ & 1) == 0;

		//uint16_t* row = data.data() + globalPt.y_ * sf.GlobalLinearShapeFunction_t::sensorWidth().value();

		
		// BGGR pattern:
		// row 0: B G B G ...
		// row 1: G R G R ...
		const uint32_t v{ data[i]};
		//const uint32_t v = row[globalPt.x_];
		uint16_t out;
		//if (evenRow) {
			//out = evenCol ? mulSatQ10(v, bQ) : mulSatQ10(v, gQ);
		//} else {
			//out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, rQ);
		//}
        if (evenRow) {
            out = evenCol ? mulSatQ10(v, rQ) : mulSatQ10(v, gQ);
        } else {
            out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, bQ);
        }
		data[i] = out;
	}
	//std::cout << "Check-Point 3" << std::endl;
}

template<size_t n>
inline void Aperture<n>::ApplyWhiteBalanceToMosaic_RGGB(
    size_t region,
    const sf_t& sf,
    std::span<uint16_t> data,
    const GainMsg& gainMsg
){
    log_verbose("[Aperture::ApplyWhiteBalanceToMosaic_RGGB]");

    const size_t elems = (region == 0) ? sf_.sf_t::nonOverlapFacet_t::size()
                                       : sf.size(region - 1);
    if (data.size() != elems) {
        std::cerr << "[WB] size mismatch: data=" << data.size()
                  << " expected=" << elems << "\n";
        std::exit(EXIT_FAILURE);
    }

    auto toQ10 = [](double g) -> uint32_t {
        if (g < 0.0) g = 0.0;
        return static_cast<uint32_t>(std::lround(g * 1024.0));
    };

    const uint32_t rQ = toQ10(gainMsg.r_gain_apply);
    const uint32_t gQ = toQ10(1.0);
    const uint32_t bQ = toQ10(gainMsg.b_gain_apply);

    auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
        uint64_t t = static_cast<uint64_t>(v) * gainQ + 512;
        t >>= 10;
        if (t > 65535u) t = 65535u;
        return static_cast<uint16_t>(t);
    };

    const size_t maskedSize = (region == 0)
        ? sf_.sf_t::nonOverlapFacet_t::size()
        : sf.indexLinearMaxs_[region - 1]->size();

    for (size_t i = 0; i < maskedSize; ++i) {
        typename sf_t::GlobalLinearShapeFunction_t::Index globalIndex{
            (region == 0)
                ? (*sf_.sf_t::nonOverlapFacet_t::indexLinearMax_)[i].value()
                : (*sf.indexLinearMaxs_[region - 1])[i].value()
        };

        typename sf_t::GlobalLinearShapeFunction_t::Point globalPt{
            sf_t::GlobalLinearShapeFunction_t::Transform(globalIndex)
        };

        const bool evenRow = (globalPt.y_ & 1) == 0;
        const bool evenCol = (globalPt.x_ & 1) == 0;

        // RGGB pattern:
        // row 0: R G R G ...
        // row 1: G B G B ...
        const uint32_t v = data[i];
        uint16_t out;
        if (evenRow) {
            out = evenCol ? mulSatQ10(v, rQ) : mulSatQ10(v, gQ);
        } else {
            out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, bQ);
        }

        data[i] = out;
    }
}
	
template<size_t n>
std::string Aperture<n>::GetBoardSerial()
{
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line))
    {
        if (line.rfind("Serial", 0) == 0) // starts with "Serial"
        {
            auto pos = line.find(':');
            if (pos != std::string::npos)
            {
                std::string serial = line.substr(pos + 1);
                // trim spaces
                serial.erase(0, serial.find_first_not_of(" \t"));
                serial.erase(serial.find_last_not_of(" \t\r\n") + 1);
                return serial;
            }
        }
    }
    return "UNKNOWN";
}

template<size_t n>
std::string Aperture<n>::ExtractCameraBusAddr(const std::string& libcameraId)
{
    // Format 1:
    //   "imx296 10-001a"
    {
        std::istringstream iss(libcameraId);
        std::string first;
        std::string second;
        if ((iss >> first) && (iss >> second)) {
            if (second.find('-') != std::string::npos) {
                return second; // e.g. "10-001a"
            }
        }
    }

    // Format 2:
    //   "/base/soc/i2c0mux/i2c@1/imx296@1a"
    {
        const auto lastSlash = libcameraId.rfind('/');
        if (lastSlash == std::string::npos) {
            throw std::runtime_error("Unexpected camera id format: " + libcameraId);
        }

        const std::string tail = libcameraId.substr(lastSlash + 1); // "imx296@1a"
        const auto atTail = tail.rfind('@');
        if (atTail == std::string::npos || atTail + 1 >= tail.size()) {
            throw std::runtime_error("Missing sensor address in camera id: " + libcameraId);
        }
        const std::string addrPart = tail.substr(atTail + 1); // "1a"

        const std::string prefix = libcameraId.substr(0, lastSlash);
        const auto prevSlash = prefix.rfind('/');
        const std::string parent = (prevSlash == std::string::npos)
            ? prefix
            : prefix.substr(prevSlash + 1);                   // "i2c@1"

        const auto atParent = parent.rfind('@');
        if (atParent == std::string::npos || atParent + 1 >= parent.size()) {
            throw std::runtime_error("Missing bus id in camera id: " + libcameraId);
        }
        const std::string busPart = parent.substr(atParent + 1); // "1"

        return busPart + "-" + addrPart; // "1-1a"
    }
}

template<size_t n>
uint32_t Aperture<n>::ParseBusAndAddressCameraId(const std::string& libcameraId)
{
    const std::string busAddr = ExtractCameraBusAddr(libcameraId);

    const auto dashPos = busAddr.find('-');
    if (dashPos == std::string::npos) {
        throw std::runtime_error("Camera bus/address missing '-': " + busAddr);
    }

    const std::string busPart  = busAddr.substr(0, dashPos);
    const std::string addrPart = busAddr.substr(dashPos + 1);

    const uint32_t bus  = static_cast<uint32_t>(std::stoul(busPart, nullptr, 10));
    const uint32_t addr = static_cast<uint32_t>(std::stoul(addrPart, nullptr, 16));

    return (bus << 16) | addr;
}

template<size_t n>
bool Aperture<n>::ReceiveGainReply()
{
    GainReply reply{};

    const ssize_t bytes = controlClnt_.ReceiveFromServer(reply);
    if (bytes <= 0) {
        return false;
    }

    if (bytes != static_cast<ssize_t>(sizeof(GainReply))) {
        std::cerr << "ReceiveGainReply: wrong size: " << bytes << '\n';
        return false;
    }

    if (reply.header.type != MessageType::GainReply) {
        return false;
    }

    if (reply.header.version != 1) {
        return false;
    }

    HandleGainReply(reply);
    return true;
}

template<size_t n>
void Aperture<n>::HandleGainReply(const GainReply& reply)
{
    if (reply.status != 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(gainMutex_);
	latestRGainApply_ = reply.r_gain_apply;
	latestBGainApply_ = reply.b_gain_apply;
	latestGainFrameId_ = reply.frame_id;
	gainValid_ = true;
}

template<size_t n>
bool Aperture<n>::RunControlLoop()
{
    while (running_) {
        if (!ReceiveGainReply()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return true;
}

template<size_t n>
template<typename InputT>
constexpr void Aperture<n>::FrameBufferTransformation(InputT&& input,
                                                   const GainMsg& gainMsg,
                                                   sfdp_t& output,
                                                   const size_t numThreads)
{
    log_verbose("[Aperture::FrameBufferTransformation]");

    if (numThreads == 0) {
        std::cerr << "[FrameBufferTransformation] numThreads must be > 0" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    output.ResetValidSizes();

    auto copyFacetToOutput = [&](size_t outIndex, auto& maskedOutput) {
        auto& dst = output[outIndex];

        std::cout << "[FrameBufferTransformation] facet=" << outIndex
                  << " masked.size()=" << maskedOutput.size()
                  << " dst.capacity()=" << dst.size()
                  << std::endl;

        if (maskedOutput.size() > dst.size()) {
            std::cerr << "[FrameBufferTransformation] overflow for facet " << outIndex
                      << ": masked.size()=" << maskedOutput.size()
                      << " dst.size()=" << dst.size()
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::memcpy(dst.data(), maskedOutput.data(), maskedOutput.size() * sizeof(uint16_t));
        output.SetRegionValidSize(outIndex, maskedOutput.size());
    };

    {
        auto maskedOutput = sf_.sf_t::nonOverlapFacet_t::FrameBufferMask(input);
        ApplyWhiteBalanceToMosaic_RGGB(
            0, sf_,
            std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            gainMsg
        );
        copyFacetToOutput(0, maskedOutput);
    }

    for (size_t i = 0; i < n_; ++i) {
        auto maskedOutput = sf_.FrameBufferMask(input, i);
        ApplyWhiteBalanceToMosaic_RGGB(
            i + 1, sf_,
            std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            gainMsg
        );
        copyFacetToOutput(i + 1, maskedOutput);
    }
    
    size_t totalValid = 0;
    for (size_t i = 0; i < NUM_REGIONS; ++i) {
        totalValid += output.RegionValidSize(i);
    }
    
    std::cout << "[FrameBufferTransformation] total valid elements="
              << totalValid << std::endl;
}

};//end namespace DASPi
