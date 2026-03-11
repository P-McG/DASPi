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

using namespace DASPi;
using namespace std::chrono;
using namespace libcamera;

// Constructor
Aperture::Aperture(const std::string clientIp, const size_t port)
:udpSrv_{UDPSrv(clientIp, port)} 
{
	 log_verbose("[Aperture::Aperture]");
	 CreateCameraManager();
	 AquireCamera();
	 Stream();
	 fpsTimer_=std::chrono::high_resolution_clock::now();
}

// Destructor
/*
 * --------------------------------------------------------------------
 * Clean Up
 * Stop the Camera, release resources and stop the CameraManager.
 * libcamera has now released all resources it owned.
*/
Aperture::~Aperture(){
    log_verbose("[Aperture::~Aperture]");
    
    CleanupCamera();
	StopUDPSender();
	StopPostProcessingThreads();
	StopRequestProcessingThreads();

}


void Aperture::PrintThreadInfo() {
    std::cout << " (std::thread ID: " << std::this_thread::get_id() << ")"
              << " is running on CPU " << sched_getcpu() << std::endl;
}

void Aperture::CleanupCamera() {
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
void Aperture::CreateCameraManager(){

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
void Aperture::AquireCamera(unsigned int index){

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
void Aperture::Stream(){
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
void Aperture::CameraConfiguration(){

	log_verbose("[Aperture::CameraConfiguration]");
	config_ = camera_->generateConfiguration( { libcamera::StreamRole::Raw } );
	
	std::string cameraId = camera_->id(); // e.g., "imx296 10-001a"
    std::string boardSerial = GetBoardSerial(); // Pi's unique CPU serial
    std::string cameraUniqueId_ = boardSerial + "_" + cameraId;
	std::cout << "Board serial : " << boardSerial << "\n";
    std::cout << "Camera ID    : " << cameraId << "\n";
    std::cout << "Unique ID    : " << cameraUniqueId_ << "\n";
}

void Aperture::StreamConfiguration(unsigned int index){
    log_verbose("[Aperture::StreamConfiguration]");
	// Modify the viewfinder configuration
	libcamera::StreamConfiguration &streamConfig = config_->at(index);
	streamConfig.size.width = sensorWidthValue_;  // Desired width
	streamConfig.size.height = sensorHeightValue_; // Desired height
	streamConfig.pixelFormat = codec_; // Desired pixel format
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
void Aperture::StreamConfigurationValidation(unsigned int index){
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
}

void Aperture::StreamConfigurationSet(){
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
void Aperture::BufferAllocation(){

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

std::unique_ptr<libcamera::Request> Aperture::CreateRequestWithControls(){
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


void Aperture::FrameCapture(unsigned int index) {
	log_verbose("[FrameCapture]");

    if (!camera_) {
        std::cerr << "camera_ is NULL, aborting FrameCapture!" << std::endl;
        exit(EXIT_FAILURE);
    }

    const auto &buffers = allocator_->buffers(config_->at(index).stream());
    std::cout << "buffers.size: " << buffers.size() << std::endl;

    for (auto &buffer : buffers) {
        std::cout << "Creating Request..." << std::endl;
		

        auto request = camera_->createRequest();
        if (!request) {
            std::cerr << "Can't create request" << std::endl;
            exit(EXIT_FAILURE);
        }

        int ret = request->addBuffer(config_->at(index).stream(), buffer.get());
        if (ret < 0) {
            std::cerr << "Can't set buffer for request" << std::endl;
            exit(EXIT_FAILURE);
        }
		
		//const std::array<int64_t, 2> duration = {33333, 33333}; // min = max = 1/30 s
		//request->controls().set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>(duration));
	
		//// Change the exposure
		//SetAeCompensationOnRequest(request.get(), +3.0f);

		bufferMap_[request.get()] = buffer.get();
        requests_.push_back(std::move(request));
    }
	
	std::cout << "Stored request pointers in requests_:" << std::endl;
	for (const auto& req : requests_) {
		std::cout << "  Request at: " << req.get() << std::endl;
	}
	
}

//Not used
void Aperture::QueueCameraRequests(bool isRequests){
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
void Aperture::RequestComplete(libcamera::Request *request){
	log_verbose("[RequestComplete]");
	if (request->status() == libcamera::Request::RequestCancelled)
		return;

    loop_.callLater(std::bind([this, request]() { this->ProcessRequest(request); }));
	
}


void Aperture::StartRequestProcessingThreads(int numThreads) {
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

void Aperture::StopRequestProcessingThreads() {
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



void Aperture::RequestProcessingThread() {
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

void Aperture::ProcessRequestImpl(libcamera::Request *request)
{
    log_verbose("[Aperture::ProcessRequestImpl]");

    if (!request) {
        std::cerr << "ProcessRequestImpl: null request\n";
        return;
    }

    const uint64_t completedFrameId = request->sequence();

    GainMsg gainMsg{};
    gainMsg.camera_id = static_cast<uint32_t>(cameraId_);
    gainMsg.frame_id = static_cast<uint32_t>(completedFrameId);
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
                postProcessingQueue_.push(PostProcessItem{
                    .frameNumber = completedFrameId,
                    .buffer = buffer,
                    .gainMsg = gainMsg
                });
            }
            postProcessingQueueCV_.notify_all();
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

void Aperture::PostProcessingThread()
{
    for (;;) {
        PostProcessItem item{};

        {
            std::unique_lock<std::mutex> lock(postProcessingQueueMutex_);
            postProcessingQueueCV_.wait(lock, [this] {
                return stopPostProcessing_ || !postProcessingQueue_.empty();
            });

            if (stopPostProcessing_ && postProcessingQueue_.empty())
                return;

            item = postProcessingQueue_.front();
            postProcessingQueue_.pop();
        }

        FrameBufferToUDP(item.frameNumber, item.buffer, item.gainMsg);
    }
}
// ProcessRequest
/*
 * When a request has completed, it is populated with a metadata control
 * list that allows an application to determine various properties of
 * the completed request. This can include the timestamp of the Sensor
 * capture, or its gain and exposure values, or properties from the IPA
 * such as the state of the 3A algorithms.
 *
 * ControlValue types have a toString, so to examine each request, print
 * all the metadata for inspection. A custom application can parse each
 * of these items and process them according to its needs.
 *
 * Each buffer has its own FrameMetadata to describe its state, or the
 * usage of each buffer. While in our simple capture we only provide one
 * buffer per request, a request can have a buffer for each stream that
 * is established when configuring the camera.
 *
 * This allows a viewfinder and a still image to be processed at the
 * same time, or to allow obtaining the RAW capture buffer from the
 * sensor along with the image as processed by the ISP.
 */
//void Aperture::ProcessRequestImpl(libcamera::Request *request) {
	//log_verbose("[Aperture::ProcessRequestImp]");

    //// Reuse & re-queue
    //request->reuse();
    //auto it = bufferMap_.find(request);
    //if (it != bufferMap_.end()) {
        //request->addBuffer(config_->at(0).stream(), it->second);
    //} else {
        //std::cerr << " No buffer mapped to request: " << request << std::endl;
    //}
	
	////// Change the exposure
	////SetAeCompensationOnRequest(request, +3.0f);

    //int ret = camera_->queueRequest(request);
    //if (ret < 0) {
        //std::cerr << "Failed to re-queue request for frame " << request->sequence() << " (ret=" << ret << ")" << std::endl;
    //}

    //// Offload to post-processing if queue not full
	////uint64_t frameNumber = request->sequence();  // ← Move this up
	//uint64_t frameNumber = globalFrameCounter_++;

	//for (auto &[stream, buffer] : request->buffers()) {
		//{
			//std::cout << "[Enqueue] Frame " << frameNumber << " for stream "
			  //<< reinterpret_cast<const void*>(stream) << " enqueued." << std::endl;

			//std::lock_guard<std::mutex> lock(postProcessingQueueMutex_);
		
			//constexpr size_t MAX_QUEUE_SIZE = 256;
			//if (postProcessingQueue_.size() >= MAX_QUEUE_SIZE) {
		//#ifdef VERBATIUM_COUT
				//std::cout << "Dropped frame: post-processing queue full" << std::endl;
		//#endif
				//return;
			//}
		
			//postProcessingQueue_.emplace([this, frameNumber, buffer]() {
				//FrameBufferToUDP(frameNumber, buffer);
			//});
		//}
		//postProcessingQueueCV_.notify_one();
	//}	

    //// Reduce pending count
    //{
        //std::lock_guard<std::mutex> lock(queueMutex_);
        //pendingRequests_--;
    //}
//}


void Aperture::ProcessRequest(libcamera::Request* request) {
	log_verbose("[Aperture::ProcessingRequest]");
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
std::string Aperture::CameraName(libcamera::Camera *camera)
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
				name = " '" + *model + "'";
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
void Aperture::SignalAndSlots(){
	log_verbose("[Aperture::SignalAndSlots]");
	camera_->requestCompleted.connect(
	    this, &DASPi::Aperture::RequestComplete
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
void Aperture::StartCapture() {
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
void Aperture::RunEventLoop(int timeout){
	log_verbose("[Aperture::RunEventLoop]");

	loop_.timeout(timeout);
	loop_.exec();
	
}

void Aperture::FrameBufferToUDP(const uint64_t frameNumber, const libcamera::FrameBuffer *buffer) {
	log_verbose("[FrameBufferToUDP]");

	static int producedFrames = 0;
	producedFrames++;
	if (producedFrames % 10 == 0) {
		std::cout << "[Produced] " << producedFrames << " frames buffered for UDP" << std::endl;
	}

	const auto &plane = buffer->planes().back();
	if (!plane.fd.isValid()) {
		std::cerr << "Invalid file descriptor for plane" << std::endl;
		exit(1);
	}
	
	// Map the plane data into memory
	//uint8_t *mappedData = static_cast<uint8_t*>(mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_PRIVATE, plane.fd.get(), 0));
	uint16_t *mappedData = static_cast<uint16_t*>(mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0));

	if (mappedData == MAP_FAILED) {
		perror("Failed to mmap plane data");
		exit(1);
	}
	std::span<uint16_t> mappedDataSpan(mappedData, plane.length / sizeof(uint16_t));
	 
	sfdp_t sfdp(sf_);
	
	// Resize the output to the largest data size because the smaller regions data size is not known. 
	// Selected the size of facet which is the largest size the overlap can be.
	if(mappedDataSpan.size() !=  sf_.GlobalLinearShapeFunction_t::size()){
		std::cerr << "mappedDataSpan.size():" << mappedDataSpan.size() << " sf_.GlobalLinearShapeFunction_t::size():" << sf_.GlobalLinearShapeFunction_t::size() << " sfdp.size():" << sfdp.size();
		std::cerr << "mappedDataSpan not the correct size" << std::endl;
		exit(1);
	}
	

	
	FrameBufferTransformation(mappedDataSpan, gainMsg, sfdp, chunkThreads_);
	
	udpSrv_.SubmitFrameOutput(
		frameNumber,
		udpSrv_.CreateFramePacket(gainMsg, sfdp.TakeContiguousMemory())
	);

	if (munmap(mappedData, plane.length) != 0) {
		perror("Failed to unmap plane data");
	}
}

void* Aperture::StartCaptureThread(void* arg){
	log_verbose("[Aperture::StartCaptureThread]");
   StartCapture();
   {
		std::lock_guard<std::mutex> lock(mutex_);
		isStartCaptureThreadDone_ = true;
	}
	condition_.notify_all();
	
   
   return nullptr;
}

bool Aperture::ContinuousCapture(unsigned int index){
	log_verbose("[Aperture::ContinuousCapture]");

    //Prep for capture
    requests_.clear();//Clear previous old requests
    SignalAndSlots();// Assign what to do with the requests
    BufferAllocation();//Allocate a buffer for a request.
	
    //Start Capture on another thread
	StartUDPSender();
    StartRequestProcessingThreads(processingThreads_); // before capture starts'
	StartPostProcessingThreads(processingThreads_);
		   
    std::thread t1([this]() {
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(0, &cpuset);  // Pin to CPU core 0
		//pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
		pthread_setname_np(pthread_self(), "Capture");

        this->StartCaptureThread(nullptr);
    });
    
    // Loop while getting external requests from udp source
	
    while(!isStartCaptureThreadDone_){
      if(isStartCaptureRunning_ && !isStartCaptureThreadDone_){// only do this if start capture is ready and running
           CaptureLoop(index);
		   break;
        }
    }
               
    // Join the thread to clean up resources
    t1.join();
    return true;
}


void Aperture::CaptureLoop(unsigned int index) {
	log_verbose("[Aperture::CaptureLoop]");
    FrameCapture(index);  // One-time request creation

	std::cout << "Queuing " << requests_.size() << " requests..." << std::endl;
	for (auto &request : requests_) {
		int ret = camera_->queueRequest(request.get());
		if (ret >= 0) pendingRequests_++;
		else std::cerr << "Failed to queue request" << std::endl;
	}
	
	std::unique_lock<std::mutex> lock(mutex_);
	condition_.wait(lock, [this] { return isStartCaptureThreadDone_.load(); });


}



void Aperture::FreeStream(unsigned int indexCount){
	log_verbose("[Aperture::FreeStream]");
    for(unsigned int i = 0; i<indexCount; i++)
         allocator_->free(config_->at(i).stream());
}

void Aperture::RequestReuse(libcamera::Request::ReuseFlag Flags){
	log_verbose("[Aperture::RequestReuse]");
    for( auto& request : requests_)
        request->reuse(Flags);
}

//void Aperture::neon_copy_and_brighten_u16(uint8_t* dest, const uint8_t* src, size_t count) {
	//log_verbose("[Apertyr]");
    //size_t i = 0;

    //for (; i + 8 <= count; i += 8) {
        //uint16x8_t input = vld1q_u16(&src[i]);
        //uint16x8_t brightened = vshlq_n_u16(input, 6);  // shift left by 6 bits (×64)
        //vst1q_u16(&dest[i], brightened);
    //}

    //for (; i < count; ++i) {
        //dest[i] = src[i] << 6;  // scalar fallback
    //}
//}


void Aperture::StartUDPSender() {
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
				std::lock_guard<std::mutex> lock(udpSrv_.outputMutex_);
				sent = udpSrv_.TrySendFramesInOrder();  // return true if at least 1 frame was sent
			}
			if (!sent)
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			
        }
    });
}

void Aperture::StopUDPSender() {
	log_verbose("[Aperture::StopUDPSender]");
	{
		std::lock_guard<std::mutex> lock(udpQueueMutex_);
		udpSending_ = false;
	}
	udpQueueCV_.notify_all();
	if (udpSenderThread_.joinable())
		udpSenderThread_.join();
}

void Aperture::StartPostProcessingThreads(int numThreads) {
	log_verbose("[StartPostProcessingThreads]");
    for (int i = 0; i < numThreads; ++i) {
		std::cout << "[PostThread] Thread " << i << " started." << std::endl;

        postProcessingThreads_.emplace_back([this, i]() {
            pthread_setname_np(pthread_self(), ("PostProc" + std::to_string(i)).c_str());
			cpu_set_t cpuset;
			CPU_ZERO(&cpuset);
			//CPU_SET(0, &cpuset);  // Pin to CPU core 2
			CPU_SET(0 + (i % 4), &cpuset);  // Spread across cores 1–3
			//pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
			pthread_setname_np(pthread_self(), "Post");
            while (running_) {
                std::function<void()> job;

                {
                    std::unique_lock lock(postProcessingQueueMutex_);
                    postProcessingQueueCV_.wait(lock, [&]() {
                        return !postProcessingQueue_.empty() || !running_;
                    });

                    if (!running_ && postProcessingQueue_.empty())
                        return;

                    job = std::move(postProcessingQueue_.front());
                    postProcessingQueue_.pop();
                }
				try {
					std::cout << "[PostThread] Executing job..." << std::endl;
					job();
					std::cout << "[PostThread] Job completed" << std::endl;
				} catch (const std::exception& e) {
					std::cerr << "[PostThread] Exception in job: " << e.what() << std::endl;
				} catch (...) {
					std::cerr << "[PostThread] Unknown exception in job!" << std::endl;
				}
            }
        });
    }
}

void Aperture::StopPostProcessingThreads() {
	log_verbose("[Aperture::StopPostProcessingThreads]");
    {
        std::lock_guard lock(postProcessingQueueMutex_);
        running_ = false;
    }
    postProcessingQueueCV_.notify_all();

    for (auto& t : postProcessingThreads_)
        if (t.joinable()) t.join();
}

/// Apply white balance gains to a RAW16 Bayer BGGR mosaic in-place.
/// Layout BGGR:
///   row 0: B G B G ...
///   row 1: G R G R ...
///   row 2: B G B G ...
///   row 3: G R G R ...
///
/// data.size() must be width*height.
//inline void Aperture::ApplyWhiteBalanceToMosaic_BGGR(std::vector<uint8_t>& data,
                                           //int width, int height,
                                           //double rGain, double gGain, double bGain)
//{
    //if (width <= 0 || height <= 0 || static_cast<size_t>(width) * height != data.size())
        //return; // or throw

    //const uint32_t maxv = 65535u;

    //for (int y = 0; y < height; ++y) {
        //const bool evenRow = (y & 1) == 0;
        //uint8_t* row = data.data() + static_cast<size_t>(y) * width;

        //for (int x = 0; x < width; ++x) {
            //const bool evenCol = (x & 1) == 0;

            //double gain;
            //if (evenRow) {
                //// row: B G B G ...
                //gain = evenCol ? bGain : gGain;
            //} else {
                //// row: G R G R ...
                //gain = evenCol ? gGain : rGain;
            //}

            //uint32_t v = row[x];
            //v = static_cast<uint32_t>(std::lround(v * gain));
            //if (v > maxv) v = maxv;
            //row[x] = static_cast<uint8_t>(v);
        //}
    //}
//}

//inline void Aperture::ApplyWhiteBalanceToMosaic_BGGR(std::span<uint16_t> data,
                                           //int width, int height,
                                           //double rGain, double gGain, double bGain)
//{
    //if (width <= 0 || height <= 0 ||
        //data.size() != static_cast<size_t>(width) * height) return; // or throw

    //const uint32_t maxv = 255u;
    //for (int y = 0; y < height; ++y) {
        //const bool evenRow = (y & 1) == 0;
        //uint16_t* row = data.data() + static_cast<size_t>(y) * width;

        //for (int x = 0; x < width; ++x) {
            //const bool evenCol = (x & 1) == 0;
            //double gain = (evenRow ? (evenCol ? bGain : gGain)
                                   //: (evenCol ? gGain : rGain));
            //uint32_t v = row[x];
            //v = static_cast<uint32_t>(std::lround(v * gain));
            //if (v > maxv) v = maxv;
            //row[x] = static_cast<uint16_t>(v);
        //}
    //}
//}

//inline void Aperture::ApplyWhiteBalanceToMosaic_BGGR(
	//std::span<uint16_t> data,
	//int width, int height,
	//double rGain, double gGain, double bGain)
//{
	//// Validate sizes (elements == pixels)
	//const size_t elems = static_cast<size_t>(width) * static_cast<size_t>(height);
	//if (width <= 0 || height <= 0 || data.size() != elems * sizeof(uint16_t)) {
		//std::cerr << "[WB] size mismatch: data=" << data.size()
				  //<< " expected=" << elems << " (w=" << width << " h=" << height << ")\n";
		//return; // or throw
	//}

	//// Convert gains to fixed point (Q10: 1.0 == 1024)
	//auto toQ10 = [](double g) -> uint32_t {
		//if (g < 0.0) g = 0.0;
		//return static_cast<uint32_t>(std::lround(g * 1024.0));
	//};
	//const uint32_t rQ = toQ10(rGain);
	//const uint32_t gQ = toQ10(gGain);
	//const uint32_t bQ = toQ10(bGain);

	//// Saturating multiply: (v * gainQ + 512) >> 10, clamped to 65535
	//auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
		//uint64_t t = static_cast<uint64_t>(v) * gainQ + 512; // +0.5 for rounding
		//t >>= 10;
		//if (t > 65535u) t = 65535u;
		//return static_cast<uint16_t>(t);
	//};

	//for (int y = 0; y < height; ++y) {
		//const bool evenRow = (y & 1) == 0;
		//uint16_t* row = data.data() + static_cast<size_t>(y) * width;

		//for (int x = 0; x < width; ++x) {
			//const bool evenCol = (x & 1) == 0;

			//// BGGR pattern:
			//// row 0: B G B G ...
			//// row 1: G R G R ...
			//const uint32_t v = row[x];
			//uint16_t out;
			//if (evenRow) {
				//out = evenCol ? mulSatQ10(v, bQ) : mulSatQ10(v, gQ);
			//} else {
				//out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, rQ);
			//}
			//row[x] = out;
		//}
	//}
//}

inline void Aperture::ApplyWhiteBalanceToMosaic_BGGR(
    size_t region,
    const sf_t& sf,
	std::span<uint16_t> data,
	const GainMsg& gainMsg
){
	log_verbose("[Aperture::ApplyWhiteBalanceToMosaic_BGGR]");
	
	// Validate sizes (elements == pixels)
	const size_t elems = (region==0) ? sf_.nonOverlapFacet_t::size() : sf.size(region-1);
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
	const uint32_t rQ = toQ10(gainMsg.r_gain);
	const uint32_t gQ = toQ10(1.0);
	const uint32_t bQ = toQ10(gainMsg.b_gain);

	// Saturating multiply: (v * gainQ + 512) >> 10, clamped to 65535
	auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
		uint64_t t = static_cast<uint64_t>(v) * gainQ + 512; // +0.5 for rounding
		t >>= 10;
		if (t > 65535u) t = 65535u;
		return static_cast<uint16_t>(t);
	};
	//std::cout << "Check-Point 2" << std::endl;

	size_t maskedSize{(region==0) ? sf.nonOverlapFacet_t::size() : sf.indexLinearMaxs_[region-1]->size()};
	//std::cout << "maskedSize: " << maskedSize << std::endl;
	
	for (size_t i = 0; i < maskedSize; ++i) {
		sf_t::GlobalLinearShapeFunction_t::Index globalIndex{
			(region==0)	? (*sf.nonOverlapFacet_t::indexLinearMax_)[i].value() : (*sf.indexLinearMaxs_[region-1])[i].value()};		
		//std::cout << "globalIndex.value():" << globalIndex.value() << std::endl;
		sf_t::GlobalLinearShapeFunction_t::Point globalPt{sf_t::GlobalLinearShapeFunction_t::Transform(globalIndex)};
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
		if (evenRow) {
			out = evenCol ? mulSatQ10(v, bQ) : mulSatQ10(v, gQ);
		} else {
			out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, rQ);
		}
		data[i] = out;
	}
	//std::cout << "Check-Point 3" << std::endl;
}
	
std::string Aperture::GetBoardSerial()
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

