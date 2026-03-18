#include <chrono>
#include <thread>
#include <execution>
#include <span>
#include <string>
#include "DASPi-logger.h"
#include "DASPi-aperture-peer.h"

namespace DASPi{
	
	template<size_t n>
	DASPi::AperturePeer<n>::AperturePeer(in_addr_t clntAddr,
										 int framePort,
										 int controlPort,
										 in_addr_t srvAddr)
		: sf_(),
		  sfdp_(this->sf_),
		  frameClnt_(clntAddr, framePort, srvAddr),
		  controlClnt_(clntAddr, controlPort, srvAddr)
	{
	}

	//template<size_t n>
	//AperturePeer<n>::AperturePeer(const in_addr_t clntAddr, const int port, const in_addr_t srvAddr)
	    //:sf_{sf_t()},
	    //sfdp_{sfdp_t(sf_)},
	    //udpClnt_{UDPClnt(clntAddr, port, srvAddr)}
	//{
	    //log_verbose("[AperturePeer]");
	    		        
	    //for(size_t i = 0; i < n_ + 1; i++){
	        //std::string fileName{ "output-" +  inAddrTToString(srvAddr) + "_" + std::to_string(i) + ".bayer"};
	        //files_[i]=std::make_unique<std::ofstream>( fileName, std::ios::binary );
	    //}
	
	    //log_verbose("[AperturePeer - End]");
	//}
	
	//template<size_t n>
	//AperturePeer<n>::~AperturePeer(){
	     //log_verbose("~AperturePeer");
	//}
	
	template<size_t n>
	std::string AperturePeer<n>::inAddrTToString(in_addr_t ip) {
	    char ipStr[INET_ADDRSTRLEN];
	    struct in_addr addr;
	    addr.s_addr = ip;
	
	    if (!inet_ntop(AF_INET, &addr, ipStr, INET_ADDRSTRLEN)) {
	        perror("inet_ntop failed");
	        return "INVALID";
	    }
	
	    return std::string(ipStr);
	}
	
	////RequestConnection
	///*
	 //* This allows the server to gather the Aperture clients network address
	 //*/
	//bool AperturePeer::RequestConnection(){
	    //udpClnt_.TransmissionRequest();
	    //return true;
	//}        
	     
	template<size_t n>
	bool AperturePeer<n>::RunFrameLoop()
	{
		log_verbose("ReceiveApertureCapture");
	
		std::vector<uint16_t> maskedBuffer(this->sfdp_.size());
	
		UDPClnt::EpollData epollData;
		if (!frameClnt_.InitEpollForSrvUDPPackets(epollData)) {
			std::cerr << "Initializing Epoll for Server UDP Packets\n";
			return false;
		}
	
		if (!frameClnt_.ReadDataFromSrvUDPPackets(epollData, maskedBuffer)) {
			std::cerr << "Read data from server FAILED\n";
			return false;
		}
	
		if (!frameClnt_.FinalizeEpollForSrvUDPPackets(epollData)) {
			std::cerr << "Finalize Epoll for Server UDP Packets\n";
			return false;
		}
	
		// rest of your existing image pipeline unchanged
		this->sfdp_.LoadToContiguousMemory(maskedBuffer);
	
		auto unmasked0{this->sf_.sf_t::nonOverlapFacet_t::FrameBufferUnmask(this->sfdp_[0])};
		this->buffer_[0].resize(unmasked0.size());
		std::memcpy(this->buffer_[0].data(), unmasked0.data(), unmasked0.size() * sizeof(uint16_t));
	
		for (size_t i = 0; i < n_; ++i) {
			auto unmasked{this->sf_.FrameBufferUnmask(this->sfdp_[i + 1], i)};
			this->buffer_[i + 1].resize(unmasked.size());
			std::memcpy(this->buffer_[i + 1].data(), unmasked.data(), unmasked.size() * sizeof(uint16_t));
		}
	
		this->BrightenImageInplace(std::span<uint16_t>(this->buffer_[0]), 6);
		for (size_t i = 0; i < n_; ++i) {
			BrightenImageInplace(std::span<uint16_t>(this->buffer_[i + 1]), 6);
		}
	
		this->BufferToFile();
		return true;
	}
		 
	//template<size_t n>   
	//bool AperturePeer<n>::ReceiveApertureCapture() {
	    //log_verbose("ReceiveApertureCapture");
	
	    //std::vector<uint16_t> maskedBuffer(sfdp_.size());
	    
	    //UDPClnt::EpollData epollData;
	    //if (!udpClnt_.InitEpollForSrvUDPPackets(epollData)) {
	        //std::cerr << "Initializing Epoll for Server UDP Packets" << std::endl;
	        //return false;//exit(1);
	    //}
	    
	    ////for(int i=0; i<1; i++){
	        //if (!udpClnt_.ReadDataFromSrvUDPPackets(epollData, maskedBuffer)) {
	            //std::cerr << "Read data from server FAILED" << std::endl;
	            //return false; //exit(1);
	        //}
	        //std::cout << "maskedBuffer.data() address: " << static_cast<void*>(maskedBuffer.data()) << std::endl;
	        //std::cout << "maskedBuffer.size() = " << maskedBuffer.size() << std::endl;
	        //std::cout << "maskedBuffer.capacity() = " << maskedBuffer.capacity() << std::endl;
	   //// }
	    
	    //if (!udpClnt_.FinalizeEpollForSrvUDPPackets(epollData)) {
	        //std::cerr << "Finalize Epoll for Server UDP Packets" << std::endl;
	        //return false;//exit(1);
	    //}
	    
	    //log_verbose("check maskedBuffer");
	
	    //if (maskedBuffer.size() == 0 || maskedBuffer.size() % sizeof(uint16_t) != 0) {
	        //log_verbose("maskedBuffer.size() = " + std::to_string(maskedBuffer.size()));
	        //return false;
	    //}
	    
	    //// 🔥 Check pointer alignment!
	    //if (reinterpret_cast<uintptr_t>(maskedBuffer.data()) % alignof(uint16_t) != 0) {
	        //std::cerr << "maskedBuffer data not properly aligned!" << std::endl;
	        //return false;
	    //}
	    
	    //// 🔥 Check expected size!
	    //std::cout << "sfdp_.size()" << sfdp_.size() << std::flush;//TROUBLESHOOTING
	    //size_t expected_bytes = sfdp_.size()/* * sizeof(uint16_t)*/;
	
	    //if (maskedBuffer.size() != expected_bytes) {
	        //std::cerr << "maskedBuffer wrong size: " << maskedBuffer.size()
	                  //<< ", expected: " << expected_bytes << std::endl;
	        //return false;
	    //}
	    
	    ////// 🔥 Safe to cast now
	    ////// Note:
	    ////// -maskedBuffer is of char
	    ////// -inputData is of uint16_t
	    ////auto* ptr = reinterpret_cast<uint16_t*>(maskedBuffer.data());
	    ////std::span<uint16_t> inputData(ptr, maskedBuffer.size()/sizeof(uint16_t));
	    
	    ////// Note: 
	    ////// -buffer_ is of char
	    ////// -GlobalLinearShapeFunction is of uint16_t.
	    ////buffer_.resize(sf_.GlobalLinearShapeFunction::size() * sizeof(uint16_t));
	
	
	////#ifdef MASKED
	    ////log_verbose("Masked Image");
	    ////if(sf_.Facet::size() != inputData.size()// inputData is of uint16_t
	       ////|| sf_.GlobalLinearShapeFunction::size() != buffer_.size()/sizeof(uint16_t)// buffer is of char
	    ////){
	        ////std::cerr << "Incorrect inputData size: " << inputData.size() << std::endl;
	        ////std::cerr << "Or, Incorrect buffer_ size: " << buffer_.size() << std::endl;
	        ////std::cerr << " for Equilateral Triangular Shape Function:  " << sf_.Facet::size() << std::endl;
	        ////exit(1);
	    ////}
	    ////auto unmasked = sf_.Facet::FrameBufferUnmask(inputData);
	    ////std::memcpy(buffer_.data(), unmasked.data(), sf_.GlobalLinearShapeFunction::size() * sizeof(uint16_t));
	////#else
	    ////if(sf_.GlobalLinearShapeFunction::size() != inputData.size()// inputData is of uint16_t
	       ////|| sf_.GlobalLinearShapeFunction::size() != buffer_.size()/sizeof(uint16_t)// buffer is of char
	    ////){
	        ////std::cerr << "Incorrect inputData size: " << inputData.size() << std::endl;
	        ////std::cerr << "Or, Incorrect buffer_ size: " << buffer_.size() << std::endl;
	        ////std::cerr << " for Global Linear Shape Function:  " << sf_.GlobalLinearShapeFunction::size() << std::endl;
	        ////exit(1);
	    ////}
	    ////std::cout << "memcpy - start" << std::endl;
	    
	    ////std::memcpy(buffer_.data(), inputData.data(), sf_.GlobalLinearShapeFunction::size() * sizeof(uint16_t));
	    ////std::cout << "memcpy - end" << std::endl;
	
	////#endif
	
	    ////sfdp_.LoadUint8tToContiguousMemory(maskedBuffer);
	    //sfdp_.LoadToContiguousMemory(maskedBuffer);
    
	    ////Post operations
		//auto unmasked{sf_.sf_t::nonOverlapFacet_t::FrameBufferUnmask(sfdp_[0])};
		//buffer_[0].resize( unmasked.size());// Note buffer is uint16_t
	    //std::memcpy(buffer_[0].data(), unmasked.data(), /*sf_.sf_t::GlobalLinearShapeFunction_t::size()*/ unmasked.size() * sizeof(uint16_t));
		
		//for(size_t i = 0; i < n_; i++){
			//auto unmasked{sf_.FrameBufferUnmask(sfdp_[i+1], i)};
			////buffer_[i+1].resize( sf_.sf_t::GlobalLinearShapeFunction_t::size());// Note buffer is uint16_t
			//buffer_[i+1].resize( unmasked.size());// Note buffer is uint16_t
		    //std::memcpy(buffer_[i+1].data(), unmasked.data(), /*sf_.sf_t::GlobalLinearShapeFunction_t::size()*/ unmasked.size() * sizeof(uint16_t));
		//}
		
		//// The following is for viewing output only.
		//BrightenImageInplace(std::span<uint16_t>(buffer_[0]), /*shift*/ 6);
		//for(size_t i = 0; i < n_; i++){
			//BrightenImageInplace(std::span<uint16_t>(buffer_[i+1]), /*shift*/ 6);
		//}
	
	    //log_verbose("BufferToFile");
	    //BufferToFile();
	    //log_verbose("Finished saving to file");
	    
	    //return true;
	//}
	
	template<size_t n>
	bool AperturePeer<n>::BufferToFile() {
	    log_verbose("[AperturePeer::BufferToFile]");
	       
	    for(size_t i=0; i<n_+1; i++){
			log_verbose("Writing file:" + std::to_string(i)) ;
	        if (!this->files_[i]->write( reinterpret_cast<const char*>(this->buffer_[i].data()),this->sf_.sf_t::GlobalLinearShapeFunction_t::size()*sizeof(uint16_t))) {
	             std::cerr << "Failed to write to file" << std::endl;
	             return false;
	        }
	        this->files_[i]->close();
	    }
	    return true;
	}
	
	template<size_t n>
	int AperturePeer<n>::GetFramePort() {
		return frameClnt_.port_;
	}
	
	template<size_t n>
	int AperturePeer<n>::GetControlPort() {
		return controlClnt_.port_;
	}
	
	template<size_t n>
	std::vector<uint16_t> AperturePeer<n>::GenerateStripedBayerBGGRImage(int width, int height, int stripeWidth) {
	    std::vector<uint16_t> image(width * height);
	
	    for (int y = 0; y < height; ++y) {
	        for (int x = 0; x < width; ++x) {
	            int stripe = (x / stripeWidth) % 2;
	            uint16_t value = stripe == 0 ? 0x0000 : 0xFFFF;
	
	            int idx = y * width + x;
	            image[idx] = value;
	        }
	    }
	
	    return image;
	}
	
	//template<size_t n>
	//inline void AperturePeer<n>::ApplyWhiteBalanceToMosaic_BGGR(
		//std::span<uint16_t> data,
	    //int width, int height,
		//double rGain, double gGain, double bGain
	//)
	//{
	    //if (width <= 0 || height <= 0 ||
	        //data.size() != static_cast<size_t>(width) * height) return; // or throw
	
	    //const uint32_t maxv = 65535u;
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
	
	template<size_t n>
	inline void AperturePeer<n>::ApplyWhiteBalanceToMosaic_BGGR(
	    std::span<uint16_t> data,
	    int width, int height,
	    double rGain, double gGain, double bGain)
	{
	    // Validate sizes (elements == pixels)
	    const size_t elems = static_cast<size_t>(width) * static_cast<size_t>(height);
	    if (width <= 0 || height <= 0 || data.size() != elems * sizeof(uint16_t)) {
	        std::cerr << "[WB] size mismatch: data=" << data.size()
	                  << " expected=" << elems << " (w=" << width << " h=" << height << ")\n";
	        return; // or throw
	    }
	
	    // Convert gains to fixed point (Q10: 1.0 == 1024)
	    auto toQ10 = [](double g) -> uint32_t {
	        if (g < 0.0) g = 0.0;
	        return static_cast<uint32_t>(std::lround(g * 1024.0));
	    };
	    const uint32_t rQ = toQ10(rGain);
	    const uint32_t gQ = toQ10(gGain);
	    const uint32_t bQ = toQ10(bGain);
	
	    // Saturating multiply: (v * gainQ + 512) >> 10, clamped to 65535
	    auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
	        uint64_t t = static_cast<uint64_t>(v) * gainQ + 512; // +0.5 for rounding
	        t >>= 10;
	        if (t > 65535u) t = 65535u;
	        return static_cast<uint16_t>(t);
	    };
	
	    for (int y = 0; y < height; ++y) {
	        const bool evenRow = (y & 1) == 0;
	        uint16_t* row = data.data() + static_cast<size_t>(y) * width;
	
	        for (int x = 0; x < width; ++x) {
	            const bool evenCol = (x & 1) == 0;
	
	            // BGGR pattern:
	            // row 0: B G B G ...
	            // row 1: G R G R ...
	            const uint32_t v = row[x];
	            uint16_t out;
	            if (evenRow) {
	                out = evenCol ? mulSatQ10(v, bQ) : mulSatQ10(v, gQ);
	            } else {
	                out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, rQ);
	            }
	            row[x] = out;
	        }
	    }
	}
	
	
	//template<size_t n>
	//auto AperturePeer<n>::BrightenImage(std::vector<uint8_t> &&input){
		//log_verbose("[Aperture::BrightenImage2]");
		//{
	       //std::vector<std::thread> workers;
	        //const size_t total = input.size();      // total number of output pixels
	        //const size_t chunk = total / processingThreads_;
	        
	        //std::cout << "[Aperture::FrameBufferTransformation2] Branching threads" << std::endl;
	        //for (size_t i = 0; i < size_t(processingThreads_); ++i) {
	            //size_t start = i * chunk;
	            //size_t end = (i == size_t(processingThreads_) - 1) ? total : (i + 1) * chunk;
	        
	            //std::cout << "[Aperture::FrameBufferTransformation2] Starting Brighten thread" << std::endl;
	            //workers.emplace_back([&, i, start, end]() {
	                //std::cout << "[Aperture::FrameBufferTransformation2] Setting Brighten thread affinity" << std::endl;
	                
	                //cpu_set_t cpuset;
	                //CPU_ZERO(&cpuset);
	                //CPU_SET(processingThreads_, &cpuset);
	                ////pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
	                //pthread_setname_np(pthread_self(), "Brighten");
	                //ScopedTimer t(("BrightenImageChunked2Thread " + std::to_string(i)).c_str());
	        
	                //std::cout << "[Thread " << i << "] started, start=" << start << ", end=" << end << std::endl;
	        
	                //try {
	                    //// Write directly into the correct segment of finalResult
	                    //std::cout << "[Aperture::BrightenImage2" << std::endl;
	                    //BrightenImageChunked2_NEON(input.data(), start, end, 6/*IMAGE BRIGHTENING SHIFT VALUE*//*, 1.5, sensorWidthValue_*/);
	                //} catch (const std::exception& e) {
	                    //std::cerr << "[Thread " << i << "] exception: " << e.what() << std::endl;
	                //} catch (...) {
	                    //std::cerr << "[Thread " << i << "] unknown exception" << std::endl;
	                //}
	        
	                //std::cout << "[Thread " << i << "] completed" << std::endl;
	            //});
	        //}
	        //for (auto& t : workers) t.join();
	    //}
		//return std::move(input);
	//}
	
	template<size_t n>
	void AperturePeer<n>::BrightenImageInplace(std::span<uint16_t> buf, size_t shift)
	{
	    log_verbose("[AperturePeer::BrightenImageInplace]");
	    if (buf.empty()) return;
	
	    const size_t threads = std::max<size_t>(1, static_cast<size_t>(this->processingThreads_));
	    const size_t total   = buf.size();
	    const size_t chunk   = (total + threads - 1) / threads; // ceil
	
	    std::vector<std::thread> workers;
	    workers.reserve(threads);
	
	    for (size_t i = 0; i < threads; ++i) {
	        const size_t start = i * chunk;
	        if (start >= total) break;
	        const size_t end   = std::min(total, start + chunk);
	
	        workers.emplace_back([this, buf, start, end, shift, i]() {
	            pthread_setname_np(pthread_self(), "Brighten");
	            //ScopedTimer t(("BrightenImageChunked2Thread " + std::to_string(i)).c_str());
	            BrightenImageChunked2_NEON(buf.data(), start, end, shift);
	        });
	    }
	    for (auto &t : workers) t.join();
	}
	
	template<size_t n>
	void AperturePeer<n>::BrightenImageChunked2_NEON(uint16_t *buffer, size_t start, size_t end, size_t shift) {
	    log_verbose("[Aperture::BrightenImageChunked2_NEON]");
	    if (shift > 15) shift = 15;
	
	    const size_t count = end - start;
	    uint16_t* data = buffer + start;
	
	    const size_t simdWidth = 8;  // 8x uint8_t per 128-bit NEON register
	    size_t i = 0;
	
	    int16x8_t shiftAmount = vdupq_n_s16(static_cast<int16_t>(shift));
	
	    for (; i + simdWidth <= count; i += simdWidth) {
	        uint16x8_t pixels = vld1q_u16(data + i);           // load 8 pixels
	        uint16x8_t bright = vshlq_u16(pixels, shiftAmount); // shift left
	        vst1q_u16(data + i, bright);                       // store back
	    }
	
	    // Remainder loop (scalar)
	    for (; i < count; ++i) {
	        data[i] <<= shift;
	    }
	}
	
	template<size_t n>
	float DASPi::AperturePeer<n>::ComputeRequestedGain(const GainMsg& msg)
	{
		if (msg.mean_brightness <= 1.0e-6f) {
			return 1.0f;
		}
	
		float gain = msg.target_brightness / msg.mean_brightness;
		return std::clamp(gain, 1.0f, 16.0f);
	}
	
	template<size_t n>
	bool DASPi::AperturePeer<n>::RunControlLoop()
	{
		GainMsg msg{};
	
		ssize_t bytes = controlClnt_.ReceiveFromServer(msg);
	
		if (bytes < 0) {
			perror("ReceiveGainMsg recvfrom");
			return false;
		}
	
		if (bytes != sizeof(GainMsg)) {
			std::cerr << "ReceiveGainMsg: wrong size\n";
			return false;
		}
	
		if (msg.header.type != MessageType::GainMsg) {
			std::cerr << "ReceiveGainMsg: invalid type\n";
			return false;
		}
	
		HandleGainMsg(msg);
		return true;
	}
	
	template<size_t n>
	bool DASPi::AperturePeer<n>::SendGainReply(const GainReply& reply)
	{
		GainReply copy = reply;
	
		ssize_t sent = controlClnt_.SendToServer(&copy, sizeof(copy));
	
		if (sent < 0) {
			perror("SendGainReply sendto");
			return false;
		}
	
		return sent == sizeof(copy);
	}
	
	template<size_t n>
	void DASPi::AperturePeer<n>::HandleGainMsg(const GainMsg& msg)
	{
		GainReply reply{};
		reply.camera_id = msg.camera_id;
		reply.frame_id = msg.frame_id;
		reply.requested_gain = ComputeRequestedGain(msg);
		reply.status = 0;
	
		SendGainReply(reply);
	
		// Optional: store locally (for diagnostics / later use)
		{
			std::lock_guard<std::mutex> lock(gainMutex_);
			latestGain_ = reply.requested_gain;
			latestFrameId_ = reply.frame_id;
			gainValid_ = true;
		}
	}
	
	//template<size_t n>
	//void AperturePeer<n>::HandleGainMsg(const GainMsg& msg)
	//{
		//GainReply reply{};
		//reply.camera_id = msg.camera_id;
		//reply.frame_id = msg.frame_id;
		//reply.requested_gain = ComputeRequestedGain(msg);
		//reply.status = 0;
	
		//SendGainReply(reply);
	//}
	
};//ending namespace DASPi

