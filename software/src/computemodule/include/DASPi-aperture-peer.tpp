#include <chrono>
#include <thread>
#include <execution>
#include <span>
#include <string>
#include <algorithm>
#include <utility>
#include "DASPi-logger.h"
#include "DASPi-udp-clnt.h"
#include "DASPi-aperture-peer.h"

namespace DASPi{
	
	template<size_t n>
	AperturePeer<n>::AperturePeer(in_addr_t clntAddr,
										 int clntFramePort,
										 int clntControlPort,
										 in_addr_t srvAddr,
										 int srvFramePort,
										 int srvControlPort)
		: sf_(),
		  sfdp_(this->sf_),
		  frameClnt_(clntAddr, clntFramePort, srvAddr, srvFramePort),
		  controlClnt_(clntAddr, clntControlPort, srvAddr, srvControlPort)
	{
		peerLabel_ = "srv=" + inAddrTToString(srvAddr) +
		             " srvFramePort=" + std::to_string(srvFramePort) +
		             " clntFramePort=" + std::to_string(clntFramePort);

		for (size_t i = 0; i < n + 1; ++i) {
			std::string name = "output-" +  inAddrTToString(srvAddr) + "_" + std::to_string(i) + ".bayer";
			files_[i] = std::make_unique<std::ofstream>(name, std::ios::binary);
			if (!files_[i] || !files_[i]->is_open()) {
				throw std::runtime_error("Failed to open " + name);
			}
		}
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
		log_verbose("[AperturePeer<n>::RunFrameLoop]");
		std::cout << "[RunFrameLoop] ENTER" << std::endl;
	
		FrameHeader frameHeader{};
		std::vector<uint16_t> maskedBuffer;
	
		UDPClnt::EpollData epollData;
		bool epollInitialized = false;
	
		auto finalizeEpoll = [&]() -> bool {
			if (!epollInitialized) {
				return true;
			}
			if (!frameClnt_.FinalizeEpollForSrvUDPPackets(epollData)) {
				std::cerr << "Finalize Epoll for Server UDP Packets\n";
				return false;
			}
			epollInitialized = false;
			return true;
		};
	
		if (!frameClnt_.InitEpollForSrvUDPPackets(epollData)) {
			std::cerr << "Initializing Epoll for Server UDP Packets\n";
			return false;
		}
		epollInitialized = true;
	
		std::cout << "[RF] after InitEpollForSrvUDPPackets" << std::endl;
		std::cout << "[RF] before ReceiveAndReassembleFramePacket" << std::endl;
	
		const bool ok = frameClnt_.ReceiveAndReassembleFramePacket(maskedBuffer, frameHeader);
	
		std::cout << "[RF] after ReceiveAndReassembleFramePacket ok=" << ok
				  << " maskedBuffer.size()=" << maskedBuffer.size()
				  << " payloadSize=" << frameHeader.payloadSize_
				  << std::endl;
	
		if (!ok) {
			std::cerr << "Read data from server FAILED\n";
			finalizeEpoll();
			return false;
		}
		
		this->fpsReceived_.Tick();
	
		if (!finalizeEpoll()) {
			return false;
		}
	
		this->sfdp_.ResetValidSizes();
	
		size_t offset = 0;
		for (size_t i = 0; i < NUM_REGIONS; ++i) {
			const size_t count = frameHeader.regionSizes_[i];
	
			if (count > this->sfdp_[i].size()) {
				std::cerr << "[RunFrameLoop] region overflow: i=" << i
						  << " count=" << count
						  << " capacity=" << this->sfdp_[i].size()
						  << std::endl;
				return false;
			}
	
			if (offset + count > maskedBuffer.size()) {
				std::cerr << "[RunFrameLoop] packed payload overflow: offset=" << offset
						  << " count=" << count
						  << " maskedBuffer.size()=" << maskedBuffer.size()
						  << std::endl;
				return false;
			}
	
			if (count != 0) {
				std::memcpy(this->sfdp_[i].data(),
							maskedBuffer.data() + offset,
							count * sizeof(uint16_t));
			}
	
			this->sfdp_.SetRegionValidSize(i, count);
			offset += count;
		}
	
		if (offset != maskedBuffer.size()) {
			std::cerr << "[RunFrameLoop] payload size mismatch after unpack: offset=" << offset
					  << " maskedBuffer.size()=" << maskedBuffer.size()
					  << std::endl;
			return false;
		}
	
		std::array<std::vector<uint16_t>, n + 1> newBuffers;
	
		auto unmasked0 =
			this->sf_.sf_t::nonOverlapFacet_t::FrameBufferUnmask(this->sfdp_[0]);
	
		newBuffers[0].resize(unmasked0.size());
		if (!unmasked0.empty()) {
			std::memcpy(newBuffers[0].data(),
						unmasked0.data(),
						unmasked0.size() * sizeof(uint16_t));
		}
	
		for (size_t i = 0; i < n_; ++i) {
			auto unmasked = this->sf_.FrameBufferUnmask(this->sfdp_[i + 1], i);
			newBuffers[i + 1].resize(unmasked.size());
			if (!unmasked.empty()) {
				std::memcpy(newBuffers[i + 1].data(),
							unmasked.data(),
							unmasked.size() * sizeof(uint16_t));
			}
		}

		auto looksBlank = [](const std::vector<uint16_t>& buf) {
			if (buf.empty()) return true;
			const size_t sampleCount = std::min<size_t>(buf.size(), 1024);
			for (size_t i = 0; i < sampleCount; ++i) {
				if (buf[i] != 0) return false;
			}
			return true;
		};

		const bool stream0Blank = looksBlank(newBuffers[0]);
		const bool stream1Blank = looksBlank(newBuffers[1]);
		if (stream0Blank && !stream1Blank) {
			std::cout << "[RunFrameLoop warning] " << peerLabel_
			          << " stream0 appears blank while stream1 has signal;"
			          << " keeping original stream mapping (no swap)"
			          << std::endl;
		}

		if (!newBuffers[0].empty() && newBuffers[1].empty()) {
			std::cout << "[RunFrameLoop trace] " << peerLabel_
			          << " stream1 empty while stream0 has data"
			          << " regionSize[0]=" << frameHeader.regionSizes_[0]
			          << " regionSize[1]=" << frameHeader.regionSizes_[1]
			          << " buffer0=" << newBuffers[0].size()
			          << " buffer1=" << newBuffers[1].size()
			          << std::endl;
		}
	
		//for (size_t i = 0; i < n_ + 1; ++i) {
			//this->BrightenImageInplace(std::span<uint16_t>(newBuffers[i]), 6);
		//}
	
		{
			std::scoped_lock lock(bufferMutex_);
			this->buffer_ = std::move(newBuffers);
		}
		
		this->fpsPublished_.Tick();
	
		std::cout << "[RunFrameLoop] published buffers" << std::endl;
		for (size_t i = 0; i < n_ + 1; ++i) {
			std::cout << "[RunFrameLoop] buffer_[" << i << "].size()="
					  << this->buffer_[i].size() << std::endl;
		}
	
		std::cout << "[RunFrameLoop] about to BufferToFile" << std::endl;
		if (!this->BufferToFile()) {
			std::cerr << "[RunFrameLoop] BufferToFile failed" << std::endl;
			return false;
		}
		
		this->fpsRunFrameLoop_.Tick();
	
		std::cout << "[RunFrameLoop] completed" << std::endl;
		return true;
	}
	
	template<size_t n>
	bool AperturePeer<n>::BufferToFile()
	{
		log_verbose("[AperturePeer::BufferToFile]");
		
		std::scoped_lock lock(bufferMutex_);
		++bufferToFileCount_;
		const bool shouldLogBuffer = ((bufferToFileCount_ % 30) == 0);

		for (size_t i = 0; i < n_ + 1; ++i) {
			log_verbose("Writing file:" + std::to_string(i));
	
			if (shouldLogBuffer) {
				std::cout << "[BufferToFile] " << peerLabel_
				          << " i=" << i
						  << " files_[i]=" << files_[i].get()
						  << " buffer_[i].size()=" << buffer_[i].size()
						  << std::endl;
			}
	
			if (!files_[i]) {
				std::cerr << "[BufferToFile] files_[" << i << "] is null\n";
				return false;
			}
	
			if (!files_[i]->is_open()) {
				std::cerr << "[BufferToFile] files_[" << i << "] is not open\n";
				return false;
			}
	
			const size_t bytesToWrite = this->buffer_[i].size() * sizeof(uint16_t);
			if (bytesToWrite == 0) {
				if (i == 1 && !this->buffer_[0].empty()) {
					std::cout << "[BufferToFile trace] " << peerLabel_
					          << " output *_1.bayer empty because buffer_[1] is empty"
					          << " while buffer_[0] has " << this->buffer_[0].size() << " pixels"
					          << std::endl;
				}
				std::cerr << "[BufferToFile] buffer_[" << i << "] is empty, skipping\n";
				continue;
			}

			if ((i <= 1) && shouldLogBuffer) {
				const auto& buf = this->buffer_[i];
				std::uint64_t sig = 1469598103934665603ull; // FNV offset basis
				const size_t sampleCount = std::min<size_t>(buf.size(), 1024);
				for (size_t j = 0; j < sampleCount; ++j) {
					sig ^= static_cast<std::uint64_t>(buf[j]);
					sig *= 1099511628211ull; // FNV prime
				}
				std::cout << "[BufferSig] " << peerLabel_
				          << " stream=" << i
				          << " sampleCount=" << sampleCount
				          << " fnv64=" << sig
				          << " first=" << buf.front()
				          << " last=" << buf.back()
				          << std::endl;
			}

			if (!this->files_[i]->write(
					reinterpret_cast<const char*>(this->buffer_[i].data()),
					static_cast<std::streamsize>(bytesToWrite))) {
				std::cerr << "[BufferToFile] Failed to write file " << i << std::endl;
				return false;
			}
	
			this->files_[i]->flush();
		}
	
		return true;
	}
	
	template<size_t n>
	bool AperturePeer<n>::CopyBuffer(size_t index, std::vector<uint16_t>& out) const
	{
		log_verbose("[AperturePeer::CopyBuffer]");
	
		if (index >= n_ + 1) {
			std::cerr << "[CopyBuffer] index out of range: " << index
					  << " (max valid: " << n_ << ")\n";
			return false;
		}
	
		std::scoped_lock lock(bufferMutex_);
	
		if (buffer_[index].empty()) {
			std::cerr << "[CopyBuffer] buffer_[" << index << "] is empty\n";
			return false;
		}
	
		out = buffer_[index];
		return true;
	}
	
	template<size_t n>
	int AperturePeer<n>::GetFramePort() {
		return frameClnt_.clntPort_;
	}
	
	template<size_t n>
	int AperturePeer<n>::GetControlPort() {
		return controlClnt_.clntPort_;
	}
	
	//template<size_t n>
	//std::vector<uint16_t> AperturePeer<n>::GenerateStripedBayerBGGRImage(int width, int height, int stripeWidth) {
	    //std::vector<uint16_t> image(width * height);
	
	    //for (int y = 0; y < height; ++y) {
	        //for (int x = 0; x < width; ++x) {
	            //int stripe = (x / stripeWidth) % 2;
	            //uint16_t value = stripe == 0 ? 0x0000 : 0xFFFF;
	
	            //int idx = y * width + x;
	            //image[idx] = value;
	        //}
	    //}
	
	    //return image;
	//}
	
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
	
	//template<size_t n>
	//inline void AperturePeer<n>::ApplyWhiteBalanceToMosaic_BGGR(
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
	
	//template<size_t n>
	//inline void Aperture<n>::ApplyWhiteBalanceToMosaic_RGGB(
		//size_t region,
		//const sf_t& sf,
		//std::span<uint16_t> data,
		//const GainMsg& gainMsg
	//){
		//log_verbose("[Aperture::ApplyWhiteBalanceToMosaic_RGGB]");
	
		//const size_t elems = (region == 0) ? sf_.sf_t::nonOverlapFacet_t::size()
										   //: sf.size(region - 1);
		//if (data.size() != elems) {
			//std::cerr << "[WB] size mismatch: data=" << data.size()
					  //<< " expected=" << elems << "\n";
			//std::exit(EXIT_FAILURE);
		//}
	
		//auto toQ10 = [](double g) -> uint32_t {
			//if (g < 0.0) g = 0.0;
			//return static_cast<uint32_t>(std::lround(g * 1024.0));
		//};
	
		//const uint32_t rQ = toQ10(gainMsg.r_gain_apply);
		//const uint32_t gQ = toQ10(1.0);
		//const uint32_t bQ = toQ10(gainMsg.b_gain_apply);
	
		//auto mulSatQ10 = [](uint32_t v, uint32_t gainQ) -> uint16_t {
			//uint64_t t = static_cast<uint64_t>(v) * gainQ + 512;
			//t >>= 10;
			//if (t > 65535u) t = 65535u;
			//return static_cast<uint16_t>(t);
		//};
	
		//const size_t maskedSize = (region == 0)
			//? sf_.sf_t::nonOverlapFacet_t::size()
			//: sf.indexLinearMaxs_[region - 1]->size();
	
		//for (size_t i = 0; i < maskedSize; ++i) {
			//typename sf_t::GlobalLinearShapeFunction_t::Index globalIndex{
				//(region == 0)
					//? (*sf_.sf_t::nonOverlapFacet_t::indexLinearMax_)[i].value()
					//: (*sf.indexLinearMaxs_[region - 1])[i].value()
			//};
	
			//typename sf_t::GlobalLinearShapeFunction_t::Point globalPt{
				//sf_t::GlobalLinearShapeFunction_t::Transform(globalIndex)
			//};
	
			//const bool evenRow = (globalPt.y_ & 1) == 0;
			//const bool evenCol = (globalPt.x_ & 1) == 0;
	
			//// RGGB pattern:
			//// row 0: R G R G ...
			//// row 1: G B G B ...
			//const uint32_t v = data[i];
			//uint16_t out;
			//if (evenRow) {
				//out = evenCol ? mulSatQ10(v, rQ) : mulSatQ10(v, gQ);
			//} else {
				//out = evenCol ? mulSatQ10(v, gQ) : mulSatQ10(v, bQ);
			//}
	
			//data[i] = out;
		//}
	//}
	
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
	
	//template<size_t n>
	//void AperturePeer<n>::BrightenImageInplace(std::span<uint16_t> buf, size_t shift)
	//{
	    //log_verbose("[AperturePeer::BrightenImageInplace]");
	    //if (buf.empty()) return;
	
	    //const size_t threads = std::max<size_t>(1, static_cast<size_t>(this->processingThreads_));
	    //const size_t total   = buf.size();
	    //const size_t chunk   = (total + threads - 1) / threads; // ceil
		
	    //std::vector<std::thread> workers;
	    //workers.reserve(threads);
	
	    //for (size_t i = 0; i < threads; ++i) {
	        //const size_t start = i * chunk;
	        //if (start >= total) break;
	        //const size_t end   = std::min(total, start + chunk);
	
	        //workers.emplace_back([this, buf, start, end, shift, i]() {
	            //pthread_setname_np(pthread_self(), "Brighten");
	            ////ScopedTimer t(("BrightenImageChunked2Thread " + std::to_string(i)).c_str());
//#if defined(__ARM_NEON) || defined(__aarch64__)
	            //this->BrightenImageChunked2_NEON(buf.data(), start, end, shift);
//#else
	            //this->BrightenImageChunked2(buf.data(), start, end, shift);
//#endif
	        //});
	    //}
	    //for (auto &t : workers) t.join();
	//}
	
	
	//template<size_t n>
	//void AperturePeer<n>::BrightenImageChunked2(uint16_t* buffer, size_t start, size_t end, size_t shift)
	//{
		//log_verbose("[Aperture::BrightenImageChunked2]");
	
		//if (!buffer || start >= end) return;
		//if (shift > 15) shift = 15;
	
//#if defined(__ARM_NEON) || defined(__aarch64__)
		//BrightenImageChunked2_NEON(buffer, start, end, shift);
//#else
		//uint16_t* data = buffer + start;
		//const size_t count = end - start;
	
		//for (size_t i = 0; i < count; ++i) {
			//data[i] <<= shift;
		//}
//#endif
//}
	
//#if defined(__ARM_NEON) || defined(__aarch64__)	
	//template<size_t n>
	//void AperturePeer<n>::BrightenImageChunked2_NEON(uint16_t *buffer, size_t start, size_t end, size_t shift) {
	    //log_verbose("[Aperture::BrightenImageChunked2_NEON]");
	    //if (shift > 15) shift = 15;
	
	    //const size_t count = end - start;
	    //uint16_t* data = buffer + start;
	
	    //const size_t simdWidth = 8;  // 8x uint8_t per 128-bit NEON register
	    //size_t i = 0;
	
	    //int16x8_t shiftAmount = vdupq_n_s16(static_cast<int16_t>(shift));
	
	    //for (; i + simdWidth <= count; i += simdWidth) {
	        //uint16x8_t pixels = vld1q_u16(data + i);           // load 8 pixels
	        //uint16x8_t bright = vshlq_u16(pixels, shiftAmount); // shift left
	        //vst1q_u16(data + i, bright);                       // store back
	    //}
	
	    //// Remainder loop (scalar)
	    //for (; i < count; ++i) {
	        //data[i] <<= shift;
	    //}
	//}
//#endif
	
	template<size_t n>
	float AperturePeer<n>::ComputeRequestedGain(const GainMsg& msg)
	{
		if (msg.mean_brightness <= 1.0e-6f) {
			return 1.0f;
		}
	
		float gain = msg.target_brightness / msg.mean_brightness;
		return std::clamp(gain, 1.0f, 16.0f);
	}
	
	template<size_t n>
	bool AperturePeer<n>::RunControlLoop()
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
	bool AperturePeer<n>::SendGainReply(const GainReply& reply)
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
	void AperturePeer<n>::HandleGainMsg(const GainMsg& msg)
	{
		GainReply reply{};
		reply.camera_id = msg.camera_id;
		reply.frame_id = msg.frame_id;
	
		reply.requested_gain = ComputeRequestedGain(msg);
		reply.r_gain_apply = reply.requested_gain;
		reply.b_gain_apply = reply.requested_gain;
		reply.status = 0;
	
		SendGainReply(reply);
	
		{
			std::lock_guard<std::mutex> lock(gainMutex_);
			latestRGainApply_ = reply.r_gain_apply;
			latestBGainApply_ = reply.b_gain_apply;
			latestFrameId_ = reply.frame_id;
			gainValid_ = true;
		}
		
		std::cout << "[GainReply] camera_id=" << reply.camera_id
          << " frame_id=" << reply.frame_id
          << " requested_gain=" << reply.requested_gain
          << " r_apply=" << reply.r_gain_apply
          << " b_apply=" << reply.b_gain_apply
          << std::endl;
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
	
	//template<size_t n> bool AperturePeer<n>::receivePeerFrame(AperturePeer<N>& peer, std::vector<std::uint16_t>& outBayer){
	//}
	
	template<size_t n>
	bool AperturePeer<n>::StitchWithPeer(AperturePeer<n>& other,
										 size_t sharedSideThis,
										 size_t sharedSideOther,
										 std::vector<uint16_t>& out,
										 bool reverseOther)
	{
		log_verbose("[AperturePeer::StitchWithPeer]");
	
		// --- Determine global output size ---
		const auto* nonOverlapMap = sf_.sf_t::nonOverlapFacet_t::indexLinearMax_.get();
		if (!nonOverlapMap) {
			std::cerr << "StitchWithPeer: null nonOverlap map\n";
			return false;
		}
	
		const size_t globalSize = nonOverlapMap->size();
	
		if (out.size() != globalSize) {
			out.assign(globalSize, 0);
		} else {
			std::fill(out.begin(), out.end(), uint16_t{0});
		}
	
		// --- 1. Scatter THIS non-overlap ---
		sf_.sf_t::nonOverlapFacet_t::FrameBufferScatterTo(
			sfdp_[0],
			nonOverlapMap,
			out.data(),
			out.size());
	
		// --- 2. Scatter OTHER non-overlap ---
		const auto* otherMap =
			other.sf_.sf_t::nonOverlapFacet_t::indexLinearMax_.get();
	
		if (!otherMap) {
			std::cerr << "StitchWithPeer: other nonOverlap map null\n";
			return false;
		}
	
		other.sf_.sf_t::nonOverlapFacet_t::FrameBufferScatterTo(
			other.sfdp_[0],
			otherMap,
			out.data(),
			out.size());
	
		// --- 3. Blend ONLY the shared overlap side ---
		const auto& blockA = sfdp_[sharedSideThis + 1];
		const auto& blockB = other.sfdp_[sharedSideOther + 1];
	
		if (blockA.size() != blockB.size()) {
			std::cerr << "StitchWithPeer: overlap size mismatch\n";
			return false;
		}
	
		const auto* mapA = sf_.indexLinearMaxs_[sharedSideThis].get();
		const auto* mapB = other.sf_.indexLinearMaxs_[sharedSideOther].get();
	
		if (!mapA || !mapB) {
			std::cerr << "StitchWithPeer: null overlap maps\n";
			return false;
		}
	
		const size_t count = blockA.size();
	
		for (size_t i = 0; i < count; ++i) {
			const size_t j = reverseOther ? (count - 1 - i) : i;
	
			const size_t gA = (*mapA)[i].value();
			const size_t gB = (*mapB)[j].value();
	
			if (gA >= out.size() || gB >= out.size()) {
				std::cerr << "StitchWithPeer: global index OOB\n";
				continue;
			}
	
			// Debug (keep temporarily)
			if (gA != gB) {
				std::cerr << "[StitchWithPeer] seam mismatch "
						  << "i=" << i
						  << " gA=" << gA
						  << " gB=" << gB
						  << std::endl;
			}
	
			const uint32_t a = static_cast<uint32_t>(blockA[i]);
			const uint32_t b = static_cast<uint32_t>(blockB[j]);
	
			// Simple average (fast)
			out[gA] = static_cast<uint16_t>((a + b) >> 1);
		}
	
		return true;
	}
	
template<size_t n>
cv::Mat AperturePeer<n>::BuildValidMask(size_t regionIndex)
{
    if (regionIndex >= n + 1) {
        throw std::out_of_range("BuildValidMask regionIndex out of range");
    }

    std::vector<uint16_t> unmasked;

    if (regionIndex == 0) {
        std::vector<uint16_t> compact(
            this->sf_.sf_t::nonOverlapFacet_t::size(),
            static_cast<uint16_t>(1));

        unmasked = this->sf_.sf_t::nonOverlapFacet_t::FrameBufferUnmask(compact);
    } else {
        const size_t overlapIdx = regionIndex - 1;

        std::vector<uint16_t> compact(
            this->sf_.size(overlapIdx),
            static_cast<uint16_t>(1));

        unmasked = this->sf_.FrameBufferUnmask(compact, overlapIdx);
    }

    if (unmasked.size() != sensorWidthValue_ * sensorHeightValue_) {
        throw std::runtime_error("BuildValidMask: unexpected unmasked buffer size");
    }

    cv::Mat mask(static_cast<int>(sensorHeightValue_),
                 static_cast<int>(sensorWidthValue_),
                 CV_8UC1,
                 cv::Scalar(0));

    for (size_t i = 0; i < unmasked.size(); ++i) {
        mask.data[i] = (unmasked[i] != 0)
                         ? static_cast<uint8_t>(255)
                         : static_cast<uint8_t>(0);
    }

    return mask;
}

template<size_t n>
bool AperturePeer<n>::CopyValidMask(size_t regionIndex, cv::Mat& out)
{
    if (regionIndex >= n + 1) {
        return false;
    }

    out = BuildValidMask(regionIndex);
    return !out.empty();
}
					  
};//ending namespace DASPi
