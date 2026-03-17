#pragma once

namespace DASPi{
	struct UdpFrame {
		uint64_t frameNumber;
		const uint8_t* data;
		size_t size;
	
	    UdpFrame(uint64_t frame, const uint8_t* ptr, size_t len)
        : frameNumber(frame), data(ptr), size(len) {};
   	};
};//namespace DASPi
