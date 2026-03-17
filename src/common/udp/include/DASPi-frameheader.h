#pragma once

#include <string>
#include "DASPi-gainmsg.h"

namespace DASPi{
	
	constexpr uint32_t MAGIC_NUMBER = 0xCAFEBABE;

    //#pragma pack(push, 1)
	struct FrameHeader {
		uint32_t magic_;         // Magic number to identify the frame
        GainMsg gainMsg;         // Info on the Frames image gain
	    uint32_t payloadSize_;   // Number of bytes in the following payload
	    uint32_t checksum_;      // Optional: CRC32 or simple sum (set to 0 if unused)
	};
	//#pragma pack(pop)

};//namespace DASPi
