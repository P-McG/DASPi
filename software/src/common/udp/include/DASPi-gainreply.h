#pragma once

namespace DASPi{
	
	struct GainReply {
	    uint32_t camera_id; // Unique identifier for the camera-zero combination
	    uint32_t ref_frame_id;     // the frame whose proposal we normalized
	    float    r_gain_apply;     // normalized gains to apply
	    float    b_gain_apply;
	};
};//namespace DASPi
