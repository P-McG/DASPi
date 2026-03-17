#pragma once

namespace DASPi{

	struct GainMsg {
	    uint32_t camera_id;  // Unique identifier for the camera-zero combination
	    uint32_t frame_id;
	    float    r_gain;           // raw AWB proposal
	    float    b_gain;
	    float    exposure_us;      // optional, useful for diagnostics
	    float    analogue_gain;    // optional
	};
};//namespace DASPi
