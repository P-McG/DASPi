//DASPi-programoptions.h
#pragma once
#include <string>
#include <vector>

namespace DASPi{

struct ProgramOptions {
    int nApertureComputeModules{0};
    std::string usbBaseIp;
    int framePort{0};
    bool reverseModuleOrder{false};
    std::vector<std::string> serverIps;

    // Optional text file:
    //   <moduleIndex> <yawDeg> <pitchDeg> <rollDeg>
    std::string cameraCalibrationPath;
};

}//DASPi
