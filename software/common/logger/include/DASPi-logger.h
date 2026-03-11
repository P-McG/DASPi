#pragma once
	
#include <iostream>
#include <string>

namespace DASPi{
	enum class Verbosity { Quiet, Normal, Verbose, VeryVerbose };
//	inline Verbosity verbosity = Verbosity::Normal;
	inline Verbosity verbosity = Verbosity::Verbose;

	//enum class LogLevel { Silent, Error, Info, Debug };
	//LogLevel logLevel = LogLevel::Info;
	
	//void log(LogLevel level, const std::string& msg) {
	    //if (static_cast<int>(level) <= static_cast<int>(logLevel)) {
	        //std::cerr << msg << std::endl;
	    //}
	//}
		
    inline void log_verbose(const std::string& msg) {
        if (verbosity == Verbosity::Verbose) {
            std::cerr << "[verbose] " << msg << std::endl;
        }
    }
    
    //void log_veryVerbose(const std::string& msg) {
        //if (verbosity == Verbosity::VeryVerbose) {
            //std::cerr << "\033[1;34m[vverbose]\033[0m " << msg << std::endl;
        //}
    //}
}//ending namespace DASPi
