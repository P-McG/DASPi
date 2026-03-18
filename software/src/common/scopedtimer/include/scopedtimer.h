#pragma once
#include <chrono>

namespace DASPi{
	//struct ScopedTimer {
	    //const char* label;
	    //std::chrono::high_resolution_clock::time_point start;
	
	    //ScopedTimer(const char* label)
	        //: label(label), start(std::chrono::high_resolution_clock::now()) {}
	
	    //~ScopedTimer() {
	        //auto end = std::chrono::high_resolution_clock::now();
	        //auto duration = std::chrono::duration<double, std::milli>(end - start).count();
	        //std::cout << "[" << label << "] " << duration << " ms" << std::endl;
	    //}
	//};
	
	struct ScopedTimer {
    std::string label;  // change from const char* to std::string
    std::chrono::high_resolution_clock::time_point start;

    ScopedTimer(std::string label)
        : label(std::move(label)), start(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << "[" << label << "] " << duration << " ms" << std::endl;
    }
};

};//ending namespace DASPi
