#include <array>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <chrono>
//#include "DASPi-globallinearshapefunction.h"
//#include "DASPi-equilateraltriangularshapefunction.h"
//#include "DASPi-hexigonalshapefunction.h"
#include "DASPi-logger.h"
#include "DASPi-overlapshapefunction.h"
#define VERBATIUM_COUT

using namespace DASPi;

template<typename framebuffer_t>
void WriteToFile(framebuffer_t &framebuffer, std::string fileName){
	log_verbose("[main::WriteToFile]");
	std::ofstream outputFile(fileName, std::ios::binary);

	// Write the mapped data to the file
	const auto *dataText = reinterpret_cast<const char *>(framebuffer.data());//OG
	
	std::cout << "Write size: " << framebuffer.size() * sizeof(uint16_t) << std::endl;
	outputFile.write(dataText, framebuffer.size() * sizeof(uint16_t));
	
	if (!outputFile) {
		std::cerr << "Failed to write to file" << std::endl;
	}
	else {
		std::cout << "Success writing to file" << std::endl;
	}
}

std::vector<uint16_t> GenerateStripedBayerBGGRImage(int width, int height, int stripeWidth) {
    log_verbose("[main::GenerateStripedBayerBGGRImage]");

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

static constexpr size_t n{3};
static constexpr size_t sensorWidth{1536};
static constexpr size_t sensorHeight{864};

void MaskUnmaskTest(){
	    log_verbose("[main::UnmaskTest]");

	    RegularPolygonalShapeFunction<n> sf(
			GlobalLinearShapeFunction::Point{static_cast<size_t>(0.5*sensorWidth), static_cast<size_t>(0.5*sensorHeight)},
			GlobalLinearShapeFunction::Direction { static_cast<long>((1.0/2.0)*(2.0/sqrt(3))*sensorHeight*sin(2.0*std::numbers::pi*0.0/(2.0*n))), 
				        static_cast<long>((1.0/2.0)*(2.0/sqrt(3))*sensorHeight*cos(2.0*std::numbers::pi*0.0/(2.0*n)))}
		);

		std::vector<uint16_t> testPattern{GenerateStripedBayerBGGRImage( sensorWidth, sensorHeight, 10)};
		std::vector<uint16_t> input(testPattern.size());

		memcpy(input.data(), testPattern.data(), testPattern.size() * sizeof(uint16_t));
 	    auto output{sf.FrameBufferUnmask(sf.FrameBufferMask2(input))};
	    WriteToFile(output, std::string("image1.bayer"));
}

void OverlapMaskUnmaskTest(){
	    log_verbose("[main::OverlapMaskUnmaskTest]");
	    OverlapShapeFunction<n> sf;

		std::vector<uint16_t> testPattern{GenerateStripedBayerBGGRImage( sensorWidth, sensorHeight, 10)};
		
		std::vector<uint16_t> input0(testPattern.size());
		memcpy(input0.data(), testPattern.data(), testPattern.size() * sizeof(uint16_t));
		
 	    auto output0{sf.Facet::FrameBufferUnmask(sf.Facet::FrameBufferMask2(input0))};
	    WriteToFile(output0, std::string("image.bayer"));
		
		for(size_t i = 0; i<n; i++){
	 	    auto output1{sf.FrameBufferUnmask(sf.FrameBufferMask2(input0,i),i)};
		    WriteToFile(output1, std::string("image" + std::to_string(i) + ".bayer"));
		}

}

// main
/*
 */ 
int main(){
    //MaskUnmaskTest();
    OverlapMaskUnmaskTest();
	
	return 0;
}
