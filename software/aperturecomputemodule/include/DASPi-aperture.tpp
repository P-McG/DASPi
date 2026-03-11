#include <omp.h> // Include OpenMP
#include "scopedtimer.h"

namespace DASPi{
   
    //template<typename input_t>
    //void constexpr Aperture::FrameBufferTransformation(input_t&& input, sfdp_t& output, const size_t numThreads){
        //log_verbose("[Aperture::FrameBufferTransformation2]");
        
        //std::vector<uint16_t> v{sf_.facet_t::FrameBufferMask(input)};

        //memcpy(output[0].data(), BrightenImage(std::move(v)).data(), output[0].size() * sizeof(uint16_t));//copy, canidate for removal
        
        //for(size_t i = 0; i < n_; i++){
            //std::vector v{sf_.FrameBufferMask(input, i)};
            //memcpy(output[i+1].data(), BrightenImage(std::move(v)).data(), output[i+1].size() * sizeof(uint16_t));//copy, canidate for removal
        //}
    //}
    
       
    //template<typename input_t>
    //void constexpr Aperture::FrameBufferTransformation(input_t&& input, sfdp_t& output, const size_t numThreads){
        //log_verbose("[Aperture::FrameBufferTransformation2]");
        
        ////std::vector<uint16_t> v{BrightenImage(input};
        //std::vector<uint16_t> v{BrightenImage(std::vector<uint16_t>(input.begin(), input.end()))};

        //memcpy(output[0].data(), sf_.nonOverlapFacet_t::FrameBufferMask(std::move(v)).data(), output[0].size() * sizeof(uint16_t));//copy, canidate for removal
        
        //for(size_t i = 0; i < n_; i++){
            ////std::vector v{BrightenImage(std::move(input))};
            //std::vector<uint16_t> v{BrightenImage(std::vector<uint16_t>(input.begin(), input.end()))};
            //memcpy(output[i+1].data(), sf_.FrameBufferMask(std::move(v)).data(), output[i+1].size() * sizeof(uint16_t), i);//copy, canidate for removal
        //}
    //}

    template<typename input_t>
    constexpr void Aperture::FrameBufferTransformation(input_t&& input, const GainMsg& gainMsg, sfdp_t& output, const size_t numThreads) {
        log_verbose("[Aperture::FrameBufferTransformation]");

        // Apply to non-overlap facet
        auto maskedOutput{sf_.nonOverlapFacet_t::FrameBufferMask(input)};
        ApplyWhiteBalanceToMosaic_BGGR(0, sf_, std::span<uint16_t>(maskedOutput), gainMsg); 
        memcpy(output[0].data(), maskedOutput.data(), maskedOutput.size() * sizeof(uint16_t)); // copy required
    
        // Apply to each of the n_ overlap facets
        for (size_t i = 0; i < n_; ++i) {
            auto maskedOutput{sf_.FrameBufferMask(input, i)};
            ApplyWhiteBalanceToMosaic_BGGR(i+1, sf_, std::span<uint16_t>(maskedOutput), gainMsg); 
            memcpy(output[i + 1].data(), maskedOutput.data(), maskedOutput.size() * sizeof(uint16_t)); // copy required
        }
    }



};//end namespace DASPi
