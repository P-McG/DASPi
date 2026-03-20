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

template<typename InputT>
constexpr void Aperture::FrameBufferTransformation(InputT&& input,
                                                   const GainMsg& gainMsg,
                                                   sfdp_t& output,
                                                   const size_t numThreads)
{
    log_verbose("[Aperture::FrameBufferTransformation]");

    if (numThreads == 0) {
        std::cerr << "[FrameBufferTransformation] numThreads must be > 0" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    output.ResetValidSizes();

    auto copyFacetToOutput = [&](size_t outIndex, auto& maskedOutput) {
        auto& dst = output[outIndex];

        std::cout << "[FrameBufferTransformation] facet=" << outIndex
                  << " masked.size()=" << maskedOutput.size()
                  << " dst.capacity()=" << dst.size()
                  << std::endl;

        if (maskedOutput.size() > dst.size()) {
            std::cerr << "[FrameBufferTransformation] overflow for facet " << outIndex
                      << ": masked.size()=" << maskedOutput.size()
                      << " dst.size()=" << dst.size()
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }

        std::memcpy(dst.data(), maskedOutput.data(), maskedOutput.size() * sizeof(uint16_t));
        output.SetRegionValidSize(outIndex, maskedOutput.size());
    };

    {
        auto maskedOutput = sf_.nonOverlapFacet_t::FrameBufferMask(input);
        ApplyWhiteBalanceToMosaic_BGGR(
            0, sf_,
            std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            gainMsg
        );
        copyFacetToOutput(0, maskedOutput);
    }

    for (size_t i = 0; i < n_; ++i) {
        auto maskedOutput = sf_.FrameBufferMask(input, i);
        ApplyWhiteBalanceToMosaic_BGGR(
            i + 1, sf_,
            std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            gainMsg
        );
        copyFacetToOutput(i + 1, maskedOutput);
    }
    
    size_t totalValid = 0;
    for (size_t i = 0; i < sfdp_t::NUM_REGIONS; ++i) {
        totalValid += output.RegionValidSize(i);
    }
    
    std::cout << "[FrameBufferTransformation] total valid elements="
              << totalValid << std::endl;
}

//template<typename InputT>
//constexpr void Aperture::FrameBufferTransformation(InputT&& input,
                                                   //const GainMsg& gainMsg,
                                                   //sfdp_t& output,
                                                   //const size_t numThreads)
//{
    //log_verbose("[Aperture::FrameBufferTransformation]");

    //if (numThreads == 0) {
        //std::cerr << "[FrameBufferTransformation] numThreads must be > 0" << std::endl;
        //std::exit(EXIT_FAILURE);
    //}

    //auto copyFacetToOutput = [&](const size_t outIndex, auto&& maskedOutput) {
        //using MaskedT = std::decay_t<decltype(maskedOutput)>;

        //auto& masked = maskedOutput;
        //auto& dst = output[outIndex];

        //std::cout << "[FrameBufferTransformation] facet=" << outIndex
                  //<< " masked.size()=" << masked.size()
                  //<< " dst.size()=" << dst.size()
                  //<< std::endl;

        //if (masked.size() != dst.size()) {
            //std::cerr << "[FrameBufferTransformation] size mismatch for facet " << outIndex
                      //<< ": masked.size()=" << masked.size()
                      //<< " dst.size()=" << dst.size()
                      //<< std::endl;
            //std::exit(EXIT_FAILURE);
        //}

        //std::memcpy(dst.data(), masked.data(), masked.size() * sizeof(typename MaskedT::value_type));
    //};

    //// Non-overlap facet
    //{
        //auto maskedOutput = sf_.nonOverlapFacet_t::FrameBufferMask(input);

        //std::cout << "[FrameBufferTransformation] non-overlap masked.size()="
                  //<< maskedOutput.size() << std::endl;

        //ApplyWhiteBalanceToMosaic_BGGR(
            //0,
            //sf_,
            //std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            //gainMsg
        //);

        //copyFacetToOutput(0, maskedOutput);
    //}

    //// Overlap facets
    //for (size_t i = 0; i < n_; ++i) {
        //auto maskedOutput = sf_.FrameBufferMask(input, i);

        //std::cout << "[FrameBufferTransformation] overlap facet=" << (i + 1)
                  //<< " masked.size()=" << maskedOutput.size()
                  //<< std::endl;

        //ApplyWhiteBalanceToMosaic_BGGR(
            //i + 1,
            //sf_,
            //std::span<uint16_t>(maskedOutput.data(), maskedOutput.size()),
            //gainMsg
        //);

        //copyFacetToOutput(i + 1, maskedOutput);
    //}
    //std::cout << "[Transform] facet 0 size=" << output[0].size() << std::endl;
    //std::cout << "[FrameBufferTransformation] completed" << std::endl;
//}



};//end namespace DASPi
