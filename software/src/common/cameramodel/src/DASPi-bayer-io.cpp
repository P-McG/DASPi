// DASPi-bayer-io.cpp
#include "DASPi-bayer-io.h"

#include <fstream>
#include <stdexcept>

namespace DASPi {

cv::Mat decodeBayer16ToBgr8(const std::uint16_t* data,
                            int width,
                            int height,
                            int bayerCode)
{
    cv::Mat raw16(height, width, CV_16UC1, const_cast<std::uint16_t*>(data));
    cv::Mat raw16Copy = raw16.clone();

    std::vector<std::uint16_t> samples;
    samples.reserve(
        static_cast<std::size_t>(width) *
        static_cast<std::size_t>(height)
    );

    for (int y = 0; y < raw16Copy.rows; ++y) {
        const auto* row = raw16Copy.ptr<std::uint16_t>(y);

        for (int x = 0; x < raw16Copy.cols; ++x) {
            const std::uint16_t v = row[x];

            // Ignore masked-out pixels.
            if (v != 0) {
                samples.push_back(v);
            }
        }
    }

    if (samples.empty()) {
        return cv::Mat::zeros(height, width, CV_8UC3);
    }

    std::sort(samples.begin(), samples.end());

    auto percentileValue =
        [&](double p) -> double
    {
        p = std::clamp(p, 0.0, 1.0);

        const std::size_t idx =
            static_cast<std::size_t>(
                p * static_cast<double>(samples.size() - 1)
            );

        return static_cast<double>(samples[idx]);
    };

    const double black = percentileValue(0.01);
    const double white = percentileValue(0.995);

    static int printCount = 0;
    if (printCount++ < 40) {
        std::cout << "[decodeBayer16ToBgr8]"
                  << " nonzeroSamples=" << samples.size()
                  << " p01=" << black
                  << " p995=" << white
                  << " minNonZero=" << samples.front()
                  << " maxNonZero=" << samples.back()
                  << '\n';
    }

    cv::Mat bgr16;
    cv::cvtColor(raw16Copy, bgr16, bayerCode);

    if (white <= black) {
        return cv::Mat::zeros(height, width, CV_8UC3);
    }

    cv::Mat bgrFloat;
    bgr16.convertTo(
        bgrFloat,
        CV_32FC3,
        1.0 / (white - black),
        -black / (white - black)
    );

    cv::threshold(bgrFloat, bgrFloat, 0.0, 0.0, cv::THRESH_TOZERO);
    cv::threshold(bgrFloat, bgrFloat, 1.0, 1.0, cv::THRESH_TRUNC);

    constexpr double gamma = 1.0 / 2.2;
    cv::pow(bgrFloat, gamma, bgrFloat);

    cv::Mat bgr8;
    bgrFloat.convertTo(bgr8, CV_8UC3, 255.0);

    return bgr8;
}

//cv::Mat decodeBayer16ToBgr8(const std::uint16_t* data,
                            //int width,
                            //int height,
                            //int bayerCode = cv::COLOR_BayerRG2BGR)
//{
    //cv::Mat raw16(height, width, CV_16UC1, const_cast<std::uint16_t*>(data));
    //cv::Mat raw16Copy = raw16.clone();

    //cv::Mat bgr16;
    //cv::cvtColor(raw16Copy, bgr16, bayerCode);

    //cv::Mat bgr8;
    //bgr16.convertTo(bgr8, CV_8UC3, 1.0 / 256.0);
    //return bgr8;
//}

//cv::Mat decodeBayer16ToBgr8(const std::uint16_t* data,
                            //int width,
                            //int height,
                            //int bayerCode = cv::COLOR_BayerBG2BGR)
//{
    //cv::Mat raw16(height, width, CV_16UC1, const_cast<std::uint16_t*>(data));
    //cv::Mat raw16Copy = raw16.clone();

    //cv::Mat bgr16;
    //cv::cvtColor(raw16Copy, bgr16, bayerCode);

    //// Convert to float in [0, 1]
    //cv::Mat bgrFloat;
    //bgr16.convertTo(bgrFloat, CV_32FC3, 1.0 / 65535.0);

    ////// Apply gamma correction
    ////constexpr float gamma = 1.0f / 2.2f;
    ////cv::pow(bgrFloat, gamma, bgrFloat);

    //// Convert to 8-bit for display
    //cv::Mat bgr8;
    //bgrFloat.convertTo(bgr8, CV_8UC3, 255.0);

    //return bgr8;
//}

}
