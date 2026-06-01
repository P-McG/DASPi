// DASPi-region0whitebalanceaggregator.h

#pragma once
namespace DASPi::detail{

class Region0WhiteBalanceAggregator {
public:
    Region0WhiteBalanceGains Update(const void* peerKey,
                                    uint32_t frameId,
                                    const Region0BayerStats& stats)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (stats.hasWhiteBalanceSamples()) {
            latestByPeer_[peerKey] = LatestStats{
                .frameId = frameId,
                .stats = stats
            };
        }

        Region0BayerStats aggregate{};

        for (const auto& [key, latest] : latestByPeer_) {
            (void)key;

            aggregate.rSum01 += latest.stats.rSum01;
            aggregate.gSum01 += latest.stats.gSum01;
            aggregate.bSum01 += latest.stats.bSum01;

            aggregate.rCount += latest.stats.rCount;
            aggregate.gCount += latest.stats.gCount;
            aggregate.bCount += latest.stats.bCount;
        }

        Region0WhiteBalanceGains result{};
        result.contributingRegion0s = latestByPeer_.size();

        if (!aggregate.hasWhiteBalanceSamples()) {
            result.rGainApply = smoothedRGain_;
            result.bGainApply = smoothedBGain_;
            result.valid = hasSmoothedGains_;
            return result;
        }

        const double rMean =
            aggregate.rSum01 / static_cast<double>(aggregate.rCount);

        const double gMean =
            aggregate.gSum01 / static_cast<double>(aggregate.gCount);

        const double bMean =
            aggregate.bSum01 / static_cast<double>(aggregate.bCount);

        auto clampGain = [](double gain) -> float {
            if (!std::isfinite(gain) || gain <= 0.0) {
                return 1.0f;
            }

            return std::clamp(
                static_cast<float>(gain),
                0.25f,
                4.0f
            );
        };

        const float rawRGain = clampGain(gMean / rMean);
        const float rawBGain = clampGain(gMean / bMean);

        constexpr float kAlpha = 0.10f;

        if (!hasSmoothedGains_) {
            smoothedRGain_ = rawRGain;
            smoothedBGain_ = rawBGain;
            hasSmoothedGains_ = true;
        } else {
            smoothedRGain_ =
                (1.0f - kAlpha) * smoothedRGain_ + kAlpha * rawRGain;

            smoothedBGain_ =
                (1.0f - kAlpha) * smoothedBGain_ + kAlpha * rawBGain;
        }

        result.rGainApply = smoothedRGain_;
        result.bGainApply = smoothedBGain_;
        result.valid = true;

        return result;
    }

private:
    struct LatestStats {
        uint32_t frameId = 0;
        Region0BayerStats stats{};
    };

    std::mutex mutex_;
    std::map<const void*, LatestStats> latestByPeer_;

    bool hasSmoothedGains_ = false;
    float smoothedRGain_ = 1.0f;
    float smoothedBGain_ = 1.0f;
};

inline Region0WhiteBalanceAggregator& Region0WhiteBalanceAggregatorInstance()
{
    static Region0WhiteBalanceAggregator aggregator;
    return aggregator;
}

}//DASPi::detail
