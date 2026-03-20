#pragma once
#include <vector>
#include <array>
#include <span>
#include <numeric>
#include <cstring>
#include <stdexcept>

#include "DASPi-logger.h"
#include "DASPi-overlapshapefunction.h"

namespace DASPi {

template<size_t n, PointData center, DirectionData direction, double nonOverlapScale>
class ShapeFunctionDataPacket {
public:
    using value_type = OverlapShapeFunction<n, center, direction, nonOverlapScale>;

private:
    static constexpr size_t n_ = n;
	static constexpr size_t NUM_REGIONS = n_ + 1;
	
    std::vector<uint16_t> contiguousMemory_;
    std::array<std::span<uint16_t>, n_ + 1> regions_{};
    std::array<size_t, n_ + 1> capacities_{};
    std::array<size_t, n_ + 1> validSizes_{};

public:
    ShapeFunctionDataPacket(const OverlapShapeFunction<n, center, direction, nonOverlapScale>& sf)
    {
        log_verbose("[ShapeFunctonDataPacket::ShapeFunctionDataPacket]");

        capacities_[0] = sf.RegularPolygonalShapeFunction<n, center, direction>::size();
        for (size_t i = 0; i < n; ++i) {
            capacities_[i + 1] = sf.size(i);
        }

        contiguousMemory_ = std::vector<uint16_t>(
            std::accumulate(capacities_.begin(), capacities_.end(), size_t{0})
        );

        regions_[0] = std::span<uint16_t>(contiguousMemory_.data(), capacities_[0]);

        size_t offset = capacities_[0];
        for (size_t i = 0; i < n; ++i) {
            regions_[i + 1] = std::span<uint16_t>(contiguousMemory_.data() + offset, capacities_[i + 1]);
            offset += capacities_[i + 1];
        }

        validSizes_.fill(0);
    }

    std::span<uint16_t>& operator[](size_t i) { return regions_[i]; }
    const std::span<uint16_t>& operator[](size_t i) const { return regions_[i]; }

    size_t RegionCapacity(size_t i) const { return capacities_[i]; }
    size_t RegionValidSize(size_t i) const { return validSizes_[i]; }

    void SetRegionValidSize(size_t i, size_t valid)
    {
        if (i >= validSizes_.size()) {
            throw std::out_of_range("region index out of range");
        }
        if (valid > capacities_[i]) {
            throw std::runtime_error("valid region size exceeds capacity");
        }
        validSizes_[i] = valid;
    }

    void ResetValidSizes()
    {
        validSizes_.fill(0);
    }

    std::vector<uint16_t> TakeContiguousMemory()
    {
        size_t totalValid = std::accumulate(validSizes_.begin(), validSizes_.end(), size_t{0});
        std::vector<uint16_t> packed;
        packed.reserve(totalValid);

        for (size_t i = 0; i < n_ + 1; ++i) {
            packed.insert(packed.end(),
                          regions_[i].begin(),
                          regions_[i].begin() + static_cast<std::ptrdiff_t>(validSizes_[i]));
        }

        return packed;
    }

    size_t size() const
    {
        return contiguousMemory_.size();
    }

    static constexpr size_t NumberOfRegions()
    {
        return NUM_REGIONS;
    }
};

} // namespace DASPi
