/*
 * @FilePath: include/ArmorTarget/ArmorTarget.hpp
 * @Description: 普通装甲：连续 3 帧未观测判定丢失，不维护角度。
 */

#pragma once

#include <cstdint>

#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

class ArmorTarget final : public TrackingBase
{
public:
    ArmorTarget(int id, const Observation &observation, std::int64_t timestamp_ns);

protected:
    int missThreshold() const override;

private:
    static constexpr int kMissThreshold = 3;
};

} // namespace rm
