/*
 * @FilePath: include/SpinningTarget/SpinningTarget.hpp
 * @Description: 旋转装甲：连续 5 帧未观测判定丢失，维护角度与角速度。
 */

#pragma once

#include <cstdint>

#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

class SpinningTarget final : public TrackingBase
{
public:
    SpinningTarget(int id, const Observation &observation, std::int64_t timestamp_ns);

protected:
    int missThreshold() const override;

    void onObserved(const Observation &observation, double dt_obs, double dt_frame) override;
    void onReacquired(const Observation &observation) override;
    void onMissedAdvanced(double dt_frame) override;

private:
    static constexpr int kMissThreshold = 5;
};

} // namespace rm
