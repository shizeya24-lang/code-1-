/*
 * @FilePath: src/SpinningTarget/SpinningTarget.cpp
 * @Description: 旋转装甲实现：丢失阈值 5，按观测时间差更新角速度与预测角。
 */

#include "SpinningTarget/SpinningTarget.hpp"

namespace rm
{

SpinningTarget::SpinningTarget(int id, const Observation &observation, std::int64_t timestamp_ns)
    : TrackingBase(id, TargetType::SPINNING, observation, timestamp_ns)
{
}

int SpinningTarget::missThreshold() const
{
    return kMissThreshold;
}

void SpinningTarget::onObserved(const Observation &observation, double dt_obs, double dt_frame)
{
    const double angular_velocity = (observation.angle - angle()) / dt_obs;
    updateAngle(observation.angle, angular_velocity, observation.angle + angular_velocity * dt_frame);
}

void SpinningTarget::onReacquired(const Observation &observation)
{
    updateAngle(observation.angle, 0.0, observation.angle);
}

void SpinningTarget::onMissedAdvanced(double dt_frame)
{
    const double angular_velocity = angularVelocity();
    const double advanced_angle = angle() + angular_velocity * dt_frame;
    updateAngle(advanced_angle, angular_velocity, advanced_angle + angular_velocity * dt_frame);
}

} // namespace rm
