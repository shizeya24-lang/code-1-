/*
 * @FilePath: src/TrackingBase/TrackingBase.cpp
 * @Description: 跟踪基类实现：状态迁移与位置/速度预测。
 */

#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

namespace
{
constexpr double kNanosecondsPerSecond = 1e9;
} // namespace

TrackingBase::TrackingBase(int id, TargetType type, const Observation &observation, std::int64_t timestamp_ns)
    : id_(id),
      type_(type),
      status_(TargetStatus::TRACK),
      x_(observation.x),
      y_(observation.y),
      velocity_x_(0.0),
      velocity_y_(0.0),
      predicted_x_(observation.x),
      predicted_y_(observation.y),
      angle_(type == TargetType::SPINNING ? observation.angle : 0.0),
      angular_velocity_(0.0),
      predicted_angle_(type == TargetType::SPINNING ? observation.angle : 0.0),
      missed_count_(0),
      last_observed_ns_(timestamp_ns)
{
}

void TrackingBase::observe(const Observation &observation, std::int64_t timestamp_ns, double dt_frame)
{
    if (status_ == TargetStatus::LOST)
    {
        // 重新追踪：不做差值计算，速度清零，预测值取本次观测。
        status_ = TargetStatus::TRACK;
        missed_count_ = 0;

        x_ = observation.x;
        y_ = observation.y;
        predicted_x_ = x_;
        predicted_y_ = y_;
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;

        onReacquired(observation);

        last_observed_ns_ = timestamp_ns;
        return;
    }

    // 整帧校验已保证时间戳严格递增，因此 dt_obs 必然大于 0。
    const double dt_obs = static_cast<double>(timestamp_ns - last_observed_ns_) / kNanosecondsPerSecond;

    velocity_x_ = (observation.x - x_) / dt_obs;
    velocity_y_ = (observation.y - y_) / dt_obs;

    x_ = observation.x;
    y_ = observation.y;

    predicted_x_ = x_ + velocity_x_ * dt_frame;
    predicted_y_ = y_ + velocity_y_ * dt_frame;

    onObserved(observation, dt_obs, dt_frame);

    missed_count_ = 0;
    last_observed_ns_ = timestamp_ns;
}

void TrackingBase::advanceMissed(double dt_frame)
{
    if (status_ != TargetStatus::TRACK)
    {
        // 已进入 LOST 的目标不再变化；基础要求允许继续计数，这里选择不再计数。
        return;
    }

    ++missed_count_;
    if (missed_count_ >= missThreshold())
    {
        // 达到丢失阈值的那一帧不再推进预测值。
        enterLost();
        return;
    }

    predicted_x_ += velocity_x_ * dt_frame;
    predicted_y_ += velocity_y_ * dt_frame;
    onMissedAdvanced(dt_frame);
}

TargetSnapshot TrackingBase::snapshot() const
{
    TargetSnapshot snapshot{};
    snapshot.id = id_;
    snapshot.type = type_;
    snapshot.status = status_;
    snapshot.x = x_;
    snapshot.y = y_;
    snapshot.velocity_x = velocity_x_;
    snapshot.velocity_y = velocity_y_;
    snapshot.predicted_x = predicted_x_;
    snapshot.predicted_y = predicted_y_;
    snapshot.angle = angle_;
    snapshot.angular_velocity = angular_velocity_;
    snapshot.predicted_angle = predicted_angle_;
    snapshot.missed_count = missed_count_;
    return snapshot;
}

void TrackingBase::enterLost()
{
    status_ = TargetStatus::LOST;
    x_ = 0.0;
    y_ = 0.0;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    predicted_x_ = 0.0;
    predicted_y_ = 0.0;
    angle_ = 0.0;
    angular_velocity_ = 0.0;
    predicted_angle_ = 0.0;
}

void TrackingBase::updateAngle(double angle, double angular_velocity, double predicted_angle)
{
    angle_ = angle;
    angular_velocity_ = angular_velocity;
    predicted_angle_ = predicted_angle;
}

void TrackingBase::onObserved(const Observation &, double, double)
{
}

void TrackingBase::onReacquired(const Observation &)
{
}

void TrackingBase::onMissedAdvanced(double)
{
}

} // namespace rm
