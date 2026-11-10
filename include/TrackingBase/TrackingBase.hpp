/*
 * @FilePath: include/TrackingBase/TrackingBase.hpp
 * @Description: 跟踪基类：维护单个目标的位置、速度、预测值与连续丢失计数，
 *               并统一「被观测」「未被观测」两种状态的公共处理流程。
 */

#pragma once

#include <cstdint>

#include "data_types.hpp"

namespace rm
{

/// 单个目标的跟踪状态。
///
/// 派生类只需要给出「连续丢失多少帧判定为 LOST」（missThreshold），
/// 以及类型特有的运动学行为（旋转装甲的角度与角速度）。
class TrackingBase
{
public:
    TrackingBase(int id, TargetType type, const Observation &observation, std::int64_t timestamp_ns);

    virtual ~TrackingBase() = default;

    TrackingBase(const TrackingBase &) = delete;
    TrackingBase &operator=(const TrackingBase &) = delete;

    int id() const { return id_; }
    TargetType type() const { return type_; }
    TargetStatus status() const { return status_; }
    bool isTracking() const { return status_ == TargetStatus::TRACK; }
    int missedCount() const { return missed_count_; }

    /// 本帧观测到该目标。
    /// @param dt_frame 当前帧的帧间隔（秒），首帧为 0；用于计算预测值。
    void observe(const Observation &observation, std::int64_t timestamp_ns, double dt_frame);

    /// 本帧未观测到该目标：连续丢失计数 +1，达到阈值时转入 LOST。
    /// @param dt_frame 当前帧的帧间隔（秒）；仍为 TRACK 时用于推进预测值。
    void advanceMissed(double dt_frame);

    /// 当前状态的快照。
    TargetSnapshot snapshot() const;

protected:
    /// 连续未观测多少帧后判定为 LOST。
    virtual int missThreshold() const = 0;

    /// 观测到该目标后的类型特有更新。
    /// @param dt_obs 目标相邻两次观测的时间差（秒）。
    virtual void onObserved(const Observation &observation, double dt_obs, double dt_frame);

    /// LOST 目标被重新观测时的类型特有复位。
    virtual void onReacquired(const Observation &observation);

    /// 仍为 TRACK、尚未达到丢失阈值时的类型特有推进。
    virtual void onMissedAdvanced(double dt_frame);

    double angle() const { return angle_; }
    double angularVelocity() const { return angular_velocity_; }
    double predictedAngle() const { return predicted_angle_; }

    /// 更新角度状态；仅旋转装甲使用，普通装甲始终为 0。
    void updateAngle(double angle, double angular_velocity, double predicted_angle);

private:
    void enterLost();

    int id_;
    TargetType type_;
    TargetStatus status_;

    double x_;
    double y_;
    double velocity_x_;
    double velocity_y_;
    double predicted_x_;
    double predicted_y_;

    double angle_;
    double angular_velocity_;
    double predicted_angle_;

    int missed_count_;
    std::int64_t last_observed_ns_;
};

} // namespace rm
