/*
 * @FilePath: src/ArmorTarget/ArmorTarget.cpp
 * @Description: 普通装甲实现：只提供丢失阈值，角度相关字段保持基类默认的 0。
 */

#include "ArmorTarget/ArmorTarget.hpp"

namespace rm
{

ArmorTarget::ArmorTarget(int id, const Observation &observation, std::int64_t timestamp_ns)
    : TrackingBase(id, TargetType::ARMOR, observation, timestamp_ns)
{
}

int ArmorTarget::missThreshold() const
{
    return kMissThreshold;
}

} // namespace rm
