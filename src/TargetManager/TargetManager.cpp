/*
 * @FilePath: src/TargetManager/TargetManager.cpp
 * @Description: 目标管理器实现：整帧校验、逐帧编排与统计。
 */

#include "TargetManager/TargetManager.hpp"

#include <unordered_set>

#include "ArmorTarget/ArmorTarget.hpp"
#include "SpinningTarget/SpinningTarget.hpp"
#include "TrackingBase/TrackingBase.hpp"

namespace rm
{

namespace
{
constexpr double kNanosecondsPerSecond = 1e9;
} // namespace

TargetManager::TargetManager() = default;

TargetManager::~TargetManager() = default;

Result TargetManager::onFrame(const InputFrame &frame)
{
    if (!validateFrame(frame))
    {
        return buildResult(false, frame.frame_id);
    }

    const double dt_frame = has_processed_frame_
                                ? static_cast<double>(frame.timestamp_ns - last_timestamp_ns_) / kNanosecondsPerSecond
                                : 0.0;

    last_frame_id_ = frame.frame_id;
    last_timestamp_ns_ = frame.timestamp_ns;
    has_processed_frame_ = true;

    std::unordered_set<int> observed_ids;
    observed_ids.reserve(frame.observations.size());

    for (const auto &observation : frame.observations)
    {
        observed_ids.insert(observation.id);

        if (TrackingBase *target = findTarget(observation.id); target != nullptr)
        {
            target->observe(observation, frame.timestamp_ns, dt_frame);
        }
        else
        {
            targets_.push_back(createTarget(observation, frame.timestamp_ns));
        }
    }

    // 本帧未被观测到的目标：推进连续丢失计数。
    for (const auto &target : targets_)
    {
        if (observed_ids.count(target->id()) == 0)
        {
            target->advanceMissed(dt_frame);
        }
    }

    return buildResult(true, frame.frame_id);
}

bool TargetManager::validateFrame(const InputFrame &frame) const
{
    // 帧号与时间戳必须严格大于最近一次成功处理的帧（相等也算非法）。
    if (has_processed_frame_ && frame.frame_id <= last_frame_id_)
    {
        return false;
    }
    if (has_processed_frame_ && frame.timestamp_ns <= last_timestamp_ns_)
    {
        return false;
    }

    std::unordered_set<int> ids;
    ids.reserve(frame.observations.size());

    for (const auto &observation : frame.observations)
    {
        if (observation.id < 0)
        {
            return false;
        }
        if (!ids.insert(observation.id).second)
        {
            return false;
        }

        // 已创建目标的类型在首次创建时确定，后续帧不得改变。
        const TrackingBase *existing = findTarget(observation.id);
        if (existing != nullptr && existing->type() != observation.type)
        {
            return false;
        }
    }

    return true;
}

TrackingBase *TargetManager::findTarget(int id) const
{
    for (const auto &target : targets_)
    {
        if (target->id() == id)
        {
            return target.get();
        }
    }
    return nullptr;
}

std::unique_ptr<TrackingBase> TargetManager::createTarget(const Observation &observation,
                                                          std::int64_t timestamp_ns) const
{
    if (observation.type == TargetType::SPINNING)
    {
        return std::make_unique<SpinningTarget>(observation.id, observation, timestamp_ns);
    }
    return std::make_unique<ArmorTarget>(observation.id, observation, timestamp_ns);
}

Result TargetManager::buildResult(bool accepted, int frame_id) const
{
    Result result{};
    result.accepted = accepted;
    result.frame_id = frame_id;

    result.targets.reserve(targets_.size());
    for (const auto &target : targets_)
    {
        result.targets.push_back(target->snapshot());
    }

    for (const auto &target : result.targets)
    {
        if (target.status == TargetStatus::TRACK)
        {
            ++result.tracking_target_count;
        }
        else
        {
            ++result.lost_target_count;
        }

        if (target.type == TargetType::ARMOR)
        {
            ++result.armor_target_count;
        }
        else
        {
            ++result.spinning_target_count;
        }
    }

    return result;
}

} // namespace rm
