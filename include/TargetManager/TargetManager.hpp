/*
 * @FilePath: include/TargetManager/TargetManager.hpp
 * @Description: 目标管理器：逐帧接收检测结果，负责整帧校验、目标更新与统计。
 */

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "data_types.hpp"

namespace rm
{

class TrackingBase;

class TargetManager
{
public:
    /// 构造/析构在 .cpp 中定义：TrackingBase 在此处仅前向声明，
    /// 需要在类型完整处创建与销毁目标。
    TargetManager();
    ~TargetManager();

    TargetManager(const TargetManager &) = delete;
    TargetManager &operator=(const TargetManager &) = delete;

    /// 处理一帧数据；校验失败时整帧拒绝，所有目标与帧记录保持不变。
    Result onFrame(const InputFrame &frame);

private:
    /// 只读校验，不修改任何状态。
    bool validateFrame(const InputFrame &frame) const;

    TrackingBase *findTarget(int id) const;

    /// 按观测类型创建对应的目标对象。
    std::unique_ptr<TrackingBase> createTarget(const Observation &observation, std::int64_t timestamp_ns) const;

    /// 用当前目标集合构造结果（快照与统计）。
    Result buildResult(bool accepted, int frame_id) const;

    std::vector<std::unique_ptr<TrackingBase>> targets_;
    int last_frame_id_{0};
    std::int64_t last_timestamp_ns_{0};
    bool has_processed_frame_{false};
};

} // namespace rm
