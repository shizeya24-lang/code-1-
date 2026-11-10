# 第二次作业 · 项目说明

## 0. 基本信息

- 姓名：师泽亚
- 学号：2026932029
- 完成日期：2026-10-31
- 是否完成全部基础功能：是

（基础要求已全部实现，28 帧测试序列的输出已按**原作业说明**第 7、10 节逐项核对通过；
原说明在本地留存（未纳入版本库）。未完成的附加项见第 3 节，已知问题见第 4 节。）

## 1. 类设计

| 类名 | 职责 | 继承自 |
| --- | --- | --- |
| `TrackingBase` | 单个目标的跟踪状态：位置、速度、预测值、角度、连续丢失计数；统一「被观测」「未被观测」「重新追踪」「丢失」四条状态迁移路径 | — |
| `ArmorTarget` | 普通装甲：声明丢失阈值为 3，角度相关字段恒为 0 | `TrackingBase` |
| `SpinningTarget` | 旋转装甲：声明丢失阈值为 5，维护角速度、当前角与预测角 | `TrackingBase` |
| `TargetManager` | 持有全部目标；实现 `onFrame`（整帧校验、逐帧编排、快照与统计） | — |

类之间的关系（文字或图片）：

```text
TargetManager
  └── std::vector<std::unique_ptr<TrackingBase>>    // 多态持有，虚析构保证正确释放
            ▲
            │ 继承
     ┌──────┴────────┐
  ArmorTarget    SpinningTarget
```

职责划分：`TrackingBase` 负责「怎么跟踪」，两个派生类只负责「丢失几帧算丢」和「要不要预测角度」；
`TargetManager` 负责「这一帧能不能收、收下之后怎么编排」。三者职责不重叠。

## 2. 一帧处理流程

按实际执行顺序简述 `onFrame`：

1. **只读校验**：帧号、时间戳必须严格大于最近成功帧；观测中不得有负数编号、不得有同帧重复编号、类型必须与已创建目标一致；
2. 任一条件失败 → 用**当前未改动**的目标集合构造 `accepted=false` 的 `Result` 返回，帧号与时间戳记录保持不变；
3. 校验通过后才更新最近成功帧号与时间戳，并计算本帧间隔 `dt_frame`（首帧取 0）；
4. 遍历本帧观测：已有目标调用 `observe`（内部区分 TRACK 与 LOST 重新追踪）；新编号按类型创建目标；
5. 再遍历全部目标，对「本帧未被观测」的目标调用 `advanceMissed`：计数 +1，未到阈值则推进预测值，达到阈值则转入 LOST 并清零；
6. 用当前全部目标生成快照，统计 `tracking/lost/armor/spinning`，返回 `accepted=true`。

**重点说明：整帧校验失败时，为什么不会留下部分修改。**

`validateFrame` 是一个 `const` 成员函数，只读不写；所有写操作（更新帧号、创建目标、调用
`observe` / `advanceMissed`）都排在它之后。失败分支里程序只做「读当前状态 → 组装 Result」，
目标容器与帧记录从未被触碰。另外，五条校验规则都集中在只读循环里，不存在
「先改了一半、再发现后面的观测非法」这种顺序，因此拒绝帧后的快照必然与上一接受帧逐字相同
（这一点由自测脚本单独检查）。

## 3. 附加项

### 3.1 完成情况

| 附加项 | 是否完成 |
| --- | --- |
| 附加一：`LOST` 目标超时删除 | 未完成（按基础要求保留 LOST 目标） |
| 附加二：类型变化时重建目标 | 未完成（类型不一致仍整帧拒绝） |
| 附加三：自主学习并使用 static | 已完成（阈值常量，见 3.2） |
| 附加四：自主学习并使用智能指针 | 已完成（目标容器 `std::unique_ptr`，见 3.3） |
| 附加五：目标排序 | 未完成（输出按创建顺序） |

### 3.2 附加三：静态成员与 static

使用位置：

- `include/ArmorTarget/ArmorTarget.hpp:24`：`static constexpr int kMissThreshold = 3;`
- `include/SpinningTarget/SpinningTarget.hpp:28`：`static constexpr int kMissThreshold = 5;`

用途：两个派生类分别声明「连续丢失多少帧判定为 LOST」。阈值属于**类型级别的配置**，
不属于某个目标实例。

选择理由：

- `static` 表示全类共享，`constexpr` 保证它是编译期常量，`missThreshold()` 直接返回它，
  比较 `missed_count_ >= missThreshold()` 时可被编译器优化；
- 不占对象存储：`sizeof(ArmorTarget)` 不会因为阈值而增加；
- 阈值只写在一处，之后调整规则只改这一行，`advanceMissed` 的比较逻辑完全不用动。

如果改用普通成员（例如 `const int miss_threshold_ = 3;`）：

- 每个目标对象都多存一份相同的值，容器里目标越多重复越多；
- 无法用在需要常量表达式的位置（数组大小、`static_assert` 等）；
- 阈值会被当成「对象状态」，与本意（类型属性）不符，容易被误认为同类型的不同对象可以有不同阈值。

补充取舍：`missThreshold()` 本身仍写成**虚函数**而不是 static 成员函数，因为取阈值需要按
**动态类型**分派——基类指针指向 `SpinningTarget` 时必须得到 5。若写成 static 就无法通过基类
指针按实际类型取值，多态就丢了。即：数据用 static，接口用 virtual。

### 3.3 附加四：智能指针

使用位置：

- `include/TargetManager/TargetManager.hpp:45`：`std::vector<std::unique_ptr<TrackingBase>> targets_;`
- `src/TargetManager/TargetManager.cpp:124`、`:126`：`std::make_unique<SpinningTarget>(...)`、
  `std::make_unique<ArmorTarget>(...)`。

理由与好处：

- 目标的具体类型要到运行时看观测才知道（普通/旋转），必须通过 `TrackingBase*` 多态使用；
  裸指针需要手工 `delete`，一旦某个分支提前返回或抛异常就会泄漏；
- `unique_ptr` 表达**独占所有权**：管理器容器是唯一拥有者，离开作用域自动析构；
  又因为基类析构是虚函数，通过基类指针删除时会正确调用派生类析构（避免未定义行为）；
- `make_unique` 把内存分配与构造参数绑定在一起，异常安全，也让「创建」集中在一个工厂函数里。

为什么不用 `shared_ptr`：目标生命周期完全由管理器容器决定，不存在共享所有权的场景。
`shared_ptr` 需要维护引用计数（额外的原子操作与内存块），还会掩盖「谁负责释放」这一信息，
在这里属于过度设计。

认为不需要智能指针的位置：`TargetSnapshot` 与 `Result` 是**值语义**的纯数据，复制成本低，
调用方（`main`）只读取，直接按值传递即可，用指针反而增加生命周期管理负担。

说明：附加二（类型变化重建）未实现，因此「删除旧目标再重建」对应的智能指针用法没有出现；
若日后实现，只需 `targets_.erase(...)` 后重新 `push_back`，旧对象由 `unique_ptr` 自动释放。

## 4. 遇到的问题 / 未完成的部分

- 遇到的问题和解决过程：
  1. `SpinningTarget::onMissedAdvanced` 中把局部变量命名为 `angle`，遮蔽了基类同名成员函数
     `angle()`，编译器报 `'angle' cannot be used as a function`。把局部变量改名 `advanced_angle`
     后解决——教训是成员函数名不要拿来当局部变量。
  2. `TargetManager` 头文件只前向声明了 `TrackingBase`，而
     `std::vector<std::unique_ptr<TrackingBase>>` 的析构需要完整类型。最初把构造/析构写成类内
     `= default`，导致 `main.cpp` 编译时报 `invalid application of 'sizeof' to incomplete type`。
     改为在 `.cpp`（已包含完整头文件）中定义构造与析构后解决。
  3. 预测值的累加规则容易写反：未观测时应「在上一帧预测值上继续累加」，而不是用当前坐标重算。
     对照测试帧 7–8（`5.33 → 5.67 → 6.00`）才确认实现正确。
- 已知未完成或有缺陷的地方：
  - 附加一、二、五未实现；
  - `LOST` 后 `missed_count` 选择「不再递增」（基础要求允许），快照里该值停在进入 LOST 时的帧数；
  - `findTarget` 为线性查找，目标数量少时足够；规模变大可换成 `std::unordered_map`。

## 5. 自测方法（补充）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
./build/tracker
```

实现过程中另外写了一个输出断言脚本（仅标准库，未纳入版本库），逐帧核对原作业说明第 7、10 节：
28 帧的帧号与接受/拒绝标记、关键速度与预测值、非法帧后状态逐字不变、每帧计数与 targets 一致。
运行结果：

```text
全部通过：28 帧，37 项目标数值检查，8 项计数检查，5 项整帧拒绝检查
```

该脚本只用于自测，不参与程序运行。
