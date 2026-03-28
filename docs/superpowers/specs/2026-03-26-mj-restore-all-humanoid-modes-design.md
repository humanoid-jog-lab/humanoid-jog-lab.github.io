# MuJoCo 风格恢复统一支持所有人形任务模式设计

日期：2026-03-26

## 背景

当前 `MuJoCo 风格恢复` 仅完整支持人形点动中的 `absolute-target` 与 `distance-target` 任务族：

- `absolute`
- `decoupled`
- `decoupled-upright`
- `distance`
- `distance-fullbody`

而以下模式仍然会回退到 `new qpJ` 或沿用各自独立控制路径：

- `relative`
- `relative-fullbody`
- `robot1`
- `robot2`
- `waist-body-link-xz`

这使得 “MuJoCo 风格恢复” 还不是一个统一覆盖所有人形点动任务的控制器，也会让对外实验和论文表述变得不干净。

本设计的目标是：在保留各模式现有任务语义不变的前提下，将所有人形任务模式统一接入同一套 `MuJoCo 风格恢复` 求解框架。

## 目标

1. 让普通 `MuJoCo 风格恢复` 最终覆盖全部人形任务模式。
2. 为 `MuJoCo 风格恢复 + w预优化` 预留与普通 `mj_restore` 相同的统一任务适配层，但本轮实现主目标仍是先完成普通 `mj_restore` 的全模式覆盖；`mj_restore_pre_w` 仅在其已支持的模式上复用新适配层，不要求本轮同步开放全部新模式。
3. 保留每种模式当前的任务定义与用户语义，不把单臂模式偷偷改成双臂协同模式，也不把腰部模式改成全身模式。
4. 消除 “表面选了 MuJoCo 风格恢复，内部却回退到 new qpJ” 的不透明行为。
5. 让所有模式最终共享同一类诊断、收敛判定与恢复逻辑。

## 非目标

1. 本轮不重写 `new qpJ` 或 `零空间投影`。
2. 本轮不调整各模式的 UI 文案与交互语义。
3. 本轮不改变 `MuJoCo 风格恢复` 当前在 `absolute / decoupled / decoupled-upright / distance / distance-fullbody` 上已经稳定的控制目标。

## 总体思路

在 `MuJoCo 风格恢复` 前增加一层统一的“任务适配层（task adapter）”。

该适配层负责将不同控制模式转换为统一任务描述，后端 `mj_restore` 只关心：

- 命令任务（command）
- 不变量任务（invariant）
- 额外约束（extra constraints）
- 活动关节集合（active joints）

这样就不再按 UI 模式名在求解入口硬分叉，而是所有模式都先被描述成同一种任务格式，再交给统一的恢复器。

## 统一任务描述

统一任务描述建议包含以下字段：

- `modeTag`
- `commandKind`
- `commandTarget`
- `invariants`
- `activeJointMask` 或等价的 active joint slice
- `extraConstraints`
- `diagnosticLabel`

其中：

- `commandKind` 用来表示当前要跟踪的任务类型
- `invariants` 是一个数组，每个元素都包含：
  - `kind`
  - `target`
  - `weight` 或等价优先级字段
  - `diagnosticLabel`
- `extraConstraints` 用于表达 `body_z`、`waist corridor` 等附加约束

## 各模式在统一框架下的语义

### 1. absolute

- command：双臂绝对位姿
- invariants：
  - 双臂相对位姿
- active joints：双臂

### 2. decoupled

- command：双臂绝对位姿
- invariants：
  - 双臂相对位姿
- active joints：腰部 + 双臂

### 3. decoupled-upright

- command：双臂绝对位姿
- invariants：
  - 双臂相对位姿
- extra constraint：`body_link` 的 `z` 轴保持竖直
- active joints：腰部 + 双臂

### 4. relative

- command：双臂相对位姿
- invariants：
  - 双臂绝对位姿
- active joints：双臂

### 5. relative-fullbody

- command：双臂相对位姿
- invariants：
  - 双臂绝对位姿
- active joints：腰部 + 双臂

### 6. distance

- command：双臂相对距离
- invariants：
  - 双臂绝对位姿
- active joints：双臂

### 7. distance-fullbody

- command：双臂相对距离
- invariants：
  - 双臂绝对位姿
- active joints：腰部 + 双臂

### 8. robot1

- command：左臂末端绝对位姿
- invariants：
  - 右臂当前关节角保持不变
  - 腰部当前关节角保持不变
- active joints：左臂

说明：这里不引入双臂协同任务。该模式仍然是“只控制左臂末端”的单臂任务，只是求解器统一成 `mj_restore`。

### 9. robot2

- command：右臂末端绝对位姿
- invariants：
  - 左臂当前关节角保持不变
  - 腰部当前关节角保持不变
- active joints：右臂

### 10. waist-body-link-xz

- command：`body_link` 的 `x/z` 二维平移
- invariants：
  - 左臂当前关节角保持不变
  - 右臂当前关节角保持不变
- active joints：腰部

说明：该模式仍保持“只控制腰部 body_link 的 X/Z 平移”的现有语义，不引入双臂任务。

## 后端任务块

建议后端统一抽象为三类任务块：

1. `pose command`
   - 8 维 DQ 位姿残差
   - 用于 `absolute / relative / robot1 / robot2`

2. `distance command`
   - 1 维相对距离残差
   - 用于 `distance / distance-fullbody`

3. `partial body command`
   - 低维 body 任务残差（本轮为 `body_link x/z`）
   - 用于 `waist-body-link-xz`

这样 `mj_restore` 核心只需要理解：

- 命令残差
- 不变量残差
- 额外约束残差

而不需要知道当前 UI 模式到底叫 `relative`、`robot1` 还是 `waist-body-link-xz`。

## 实现策略

### 阶段 1：统一任务适配层

新增一个适配层模块，将：

- `controlMode`
- 当前关节角
- 当前/目标位姿或距离信息

转换成统一任务描述。

该阶段先不改变 `mj_restore` 内核，只把模式语义集中整理出来。

### 阶段 2：接入 relative / relative-fullbody

优先将 `relative` 任务族接入统一恢复器，因为它与当前 `absolute` 任务族最对称，语义最清晰。

目标：

- 不再在 `relative-target` 下回退到 `new qpJ`
- 统一发布 `MuJoCo 风格恢复` 诊断

### 阶段 3：接入特殊模式

最后接入：

- `robot1`
- `robot2`
- `waist-body-link-xz`

这三类模式的活动关节集合更简单，但任务语义更特殊，适合在 `relative` 稳定后单独收敛。

## 失败与回退策略

推荐保留“内部安全失败”能力，但不再对用户伪装为 silent fallback。

也就是说：

- 如果统一任务适配失败，应明确发布 `mode_adapter_failed`
- 如果某类任务构造失败，应发布 `command_builder_failed` 或等价原因
- 普通 `mj_restore` 在这些失败场景下应作为 **hard no-commit failure**
  - 不提交任何新的关节角更新
  - 明确给出用户可见状态文案
  - 诊断中保留失败原因
- 不再显示“当前为 MuJoCo 风格恢复”，但实际内部跑的是 `new qpJ`

对于 `mj_restore_pre_w`：

- 若 pre-w 所在模式仍未接入统一任务适配层，可继续显式显示 `pre_w: disabled_for_mode`
- 但正式第二阶段若标记为 `mj_restore`，则同样不得 silently fallback 到 `new qpJ`

这样实验、诊断和论文表述都会更干净。

## 诊断输出

统一支持后，诊断中建议额外明确以下信息：

- `modeTag`
- `commandKind`
- `invariants`
  - 每个 invariant 的 `kind`
  - 每个 invariant 的 `diagnosticLabel`
- `active_joints`
- `extra constraints`

这样在 `relative / robot1 / waist-body-link-xz` 这些模式下，可以直接看出当前 `mj_restore` 实际在解什么任务。

## 测试策略

### 单元测试

为任务适配层补测试，验证每种模式都能生成正确的统一任务描述：

- `absolute`
- `decoupled`
- `decoupled-upright`
- `relative`
- `relative-fullbody`
- `distance`
- `distance-fullbody`
- `robot1`
- `robot2`
- `waist-body-link-xz`

### 集成测试

覆盖以下行为：

1. `MuJoCo 风格恢复` 在 `relative` 下不再发布 fallback 到 `new qpJ` 的文案
2. `MuJoCo 风格恢复` 在 `distance` 下继续可用
3. `robot1 / robot2 / waist-body-link-xz` 会进入统一恢复链，而不是沿用旧单独路径
4. `mode_adapter_failed / command_builder_failed` 会 hard no-commit，且不会 silently fallback

### 回归测试

重点确保以下模式的现有行为不被带坏：

- `absolute`
- `decoupled`
- `decoupled-upright`
- `distance`
- `distance-fullbody`

并新增语义保持测试：

- `relative`：点动后绝对位姿保持不变，仅相对位姿按命令变化
- `relative-fullbody`：点动后绝对位姿保持不变，腰部可参与但不破坏定义
- `robot1`：点动左臂时右臂与腰部关节角保持不变
- `robot2`：点动右臂时左臂与腰部关节角保持不变
- `waist-body-link-xz`：点动后双臂关节角保持不变，仅腰部与 body_link X/Z 发生对应变化

## 风险

1. `relative` 任务族是“command / invariant”镜像关系，若残差方向或目标保持语义构造错误，容易出现“看似收敛但任务跑反”。
2. `robot1 / robot2` 若 invariant 设计过重，可能造成单臂模式动作过硬或近似不可动。
3. `waist-body-link-xz` 是低维局部任务，若强行套用完整 pose 语义，可能带来多余约束。

## 推荐顺序

1. 先实现统一任务适配层
2. 再接 `relative / relative-fullbody`
3. 最后接 `robot1 / robot2 / waist-body-link-xz`

该顺序能最大化复用当前 `absolute / distance` 路径，同时把风险控制在最小范围内。
