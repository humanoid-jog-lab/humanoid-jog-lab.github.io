# Left Ruled-Surface Polishing Design

## Goal

在 `dq_vizlab_academic` 中新增一个全新的单手人形模式 `left-ruled-surface-polishing`，让左手末端按照论文中的单手 ruled-surface primitive 进行显式参数雅可比控制。这个模式需要在界面中可选，并且走显式 `J_surf^(y)` 求解链，而不是复用旧的 `surface-slide` 模式定义。

## Constraints

- 这是一个新模式，不恢复旧的 `surface-slide` / `surface-slide-absolute`
- 只做左手版本
- 右臂和腰部在该模式下保持固定
- 保留当前工程增强：
  - 仍允许使用 surface-attached `quaternionOffset`
  - 仍允许使用 Plucker tube invariant 保护接触带
  - 继续使用 `mj_restore` 系列求解器外壳

## UI Behavior

- 在 humanoid 控制模式下拉中新增 `Left Ruled-Surface Polishing`
- 该模式只在 humanoid UI 中显示，在 dual-arm UI 中隐藏
- teach pendant 映射改为：
  - `x -> u`
  - `y -> v`
  - `z -> n`
- 所有旋转按钮在该模式下禁用
- 切换到该模式时初始化单手 ruled-surface 状态，并刷新当前 surface 可视化

## Solver Design

- 复用现有 surface 投影、surface marker、surface demo geometry 和 tube invariant 工具
- 基于已新增的 `buildSurfaceAttachedPoseJacobian(...)` 构建单手显式 `J_surf^(y)`
- 用 `Δy = [Δu, Δv, Δn]` 生成 8 维 DQ command differential
- 新增单手 differential restore helper，使左手走：
  - `J_x Δq ≈ J_surf^(y) Δy`
- 当前左手 pose 作为 differential 基准
- 右臂 pose 作为 invariant 固定
- 不再走旧的“先生成 pose target，再恢复”的 single-arm surface-slide 路径

## Implementation Areas

- `index.html`
  - 新增 humanoid 模式选项
- `src/ui/mode-switcher.js`
  - 新增模式说明
  - 新增 `u / v / n` 标签和按钮禁用规则
- `src/ui/event-handlers.js`
  - 将新模式分发到新的 left-hand ruled-surface move handler
- `src/core/humanoid-jog-mj-task-adapter.js`
  - 新增 `left-ruled-surface-polishing` descriptor
- `src/core/humanoid-jog-mj-mode-support.js`
  - 允许该模式进入 `mj_restore` 支持链
- `src/core/humanoid-jog-mj-profile.js`
  - 为该模式分配与 cooperative polishing 一致的 restoration profile
- `src/visualizer.js`
  - 新增单手 ruled-surface state 初始化
  - 新增单手显式 command builder
  - 新增单手 differential restore helper
  - 新增按 `u / v / n` 推进参数的入口

## Testing

- UI 测试：
  - humanoid 下拉中出现 `left-ruled-surface-polishing`
  - 旧 `surface-slide` 相关 mode 名仍不出现在界面
- mode switcher 测试：
  - 该模式下按钮标签为 `u / v / n`
  - 旋转按钮被禁用
- Jacobian 测试：
  - 平面表面下 `J_surf^(y)` 的三列与期望一致
- behavior 测试：
  - 参数按钮触发新的显式 differential restore helper
  - 右臂固定 invariant 会被正确带入 restoration
- verification：
  - 相关 Node 测试通过
  - `npm run build` 通过
