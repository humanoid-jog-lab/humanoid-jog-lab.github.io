# Default World Frame Design

## Goal

让所有点动任务在首次进入页面时默认使用世界坐标，而不是局部增量。

## Scope

- 将 `teach-multiply-order-toggle` 的默认初始化语义改为世界坐标
- 为该开关增加 `localStorage` 持久化，刷新后恢复用户上次选择
- 保持现有点动执行逻辑不变，只调整默认值与初始化来源

## Behavior

- 首次进入页面时，点动模式默认为 `世界坐标（Δ × 当前）`
- 用户手动切换为局部坐标后，刷新页面应恢复该选择
- 若本地没有已保存值，则始终回退到世界坐标默认值

## Verification

- 页面初始化时状态文案显示为世界坐标
- `visualizer.setTeachMultiplicationOrder(...)` 在首次加载时收到 `current-first`
- 手动切换后，刷新能够恢复上次选中的乘法顺序
