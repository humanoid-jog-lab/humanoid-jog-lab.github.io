# Remove Planned Motion Design

## Goal

从 `dq_vizlab_academic` 中彻底移除“规划运动”和“执行关节实时监控”前端功能，避免界面、事件接线、状态槽位和独立监控页继续残留。

## Scope

- 删除 `index.html` 中的 `planned-motion-section`
- 删除 `src/features/planned-motion.js`
- 删除 `src/features/joint-monitor-window.js` 与 `joint-monitor.html`
- 删除 `src/main.js`、`src/ui/event-handlers.js`、`src/ui/mode-switcher.js`、`src/core/state.js` 中的接线与状态占位
- 删除 `src/styles.css` 中全部 `.planned-motion-*` 样式

## Verification

- 回归测试确认源码树中不再存在 `planned-motion` 面板和监控页入口
- `rg` 检查业务源码中不再引用 `planned-motion` / `joint-monitor-window`
