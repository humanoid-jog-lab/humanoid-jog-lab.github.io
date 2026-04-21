# Bilingual UI Design

## Goal

为 `dq_vizlab_academic` 增加中英文界面切换，默认显示英文，并在用户切换后记住上次选择。切换语言时，静态界面文案、运行时按钮状态、提示文案、弹窗提示和页面标题都需要同步切换。

## Scope

- 默认语言改为英文
- 在主控制面板增加语言切换入口
- 使用 `localStorage` 持久化语言选择
- 静态 DOM 文案支持中英文切换
- 运行时动态文案支持中英文切换
  - 按钮状态文本
  - 状态提示文本
  - 弹窗 / prompt / alert 文案
  - 主要模式标签与说明
- 不翻译用户输入、URDF/机器人原始名称和控制台日志注释

## Architecture

新增一个轻量 i18n 模块，负责：

- 管理当前语言与本地持久化
- 提供 `t(key, params)` 翻译函数
- 提供静态 DOM 应用函数
- 提供语言切换订阅机制，供运行时模块刷新当前显示状态

静态页面文案以英文为默认内容，语言模块在初始化时根据保存的设置统一重刷。动态模块中的硬编码中文改为翻译 key，在状态变化或语言切换时重新渲染当前文本。

## Initial Coverage

- `index.html`
- `src/main.js`
- `src/ui/event-handlers.js`
- `src/ui/mode-switcher.js`
- `src/ui/environment-panel.js`
- `src/ui/drag-drop.js`
- `src/ui/joint-controls.js`
- `src/ros2/ros2-panel.js`
- `src/core/humanoid-jog-solver-presentation.js`

## Verification

- 回归测试验证默认语言为英文、语言选择会持久化、静态翻译和动态翻译 helper 正常工作
- 前端构建通过
- 源码搜索确认语言入口与 i18n 初始化已接入主流程
