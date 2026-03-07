/**
 * DQ Robotics 可视化实验室 — 入口文件
 *
 * 本文件仅负责：
 *  1. 导入各功能模块
 *  2. 注册跨模块回调
 *  3. 执行初始化流程
 *
 * 所有业务逻辑已拆分到 core/ utils/ ui/ features/ humanoid/ ros2/ 目录。
 */

// ── WASM 初始化 ──────────────────────────────
import { initDQRobotics } from './dqrobotics.js';

// ── 核心模块 ──────────────────────────────────
import { initVisualizer, initDefaultIARobot, registerRobotManagerCallbacks } from './core/robot-manager.js';

// ── UI 模块 ──────────────────────────────────
import { registerJointControlsCallbacks } from './ui/joint-controls.js';
import { registerDisplayCallbacks } from './ui/display.js';
import { registerEnvironmentPanelCallbacks } from './ui/environment-panel.js';
import { registerPanelManagerCallbacks } from './ui/panel-manager.js';
import { registerModeSwitcherCallbacks } from './ui/mode-switcher.js';
import { registerEventHandlersCallbacks } from './ui/event-handlers.js';
import { setupEventListeners } from './ui/event-handlers.js';

// ── 功能模块 ──────────────────────────────────
import { registerIKControllerCallbacks } from './features/ik-controller.js';
import { registerDQCalculatorCallbacks } from './features/dq-calculator.js';
import { registerJointClipboardCallbacks } from './features/joint-clipboard.js';
import { registerPlannedMotionCallbacks } from './features/planned-motion.js';

// ── 人形机器人 ────────────────────────────────
import { registerHumanoidBaseControlCallbacks } from './humanoid/base-control.js';

// ── ROS 2 通讯 ────────────────────────────────
import { registerROS2PanelCallbacks } from './ros2/ros2-panel.js';

// ══════════════════════════════════════════════
//  回调注册（必须在任何业务逻辑之前完成）
// ══════════════════════════════════════════════

function registerAllCallbacks() {
    // 核心
    registerRobotManagerCallbacks();

    // UI
    registerJointControlsCallbacks();
    registerDisplayCallbacks();
    registerEnvironmentPanelCallbacks();
    registerPanelManagerCallbacks();
    registerModeSwitcherCallbacks();
    registerEventHandlersCallbacks();

    // 功能
    registerIKControllerCallbacks();
    registerDQCalculatorCallbacks();
    registerJointClipboardCallbacks();
    registerPlannedMotionCallbacks();

    // 人形
    registerHumanoidBaseControlCallbacks();

    // ROS 2
    registerROS2PanelCallbacks();
}

// ══════════════════════════════════════════════
//  应用初始化
// ══════════════════════════════════════════════

async function init() {
    try {
        // 1. 初始化 WASM 模块（DQ Robotics）
        await initDQRobotics();

        // 2. 注册所有跨模块回调
        registerAllCallbacks();

        // 3. 隐藏加载界面
        document.getElementById('loading').classList.add('hidden');

        // 4. 初始化三维可视化器
        await initVisualizer();

        // 5. 绑定所有事件监听器
        setupEventListeners();

        // 6. 默认加载机器人（略作延迟，确保 UI 就绪）
        setTimeout(() => {
            initDefaultIARobot();
        }, 100);

        console.log('✓ 应用初始化完成');
    } catch (error) {
        console.error('初始化失败:', error);
        document.getElementById('loading').innerHTML = `
            <div style="color: #ef4444;">
                <h2>❌ 加载失败</h2>
                <p>${error.message}</p>
                <p style="font-size: 0.875rem; margin-top: 1rem;">
                    请确保已正确构建 WASM 模块
                </p>
            </div>
        `;
    }
}

// ── 启动 ──────────────────────────────────────

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
