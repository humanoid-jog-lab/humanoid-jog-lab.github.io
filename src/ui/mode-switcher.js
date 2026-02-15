/**
 * UI 模式切换模块
 * - switchToHumanoidUI: 切换到人形模式显示
 * - switchToDualArmUI: 切换到双臂模式显示
 * - switchToSingleArmUI: 切换到单臂模式显示
 * - updateROS2ControlModeDisplay: 更新 ROS2 控制模式显示
 */

import { appState, callbacks } from '../core/state.js';

// ──────────────────────────────────────────────────────────────
// switchToHumanoidUI
// ──────────────────────────────────────────────────────────────

export function switchToHumanoidUI() {
    callbacks.startHumanoidBaseControlLoop?.();
    // 显示规划运动面板（仅人形模式）
    const plannedMotionSection = document.getElementById('planned-motion-section');
    if (plannedMotionSection) plannedMotionSection.style.display = 'block';
    // 与双臂共用：显示双臂位姿面板（左臂=robot1，右臂=robot2）
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) singleArmPose.style.display = 'none';
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'block';
        dualArmPose.classList.add('active');
    }
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) dualArmControlMode.style.display = 'block';
    const modeLabel = document.getElementById('dual-arm-mode-label');
    if (modeLabel) modeLabel.textContent = '人形：左臂/右臂位姿与相对/绝对/距离控制';
    // 位姿面板标签：左臂/右臂
    const r1Pos = document.getElementById('dual-arm-robot1-pos-label');
    const r1Quat = document.getElementById('dual-arm-robot1-quat-label');
    const r1Dq = document.getElementById('dual-arm-robot1-dq-label');
    const r2Pos = document.getElementById('dual-arm-robot2-pos-label');
    const r2Quat = document.getElementById('dual-arm-robot2-quat-label');
    const r2Dq = document.getElementById('dual-arm-robot2-dq-label');
    if (r1Pos) r1Pos.textContent = '左臂位置 (m)';
    if (r1Quat) r1Quat.textContent = '左臂四元数';
    if (r1Dq) r1Dq.textContent = '左臂 DQ';
    if (r2Pos) r2Pos.textContent = '右臂位置 (m)';
    if (r2Quat) r2Quat.textContent = '右臂四元数';
    if (r2Dq) r2Dq.textContent = '右臂 DQ';
    // 控制模式下拉：robot1/robot2 显示为 左臂/右臂，并更新模式说明
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const teachPendantControls = document.getElementById('teach-pendant-controls');
    const distanceControls = document.getElementById('dual-arm-distance-controls');
    if (controlModeSelect) {
        Array.from(controlModeSelect.options).forEach(opt => {
            if (opt.value === 'robot1') opt.textContent = '左臂';
            else if (opt.value === 'robot2') opt.textContent = '右臂';
            else if (opt.value === 'waist-body-link-xz') opt.hidden = false;
        });
        const modeLabelEl = document.getElementById('dual-arm-mode-label');
        const updateHumanoidModeLabel = () => {
            const mode = controlModeSelect.value;
            if (typeof window !== 'undefined') {
                window.currentDualArmControlMode = mode;
            }
            // 控制按钮显示：相对距离模式只显示 d-/d+
            if (teachPendantControls && distanceControls) {
                if (mode === 'distance' || mode === 'distance-fullbody') {
                    teachPendantControls.style.display = 'none';
                    distanceControls.style.display = 'grid';
                } else {
                    teachPendantControls.style.display = 'grid';
                    distanceControls.style.display = 'none';
                }
            }
            const teachButtons = document.querySelectorAll('#teach-pendant-controls .teach-btn');
            teachButtons.forEach(btn => {
                const axis = btn.dataset.axis;
                const type = btn.dataset.type || 'translation';
                const enabled = mode !== 'waist-body-link-xz' || (type !== 'rotation' && (axis === 'x' || axis === 'z'));
                btn.disabled = !enabled;
            });
            if (!modeLabelEl) return;
            if (mode === 'robot1') modeLabelEl.textContent = '左臂模式：控制左臂末端';
            else if (mode === 'robot2') modeLabelEl.textContent = '右臂模式：控制右臂末端';
            else if (mode === 'relative') modeLabelEl.textContent = '相对位姿模式：控制相对位姿，保持绝对位姿不变';
            else if (mode === 'distance') modeLabelEl.textContent = '相对距离模式：控制两末端距离，保持绝对位姿与相对姿态不变';
            else if (mode === 'distance-fullbody') modeLabelEl.textContent = '全身相对距离：腰部+双臂联动，控制两末端距离，保持绝对位姿与相对姿态不变';
            else if (mode === 'decoupled') modeLabelEl.textContent = '全身绝对位姿：腰部+双臂联动，控制绝对位姿，保持相对位姿不变';
            else if (mode === 'decoupled-upright') modeLabelEl.textContent = '全身绝对位姿（腰部竖直）：腰部+双臂联动，控制绝对位姿，同时保持 body_link 的 z 轴竖直地面';
            else if (mode === 'waist-body-link-xz') modeLabelEl.textContent = '腰部 body_link 点动：仅控制 body_link 的 X/Z 平移';
            else if (mode === 'relative-fullbody') modeLabelEl.textContent = '全身相对位姿：腰部+双臂联动，控制相对位姿，保持绝对位姿不变';
            else modeLabelEl.textContent = '绝对位姿模式：控制绝对位姿，保持相对位姿不变';
        };
        controlModeSelect.removeEventListener('change', window._humanoidModeLabelHandler);
        window._humanoidModeLabelHandler = updateHumanoidModeLabel;
        controlModeSelect.addEventListener('change', updateHumanoidModeLabel);
        updateHumanoidModeLabel();
    }
    callbacks.updateIKRobotSelection?.();
    callbacks.updateDQCalculatorRobotSelection?.();
    callbacks.updateRobotPose?.();
}

// ──────────────────────────────────────────────────────────────
// switchToDualArmUI
// ──────────────────────────────────────────────────────────────

/**
 * 切换到双臂UI显示
 */
export function switchToDualArmUI() {
    callbacks.stopHumanoidBaseControlLoop?.();
    // 隐藏规划运动面板
    const plannedMotionSection = document.getElementById('planned-motion-section');
    if (plannedMotionSection) plannedMotionSection.style.display = 'none';
    // 隐藏单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'none';
    }

    // 显示双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'block';
        dualArmPose.classList.add('active');
    }

    // 显示双臂控制模式切换开关
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) {
        dualArmControlMode.style.display = 'block';
    }

    // 移除人形模式时绑定的标签更新（若有）
    const controlModeSelectForDual = document.getElementById('dual-arm-control-mode-select');
    if (controlModeSelectForDual && window._humanoidModeLabelHandler) {
        controlModeSelectForDual.removeEventListener('change', window._humanoidModeLabelHandler);
        window._humanoidModeLabelHandler = null;
    }
    // 设置控制模式选择器的事件监听器
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const modeLabel = document.getElementById('dual-arm-mode-label');
    const teachPendantControls = document.getElementById('teach-pendant-controls');
    const distanceControls = document.getElementById('dual-arm-distance-controls');
    const waistBodyOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'waist-body-link-xz')
        : null;
    if (waistBodyOption) {
        waistBodyOption.hidden = true;
        if (controlModeSelect.value === 'waist-body-link-xz') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (controlModeSelect && modeLabel) {
        const updateModeLabel = () => {
            const mode = controlModeSelect.value;
            if (typeof window !== 'undefined') {
                window.currentDualArmControlMode = mode;
            }
            // 控制按钮显示：相对距离模式只显示 distance-/distance+
            if (teachPendantControls && distanceControls) {
                if (mode === 'distance') {
                    teachPendantControls.style.display = 'none';
                    distanceControls.style.display = 'grid';
                } else {
                    teachPendantControls.style.display = 'grid';
                    distanceControls.style.display = 'none';
                }
            }
            const teachButtons = document.querySelectorAll('#teach-pendant-controls .teach-btn');
            teachButtons.forEach(btn => { btn.disabled = false; });
            switch (mode) {
                case 'robot1':
                    modeLabel.textContent = '机器人1模式：控制机器人1的末端执行器';
                    break;
                case 'robot2':
                    modeLabel.textContent = '机器人2模式：控制机器人2的末端执行器';
                    break;
                case 'relative':
                    modeLabel.textContent = '相对位姿模式：控制相对位姿，保持绝对位姿不变';
                    break;
                case 'relative-fullbody':
                    modeLabel.textContent = '全身相对位姿：腰部+双臂联动，控制相对位姿，保持绝对位姿不变';
                    break;
                case 'distance':
                    modeLabel.textContent = '相对距离模式：控制两末端距离，保持绝对位姿与相对姿态不变';
                    break;
                case 'decoupled':
                    modeLabel.textContent = '全身绝对位姿：腰部+双臂联动，控制绝对位姿，保持相对位姿不变';
                    break;
                case 'decoupled-upright':
                    modeLabel.textContent = '全身绝对位姿（腰部竖直）：腰部+双臂联动，控制绝对位姿，同时保持 body_link 的 z 轴竖直地面';
                    break;
                case 'absolute':
                default:
                    modeLabel.textContent = '绝对位姿模式：控制绝对位姿，保持相对位姿不变';
                    break;
            }
            callbacks.renderEnvironmentObjectList?.();
        };
        updateModeLabel();
        controlModeSelect.addEventListener('change', updateModeLabel);
    }

    // 双臂位姿面板标签：机器人1/机器人2
    const r1Pos = document.getElementById('dual-arm-robot1-pos-label');
    const r1Quat = document.getElementById('dual-arm-robot1-quat-label');
    const r1Dq = document.getElementById('dual-arm-robot1-dq-label');
    const r2Pos = document.getElementById('dual-arm-robot2-pos-label');
    const r2Quat = document.getElementById('dual-arm-robot2-quat-label');
    const r2Dq = document.getElementById('dual-arm-robot2-dq-label');
    if (r1Pos) r1Pos.textContent = '机器人1位置 (m)';
    if (r1Quat) r1Quat.textContent = '机器人1四元数';
    if (r1Dq) r1Dq.textContent = '机器人1 DQ';
    if (r2Pos) r2Pos.textContent = '机器人2位置 (m)';
    if (r2Quat) r2Quat.textContent = '机器人2四元数';
    if (r2Dq) r2Dq.textContent = '机器人2 DQ';
    const controlModeSelectDual = document.getElementById('dual-arm-control-mode-select');
    if (controlModeSelectDual) {
        Array.from(controlModeSelectDual.options).forEach(opt => {
            if (opt.value === 'robot1') opt.textContent = '机器人1';
            else if (opt.value === 'robot2') opt.textContent = '机器人2';
        });
    }
    // 更新标签页，确保显示正确的标签内容
    if (dualArmPose && document.querySelector('[data-tab="pose"]')) {
        callbacks.switchTab?.('pose');
    }

    callbacks.renderEnvironmentObjectList?.();
    callbacks.updateIKRobotSelection?.();
    callbacks.updateDQCalculatorRobotSelection?.();
}

// ──────────────────────────────────────────────────────────────
// switchToSingleArmUI
// ──────────────────────────────────────────────────────────────

/**
 * 切换到单臂UI显示
 */
export function switchToSingleArmUI() {
    callbacks.stopHumanoidBaseControlLoop?.();
    // 隐藏规划运动面板
    const plannedMotionSection = document.getElementById('planned-motion-section');
    if (plannedMotionSection) plannedMotionSection.style.display = 'none';
    // 显示单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'block';
    }

    // 隐藏双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'none';
        dualArmPose.classList.remove('active');
    }

    // 隐藏双臂控制模式切换开关
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) {
        dualArmControlMode.style.display = 'none';
    }

    if (typeof window !== 'undefined') {
        window.currentDualArmControlMode = 'absolute';
    }

    const teachButtons = document.querySelectorAll('#teach-pendant-controls .teach-btn');
    teachButtons.forEach(btn => { btn.disabled = false; });

    callbacks.renderEnvironmentObjectList?.();
    callbacks.updateIKRobotSelection?.();
    callbacks.updateDQCalculatorRobotSelection?.();
}

// ──────────────────────────────────────────────────────────────
// updateROS2ControlModeDisplay
// ──────────────────────────────────────────────────────────────

/**
 * 更新 ROS 2 控制模式显示
 */
export function updateROS2ControlModeDisplay(mode) {
    const controlModeSelect = document.getElementById('ros2-control-mode');
    if (controlModeSelect) {
        controlModeSelect.value = mode;

        // 更新状态显示
        const modeStatusEl = document.getElementById('ros2-mode-status');
        if (modeStatusEl) {
            const modeText = mode === 'manual' ? '手动模式 - 可直接控制机器人关节' :
                           mode === 'teleop' ? '遥操作模式 - 通过VR设备控制' : '未知模式';
            modeStatusEl.textContent = `当前模式：${modeText}`;
        }
    }
}

// ──────────────────────────────────────────────────────────────
// 回调注册
// ──────────────────────────────────────────────────────────────

export function registerModeSwitcherCallbacks() {
    callbacks.switchToSingleArmUI = switchToSingleArmUI;
    callbacks.switchToDualArmUI = switchToDualArmUI;
    callbacks.switchToHumanoidUI = switchToHumanoidUI;
    callbacks.updateROS2ControlModeDisplay = updateROS2ControlModeDisplay;
}
