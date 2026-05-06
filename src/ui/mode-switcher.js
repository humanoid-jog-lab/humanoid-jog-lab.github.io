/**
 * UI 模式切换模块
 * - switchToHumanoidUI: 切换到人形模式显示
 * - switchToDualArmUI: 切换到双臂模式显示
 * - switchToSingleArmUI: 切换到单臂模式显示
 * - updateROS2ControlModeDisplay: 更新 ROS2 控制模式显示
 */

import { appState, callbacks } from '../core/state.js';
import { onLanguageChange, t } from '../i18n.js';

let modeSwitcherLanguageBound = false;
const HUMANOID_QUICK_JOG_AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];

function isHumanoidCooperativePolishingMode(mode) {
    return (
        mode === 'cooperative-ruled-surface-polishing-dq-setpoint'
        || mode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'
    );
}

function isHumanoidLeftPolishingMode(mode) {
    return mode === 'left-ruled-surface-polishing';
}

export function getHumanoidQuickJogControlState(mode = 'absolute') {
    const showDistanceControls = mode === 'distance' || mode === 'distance-fullbody';
    const axisLabels = {
        x: 'X',
        y: 'Y',
        z: 'Z',
        rx: 'Rx',
        ry: 'Ry',
        rz: 'Rz'
    };
    const visibleAxes = new Set(HUMANOID_QUICK_JOG_AXES);

    if (showDistanceControls) {
        visibleAxes.clear();
    } else if (isHumanoidCooperativePolishingMode(mode)) {
        Object.assign(axisLabels, {
            x: 'u',
            y: 'vc',
            z: 'n',
            rx: 'Δv'
        });
        visibleAxes.clear();
        ['x', 'y', 'z', 'rx'].forEach((axis) => visibleAxes.add(axis));
    } else if (isHumanoidLeftPolishingMode(mode)) {
        Object.assign(axisLabels, {
            x: 'u',
            y: 'v',
            z: 'n'
        });
        visibleAxes.clear();
        ['x', 'y', 'z'].forEach((axis) => visibleAxes.add(axis));
    } else if (mode === 'waist-body-link-xz') {
        visibleAxes.clear();
        ['x', 'z'].forEach((axis) => visibleAxes.add(axis));
    }

    const axes = Object.fromEntries(
        HUMANOID_QUICK_JOG_AXES.map((axis) => [
            axis,
            {
                label: axisLabels[axis],
                visible: visibleAxes.has(axis)
            }
        ])
    );

    return {
        showDistanceControls,
        axes
    };
}

export function updateHumanoidQuickJogControls(mode = 'absolute') {
    const state = getHumanoidQuickJogControlState(mode);
    const quickJogControls = document.querySelector('.quick-jog-controls');
    const quickDistanceControls = document.getElementById('quick-distance-controls');

    if (quickJogControls) {
        quickJogControls.style.display = state.showDistanceControls ? 'none' : 'flex';
    }
    if (quickDistanceControls) {
        quickDistanceControls.style.display = state.showDistanceControls ? 'flex' : 'none';
    }

    Object.entries(state.axes).forEach(([axis, axisState]) => {
        const negativeBtn = document.querySelector(`.quick-jog-controls .quick-jog-btn[data-axis="${axis}"][data-direction="negative"]`);
        const positiveBtn = document.querySelector(`.quick-jog-controls .quick-jog-btn[data-axis="${axis}"][data-direction="positive"]`);
        const group = negativeBtn?.closest('.quick-axis-control') ?? positiveBtn?.closest('.quick-axis-control');
        const label = group?.querySelector('.quick-axis-label') ?? group?.querySelector('span');
        if (group) group.hidden = !axisState.visible;
        if (label) label.textContent = axisState.label;
        if (negativeBtn) negativeBtn.title = `${axisState.label} negative`;
        if (positiveBtn) positiveBtn.title = `${axisState.label} positive`;
    });
}

function getHumanoidModeDescription(mode) {
    if (mode === 'robot1') return t('Left Arm Mode: control the left end-effector');
    if (mode === 'robot1-waist') return t('Left Arm + Waist Mode: control the left end-effector with waist cooperation while keeping the right end-effector fixed');
    if (mode === 'robot2') return t('Right Arm Mode: control the right end-effector');
    if (mode === 'robot2-waist') return t('Right Arm + Waist Mode: control the right end-effector with waist cooperation while keeping the left end-effector fixed');
    if (mode === 'left-ruled-surface-polishing') return t('Left Ruled-Surface Polishing: control u, v, and n while keeping the right arm and waist fixed');
    if (mode === 'cooperative-ruled-surface-polishing-dq-setpoint') return t('Cooperative Surface Polishing (DQ Set-point, Waist Maintained): use the same shared strip parameters and true dual-quaternion pose targets while keeping waist participation and body-z maintenance');
    if (mode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only') return t('Cooperative Surface Polishing (DQ Set-point, Arms Only): use the same shared strip parameters and true dual-quaternion pose targets without waist participation');
    if (mode === 'relative') return t('Relative Pose Mode: control the relative pose while keeping the absolute pose unchanged');
    if (mode === 'distance') return t('Relative Distance Mode: control the distance between the two end-effectors while keeping absolute pose and relative orientation unchanged');
    if (mode === 'distance-fullbody') return t('Whole-Body Relative Distance: coordinate waist and both arms to control the end-effector distance while keeping absolute pose and relative orientation unchanged');
    if (mode === 'decoupled') return t('Whole-Body Absolute Pose Mode: coordinate waist and both arms to control the absolute pose while keeping the relative pose unchanged');
    if (mode === 'decoupled-upright') return t('Whole-Body Absolute Pose Mode (Upright Waist): coordinate waist and both arms to control the absolute pose while keeping the body_link z-axis upright');
    if (mode === 'waist-body-link-xz') return t('Waist body_link Jog Mode: control only the body_link X/Z translation');
    if (mode === 'relative-fullbody') return t('Whole-Body Relative Pose Mode: coordinate waist and both arms to control the relative pose while keeping the absolute pose unchanged');
    return t('Absolute Pose Mode: control the absolute pose while keeping the relative pose unchanged');
}

function getDualArmModeDescription(mode) {
    if (mode === 'robot1') return t('Robot 1 Mode: control robot 1 end-effector');
    if (mode === 'robot1-waist') return t('Left Arm + Waist Mode: control the left end-effector with waist cooperation while keeping the right end-effector fixed');
    if (mode === 'robot2') return t('Robot 2 Mode: control robot 2 end-effector');
    if (mode === 'robot2-waist') return t('Right Arm + Waist Mode: control the right end-effector with waist cooperation while keeping the left end-effector fixed');
    if (mode === 'relative') return t('Relative Pose Mode: control the relative pose while keeping the absolute pose unchanged');
    if (mode === 'relative-fullbody') return t('Whole-Body Relative Pose Mode: coordinate waist and both arms to control the relative pose while keeping the absolute pose unchanged');
    if (mode === 'distance') return t('Relative Distance Mode: control the distance between the two end-effectors while keeping absolute pose and relative orientation unchanged');
    if (mode === 'decoupled') return t('Whole-Body Absolute Pose Mode: coordinate waist and both arms to control the absolute pose while keeping the relative pose unchanged');
    if (mode === 'decoupled-upright') return t('Whole-Body Absolute Pose Mode (Upright Waist): coordinate waist and both arms to control the absolute pose while keeping the body_link z-axis upright');
    return t('Absolute Pose Mode: control the absolute pose while keeping the relative pose unchanged');
}

function setTeachPendantSurfaceLabels(mode = 'none') {
    const translationSchemas = {
        none: {
            x: { label: t('X Axis'), negative: 'X-', positive: 'X+' },
            y: { label: t('Y Axis'), negative: 'Y-', positive: 'Y+' },
            z: { label: t('Z Axis'), negative: 'Z-', positive: 'Z+' }
        },
        cooperative: {
            x: { label: 'u', negative: 'u-', positive: 'u+' },
            y: { label: 'vc', negative: 'vc-', positive: 'vc+' },
            z: { label: 'n', negative: 'n-', positive: 'n+' }
        },
        single: {
            x: { label: 'u', negative: 'u-', positive: 'u+' },
            y: { label: 'v', negative: 'v-', positive: 'v+' },
            z: { label: 'n', negative: 'n-', positive: 'n+' }
        }
    };
    const rotationSchemas = {
        none: {
            rx: { label: 'Rx', negative: 'Rx-', positive: 'Rx+' },
            ry: { label: 'Ry', negative: 'Ry-', positive: 'Ry+' },
            rz: { label: 'Rz', negative: 'Rz-', positive: 'Rz+' }
        },
        cooperative: {
            rx: { label: 'Δv', negative: 'Δv-', positive: 'Δv+' },
            ry: { label: 'Ry', negative: 'Ry-', positive: 'Ry+' },
            rz: { label: 'Rz', negative: 'Rz-', positive: 'Rz+' }
        }
    };
    const translationMode = mode === 'cooperative'
        ? 'cooperative'
        : (mode === 'single' ? 'single' : 'none');
    const rotationMode = mode === 'cooperative' ? 'cooperative' : 'none';

    Object.entries(translationSchemas[translationMode]).forEach(([axis, schema]) => {
        const negativeBtn = document.querySelector(`#teach-pendant-controls .teach-btn[data-axis="${axis}"][data-direction="negative"]:not([data-type="rotation"])`);
        const positiveBtn = document.querySelector(`#teach-pendant-controls .teach-btn[data-axis="${axis}"][data-direction="positive"]:not([data-type="rotation"])`);
        const axisLabel = negativeBtn?.closest('.axis-control-group')?.querySelector('.axis-label');
        if (axisLabel) axisLabel.textContent = schema.label;
        if (negativeBtn) negativeBtn.textContent = schema.negative;
        if (positiveBtn) positiveBtn.textContent = schema.positive;
    });

    Object.entries(rotationSchemas[rotationMode]).forEach(([axis, schema]) => {
        const negativeBtn = document.querySelector(`#teach-pendant-controls .teach-btn[data-axis="${axis}"][data-direction="negative"][data-type="rotation"]`);
        const positiveBtn = document.querySelector(`#teach-pendant-controls .teach-btn[data-axis="${axis}"][data-direction="positive"][data-type="rotation"]`);
        const axisLabel = negativeBtn?.closest('.axis-control-group')?.querySelector('.axis-label');
        if (axisLabel) axisLabel.textContent = schema.label;
        if (negativeBtn) negativeBtn.textContent = schema.negative;
        if (positiveBtn) positiveBtn.textContent = schema.positive;
    });
}

// ──────────────────────────────────────────────────────────────
// switchToHumanoidUI
// ──────────────────────────────────────────────────────────────

export function switchToHumanoidUI() {
    callbacks.startHumanoidBaseControlLoop?.();
    // 与双臂共用：显示双臂位姿面板（左臂=robot1，右臂=robot2）
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) singleArmPose.style.display = 'none';
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'block';
    }
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) dualArmControlMode.style.display = 'block';
    const modeLabel = document.getElementById('dual-arm-mode-label');
    if (modeLabel) modeLabel.textContent = t('Humanoid: left/right arm pose with relative/absolute/distance control');
    // 位姿面板标签：左臂/右臂
    const r1Pos = document.getElementById('dual-arm-robot1-pos-label');
    const r1Quat = document.getElementById('dual-arm-robot1-quat-label');
    const r1Dq = document.getElementById('dual-arm-robot1-dq-label');
    const r2Pos = document.getElementById('dual-arm-robot2-pos-label');
    const r2Quat = document.getElementById('dual-arm-robot2-quat-label');
    const r2Dq = document.getElementById('dual-arm-robot2-dq-label');
    if (r1Pos) r1Pos.textContent = t('Left Arm Position (m)');
    if (r1Quat) r1Quat.textContent = t('Left Arm Quaternion');
    if (r1Dq) r1Dq.textContent = t('Left Arm DQ');
    if (r2Pos) r2Pos.textContent = t('Right Arm Position (m)');
    if (r2Quat) r2Quat.textContent = t('Right Arm Quaternion');
    if (r2Dq) r2Dq.textContent = t('Right Arm DQ');
    // 控制模式下拉：robot1/robot2 显示为 左臂/右臂，并更新模式说明
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const teachPendantControls = document.getElementById('teach-pendant-controls');
    const distanceControls = document.getElementById('dual-arm-distance-controls');
    if (controlModeSelect) {
        Array.from(controlModeSelect.options).forEach(opt => {
            if (opt.value === 'robot1') opt.textContent = t('Left Arm');
            else if (opt.value === 'robot1-waist') opt.textContent = t('Left Arm + Waist');
            else if (opt.value === 'robot2') opt.textContent = t('Right Arm');
            else if (opt.value === 'robot2-waist') opt.textContent = t('Right Arm + Waist');
            else if (opt.value === 'left-ruled-surface-polishing') {
                opt.textContent = t('Left Ruled-Surface Polishing');
                opt.hidden = false;
            }
            else if (opt.value === 'cooperative-ruled-surface-polishing-dq-setpoint') {
                opt.textContent = t('Cooperative Surface Polishing (DQ Set-point, Waist Maintained)');
                opt.hidden = false;
            }
            else if (opt.value === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only') {
                opt.textContent = t('Cooperative Surface Polishing (DQ Set-point, Arms Only)');
                opt.hidden = false;
            }
            else if (opt.value === 'waist-body-link-xz') opt.hidden = false;
        });
        const modeLabelEl = document.getElementById('dual-arm-mode-label');
        const updateHumanoidModeLabel = () => {
            const mode = controlModeSelect.value;
            if (typeof window !== 'undefined') {
                window.currentDualArmControlMode = mode;
                window.refreshHumanoidJogMetricOptionLabels?.(mode);
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
            const isLeftRuledSurfacePolishingMode = mode === 'left-ruled-surface-polishing';
            const isCooperativePolishingMode =
                mode === 'cooperative-ruled-surface-polishing-dq-setpoint'
                || mode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only';
            setTeachPendantSurfaceLabels(
                isCooperativePolishingMode
                    ? 'cooperative'
                    : (isLeftRuledSurfacePolishingMode ? 'single' : 'none')
            );
            updateHumanoidQuickJogControls(mode);
            const teachButtons = document.querySelectorAll('#teach-pendant-controls .teach-btn');
            teachButtons.forEach(btn => {
                const axis = btn.dataset.axis;
                const type = btn.dataset.type || 'translation';
                const enabled = isCooperativePolishingMode
                    ? (
                        ((type !== 'rotation') && (axis === 'x' || axis === 'y' || axis === 'z'))
                        || ((type === 'rotation') && axis === 'rx')
                    )
                    : isLeftRuledSurfacePolishingMode
                        ? (type !== 'rotation' && (axis === 'x' || axis === 'y' || axis === 'z'))
                        : (mode !== 'waist-body-link-xz' || (type !== 'rotation' && (axis === 'x' || axis === 'z')));
                btn.disabled = !enabled;
            });
            if (isLeftRuledSurfacePolishingMode) {
                appState.visualizer?.prepareHumanoidLeftRuledSurfacePolishingMode?.();
            }
            appState.visualizer?.refreshHumanoidSurfaceSlideVisualization?.();
            if (!modeLabelEl) return;
            modeLabelEl.textContent = getHumanoidModeDescription(mode);
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
    // 隐藏单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'none';
    }

    // 显示双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'block';
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
    const robot1WaistOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'robot1-waist')
        : null;
    const robot2WaistOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'robot2-waist')
        : null;
    const cooperativePolishingDQSetpointOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'cooperative-ruled-surface-polishing-dq-setpoint')
        : null;
    const cooperativePolishingDQSetpointArmOnlyOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only')
        : null;
    const leftRuledSurfacePolishingOption = controlModeSelect
        ? Array.from(controlModeSelect.options).find(opt => opt.value === 'left-ruled-surface-polishing')
        : null;
    if (waistBodyOption) {
        waistBodyOption.hidden = true;
        if (controlModeSelect.value === 'waist-body-link-xz') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (robot1WaistOption) {
        robot1WaistOption.hidden = true;
        if (controlModeSelect.value === 'robot1-waist') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (robot2WaistOption) {
        robot2WaistOption.hidden = true;
        if (controlModeSelect.value === 'robot2-waist') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (cooperativePolishingDQSetpointOption) {
        cooperativePolishingDQSetpointOption.hidden = true;
        if (controlModeSelect.value === 'cooperative-ruled-surface-polishing-dq-setpoint') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (cooperativePolishingDQSetpointArmOnlyOption) {
        cooperativePolishingDQSetpointArmOnlyOption.hidden = true;
        if (controlModeSelect.value === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (leftRuledSurfacePolishingOption) {
        leftRuledSurfacePolishingOption.hidden = true;
        if (controlModeSelect.value === 'left-ruled-surface-polishing') {
            controlModeSelect.value = 'absolute';
        }
    }
    if (controlModeSelect && modeLabel) {
        const updateModeLabel = () => {
            const mode = controlModeSelect.value;
            if (typeof window !== 'undefined') {
                window.currentDualArmControlMode = mode;
                window.refreshHumanoidJogMetricOptionLabels?.(mode);
            }
            setTeachPendantSurfaceLabels('none');
            updateHumanoidQuickJogControls(mode);
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
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'robot1-waist':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'robot2':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'robot2-waist':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'relative':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'relative-fullbody':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'distance':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'decoupled':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'decoupled-upright':
                    modeLabel.textContent = getDualArmModeDescription(mode);
                    break;
                case 'absolute':
                default:
                    modeLabel.textContent = getDualArmModeDescription(mode);
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
    if (r1Pos) r1Pos.textContent = t('Robot 1 Position (m)');
    if (r1Quat) r1Quat.textContent = t('Robot 1 Quaternion');
    if (r1Dq) r1Dq.textContent = t('Robot 1 DQ');
    if (r2Pos) r2Pos.textContent = t('Robot 2 Position (m)');
    if (r2Quat) r2Quat.textContent = t('Robot 2 Quaternion');
    if (r2Dq) r2Dq.textContent = t('Robot 2 DQ');
    const controlModeSelectDual = document.getElementById('dual-arm-control-mode-select');
    if (controlModeSelectDual) {
        Array.from(controlModeSelectDual.options).forEach(opt => {
            if (opt.value === 'robot1') opt.textContent = t('Robot 1');
            else if (opt.value === 'robot1-waist') opt.hidden = true;
            else if (opt.value === 'robot2') opt.textContent = t('Robot 2');
            else if (opt.value === 'robot2-waist') opt.hidden = true;
            else if (opt.value === 'left-ruled-surface-polishing') opt.hidden = true;
        });
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
    // 显示单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'block';
    }

    // 隐藏双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'none';
    }

    // 隐藏双臂控制模式切换开关
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) {
        dualArmControlMode.style.display = 'none';
    }

    if (typeof window !== 'undefined') {
        window.currentDualArmControlMode = 'absolute';
    }

    setTeachPendantSurfaceLabels('none');
    updateHumanoidQuickJogControls('absolute');
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
            const modeText = mode === 'manual'
                ? t('Manual Mode - direct joint control available')
                : mode === 'teleop'
                    ? t('Teleoperation Mode - controlled via VR devices')
                    : t('Unknown Mode');
            modeStatusEl.textContent = `${t('Current Mode:')} ${modeText}`;
        }
    }
}

// ──────────────────────────────────────────────────────────────
// 回调注册
// ──────────────────────────────────────────────────────────────

export function registerModeSwitcherCallbacks() {
    if (!modeSwitcherLanguageBound) {
        modeSwitcherLanguageBound = true;
        onLanguageChange(() => {
            if (appState.humanoidMode) {
                switchToHumanoidUI();
            } else if (appState.dualArmMode) {
                switchToDualArmUI();
            } else {
                switchToSingleArmUI();
            }
            const ros2Mode = document.getElementById('ros2-control-mode')?.value;
            if (ros2Mode) {
                updateROS2ControlModeDisplay(ros2Mode);
            }
        });
    }
    callbacks.switchToSingleArmUI = switchToSingleArmUI;
    callbacks.switchToDualArmUI = switchToDualArmUI;
    callbacks.switchToHumanoidUI = switchToHumanoidUI;
    callbacks.updateROS2ControlModeDisplay = updateROS2ControlModeDisplay;
}
