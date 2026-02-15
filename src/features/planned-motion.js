/**
 * 规划运动模块
 *
 * 实现两阶段绝对位姿运动规划：
 *   阶段 1（可选）：如果位置偏差 > 阈值，先通过底盘移动到目标附近
 *   阶段 2：底盘到位后，通过机械臂 IK 精确补偿残余误差
 *
 * 状态机：IDLE → BASE_MOVING → ARM_COMPENSATING → DONE
 */

import { appState, callbacks } from '../core/state.js';
import { getDQModule } from '../dqrobotics.js';
import { wrapAngle } from '../utils/math-utils.js';

// ── 状态机状态 ──────────────────────────────────
const STATE = {
    IDLE: 'idle',
    BASE_MOVING: 'base_moving',
    ARM_COMPENSATING: 'arm_compensating',
    DONE: 'done',
    ERROR: 'error',
};

// ── 默认参数 ──────────────────────────────────
const DEFAULT_PARAMS = {
    baseThreshold: 0.40,        // 底盘启动阈值（米）
    baseSettleCycles: 10,       // 底盘收敛判定次数
    armIKIncludeWaist: true,    // 手臂 IK 是否包含腰部关节
    pollInterval: 50,           // 状态轮询间隔（ms）
    armCompensationDuration: 0.5,  // 手臂补偿动画时长（秒）
    armCompensationEase: 'easeInOutCubic',
};

// ── 模块状态 ──────────────────────────────────
let motionState = STATE.IDLE;
let motionTarget = null;        // 目标绝对位姿 DQ (8 元素数组)
let motionParams = { ...DEFAULT_PARAMS };
let motionPollTimer = null;
let motionArmAnimationId = null;  // requestAnimationFrame ID，用于取消手臂动画
let motionStatusCallback = null; // UI 状态更新回调

// 缓动：t in [0,1] -> [0,1]
function easeInOutCubic(t) {
    return t <= 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

// ── 公共 API ──────────────────────────────────

/**
 * 获取当前运动状态
 */
export function getPlannedMotionState() {
    return motionState;
}

/**
 * 是否正在执行规划运动
 */
export function isPlannedMotionRunning() {
    return motionState === STATE.BASE_MOVING || motionState === STATE.ARM_COMPENSATING;
}

/**
 * 设置状态更新回调
 */
export function setPlannedMotionStatusCallback(cb) {
    motionStatusCallback = cb;
}

/**
 * 执行规划运动
 * @param {number[]} targetDQArray - 目标绝对位姿（世界坐标系，8 元素 DQ 数组）
 * @param {Object} [options] - 可选参数覆盖
 * @param {number} [options.baseThreshold] - 底盘启动阈值（米）
 */
export function executePlannedMotion(targetDQArray, options = {}) {
    if (!appState.humanoidMode || !appState.visualizer) {
        updateStatus(STATE.ERROR, '仅在人形模式下可用');
        return false;
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        updateStatus(STATE.ERROR, 'DQ 模块未加载');
        return false;
    }

    if (!Array.isArray(targetDQArray) || targetDQArray.length !== 8) {
        updateStatus(STATE.ERROR, '目标 DQ 格式无效（需要 8 个数）');
        return false;
    }

    // 停止之前可能正在运行的规划
    cancelPlannedMotion();

    motionTarget = [...targetDQArray];
    motionParams = { ...DEFAULT_PARAMS, ...options };

    // 计算当前绝对位姿（世界坐标系）
    const currentWorldAbs = getCurrentWorldAbsolutePose(DQ);
    if (!currentWorldAbs) {
        updateStatus(STATE.ERROR, '无法获取当前绝对位姿');
        return false;
    }

    // 计算位置偏差
    const posError = computePositionError(DQ, currentWorldAbs, motionTarget);
    console.log(`规划运动：位置偏差 = ${(posError * 100).toFixed(1)} cm，阈值 = ${(motionParams.baseThreshold * 100).toFixed(1)} cm`);

    if (posError > motionParams.baseThreshold) {
        // 偏差大 → 先移动底盘
        startBaseMovement(DQ, currentWorldAbs, motionTarget);
    } else {
        // 偏差小 → 直接手臂补偿
        startArmCompensation();
    }

    return true;
}

/**
 * 取消正在执行的规划运动
 */
export function cancelPlannedMotion() {
    if (motionPollTimer) {
        clearInterval(motionPollTimer);
        motionPollTimer = null;
    }
    if (motionArmAnimationId != null) {
        cancelAnimationFrame(motionArmAnimationId);
        motionArmAnimationId = null;
    }
    if (motionState !== STATE.IDLE) {
        motionState = STATE.IDLE;
        motionTarget = null;
        updateStatus(STATE.IDLE, '已取消');
    }
}

// ── 内部逻辑 ──────────────────────────────────

/**
 * 获取当前世界坐标系下的绝对位姿 DQ 数组
 */
function getCurrentWorldAbsolutePose(DQ) {
    const vis = appState.visualizer;
    if (!vis || !vis._getHumanoidAbsolutePose || !vis._composeHumanoidWorldPose) return null;

    const q = window.humanoidQ;
    if (!q || q.length === 0) return null;

    const absLocal = vis._getHumanoidAbsolutePose(q);
    if (!absLocal) return null;

    const basePose = appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 };
    return vis._composeHumanoidWorldPose(basePose, absLocal) || absLocal;
}

/**
 * 从 DQ 数组提取 XY 平面上的位置偏差
 */
function computePositionError(DQ, currentDQArray, targetDQArray) {
    const dqCur = DQ.createFromArray(currentDQArray);
    const dqTgt = DQ.createFromArray(targetDQArray);
    const tCur = DQ.getTranslation(dqCur); // [tw, tx, ty, tz]
    const tTgt = DQ.getTranslation(dqTgt);

    const dx = tTgt[1] - tCur[1];
    const dy = tTgt[2] - tCur[2];
    const dz = tTgt[3] - tCur[3];
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * 阶段 1：启动底盘移动
 */
function startBaseMovement(DQ, currentWorldAbs, targetWorldAbs) {
    // 计算目标位置的 XY 平移量
    const dqCur = DQ.createFromArray(currentWorldAbs);
    const dqTgt = DQ.createFromArray(targetWorldAbs);
    const tCur = DQ.getTranslation(dqCur);
    const tTgt = DQ.getTranslation(dqTgt);

    const deltaX = tTgt[1] - tCur[1];
    const deltaY = tTgt[2] - tCur[2];

    // 底盘目标 = 当前底盘位置 + 绝对位姿的 XY 偏差
    const basePose = appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 };
    const baseTargetX = (basePose.x || 0) + deltaX;
    const baseTargetY = (basePose.y || 0) + deltaY;

    // 底盘朝向：朝向目标方向
    const baseTargetYaw = Math.atan2(deltaY, deltaX);

    // 设置底盘目标
    appState.humanoidBaseTargetPose.x = baseTargetX;
    appState.humanoidBaseTargetPose.y = baseTargetY;
    appState.humanoidBaseTargetPose.yaw = baseTargetYaw;
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidBaseReverseDrive = false;

    // 确保底盘控制模式为 point-to-point
    appState.humanoidBaseControlMode = 'point';

    // 确保底盘控制循环在运行
    if (!appState.humanoidBaseControlTimer) {
        callbacks.startHumanoidBaseControlLoop?.();
    }

    updateStatus(STATE.BASE_MOVING, `底盘移动中… 偏差 ${(Math.hypot(deltaX, deltaY) * 100).toFixed(1)} cm`);

    // 启动轮询，等待底盘到位
    motionPollTimer = setInterval(() => {
        pollBaseMovement();
    }, motionParams.pollInterval);
}

/**
 * 轮询底盘移动状态
 */
function pollBaseMovement() {
    if (motionState !== STATE.BASE_MOVING) {
        clearInterval(motionPollTimer);
        motionPollTimer = null;
        return;
    }

    const converged = appState.humanoidBaseConvergedCount >= motionParams.baseSettleCycles;

    // 更新状态显示
    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    const posErr = Math.hypot(dx, dy);
    updateStatus(STATE.BASE_MOVING, `底盘移动中… 剩余 ${(posErr * 100).toFixed(1)} cm`);

    if (converged) {
        clearInterval(motionPollTimer);
        motionPollTimer = null;
        console.log('规划运动：底盘到位，开始手臂补偿');
        startArmCompensation();
    }
}

/**
 * 关节角度插值（考虑周期 wrap，用于旋转关节）
 */
function lerpJoint(a, b, t) {
    let d = b - a;
    const twoPi = 2 * Math.PI;
    while (d > Math.PI) d -= twoPi;
    while (d < -Math.PI) d += twoPi;
    return a + d * t;
}

/**
 * 阶段 2：先求解 IK 得到目标关节角，再以动画插值到目标
 */
function startArmCompensation() {
    updateStatus(STATE.ARM_COMPENSATING, '手臂补偿中…');

    const vis = appState.visualizer;
    if (!vis || !motionTarget) {
        updateStatus(STATE.ERROR, '可视化器或目标丢失');
        return;
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        updateStatus(STATE.ERROR, 'DQ 模块不可用');
        return;
    }

    const basePose = appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 };
    const baseDQ = vis._makeHumanoidBaseDQ?.(basePose);
    if (!baseDQ) {
        updateStatus(STATE.ERROR, '无法构建底盘 DQ');
        return;
    }
    const baseDQInv = DQ.inverse(baseDQ);
    const worldTargetDQ = DQ.createFromArray(motionTarget);
    const localTargetDQ = DQ.multiply(baseDQInv, worldTargetDQ);
    const localTargetArray = DQ.toArray(localTargetDQ);

    const qRef = appState.humanoidQ || window.humanoidQ;
    if (!qRef || qRef.length === 0) {
        updateStatus(STATE.ERROR, '人形关节未初始化');
        return;
    }

    const initialQ = qRef.slice();
    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

    let targetQ;
    try {
        const success = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
            localTargetArray,
            initialQ,
            enableLimitAvoidance,
            motionParams.armIKIncludeWaist
        );
        if (!success) {
            updateStatus(STATE.DONE, '手臂 IK 未收敛');
            motionTarget = null;
            return;
        }
        targetQ = (appState.humanoidQ || window.humanoidQ).slice();
    } catch (e) {
        console.error('规划运动 - 手臂 IK 失败:', e);
        updateStatus(STATE.ERROR, `手臂 IK 失败: ${e.message}`);
        motionTarget = null;
        return;
    }

    // 恢复为初始姿态，准备做插值动画
    for (let i = 0; i < qRef.length; i++) {
        qRef[i] = initialQ[i];
    }
    if (appState.humanoidQ) {
        for (let i = 0; i < appState.humanoidQ.length; i++) {
            appState.humanoidQ[i] = initialQ[i];
        }
    }
    vis.setHumanoidJoints?.(qRef);
    callbacks.updateRobotPose?.();

    const durationMs = (motionParams.armCompensationDuration || 0.5) * 1000;
    const easeName = motionParams.armCompensationEase || 'easeInOutCubic';
    const easeFn = easeName === 'easeInOutCubic' ? easeInOutCubic : (t) => t;

    const startTime = performance.now();

    function animateArm(now) {
        if (motionState !== STATE.ARM_COMPENSATING) {
            motionArmAnimationId = null;
            return;
        }
        const elapsed = now - startTime;
        const t = Math.min(1, elapsed / durationMs);
        const s = easeFn(t);

        for (let i = 0; i < qRef.length; i++) {
            qRef[i] = lerpJoint(initialQ[i], targetQ[i], s);
        }
        if (appState.humanoidQ) {
            for (let i = 0; i < appState.humanoidQ.length; i++) {
                appState.humanoidQ[i] = qRef[i];
            }
        }
        vis.setHumanoidJoints?.(qRef);
        callbacks.updateRobotPose?.();
        callbacks.refreshJointControlDisplay?.();

        if (t >= 1) {
            motionArmAnimationId = null;
            const finalWorldAbs = getCurrentWorldAbsolutePose(DQ);
            if (finalWorldAbs) {
                const finalError = computePositionError(DQ, finalWorldAbs, motionTarget);
                updateStatus(STATE.DONE, `完成！残余误差 ${(finalError * 1000).toFixed(1)} mm`);
            } else {
                updateStatus(STATE.DONE, '手臂补偿完成');
            }
            motionTarget = null;
            return;
        }
        motionArmAnimationId = requestAnimationFrame(animateArm);
    }

    motionArmAnimationId = requestAnimationFrame(animateArm);
}

/**
 * 更新状态和 UI
 */
function updateStatus(newState, message) {
    motionState = newState;
    if (motionStatusCallback) {
        motionStatusCallback(newState, message);
    }
}

// ── UI 事件绑定 ──────────────────────────────────

/**
 * 初始化规划运动 UI 事件监听
 */
export function setupPlannedMotionListeners() {
    const executeBtn = document.getElementById('planned-motion-execute-btn');
    const cancelBtn = document.getElementById('planned-motion-cancel-btn');
    const input = document.getElementById('planned-motion-dq-input');
    const statusEl = document.getElementById('planned-motion-status');
    const thresholdInput = document.getElementById('planned-motion-threshold');
    const thresholdValue = document.getElementById('planned-motion-threshold-value');
    const copyCurrentBtn = document.getElementById('planned-motion-copy-current');

    // 状态回调
    setPlannedMotionStatusCallback((state, msg) => {
        if (!statusEl) return;
        statusEl.textContent = msg || state;
        statusEl.className = 'planned-motion-status';
        if (state === STATE.ERROR) statusEl.classList.add('status-error');
        else if (state === STATE.DONE) statusEl.classList.add('status-done');
        else if (state === STATE.BASE_MOVING || state === STATE.ARM_COMPENSATING) statusEl.classList.add('status-running');

        // 按钮状态
        if (executeBtn) executeBtn.disabled = isPlannedMotionRunning();
        if (cancelBtn) cancelBtn.disabled = !isPlannedMotionRunning();
    });

    // 阈值滑块
    if (thresholdInput && thresholdValue) {
        thresholdInput.addEventListener('input', () => {
            thresholdValue.textContent = thresholdInput.value;
        });
    }

    // 复制当前绝对位姿
    if (copyCurrentBtn) {
        copyCurrentBtn.addEventListener('click', () => {
            const DQ = getDQModule()?.DQWrapper;
            if (!DQ || !appState.humanoidMode) return;
            const absWorld = getCurrentWorldAbsolutePose(DQ);
            if (absWorld && input) {
                input.value = absWorld.map(v => v.toFixed(6)).join(' ');
            }
        });
    }

    // 执行按钮
    if (executeBtn) {
        executeBtn.addEventListener('click', () => {
            if (!input) return;
            const text = input.value.trim();
            if (!text) {
                updateStatus(STATE.ERROR, '请输入目标双四元数');
                return;
            }
            const parts = text.split(/[\s,]+/).filter(Boolean).map(Number);
            if (parts.length !== 8 || parts.some(isNaN)) {
                updateStatus(STATE.ERROR, '请输入 8 个数值（DQ 格式）');
                return;
            }
            const threshold = thresholdInput ? Number(thresholdInput.value) / 100 : DEFAULT_PARAMS.baseThreshold;
            executePlannedMotion(parts, { baseThreshold: threshold });
        });
    }

    // 取消按钮
    if (cancelBtn) {
        cancelBtn.addEventListener('click', () => {
            cancelPlannedMotion();
        });
    }

    // 初始化按钮状态
    if (cancelBtn) cancelBtn.disabled = true;
}

/**
 * 注册回调
 */
export function registerPlannedMotionCallbacks() {
    callbacks.executePlannedMotion = executePlannedMotion;
    callbacks.cancelPlannedMotion = cancelPlannedMotion;
    callbacks.setupPlannedMotionListeners = setupPlannedMotionListeners;
}
