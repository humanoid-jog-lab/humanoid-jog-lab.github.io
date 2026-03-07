/**
 * 规划运动模块
 *
 * 新流程（人形模式）：
 *  1) 先规划并播放残影预览（不执行真实底盘/关节动作）
 *  2) 用户确认后再执行真实运动（底盘阶段 + 手臂补偿阶段）
 */

import * as THREE from 'three';
import { appState, callbacks } from '../core/state.js';
import { getDQModule, retimeWaypointsWithTOPPRA } from '../dqrobotics.js';

// ── 状态机状态 ──────────────────────────────────
const STATE = {
    IDLE: 'idle',
    PREVIEWING: 'previewing',
    PLANNED: 'planned',
    WAIST_ALIGNING: 'waist_aligning',
    BASE_MOVING: 'base_moving',
    ARM_COMPENSATING: 'arm_compensating',
    DONE: 'done',
    ERROR: 'error',
};

const TASK_TYPE = {
    ABSOLUTE: 'absolute',
    DUAL_ARM_SEQUENCE_PLACE: 'dual-arm-sequence-place',
};

// ── 默认参数 ──────────────────────────────────
const DEFAULT_PARAMS = {
    baseThreshold: 0.40,           // 底盘启动阈值（米）
    baseSettleCycles: 10,          // 底盘收敛判定次数
    armIKIncludeWaist: true,       // 手臂 IK 是否包含腰部关节
    waistYawPreAlignEnabled: true, // 规划时是否先做“腰部 yaw 预对齐”
    waistYawPreAlignMinDeg: 1.0,   // 触发腰部 yaw 预对齐的最小角度（度）
    waistLimitSoftMarginDeg: 4.0,  // 腰部限位软边界（度），用于候选解惩罚
    waistLimitHardMarginDeg: 1.0,  // 腰部限位硬边界（度），用于候选解屏蔽
    waistPreAlignAllowNearLimit: false, // 腰部预对齐是否允许贴近限位
    singleArmIKIncludeWaist: true, // 单臂目标求解是否包含腰部关节
    singleArmIKKeepBodyZVertical: true, // 单臂+腰部求解时是否保持 body_link z 轴竖直
    armIKKeepBodyZVertical: false, // 绝对位姿控制时是否保持 body_link z 轴竖直
    pollInterval: 50,              // 状态轮询间隔（ms）
    armCompensationDuration: 0.5,  // 手臂补偿动画时长（秒）
    armCompensationEase: 'easeInOutCubic',
    armCompensationUseToppra: true, // 手臂补偿是否使用 TOPPRA 时间参数化
    armCompensationToppraSingleArmOnly: true, // 仅单臂步骤（左臂/右臂顺序任务）启用 TOPPRA
    armCompensationToppraMaxVelDeg: 120.0, // TOPPRA 关节速度上限（度/秒）
    armCompensationToppraMaxAccDeg: 240.0, // TOPPRA 关节加速度上限（度/秒^2）
    armCompensationToppraGridpoints: 0, // TOPPRA 网格段数（0 自动）
    armCompensationToppraWaypoints: 7, // TOPPRA 关节路径离散点数（>=2）
    waistPreAlignDuration: 0.45,   // 执行时腰部预对齐动画时长（秒）
    waistPreAlignExecutionMode: 'during-base', // 执行腰部预对齐策略: during-base | pre-stage
    waistBlendDuringBaseBlend: 0.34, // 底盘阶段腰部渐进对齐平滑系数
    waistBlendDuringBaseMaxStepDeg: 2.8, // 底盘阶段腰部每周期最大步长（度）
    waistBlendDuringBaseMinStepDeg: 0.22, // 底盘阶段腰部每周期最小步长（度）
    syncArmConvergeTolDeg: 0.35, // 同步阶段双臂（非腰部）到位阈值（度）
    syncWaistConvergeTolDeg: 0.10, // 同步阶段腰部到位阈值（度）
    syncRelativeTolDeg: 2.00, // 同步阶段双臂相对位姿旋转误差阈值（度）
    syncAbsolutePosTolMm: 20.0, // 同步阶段绝对位姿平移误差阈值（毫米）
    syncAbsoluteRotTolDeg: 4.0, // 同步阶段绝对位姿旋转误差阈值（度）
    syncRelativeConstraintMode: 'runtime-lock', // 同步阶段相对约束来源: runtime-lock | planned-sync
    syncMaxWaitCycles: 80, // 底盘已到位后，同步阶段最长等待轮询次数
    armRefineMaxPasses: 4,         // 手臂 IK 自动精修轮次
    finalPosTolMm: 0.5,            // 期望最终平移误差阈值（毫米）
    finalRotTolDeg: 0.2,           // 期望最终旋转误差阈值（度）
    previewDuration: 1.2,          // 残影动画总时长（秒）
    previewUseToppraTiming: true,  // 预览路径时间轴是否使用 TOPPRA（路点->时间）
    previewToppraMaxVel: 0.45,     // TOPPRA 速度上限（m/s，每维）
    previewToppraMaxAcc: 0.90,     // TOPPRA 加速度上限（m/s^2，每维）
    previewToppraGridpoints: 0,    // TOPPRA 网格段数（0 自动）
    previewBaseSamples: 16,        // 预览底盘阶段采样数
    previewArmSamples: 24,         // 预览手臂阶段采样数
    previewRobotSamples: 8,        // 机器人残影关键帧数量（越小越稀疏）
    baseMoveTrackingEnabled: true, // 底盘移动中是否做局部目标在线跟踪
    baseMoveTrackingPasses: 1,     // 每个轮询周期的在线跟踪迭代次数
    baseMoveTrackingBlend: 0.25,   // 在线跟踪平滑系数（0~1，越大越快）
    baseMoveTrackingMaxJointStepDeg: 2.0, // 在线跟踪每周期最大关节步长（度）
    baseMoveTrackingStartupHoldMs: 180, // 底盘起步时上肢跟踪延时，抑制开头抖动（毫秒）
    baseMoveTrackingJitterDeadbandDeg: 0.08, // 在线跟踪微小关节变化死区（度）
    baseMoveTrackingWarmupMs: 1200, // 在线跟踪暖启动时长（毫秒）
    baseMoveTrackingWarmupMinScale: 0.30, // 在线跟踪初始速度比例（0~1）
};

// ── 模块状态 ──────────────────────────────────
let motionState = STATE.IDLE;
let motionTarget = null;             // 当前执行目标 DQ (8 元素数组)
let motionParams = { ...DEFAULT_PARAMS };

let motionPreparedPlan = null;       // 已规划未执行
let motionActivePlan = null;         // 当前正在执行的计划
let motionPreparedTaskPlan = null;   // 已规划未执行的任务（如左右臂顺序放置）
let motionActiveTaskPlan = null;     // 当前执行中的任务

let motionPollTimer = null;
let motionArmAnimationId = null;
let motionPreviewAnimationId = null;
let motionDebugAnimationId = null;
let motionWaistAnimationId = null;

let motionPreviewGroup = null;
let motionPreviewItems = null;

let motionStatusCallback = null;     // UI 状态更新回调

let motionDebugSelectedTaskStepIndex = 0;
let motionDebugLastStageKey = 'start';
let motionDebugFlowEnabled = true;

const MOTION_JOINT_MONITOR_INTERVAL_MS = 80;
const MOTION_JOINT_MONITOR_HISTORY_LIMIT = 150;
const MOTION_JOINT_MONITOR_RENDER_INTERVAL_IDLE_MS = 120;
const MOTION_JOINT_MONITOR_RENDER_INTERVAL_EXEC_MS = 180;
const MOTION_JOINT_MONITOR_CHANNEL_NAME = 'dq-vizlab-joint-monitor-v1';
const MOTION_JOINT_MONITOR_COLORS = [
    '#22d3ee',
    '#60a5fa',
    '#34d399',
    '#f59e0b',
    '#f472b6',
    '#a78bfa',
    '#f87171',
    '#facc15',
];
let motionJointMonitorTimer = null;
let motionJointMonitorHistory = [];
let motionJointMonitorLastQDim = 0;
let motionJointMonitorLastRenderTs = 0;
let motionJointMonitorLastStateText = '';
let motionJointMonitorChannel = null;
let motionJointMonitorChannelUnsupported = false;
let motionPreviewDeferredDisposeQueue = [];
let motionPreviewDeferredDisposeScheduled = false;

// 缓动：t in [0,1] -> [0,1]
function easeInOutCubic(t) {
    return t <= 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

function wrapAngle(angle) {
    const twoPi = 2 * Math.PI;
    let a = angle;
    while (a > Math.PI) a -= twoPi;
    while (a < -Math.PI) a += twoPi;
    return a;
}

function toNumericTuple(input, expectedLength) {
    if (input == null) return null;

    let arr = null;
    if (Array.isArray(input)) {
        arr = input;
    } else if (typeof input?.toArray === 'function') {
        try {
            const converted = input.toArray();
            if (Array.isArray(converted) || ArrayBuffer.isView(converted)) {
                arr = Array.from(converted);
            }
        } catch {
            arr = null;
        }
    } else if (ArrayBuffer.isView(input)) {
        arr = Array.from(input);
    } else if (typeof input === 'object' && Number.isFinite(Number(input.length))) {
        const len = Number(input.length);
        arr = [];
        for (let i = 0; i < len; i++) arr.push(input[i]);
    }

    if (!arr || arr.length < expectedLength) return null;
    const out = arr.slice(0, expectedLength).map(Number);
    if (out.some(v => !Number.isFinite(v))) return null;
    return out;
}

function normalizeQuaternion(quat) {
    const q = toNumericTuple(quat, 4);
    if (!q) return null;
    const n = Math.hypot(q[0], q[1], q[2], q[3]);
    if (!Number.isFinite(n) || n < 1e-12) return null;
    return q.map(v => v / n);
}

function multiplyQuaternion(a, b) {
    const qa = toNumericTuple(a, 4);
    const qb = toNumericTuple(b, 4);
    if (!qa || !qb) return null;
    const [aw, ax, ay, az] = qa;
    const [bw, bx, by, bz] = qb;
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ];
}

function quaternionYaw(quat) {
    const q = toNumericTuple(quat, 4);
    if (!q) return null;
    const qw = q[0];
    const qx = q[1];
    const qy = q[2];
    const qz = q[3];
    const siny = 2 * (qw * qz + qx * qy);
    const cosy = 1 - 2 * (qy * qy + qz * qz);
    return Math.atan2(siny, cosy);
}

function buildPoseDQFromRotationTranslation(rotationQuat, translationVec3) {
    const qr = normalizeQuaternion(rotationQuat);
    const t = toNumericTuple(translationVec3, 3);
    if (!qr || !t) return null;

    const tx = t[0];
    const ty = t[1];
    const tz = t[2];

    const tQuat = [0, tx, ty, tz];
    const tMulR = multiplyQuaternion(tQuat, qr);
    if (!tMulR) return null;

    const qd = tMulR.map(v => 0.5 * v);
    return normalizeDualQuaternionArray([...qr, ...qd]);
}

function cloneBasePose(basePose = {}) {
    return {
        x: Number(basePose.x) || 0,
        y: Number(basePose.y) || 0,
        yaw: Number(basePose.yaw) || 0,
    };
}

function lerpAngle(a, b, t) {
    return a + wrapAngle(b - a) * t;
}

function lerpBasePose(a, b, t) {
    return {
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t,
        yaw: lerpAngle(a.yaw, b.yaw, t),
    };
}

function hasPreparedPlan() {
    return !!motionPreparedPlan || !!motionPreparedTaskPlan;
}

function normalizePlannedMotionTaskType(taskType) {
    return taskType === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE
        ? TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE
        : TASK_TYPE.ABSOLUTE;
}

function getPlannedMotionTaskType() {
    const taskTypeSelect = document.getElementById('planned-motion-task-type-select');
    return normalizePlannedMotionTaskType(taskTypeSelect?.value);
}

function normalizePlannedMotionControlMode(mode) {
    return mode === 'decoupled-upright' ? 'decoupled-upright' : 'decoupled';
}

function getPlannedMotionControlMode() {
    const plannedModeSelect = document.getElementById('planned-motion-control-mode-select');
    if (plannedModeSelect) {
        return normalizePlannedMotionControlMode(plannedModeSelect.value);
    }

    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const controlMode = controlModeSelect?.value || (typeof window !== 'undefined' ? window.currentDualArmControlMode : null);
    return normalizePlannedMotionControlMode(controlMode);
}

function detectUprightControlMode() {
    const mode = getPlannedMotionControlMode();
    return mode === 'decoupled-upright';
}

function resolvePlannedMotionParams(options = {}) {
    const merged = { ...DEFAULT_PARAMS, ...options };
    if (typeof options.armIKKeepBodyZVertical !== 'boolean') {
        merged.armIKKeepBodyZVertical = detectUprightControlMode();
    }
    return merged;
}

// ── 公共 API ──────────────────────────────────

/**
 * 获取当前运动状态
 */
export function getPlannedMotionState() {
    return motionState;
}

/**
 * 是否处于“正在进行中”的状态（预览或执行）
 */
export function isPlannedMotionRunning() {
    return (
        motionState === STATE.PREVIEWING ||
        motionState === STATE.WAIST_ALIGNING ||
        motionState === STATE.BASE_MOVING ||
        motionState === STATE.ARM_COMPENSATING
    );
}

/**
 * 设置状态更新回调
 */
export function setPlannedMotionStatusCallback(cb) {
    motionStatusCallback = cb;
}

/**
 * 规划运动（含残影预览，不执行真实动作）
 * @param {number[]} targetDQArray - 目标绝对位姿（世界坐标系，8 元素 DQ 数组）
 * @param {Object} [options] - 可选参数覆盖
 * @param {boolean} [options.playPreview=true] - 是否播放残影动画
 * @param {boolean} [options.armIKKeepBodyZVertical] - 是否保持 body_link z 轴竖直（默认跟随控制模式）
 * @param {number} [options.baseTargetYaw] - 指定底盘目标 yaw（弧度）
 */
export function planPlannedMotion(targetDQArray, options = {}) {
    const validation = validatePlanInput(targetDQArray);
    if (!validation.ok) {
        updateStatus(STATE.ERROR, validation.message);
        return false;
    }
    const normalizedTargetDQ = normalizeDualQuaternionArray(targetDQArray);
    if (!normalizedTargetDQ) {
        updateStatus(STATE.ERROR, '目标 DQ 无法单位化，请检查输入');
        return false;
    }

    // 开始新规划前，先清理当前运行态和旧残影
    stopAllRuntimeTimers();
    clearPreviewVisualization({ deferDispose: false });
    motionPreparedPlan = null;
    motionActivePlan = null;
    motionPreparedTaskPlan = null;
    motionActiveTaskPlan = null;
    motionTarget = null;
    motionDebugSelectedTaskStepIndex = 0;

    motionParams = resolvePlannedMotionParams(options);
    const requestedBaseYaw = Number.isFinite(options.baseTargetYaw)
        ? wrapAngle(Number(options.baseTargetYaw))
        : null;

    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;

    const plan = buildMotionPlan(DQ, vis, normalizedTargetDQ, motionParams, requestedBaseYaw);
    if (!plan) {
        // buildMotionPlan 内部会给出状态信息
        return false;
    }

    motionPreparedPlan = plan;

    const playPreview = options.playPreview !== false;
    if (!playPreview) {
        updateStatus(
            STATE.PLANNED,
            '规划完成，点击“执行已规划运动”开始真实执行'
        );
        return true;
    }

    const started = startPreviewAnimation(plan);
    if (!started) {
        updateStatus(
            STATE.PLANNED,
            '规划完成（预览不可用），点击“执行已规划运动”开始真实执行'
        );
        return true;
    }

    return true;
}

/**
 * 规划左右臂顺序放置任务：
 *  1) 左臂先放到 A 点
 *  2) 右臂再放到 B 点
 */
export function planDualArmSequentialPlaceTask(leftTargetDQArray, rightTargetDQArray, options = {}) {
    const contextValidation = validatePlannerContext();
    if (!contextValidation.ok) {
        updateStatus(STATE.ERROR, contextValidation.message);
        return false;
    }
    const leftValidation = validateDQArray(leftTargetDQArray, '左臂目标 DQ');
    if (!leftValidation.ok) {
        updateStatus(STATE.ERROR, leftValidation.message);
        return false;
    }
    const rightValidation = validateDQArray(rightTargetDQArray, '右臂目标 DQ');
    if (!rightValidation.ok) {
        updateStatus(STATE.ERROR, rightValidation.message);
        return false;
    }
    const normalizedLeftTargetDQ = normalizeDualQuaternionArray(leftTargetDQArray);
    if (!normalizedLeftTargetDQ) {
        updateStatus(STATE.ERROR, '左臂目标 DQ 无法单位化，请检查输入');
        return false;
    }
    const normalizedRightTargetDQ = normalizeDualQuaternionArray(rightTargetDQArray);
    if (!normalizedRightTargetDQ) {
        updateStatus(STATE.ERROR, '右臂目标 DQ 无法单位化，请检查输入');
        return false;
    }

    stopAllRuntimeTimers();
    clearPreviewVisualization({ deferDispose: false });
    motionPreparedPlan = null;
    motionActivePlan = null;
    motionPreparedTaskPlan = null;
    motionActiveTaskPlan = null;
    motionTarget = null;
    motionDebugSelectedTaskStepIndex = 0;

    motionParams = resolvePlannedMotionParams(options);
    const requestedBaseYaw = Number.isFinite(options.baseTargetYaw)
        ? wrapAngle(Number(options.baseTargetYaw))
        : 0;
    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;

    const taskPlan = buildDualArmSequentialPlaceTaskPlan(
        DQ,
        vis,
        normalizedLeftTargetDQ,
        normalizedRightTargetDQ,
        motionParams,
        requestedBaseYaw
    );
    if (!taskPlan) return false;

    motionPreparedTaskPlan = taskPlan;
    motionDebugSelectedTaskStepIndex = 0;
    updateStatus(
        STATE.PLANNED,
        `任务规划完成（先左臂到 A 点，再右臂到 B 点，底盘yaw期望 ${(taskPlan.baseTargetYaw * 180 / Math.PI).toFixed(1)}deg），点击“执行已规划运动”开始执行`
    );
    return true;
}

/**
 * 执行规划运动
 *
 * 兼容两种用法：
 *  1) executePlannedMotion()：执行当前“已规划”结果
 *  2) executePlannedMotion(targetDQ, options)：兼容旧调用，直接规划并立即执行（跳过预览）
 */
export function executePlannedMotion(targetDQArray = null, options = {}) {
    // 兼容旧模式：直接传目标并执行
    if (Array.isArray(targetDQArray)) {
        const planned = planPlannedMotion(targetDQArray, { ...options, playPreview: false });
        if (!planned) return false;
        return executePreparedPlan();
    }

    // 新模式：执行已规划结果
    return executePreparedPlan();
}

/**
 * 取消规划/预览/执行，并清理残影
 */
export function cancelPlannedMotion() {
    const hadAnyState = (
        motionState !== STATE.IDLE ||
        motionPreparedPlan ||
        motionActivePlan ||
        motionPreparedTaskPlan ||
        motionActiveTaskPlan ||
        motionTarget
    );

    stopAllRuntimeTimers();
    clearPreviewVisualization();

    motionPreparedPlan = null;
    motionActivePlan = null;
    motionPreparedTaskPlan = null;
    motionActiveTaskPlan = null;
    motionTarget = null;
    motionDebugSelectedTaskStepIndex = 0;
    setPlannedMotionDebugInfo('-');

    if (hadAnyState) {
        updateStatus(STATE.IDLE, '已取消');
    } else {
        updateStatus(STATE.IDLE, '就绪');
    }
}

// ── 内部逻辑 ──────────────────────────────────

function validatePlannerContext() {
    if (!appState.humanoidMode || !appState.visualizer) {
        return { ok: false, message: '仅在人形模式下可用' };
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        return { ok: false, message: 'DQ 模块未加载' };
    }
    return { ok: true };
}

function normalizeDualQuaternionArray(dqArray) {
    if (!Array.isArray(dqArray) || dqArray.length !== 8) return null;
    const vals = dqArray.map(Number);
    if (vals.some(v => !Number.isFinite(v))) return null;

    const qr = vals.slice(0, 4);
    const qd = vals.slice(4, 8);
    const normQr = Math.hypot(qr[0], qr[1], qr[2], qr[3]);
    if (!Number.isFinite(normQr) || normQr < 1e-12) return null;

    const qrN = qr.map(v => v / normQr);
    const qdScaled = qd.map(v => v / normQr);

    // Study 条件投影：qr·qd = 0
    const dot = qrN[0] * qdScaled[0] + qrN[1] * qdScaled[1] + qrN[2] * qdScaled[2] + qrN[3] * qdScaled[3];
    const qdProj = qdScaled.map((v, i) => v - dot * qrN[i]);

    const out = [...qrN, ...qdProj];
    if (out.some(v => !Number.isFinite(v))) return null;
    return out;
}

function validateDQArray(targetDQArray, label = '目标 DQ') {
    if (!Array.isArray(targetDQArray) || targetDQArray.length !== 8) {
        return { ok: false, message: `${label} 格式无效（需要 8 个数）` };
    }

    if (targetDQArray.some(v => Number.isNaN(Number(v)))) {
        return { ok: false, message: `${label} 包含非法数字` };
    }

    return { ok: true };
}

function safeCreatePoseDQ(DQ, dqArray) {
    const normalized = normalizeDualQuaternionArray(dqArray);
    if (!normalized) return null;
    try {
        const dq = DQ.createFromArray(normalized);
        const t = DQ.getTranslation(dq);
        const q = DQ.getRotation(dq);
        return { dq, normalized, t, q };
    } catch (e) {
        console.warn('DQ 单位化后仍无法构建位姿:', e);
        return null;
    }
}

function validatePlanInput(targetDQArray) {
    const contextValidation = validatePlannerContext();
    if (!contextValidation.ok) return contextValidation;
    return validateDQArray(targetDQArray, '目标 DQ');
}

function stopAllRuntimeTimers() {
    if (motionPollTimer) {
        clearInterval(motionPollTimer);
        motionPollTimer = null;
    }

    if (motionArmAnimationId != null) {
        cancelAnimationFrame(motionArmAnimationId);
        motionArmAnimationId = null;
    }

    if (motionPreviewAnimationId != null) {
        cancelAnimationFrame(motionPreviewAnimationId);
        motionPreviewAnimationId = null;
    }

    if (motionDebugAnimationId != null) {
        cancelAnimationFrame(motionDebugAnimationId);
        motionDebugAnimationId = null;
    }

    if (motionWaistAnimationId != null) {
        cancelAnimationFrame(motionWaistAnimationId);
        motionWaistAnimationId = null;
    }
}

function buildMotionPlan(DQ, vis, targetDQArray, params, forcedBaseYaw = null) {
    const currentWorldAbs = getCurrentWorldAbsolutePose(DQ);
    if (!currentWorldAbs) {
        updateStatus(STATE.ERROR, '无法获取当前绝对位姿');
        return null;
    }
    const qCurrent = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qCurrent) || qCurrent.length === 0) {
        updateStatus(STATE.ERROR, '人形关节未初始化');
        return null;
    }

    const poseErrorBeforeWaist = computePoseError(DQ, currentWorldAbs, targetDQArray);
    const baseStartPose = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });
    const hasForcedBaseYaw = Number.isFinite(forcedBaseYaw);
    const baseYawTol = Math.max(1e-3, Number(appState.humanoidBaseControlParams?.tolYaw) || 0.05);
    if (hasForcedBaseYaw && Math.abs(wrapAngle(forcedBaseYaw - baseStartPose.yaw)) > baseYawTol) {
        console.warn('规划运动：当前流程姿态对齐由腰部完成，已忽略底盘目标 yaw 设定');
    }

    // 先做“腰部 yaw 预对齐”求解，再基于预对齐后的姿态去估计底盘 XY 收敛目标。
    const waistYawPreAlign = planWaistYawPreAlignment(
        DQ,
        vis,
        currentWorldAbs,
        targetDQArray,
        baseStartPose,
        params
    );
    const planningWorldAbs = waistYawPreAlign?.worldAbsAfterAlign || currentWorldAbs;
    const poseError = computePoseError(DQ, planningWorldAbs, targetDQArray);

    // 底盘阶段只负责 XY，姿态对齐交给腰部/上肢 IK
    const needBaseMoveForXY = poseError.posErrXY > params.baseThreshold;
    const needBaseYawMove = false;
    const needBaseMove = needBaseMoveForXY;

    // 以“腰部预对齐后”的绝对位姿为起点，规划底盘 XY 收敛。
    const baseTargetXY = computeBaseTargetXY(DQ, planningWorldAbs, targetDQArray, baseStartPose, needBaseMoveForXY);

    // 固定 yaw：底盘保持当前朝向，不参与姿态对齐
    const fixedYaw = baseStartPose.yaw || 0;
    const qSeedAfterWaist =
        Array.isArray(waistYawPreAlign?.qSeed) && waistYawPreAlign.qSeed.length > 0
            ? waistYawPreAlign.qSeed.slice()
            : qCurrent.slice();
    const relativePoseConstraintForFullBody =
        Array.isArray(waistYawPreAlign?.relativePoseConstraint) && waistYawPreAlign.relativePoseConstraint.length === 8
            ? waistYawPreAlign.relativePoseConstraint.slice()
            : getHumanoidRelativePoseConstraint(vis, qSeedAfterWaist);
    const baseTargetPose = {
        x: baseTargetXY.x,
        y: baseTargetXY.y,
        yaw: fixedYaw,
    };

    const baseStages = needBaseMove ? buildBaseMotionStages(baseStartPose, baseTargetPose) : [];
    const needBaseMovePlanned = baseStages.length > 0;

    // 虚拟状态3（规划）：在“虚拟状态2”的底盘位姿下做全身绝对位姿补偿。
    const solveResult = solveArmTargetForBasePose(
        DQ,
        vis,
        targetDQArray,
        baseTargetPose,
        params,
        relativePoseConstraintForFullBody,
        qSeedAfterWaist
    );
    if (!solveResult.ok) {
        console.warn(
            `规划运动：固定 yaw 求解失败 (yaw=${(fixedYaw * 180 / Math.PI).toFixed(1)}deg)，原因: ${solveResult.message || '未知'}`
        );
        updateStatus(STATE.ERROR, '手臂 IK 规划失败');
        return null;
    }

    // 虚拟状态1：腰部 yaw 预对齐目标（仅腰部变化）
    const virtualStateRef = { qStart: solveResult.qStart, waistYawPreAlign };
    const qVirtualState1 = buildWaistPreAlignTargetQ(virtualStateRef, solveResult.qStart);
    // 虚拟状态2：底盘 XY 补偿后，关节保持虚拟状态1
    const qVirtualState2 = qVirtualState1.slice();
    // 虚拟状态3（同步阶段目标）：沿用虚拟状态1的腰部，非腰部取自虚拟状态3全身求解结果
    const qVirtualState3FullBody = adaptJointVectorToReference(solveResult.qTarget, solveResult.qStart);
    const waistCount = getWaistJointCount({ qStart: solveResult.qStart });
    const qVirtualState3Sync = mergeWaistAndNonWaistTargets(
        qVirtualState1,
        qVirtualState3FullBody,
        waistCount
    );
    const qSyncRef = adaptJointVectorToReference(qVirtualState3Sync, solveResult.qStart);
    const arm1BodyRelativeTarget = computeArmPoseRelativeToBody(DQ, vis, qSyncRef, 1);
    const arm2BodyRelativeTarget = computeArmPoseRelativeToBody(DQ, vis, qSyncRef, 2);

    const syncRelativePoseConstraint =
        getHumanoidRelativePoseConstraint(vis, qVirtualState3Sync) ||
        getHumanoidRelativePoseConstraint(vis, qVirtualState3FullBody) ||
        (Array.isArray(relativePoseConstraintForFullBody) ? relativePoseConstraintForFullBody.slice() : null);
    const fullBodyRelativePoseConstraint =
        getHumanoidRelativePoseConstraint(vis, qVirtualState3FullBody) ||
        (Array.isArray(relativePoseConstraintForFullBody) ? relativePoseConstraintForFullBody.slice() : null);

    const jointCost = computeJointMotionCost(solveResult.qStart, qVirtualState3FullBody);
    const baseMoveLocalTarget = computeLocalTargetForBasePose(DQ, vis, targetDQArray, baseTargetPose);

    const plan = {
        targetDQ: [...targetDQArray],
        params: { ...params },
        needBaseMove: needBaseMovePlanned,
        needBaseMoveForXY,
        needBaseYawMove,
        poseErrorBeforeWaist,
        poseError,
        waistYawPreAlign,
        baseStartPose,
        baseTargetPose,
        baseStages,
        baseMoveLocalTarget,
        // 执行阶段最终全身补偿（步骤4）使用的相对位姿约束
        relativePoseConstraint: Array.isArray(fullBodyRelativePoseConstraint) ? fullBodyRelativePoseConstraint.slice() : null,
        // 底盘同步阶段（步骤1/2/3并行）使用的相对位姿约束
        syncRelativePoseConstraint: Array.isArray(syncRelativePoseConstraint) ? syncRelativePoseConstraint.slice() : null,
        // 虚拟状态缓存（便于 Debug 与执行一致）
        qVirtualState1,
        qVirtualState2,
        qVirtualState3FullBody,
        qSyncTarget: qVirtualState3Sync,
        qWaistVirtualTarget: qVirtualState1,
        syncArmBodyRelativeTargets: {
            arm1: Array.isArray(arm1BodyRelativeTarget) ? arm1BodyRelativeTarget.slice() : null,
            arm2: Array.isArray(arm2BodyRelativeTarget) ? arm2BodyRelativeTarget.slice() : null,
        },
        qStart: solveResult.qStart,
        // 最终补偿目标（步骤4，全身）
        qTarget: qVirtualState3FullBody,
        solvedError: solveResult.solvedError,
        createdAt: Date.now(),
    };

    let waistPreAlignText = '未执行';
    if (waistYawPreAlign?.applied) {
        const yawTargetDeg = (waistYawPreAlign.yawTarget * 180) / Math.PI;
        const yawAfterDeg = (waistYawPreAlign.yawAfter * 180) / Math.PI;
        const remainText = Number.isFinite(waistYawPreAlign.waistLimitMinRemainDeg)
            ? `, 限位余量${Number(waistYawPreAlign.waistLimitMinRemainDeg).toFixed(2)}deg`
            : '';
        waistPreAlignText = `已执行(目标${yawTargetDeg.toFixed(1)}deg, 结果${yawAfterDeg.toFixed(1)}deg${remainText})`;
    } else if (waistYawPreAlign?.reason === 'small_delta') {
        const deltaDeg = Math.abs((waistYawPreAlign.yawDeltaRad || 0) * 180 / Math.PI);
        waistPreAlignText = `跳过(Δyaw=${deltaDeg.toFixed(2)}deg)`;
    } else if (waistYawPreAlign?.reason === 'limit_blocked') {
        const remainText = Number.isFinite(waistYawPreAlign.waistLimitMinRemainDeg)
            ? `余量${Number(waistYawPreAlign.waistLimitMinRemainDeg).toFixed(2)}deg`
            : '贴近限位';
        waistPreAlignText = `跳过(${remainText})`;
    } else if (waistYawPreAlign?.reason === 'solve_failed') {
        waistPreAlignText = '失败回退';
    } else if (waistYawPreAlign?.reason) {
        waistPreAlignText = `未执行(${waistYawPreAlign.reason})`;
    }

    console.log(
        `规划运动：偏差 XY=${(poseErrorBeforeWaist.posErrXY * 100).toFixed(1)}→${(poseError.posErrXY * 100).toFixed(1)} cm, ` +
        `3D=${(poseErrorBeforeWaist.posErr3D * 100).toFixed(1)}→${(poseError.posErr3D * 100).toFixed(1)} cm, ` +
        `rot=${poseError.rotErrDeg.toFixed(2)} deg, ` +
        `腰部yaw预对齐=${waistPreAlignText}, ` +
        `底盘阈值=${(params.baseThreshold * 100).toFixed(1)} cm, ` +
        `底盘阶段=${needBaseMovePlanned ? `需要(${baseStages.length}段)` : '跳过'}(仅XY), ` +
        `同步阶段=腰部锁定虚拟状态1, ` +
        `腰部竖直约束=${params.armIKKeepBodyZVertical ? '开' : '关'}, ` +
        `选中yaw=${(baseTargetPose.yaw * 180 / Math.PI).toFixed(1)} deg(底盘保持), ` +
        `jointCost=${jointCost.toFixed(3)}`
    );

    return plan;
}

function buildSingleArmMotionPlan(
    DQ,
    vis,
    targetDQArray,
    params,
    armIndex,
    armLabel = '手臂',
    pointLabel = '',
    forcedBaseYaw = 0
) {
    const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qNow) || qNow.length === 0) {
        updateStatus(STATE.ERROR, '人形关节未初始化');
        return null;
    }

    const baseStartPose = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });
    const currentArmWorld = getHumanoidArmWorldPose(DQ, vis, qNow, baseStartPose, armIndex);
    if (!currentArmWorld) {
        updateStatus(STATE.ERROR, `${armLabel}当前位姿获取失败`);
        return null;
    }

    const poseErrorBeforeWaist = computePoseError(DQ, currentArmWorld, targetDQArray);

    // 虚拟状态1：腰部先做 yaw + 高度预补偿（围绕目标臂误差求解）
    const waistYawPreAlign = planSingleArmWaistPreAlignment(
        DQ,
        vis,
        currentArmWorld,
        targetDQArray,
        baseStartPose,
        params,
        armIndex
    );
    const qSeedAfterWaist =
        Array.isArray(waistYawPreAlign?.qSeed) && waistYawPreAlign.qSeed.length === qNow.length
            ? waistYawPreAlign.qSeed.slice()
            : qNow.slice();
    const planningArmWorld =
        getHumanoidArmWorldPose(DQ, vis, qSeedAfterWaist, baseStartPose, armIndex) ||
        waistYawPreAlign?.armWorldAfterAlign ||
        currentArmWorld;

    const poseError = computePoseError(DQ, planningArmWorld, targetDQArray);
    const needBaseMoveXY = poseError.posErrXY > params.baseThreshold;
    const targetBaseYaw = Number.isFinite(forcedBaseYaw) ? wrapAngle(Number(forcedBaseYaw)) : 0;
    const baseYawTol = Math.max(1e-3, Number(appState.humanoidBaseControlParams?.tolYaw) || 0.05);
    const needBaseMoveYaw = Math.abs(wrapAngle(targetBaseYaw - (baseStartPose.yaw || 0))) > baseYawTol;
    const needBaseMove = needBaseMoveXY || needBaseMoveYaw;
    // 虚拟状态2：以虚拟状态1的目标臂位姿做底盘 XY 补偿
    const baseTargetXY = computeBaseTargetXY(DQ, planningArmWorld, targetDQArray, baseStartPose, needBaseMoveXY);
    const baseTargetPose = {
        x: baseTargetXY.x,
        y: baseTargetXY.y,
        yaw: targetBaseYaw,
    };
    const baseStages = needBaseMove ? buildBaseMotionStages(baseStartPose, baseTargetPose) : [];
    const needBaseMovePlanned = baseStages.length > 0;
    const solveResult = solveSingleArmTargetForBasePose(
        DQ,
        vis,
        targetDQArray,
        baseTargetPose,
        params,
        armIndex,
        qSeedAfterWaist
    );
    if (!solveResult.ok) {
        console.warn(
            `顺序放置规划(${armLabel}${pointLabel ? `->${pointLabel}` : ''})固定yaw求解失败 ` +
            `(yaw=${(baseTargetPose.yaw * 180 / Math.PI).toFixed(1)}deg): ${solveResult.message || '未知'}`
        );
        updateStatus(STATE.ERROR, `${armLabel} IK 规划失败`);
        return null;
    }

    const qStartRef = adaptJointVectorToReference(solveResult.qStart, solveResult.qStart);
    const qVirtualState1 = buildWaistPreAlignTargetQ({ qStart: qStartRef, waistYawPreAlign }, qStartRef);
    const qVirtualState2 = qVirtualState1.slice();
    const qVirtualState3FullBody = adaptJointVectorToReference(solveResult.qTarget, qStartRef);
    // 最终补偿：仅允许“腰部 + 当前目标臂”变化，另一臂保持当前状态
    const qVirtualState3Final = buildSingleArmWaistAndArmJointTarget(
        qVirtualState1,
        qVirtualState3FullBody,
        armIndex,
        { qStart: qVirtualState1 }
    );
    // 同步阶段：腰部锁定在虚拟状态1，仅目标臂朝虚拟状态3逼近
    const qVirtualState3Sync = buildSingleArmSyncJointTarget(
        qVirtualState1,
        qVirtualState3Final,
        armIndex,
        { qStart: qVirtualState1 }
    );
    const qSyncTarget = adaptJointVectorToReference(qVirtualState3Sync, qStartRef);
    const qFinalRef = qVirtualState3Final.slice();
    const arm1BodyStart = computeArmPoseRelativeToBody(DQ, vis, qVirtualState1, 1);
    const arm2BodyStart = computeArmPoseRelativeToBody(DQ, vis, qVirtualState1, 2);
    const arm1BodySync = computeArmPoseRelativeToBody(DQ, vis, qSyncTarget, 1);
    const arm2BodySync = computeArmPoseRelativeToBody(DQ, vis, qSyncTarget, 2);
    const syncArmBodyRelativeTargets = {
        arm1: Array.isArray(arm1BodyStart) && arm1BodyStart.length === 8
            ? (armIndex === 1 && Array.isArray(arm1BodySync) ? arm1BodySync.slice() : arm1BodyStart.slice())
            : null,
        arm2: Array.isArray(arm2BodyStart) && arm2BodyStart.length === 8
            ? (armIndex === 2 && Array.isArray(arm2BodySync) ? arm2BodySync.slice() : arm2BodyStart.slice())
            : null,
    };
    const baseMoveLocalTarget = computeLocalTargetForBasePose(DQ, vis, targetDQArray, baseTargetPose);

    const plan = {
        planKind: 'single-arm',
        armIndex,
        armLabel,
        pointLabel,
        targetDQ: [...targetDQArray],
        params: { ...params },
        needBaseMove: needBaseMovePlanned,
        poseErrorBeforeWaist,
        poseError,
        waistYawPreAlign,
        baseStartPose,
        baseTargetPose,
        baseStages,
        baseMoveLocalTarget,
        requestedBaseYaw: targetBaseYaw,
        qVirtualState1,
        qVirtualState2,
        qVirtualState3FullBody,
        qStart: qStartRef,
        qSyncTarget,
        qTarget: qFinalRef,
        qWaistVirtualTarget: qVirtualState1.slice(),
        syncArmBodyRelativeTargets,
        solvedError: solveResult.solvedError,
        createdAt: Date.now(),
    };
    const jointCost = computeJointMotionCost(qStartRef, qFinalRef);
    let waistPreAlignText = '未执行';
    if (waistYawPreAlign?.applied) {
        const yawDeg = Math.abs((waistYawPreAlign.yawDeltaRad || 0) * 180 / Math.PI);
        const zMm = (waistYawPreAlign.heightDeltaM || 0) * 1000;
        const remainText = Number.isFinite(waistYawPreAlign.waistLimitMinRemainDeg)
            ? `, 限位余量${Number(waistYawPreAlign.waistLimitMinRemainDeg).toFixed(2)}deg`
            : '';
        waistPreAlignText = `已执行(Δyaw=${yawDeg.toFixed(2)}deg, Δz=${zMm.toFixed(1)}mm${remainText})`;
    } else if (waistYawPreAlign?.reason === 'small_delta') {
        waistPreAlignText = '跳过(变化小)';
    } else if (waistYawPreAlign?.reason === 'limit_blocked') {
        const remainText = Number.isFinite(waistYawPreAlign.waistLimitMinRemainDeg)
            ? `限位余量${Number(waistYawPreAlign.waistLimitMinRemainDeg).toFixed(2)}deg`
            : '贴近限位';
        waistPreAlignText = `跳过(${remainText})`;
    } else if (waistYawPreAlign?.reason) {
        waistPreAlignText = `未执行(${waistYawPreAlign.reason})`;
    }

    console.log(
        `顺序放置子规划(${armLabel}${pointLabel ? `->${pointLabel}` : ''})：` +
        `偏差XY=${(poseErrorBeforeWaist.posErrXY * 100).toFixed(1)}->${(poseError.posErrXY * 100).toFixed(1)}cm, ` +
        `3D=${(poseErrorBeforeWaist.posErr3D * 100).toFixed(1)}->${(poseError.posErr3D * 100).toFixed(1)}cm, ` +
        `rot=${poseError.rotErrDeg.toFixed(2)}deg, ` +
        `腰部预对齐=${waistPreAlignText}, ` +
        `底盘阶段=${needBaseMovePlanned ? `需要(${baseStages.length}段)` : '跳过'}(XY=${needBaseMoveXY ? 'Y' : 'N'}, yaw=${needBaseMoveYaw ? 'Y' : 'N'}), ` +
        `选中yaw=${(baseTargetPose.yaw * 180 / Math.PI).toFixed(1)}deg, ` +
        `jointCost=${jointCost.toFixed(3)}`
    );

    return plan;
}

/**
 * 获取当前世界坐标系下的绝对位姿 DQ 数组
 */
function getCurrentWorldAbsolutePose(DQ) {
    const vis = appState.visualizer;
    if (!vis || !vis._getHumanoidAbsolutePose || !vis._composeHumanoidWorldPose) return null;

    const q = window.humanoidQ;
    if (!q || q.length === 0) return null;

    const basePose = appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 };
    return getWorldAbsolutePoseFromBaseAndJoints(vis, q, basePose);
}

function getWorldAbsolutePoseFromBaseAndJoints(vis, q, basePose) {
    const absLocal = vis?._getHumanoidAbsolutePose?.(q);
    if (!absLocal) return null;
    return vis?._composeHumanoidWorldPose?.(basePose, absLocal) || absLocal;
}

function getHumanoidWaistWorldPose(vis, q, basePose) {
    if (!vis || !Array.isArray(q) || q.length === 0) return null;
    const waistLocal = vis?.humanoidModel?.fkmEffector?.(q, 0);
    if (!waistLocal) return null;
    return vis?._composeHumanoidWorldPose?.(basePose, waistLocal) || waistLocal;
}

/**
 * 计算绝对位姿误差（平移 + 旋转）
 * @returns {{posErr3D: number, posErrXY: number, rotErrRad: number, rotErrDeg: number}}
 */
function computePoseError(DQ, currentDQArray, targetDQArray) {
    const curPose = safeCreatePoseDQ(DQ, currentDQArray);
    const tgtPose = safeCreatePoseDQ(DQ, targetDQArray);
    if (!curPose || !tgtPose) {
        return {
            posErr3D: Number.POSITIVE_INFINITY,
            posErrXY: Number.POSITIVE_INFINITY,
            rotErrRad: Number.POSITIVE_INFINITY,
            rotErrDeg: Number.POSITIVE_INFINITY,
        };
    }
    const tCur = curPose.t;
    const tTgt = tgtPose.t;

    const dx = tTgt[1] - tCur[1];
    const dy = tTgt[2] - tCur[2];
    const dz = tTgt[3] - tCur[3];
    const posErr3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const posErrXY = Math.hypot(dx, dy);

    const qCur = curPose.q;
    const qTgt = tgtPose.q;
    const dot = Math.min(1, Math.max(-1,
        Math.abs(
            (qCur[0] || 0) * (qTgt[0] || 0) +
            (qCur[1] || 0) * (qTgt[1] || 0) +
            (qCur[2] || 0) * (qTgt[2] || 0) +
            (qCur[3] || 0) * (qTgt[3] || 0)
        )
    ));
    const rotErrRad = 2 * Math.acos(dot);
    const rotErrDeg = (rotErrRad * 180) / Math.PI;

    return { posErr3D, posErrXY, rotErrRad, rotErrDeg };
}

function planWaistYawPreAlignment(DQ, vis, currentWorldAbs, targetWorldAbs, basePose, params) {
    const result = {
        applied: false,
        reason: 'not_needed',
        yawCurrent: null,
        yawTarget: null,
        yawAfter: null,
        yawDeltaRad: 0,
        waistLimitPenalty: 0,
        waistLimitHardHit: false,
        waistLimitMinRemainDeg: null,
        qSeed: null,
        relativePoseConstraint: null,
        worldAbsAfterAlign: currentWorldAbs,
    };

    if (!params?.waistYawPreAlignEnabled) {
        result.reason = 'disabled';
        return result;
    }

    const currentQ = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(currentQ) || currentQ.length === 0) {
        result.reason = 'q_unavailable';
        return result;
    }

    const curPose = safeCreatePoseDQ(DQ, currentWorldAbs);
    const tgtPose = safeCreatePoseDQ(DQ, targetWorldAbs);
    if (!curPose || !tgtPose) {
        result.reason = 'pose_invalid';
        return result;
    }

    const yawCurrent = quaternionYaw(curPose.q);
    const yawTarget = quaternionYaw(tgtPose.q);
    if (!Number.isFinite(yawCurrent) || !Number.isFinite(yawTarget)) {
        result.reason = 'yaw_invalid';
        return result;
    }

    const yawDelta = wrapAngle(yawTarget - yawCurrent);
    const yawDeltaDeg = Math.abs(yawDelta * 180 / Math.PI);
    const minYawDeltaDeg = Math.max(0, Number(params.waistYawPreAlignMinDeg) || 1.0);

    result.yawCurrent = yawCurrent;
    result.yawTarget = yawTarget;
    result.yawDeltaRad = yawDelta;

    if (yawDeltaDeg < minYawDeltaDeg) {
        result.reason = 'small_delta';
        return result;
    }

    const waistWorldNow = getHumanoidWaistWorldPose(vis, currentQ, basePose);
    const waistPoseNow = safeCreatePoseDQ(DQ, waistWorldNow);
    if (!waistPoseNow) {
        result.reason = 'waist_pose_invalid';
        return result;
    }

    const waistYawNow = quaternionYaw(waistPoseNow.q);
    if (!Number.isFinite(waistYawNow)) {
        result.reason = 'waist_yaw_invalid';
        return result;
    }

    // 仅腰部预对齐：把“绝对位姿需要的 yaw 增量”映射到 body_link 的 yaw 调整。
    const yawDeltaQuat = [Math.cos(yawDelta / 2), 0, 0, Math.sin(yawDelta / 2)];
    const yawAlignedQuat = normalizeQuaternion(multiplyQuaternion(yawDeltaQuat, waistPoseNow.q));
    const yawAlignWorldTarget = buildPoseDQFromRotationTranslation(
        yawAlignedQuat,
        [waistPoseNow.t[1] || 0, waistPoseNow.t[2] || 0, waistPoseNow.t[3] || 0]
    );
    if (!yawAlignWorldTarget) {
        result.reason = 'target_build_failed';
        return result;
    }

    const solveResult = solveWaistTargetForBasePose(
        DQ,
        vis,
        yawAlignWorldTarget,
        basePose,
        params,
        currentQ
    );
    if (!solveResult.ok || !Array.isArray(solveResult.qTarget) || solveResult.qTarget.length !== currentQ.length) {
        result.reason = 'solve_failed';
        return result;
    }

    const waistLimits = getHumanoidWaistJointLimits(vis, currentQ, getWaistJointCount({ qStart: currentQ }));
    const limitEval = evaluateWaistLimitPenalty(solveResult.qTarget, waistLimits, params);
    result.waistLimitPenalty = limitEval.penalty;
    result.waistLimitHardHit = !!(limitEval.hardHit || limitEval.outside);
    result.waistLimitMinRemainDeg = limitEval.minRemainDeg;
    const allowNearLimit = params?.waistPreAlignAllowNearLimit === true;
    if (result.waistLimitHardHit && !allowNearLimit) {
        result.reason = 'limit_blocked';
        return result;
    }

    const worldAfter = getWorldAbsolutePoseFromBaseAndJoints(vis, solveResult.qTarget, basePose) || currentWorldAbs;
    const afterPose = safeCreatePoseDQ(DQ, worldAfter);

    result.applied = true;
    result.reason = result.waistLimitHardHit ? 'applied_near_limit' : 'applied';
    result.qSeed = solveResult.qTarget.slice();
    result.relativePoseConstraint = getHumanoidRelativePoseConstraint(vis, solveResult.qTarget);
    result.worldAbsAfterAlign = worldAfter;
    result.yawAfter = afterPose ? quaternionYaw(afterPose.q) : yawCurrent;

    return result;
}

function planSingleArmWaistPreAlignment(
    DQ,
    vis,
    currentArmWorldAbs,
    targetArmWorldAbs,
    basePose,
    params,
    armIndex
) {
    const result = {
        applied: false,
        reason: 'not_needed',
        yawCurrent: null,
        yawTarget: null,
        yawAfter: null,
        yawDeltaRad: 0,
        heightCurrentM: null,
        heightTargetM: null,
        heightDeltaM: 0,
        waistLimitPenalty: 0,
        waistLimitHardHit: false,
        waistLimitMinRemainDeg: null,
        qSeed: null,
        armWorldAfterAlign: currentArmWorldAbs,
    };

    if (!params?.waistYawPreAlignEnabled) {
        result.reason = 'disabled';
        return result;
    }

    const currentQ = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(currentQ) || currentQ.length === 0) {
        result.reason = 'q_unavailable';
        return result;
    }

    const curPose = safeCreatePoseDQ(DQ, currentArmWorldAbs);
    const tgtPose = safeCreatePoseDQ(DQ, targetArmWorldAbs);
    if (!curPose || !tgtPose) {
        result.reason = 'pose_invalid';
        return result;
    }

    const yawCurrent = quaternionYaw(curPose.q);
    const yawTarget = quaternionYaw(tgtPose.q);
    const yawDeltaRaw = (Number.isFinite(yawCurrent) && Number.isFinite(yawTarget))
        ? wrapAngle(yawTarget - yawCurrent)
        : 0;
    const yawDeltaDeg = Math.abs(yawDeltaRaw * 180 / Math.PI);
    const minYawDeltaDeg = Math.max(0, Number(params.waistYawPreAlignMinDeg) || 1.0);
    const needYawAlign = Number.isFinite(yawCurrent) && Number.isFinite(yawTarget) && yawDeltaDeg >= minYawDeltaDeg;

    const heightCurrent = Number(curPose.t?.[3]) || 0;
    const heightTarget = Number(tgtPose.t?.[3]) || 0;
    const heightDelta = heightTarget - heightCurrent;
    const minHeightDeltaM = Math.max(1e-4, (Number(params.finalPosTolMm) || 0.5) / 1000);
    const needHeightAlign = Math.abs(heightDelta) >= minHeightDeltaM;

    result.yawCurrent = Number.isFinite(yawCurrent) ? yawCurrent : null;
    result.yawTarget = Number.isFinite(yawTarget) ? yawTarget : null;
    result.yawDeltaRad = needYawAlign ? yawDeltaRaw : 0;
    result.heightCurrentM = heightCurrent;
    result.heightTargetM = heightTarget;
    result.heightDeltaM = needHeightAlign ? heightDelta : 0;

    if (!needYawAlign && !needHeightAlign) {
        result.reason = Number.isFinite(yawCurrent) && Number.isFinite(yawTarget) ? 'small_delta' : 'yaw_invalid';
        return result;
    }

    const waistWorldNow = getHumanoidWaistWorldPose(vis, currentQ, basePose);
    const waistPoseNow = safeCreatePoseDQ(DQ, waistWorldNow);
    if (!waistPoseNow) {
        result.reason = 'waist_pose_invalid';
        return result;
    }

    const candidateYawDeltas = [];
    if (needYawAlign) {
        const raw = wrapAngle(yawDeltaRaw);
        candidateYawDeltas.push(raw);
        candidateYawDeltas.push(wrapAngle(-raw));
        candidateYawDeltas.push(wrapAngle(raw + Math.PI));
        candidateYawDeltas.push(wrapAngle(raw - Math.PI));
    } else {
        candidateYawDeltas.push(0);
    }
    // 去重，避免重复求解
    const uniqYawDeltas = [];
    for (const v of candidateYawDeltas) {
        if (!Number.isFinite(v)) continue;
        const exists = uniqYawDeltas.some(u => Math.abs(wrapAngle(u - v)) < 1e-4);
        if (!exists) uniqYawDeltas.push(v);
    }
    if (uniqYawDeltas.length === 0) uniqYawDeltas.push(0);

    const zTarget = (waistPoseNow.t[3] || 0) + (needHeightAlign ? heightDelta : 0);
    const waistLimits = getHumanoidWaistJointLimits(vis, currentQ, getWaistJointCount({ qStart: currentQ }));
    const allowNearLimit = params?.waistPreAlignAllowNearLimit === true;
    let best = null;
    let bestAny = null;

    for (const yawDelta of uniqYawDeltas) {
        const yawDeltaQuat = [Math.cos(yawDelta / 2), 0, 0, Math.sin(yawDelta / 2)];
        const waistQuatTarget = normalizeQuaternion(multiplyQuaternion(yawDeltaQuat, waistPoseNow.q));
        if (!waistQuatTarget) continue;

        const waistTargetWorld = buildPoseDQFromRotationTranslation(
            waistQuatTarget,
            [waistPoseNow.t[1] || 0, waistPoseNow.t[2] || 0, zTarget]
        );
        if (!waistTargetWorld) continue;

        const solveResult = solveWaistTargetForBasePose(
            DQ,
            vis,
            waistTargetWorld,
            basePose,
            params,
            currentQ
        );
        if (!solveResult.ok || !Array.isArray(solveResult.qTarget) || solveResult.qTarget.length !== currentQ.length) {
            continue;
        }

        const qSeed = solveResult.qTarget.slice();
        const armWorldAfterAlign = getHumanoidArmWorldPose(DQ, vis, qSeed, basePose, armIndex) || currentArmWorldAbs;
        const err = computePoseError(DQ, armWorldAfterAlign, targetArmWorldAbs);
        const posMm = (Number(err?.posErr3D) || Number.POSITIVE_INFINITY) * 1000;
        const rotDeg = Number(err?.rotErrDeg) || Number.POSITIVE_INFINITY;
        const yawMagnitudeDeg = Math.abs(yawDelta * 180 / Math.PI);
        const yawDeviationDeg = Math.abs(wrapAngle(yawDelta - (needYawAlign ? yawDeltaRaw : 0))) * 180 / Math.PI;
        const waistCount = getWaistJointCount({ qStart: currentQ });
        let waistMotionMaxDeg = 0;
        for (let i = 0; i < waistCount && i < currentQ.length && i < qSeed.length; i++) {
            const d = Math.abs(wrapAngle((qSeed[i] || 0) - (currentQ[i] || 0))) * 180 / Math.PI;
            if (d > waistMotionMaxDeg) waistMotionMaxDeg = d;
        }
        // 右臂步骤容易出现“可行但不自然”的大角度腰部候选，增加方向一致性与连续性惩罚。
        const yawPenalty = yawMagnitudeDeg * 0.08 + yawDeviationDeg * 0.45;
        const continuityPenalty = waistMotionMaxDeg * 0.20;
        const limitEval = evaluateWaistLimitPenalty(qSeed, waistLimits, params);
        const hardHit = !!(limitEval.hardHit || limitEval.outside);
        const limitPenalty = Number(limitEval.penalty) || 0;
        const score = posMm + 3.0 * rotDeg + yawPenalty + continuityPenalty + 120.0 * limitPenalty;
        const candidate = {
            score,
            yawDelta,
            yawDeviationDeg,
            qSeed,
            armWorldAfterAlign,
            limitEval,
        };

        if (!bestAny || score < bestAny.score - 1e-9) {
            bestAny = candidate;
        }
        if (hardHit && !allowNearLimit) {
            continue;
        }

        if (
            !best ||
            score < best.score - 1e-9 ||
            (Math.abs(score - best.score) <= 1e-9 && (candidate.yawDeviationDeg || 0) < (best.yawDeviationDeg || 0))
        ) {
            best = candidate;
        }
    }

    if (!best) {
        if (bestAny && !allowNearLimit) {
            result.reason = 'limit_blocked';
            result.waistLimitPenalty = Number(bestAny.limitEval?.penalty) || 0;
            result.waistLimitHardHit = true;
            result.waistLimitMinRemainDeg = bestAny.limitEval?.minRemainDeg ?? null;
            return result;
        }
        result.reason = 'solve_failed';
        return result;
    }

    const armPoseAfterAlign = safeCreatePoseDQ(DQ, best.armWorldAfterAlign);
    result.applied = true;
    result.reason = (best.limitEval?.hardHit || best.limitEval?.outside) ? 'applied_near_limit' : 'applied';
    result.yawDeltaRad = best.yawDelta;
    result.waistLimitPenalty = Number(best.limitEval?.penalty) || 0;
    result.waistLimitHardHit = !!(best.limitEval?.hardHit || best.limitEval?.outside);
    result.waistLimitMinRemainDeg = best.limitEval?.minRemainDeg ?? null;
    result.qSeed = best.qSeed;
    result.armWorldAfterAlign = best.armWorldAfterAlign;
    result.yawAfter = armPoseAfterAlign ? quaternionYaw(armPoseAfterAlign.q) : result.yawCurrent;
    return result;
}

/**
 * 在给定底盘位姿下评估某组关节对应的世界绝对位姿误差
 */
function evaluateHumanoidWorldError(DQ, vis, q, basePose, worldTargetDQArray) {
    const worldAbs = getWorldAbsolutePoseFromBaseAndJoints(vis, q, basePose);
    if (!worldAbs) return null;
    return computePoseError(DQ, worldAbs, worldTargetDQArray);
}

function evaluateHumanoidWaistWorldError(DQ, vis, q, basePose, worldTargetDQArray) {
    const waistWorld = getHumanoidWaistWorldPose(vis, q, basePose);
    if (!waistWorld) return null;
    return computePoseError(DQ, waistWorld, worldTargetDQArray);
}

function evaluateHumanoidRelativeError(DQ, vis, q, relativeTargetDQArray) {
    if (!DQ || !vis || !Array.isArray(q) || q.length === 0) return null;
    if (!Array.isArray(relativeTargetDQArray) || relativeTargetDQArray.length !== 8) return null;
    const relNow = vis._getHumanoidRelativePose?.(q);
    if (!Array.isArray(relNow) || relNow.length !== 8) return null;
    return computePoseError(DQ, relNow, relativeTargetDQArray);
}

function getHumanoidRelativePoseConstraint(vis, q = null) {
    if (!vis || typeof vis._getHumanoidRelativePose !== 'function') return null;
    const qRef = Array.isArray(q) && q.length > 0 ? q : (window.humanoidQ || appState.humanoidQ);
    if (!Array.isArray(qRef) || qRef.length === 0) return null;
    const rel = vis._getHumanoidRelativePose(qRef);
    return normalizeDualQuaternionArray(rel);
}

function computeLocalTargetForBasePose(DQ, vis, worldTargetDQArray, basePose) {
    if (!DQ || !vis || !Array.isArray(worldTargetDQArray) || worldTargetDQArray.length !== 8) return null;
    const baseDQ = vis._makeHumanoidBaseDQ?.(basePose);
    if (!baseDQ) return null;
    const normalizedTarget = normalizeDualQuaternionArray(worldTargetDQArray);
    if (!normalizedTarget) return null;
    const localTargetDQ = DQ.multiply(DQ.inverse(baseDQ), DQ.createFromArray(normalizedTarget));
    return DQ.toArray(localTargetDQ);
}

/**
 * 计算底盘目标平移（仅 x/y）
 */
function computeBaseTargetXY(DQ, currentWorldAbs, targetWorldAbs, basePose, needBaseMove) {
    if (!needBaseMove) {
        return {
            x: basePose.x || 0,
            y: basePose.y || 0,
        };
    }

    const curPose = safeCreatePoseDQ(DQ, currentWorldAbs);
    const tgtPose = safeCreatePoseDQ(DQ, targetWorldAbs);
    if (!curPose || !tgtPose) {
        return {
            x: basePose.x || 0,
            y: basePose.y || 0,
        };
    }
    const tCur = curPose.t;
    const tTgt = tgtPose.t;

    const deltaX = tTgt[1] - tCur[1];
    const deltaY = tTgt[2] - tCur[2];

    return {
        x: (basePose.x || 0) + deltaX,
        y: (basePose.y || 0) + deltaY,
    };
}

function buildBaseMotionStages(baseStartPose, baseTargetPose) {
    const start = cloneBasePose(baseStartPose || { x: 0, y: 0, yaw: 0 });
    const target = cloneBasePose(baseTargetPose || start);
    const pRaw = appState.humanoidBaseControlParams || {};
    const tolPos = Math.max(1e-4, Number(pRaw.tolPos) || 0.03);
    const tolYaw = Math.max(1e-4, Number(pRaw.tolYaw) || 0.05);

    const dx = target.x - start.x;
    const dy = target.y - start.y;
    const travelDist = Math.hypot(dx, dy);
    const yawDelta = Math.abs(wrapAngle(target.yaw - start.yaw));

    const stages = [];
    if (travelDist > tolPos || yawDelta > tolYaw) {
        let label = '底盘调整';
        if (travelDist > tolPos && yawDelta > tolYaw) label = '底盘平移+转向';
        else if (travelDist > tolPos) label = '底盘平移';
        else if (yawDelta > tolYaw) label = '底盘转向';
        stages.push({
            key: 'move-xy',
            label,
            targetPose: { x: target.x, y: target.y, yaw: target.yaw },
        });
    }
    return stages;
}

function computeJointMotionCost(qStart, qTarget) {
    if (!Array.isArray(qStart) || !Array.isArray(qTarget) || qStart.length !== qTarget.length || qStart.length === 0) {
        return Number.POSITIVE_INFINITY;
    }

    let sumSq = 0;
    let maxAbs = 0;
    for (let i = 0; i < qStart.length; i++) {
        const d = Math.abs(wrapAngle((qTarget[i] || 0) - (qStart[i] || 0)));
        sumSq += d * d;
        if (d > maxAbs) maxAbs = d;
    }
    const rms = Math.sqrt(sumSq / qStart.length);
    return rms + 0.25 * maxAbs;
}

function clampValue(v, min, max) {
    return Math.max(min, Math.min(max, v));
}

function sampleTrajectoryByCount(trajectory, count) {
    if (!Array.isArray(trajectory) || trajectory.length === 0) return [];
    if (!Number.isFinite(count) || count <= 1) return [trajectory[0], trajectory[trajectory.length - 1]].filter(Boolean);
    if (trajectory.length <= count) return trajectory.slice();

    const sampled = [];
    for (let i = 0; i < count; i++) {
        const idx = Math.round((i * (trajectory.length - 1)) / (count - 1));
        sampled.push(trajectory[Math.max(0, Math.min(trajectory.length - 1, idx))]);
    }
    return sampled;
}

function sampleTrajectoryByCountWithIndices(trajectory, count) {
    if (!Array.isArray(trajectory) || trajectory.length === 0) return [];
    if (!Number.isFinite(count) || count <= 1) {
        const first = trajectory[0];
        const last = trajectory[trajectory.length - 1];
        const out = [];
        if (first) out.push({ pose: first, idx: 0 });
        if (last && trajectory.length > 1) out.push({ pose: last, idx: trajectory.length - 1 });
        return out;
    }
    if (trajectory.length <= count) {
        return trajectory.map((pose, idx) => ({ pose, idx }));
    }

    const sampled = [];
    for (let i = 0; i < count; i++) {
        const idx = Math.round((i * (trajectory.length - 1)) / (count - 1));
        const clampedIdx = Math.max(0, Math.min(trajectory.length - 1, idx));
        sampled.push({
            pose: trajectory[clampedIdx],
            idx: clampedIdx,
        });
    }
    return sampled;
}

function stepQTowardsGoal(currentQ, goalQ, blend, maxStepRad, gainScale = 1) {
    if (!Array.isArray(currentQ) || !Array.isArray(goalQ) || currentQ.length !== goalQ.length) {
        return Array.isArray(currentQ) ? currentQ.slice() : [];
    }
    const scale = clampValue(Number(gainScale) || 1, 0.01, 1.0);
    const nextQ = [];
    for (let i = 0; i < currentQ.length; i++) {
        const start = currentQ[i] || 0;
        const goal = goalQ[i] || start;
        const d = wrapAngle(goal - start);
        const step = clampValue(d * blend * scale, -maxStepRad * scale, maxStepRad * scale);
        nextQ.push(start + step);
    }
    return nextQ;
}

function blendWaistPreAlignForPreviewStep(currentQ, waistTargetQ, waistCount, params) {
    if (!Array.isArray(currentQ) || currentQ.length === 0) return [];
    if (!Array.isArray(waistTargetQ) || waistTargetQ.length === 0) return currentQ.slice();
    const nWaist = Math.max(0, Math.min(Number(waistCount) || 0, currentQ.length, waistTargetQ.length));
    if (nWaist <= 0) return currentQ.slice();

    const blend = clampValue(Number(params?.waistBlendDuringBaseBlend) || 0.34, 0.01, 1.0);
    const maxStepRad = Math.max(0.1, Number(params?.waistBlendDuringBaseMaxStepDeg) || 2.8) * Math.PI / 180;
    const minStepRad = Math.max(0, Number(params?.waistBlendDuringBaseMinStepDeg) || 0.22) * Math.PI / 180;

    const nextQ = currentQ.slice();
    for (let i = 0; i < nWaist; i++) {
        const cur = currentQ[i] || 0;
        const goal = waistTargetQ[i] || cur;
        const d = wrapAngle(goal - cur);
        const absD = Math.abs(d);
        if (absD <= 1e-12) {
            nextQ[i] = goal;
            continue;
        }
        const stepMag = clampValue(absD * blend, minStepRad, maxStepRad);
        const step = Math.sign(d) * Math.min(absD, stepMag);
        nextQ[i] = cur + step;
    }
    return nextQ;
}

function computeTrackingWarmupScale(stepIndex, cycleMs, warmupMs, minScale) {
    const warmMs = Math.max(0, Number(warmupMs) || 0);
    const minS = clampValue(Number(minScale) || 0.3, 0.05, 1.0);
    if (warmMs <= 0) return 1;
    const elapsedMs = Math.max(0, Number(stepIndex) || 0) * Math.max(1, Number(cycleMs) || 1);
    const t = clampValue(elapsedMs / warmMs, 0, 1);
    return minS + (1 - minS) * easeInOutCubic(t);
}

function applyHumanoidBasePoseToState(vis, basePose) {
    const pose = cloneBasePose(basePose || { x: 0, y: 0, yaw: 0 });

    if (appState.humanoidBasePose && typeof appState.humanoidBasePose === 'object') {
        appState.humanoidBasePose.x = pose.x;
        appState.humanoidBasePose.y = pose.y;
        appState.humanoidBasePose.yaw = pose.yaw;
    } else {
        appState.humanoidBasePose = { ...pose };
    }

    if (typeof window !== 'undefined') {
        if (window.humanoidBasePose && typeof window.humanoidBasePose === 'object') {
            window.humanoidBasePose.x = pose.x;
            window.humanoidBasePose.y = pose.y;
            window.humanoidBasePose.yaw = pose.yaw;
        } else {
            window.humanoidBasePose = { ...pose };
        }
    }

    vis?.setHumanoidBasePose?.(pose);
}

function withTemporaryHumanoidBasePose(vis, tempPose, workFn) {
    if (!tempPose || typeof workFn !== 'function') return workFn?.();

    const restorePose = cloneBasePose(
        appState.humanoidBasePose ||
        (typeof window !== 'undefined' ? window.humanoidBasePose : null) ||
        { x: 0, y: 0, yaw: 0 }
    );

    applyHumanoidBasePoseToState(vis, tempPose);
    try {
        return workFn();
    } finally {
        applyHumanoidBasePoseToState(vis, restorePose);
    }
}

function computeTrackingGoalQForPreview(
    vis,
    localTargetArray,
    qStart,
    params,
    previewBasePose = null,
    fallbackQ = null,
    relativePoseConstraint = null,
    includeWaistOverride = null,
    bodyRelativeTargets = null,
    singleArmIndex = null
) {
    if (!vis) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart?.length) return fallbackQ.slice();
        return Array.isArray(qStart) ? qStart.slice() : [];
    }
    if (!Array.isArray(qStart) || qStart.length === 0) return [];

    const qGlobalBefore = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qGlobalBefore) || qGlobalBefore.length === 0) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart.length) return fallbackQ.slice();
        return qStart.slice();
    }

    const DQ = getDQModule()?.DQWrapper;
    const hasLocalTarget = Array.isArray(localTargetArray) && localTargetArray.length === 8;
    const previewArmIndex = Number(singleArmIndex);
    const hasSingleArmPreviewTarget =
        (previewArmIndex === 1 || previewArmIndex === 2) &&
        typeof vis.updateHumanoidSingleArmPoseTarget === 'function';
    const singleArmBodyRelativeTarget =
        hasSingleArmPreviewTarget &&
        Array.isArray(bodyRelativeTargets?.[`arm${previewArmIndex}`]) &&
        bodyRelativeTargets[`arm${previewArmIndex}`].length === 8
            ? bodyRelativeTargets[`arm${previewArmIndex}`]
            : null;
    const hasSingleArmBodyRelativeTarget =
        Array.isArray(singleArmBodyRelativeTarget) &&
        singleArmBodyRelativeTarget.length === 8 &&
        !!DQ;
    const hasBodyRelativeTargets =
        !hasSingleArmPreviewTarget &&
        typeof vis.updateHumanoidSingleArmPoseTarget === 'function' &&
        Array.isArray(bodyRelativeTargets?.arm1) &&
        bodyRelativeTargets.arm1.length === 8 &&
        Array.isArray(bodyRelativeTargets?.arm2) &&
        bodyRelativeTargets.arm2.length === 8;
    if (!hasLocalTarget && !hasBodyRelativeTargets && !hasSingleArmBodyRelativeTarget) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart.length) return fallbackQ.slice();
        return qStart.slice();
    }

    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
    const passes = Math.max(1, Math.floor(Number(params.baseMoveTrackingPasses) || 1));
    const includeWaist = typeof includeWaistOverride === 'boolean'
        ? includeWaistOverride
        : !!params.armIKIncludeWaist;

    const solveTrackingGoal = () => {
        // 在不改变最终真实状态的前提下，临时求一帧在线跟踪目标。
        restoreHumanoidQ(vis, qStart);
        const qRef = window.humanoidQ || appState.humanoidQ;
        let okAny = false;

        if (hasSingleArmPreviewTarget && (hasLocalTarget || hasSingleArmBodyRelativeTarget)) {
            for (let i = 0; i < passes; i++) {
                const localTargetNow = hasSingleArmBodyRelativeTarget
                    ? composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, singleArmBodyRelativeTarget)
                    : localTargetArray;
                if (!Array.isArray(localTargetNow) || localTargetNow.length !== 8) break;
                const ok = vis.updateHumanoidSingleArmPoseTarget(
                    localTargetNow,
                    qRef,
                    previewArmIndex,
                    false,
                    false
                );
                if (!ok) break;
                okAny = true;
            }
        } else if (hasBodyRelativeTargets && DQ) {
            for (let i = 0; i < passes; i++) {
                const arm1TargetLocal = composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, bodyRelativeTargets.arm1);
                const arm2TargetLocal = composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, bodyRelativeTargets.arm2);
                if (!arm1TargetLocal || !arm2TargetLocal) break;

                const ok1 = vis.updateHumanoidSingleArmPoseTarget(
                    arm1TargetLocal,
                    qRef,
                    1,
                    false,
                    false
                );
                const ok2 = vis.updateHumanoidSingleArmPoseTarget(
                    arm2TargetLocal,
                    qRef,
                    2,
                    false,
                    false
                );
                if (!ok1 && !ok2) break;
                okAny = true;
            }
        } else {
            for (let i = 0; i < passes; i++) {
                const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                    localTargetArray,
                    qRef,
                    enableLimitAvoidance,
                    includeWaist,
                    !!params.armIKKeepBodyZVertical,
                    relativePoseConstraint
                );
                if (!ok) break;
                okAny = true;
            }
        }

        const qSolved = (window.humanoidQ || appState.humanoidQ)?.slice();
        restoreHumanoidQ(vis, qGlobalBefore);
        return { okAny, qSolved };
    };

    const { okAny, qSolved } = withTemporaryHumanoidBasePose(
        vis,
        previewBasePose,
        solveTrackingGoal
    ) || { okAny: false, qSolved: null };

    if (!okAny || !Array.isArray(qSolved) || qSolved.length !== qStart.length) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart.length) return fallbackQ.slice();
        return qStart.slice();
    }
    return qSolved;
}

/**
 * 预览底盘 point-to-point 轨迹（近似真实执行控制律）
 * 这样即便起终 yaw 相同，预览也能显示运动过程中的“先转向再走”的姿态变化。
 */
function simulateBaseTrajectoryForPreview(startPose, targetPose, settleCycles = 10) {
    const pRaw = appState.humanoidBaseControlParams || {};
    const p = {
        wheelRadius: Number(pRaw.wheelRadius) || 0.10,
        wheelDistance: Number(pRaw.wheelDistance) || 0.425,
        kRho: Number(pRaw.kRho) || 1.2,
        kAlpha: Number(pRaw.kAlpha) || 3.2,
        kBeta: Number(pRaw.kBeta) || -0.9,
        kYawFinal: Number(pRaw.kYawFinal) || 3.0,
        maxLinear: Number(pRaw.maxLinear) || 0.8,
        maxAngular: Number(pRaw.maxAngular) || 2.0,
        maxWheelSpeed: Number(pRaw.maxWheelSpeed) || 18.0,
        tolPos: Number(pRaw.tolPos) || 0.03,
        tolYaw: Number(pRaw.tolYaw) || 0.05,
    };

    const pose = cloneBasePose(startPose);
    const target = cloneBasePose(targetPose);

    const dt = 0.02;      // 与真实控制循环一致（20ms）
    const maxSteps = 600; // 最长约 12s
    const settleNeed = Math.max(1, Math.floor(Number(settleCycles) || 10));

    let reverseDrive = false;
    let convergedCount = 0;

    const trajectory = [cloneBasePose(pose)];

    for (let step = 0; step < maxSteps; step++) {
        const dx = (target.x || 0) - (pose.x || 0);
        const dy = (target.y || 0) - (pose.y || 0);
        const theta = pose.yaw || 0;
        const thetaTarget = target.yaw || 0;
        const posErr = Math.hypot(dx, dy);
        const eYaw = wrapAngle(thetaTarget - theta);

        let v = 0;
        let omega = 0;
        let converged = false;

        if (posErr < p.tolPos) {
            reverseDrive = false;
            v = 0;
            if (Math.abs(eYaw) < p.tolYaw) {
                omega = 0;
                converged = true;
            } else {
                omega = clampValue(p.kYawFinal * eYaw, -p.maxAngular, p.maxAngular);
            }
        } else {
            const headingToTarget = Math.atan2(dy, dx);
            const alphaForward = wrapAngle(headingToTarget - theta);
            const alphaReverse = wrapAngle(alphaForward - Math.PI);

            const reverseEnter = (100 * Math.PI) / 180;
            const reverseExit = (70 * Math.PI) / 180;

            if (reverseDrive) {
                if (Math.abs(alphaReverse) >= reverseExit) reverseDrive = false;
            } else if (Math.abs(alphaForward) > reverseEnter) {
                reverseDrive = true;
            }

            const alpha = reverseDrive ? alphaReverse : alphaForward;
            const rhoSigned = reverseDrive ? -posErr : posErr;
            const beta = wrapAngle(thetaTarget - theta - alpha);
            const yawBlendDist = Math.max(0.15, 5 * p.tolPos);
            const yawBlend = clampValue((yawBlendDist - posErr) / yawBlendDist, 0, 1);

            v = clampValue(p.kRho * rhoSigned, -p.maxLinear, p.maxLinear);
            omega = clampValue(p.kAlpha * alpha + yawBlend * p.kBeta * beta, -p.maxAngular, p.maxAngular);
        }

        // 与真实控制一致：先转轮速并限幅，再积分
        let wr = (v / p.wheelRadius) + (p.wheelDistance / (2 * p.wheelRadius)) * omega;
        let wl = (v / p.wheelRadius) - (p.wheelDistance / (2 * p.wheelRadius)) * omega;
        wr = clampValue(wr, -p.maxWheelSpeed, p.maxWheelSpeed);
        wl = clampValue(wl, -p.maxWheelSpeed, p.maxWheelSpeed);

        const vInt = (p.wheelRadius / 2) * (wr + wl);
        const omegaInt = (p.wheelRadius / p.wheelDistance) * (wr - wl);

        pose.x += dt * vInt * Math.cos(theta);
        pose.y += dt * vInt * Math.sin(theta);
        pose.yaw = wrapAngle(theta + dt * omegaInt);
        trajectory.push(cloneBasePose(pose));

        if (converged) convergedCount += 1;
        else convergedCount = 0;
        if (convergedCount >= settleNeed) break;
    }

    return trajectory;
}

function simulateBaseTrajectoryStagesForPreview(baseStartPose, baseStages, settleCycles = 10) {
    const start = cloneBasePose(baseStartPose || { x: 0, y: 0, yaw: 0 });
    const stages = Array.isArray(baseStages) ? baseStages : [];
    if (stages.length === 0) return [start];

    const trajectory = [start];
    let stageStart = start;

    for (let i = 0; i < stages.length; i++) {
        const stagePose = cloneBasePose(stages[i]?.targetPose || stages[i]);
        const seg = simulateBaseTrajectoryForPreview(stageStart, stagePose, settleCycles);
        if (Array.isArray(seg) && seg.length > 0) {
            for (let j = 1; j < seg.length; j++) {
                trajectory.push(seg[j]);
            }
            stageStart = cloneBasePose(seg[seg.length - 1]);
        } else {
            trajectory.push(stagePose);
            stageStart = stagePose;
        }
    }

    return trajectory;
}

/**
 * 恢复/同步 humanoid 关节（避免规划阶段把真实姿态留在中间值）
 */
function restoreHumanoidQ(vis, qRestore) {
    if (!Array.isArray(qRestore) || qRestore.length === 0) return;

    const qRef = window.humanoidQ || appState.humanoidQ;
    if (Array.isArray(qRef)) {
        for (let i = 0; i < qRef.length && i < qRestore.length; i++) {
            qRef[i] = qRestore[i];
        }
    }

    if (Array.isArray(appState.humanoidQ)) {
        for (let i = 0; i < appState.humanoidQ.length && i < qRestore.length; i++) {
            appState.humanoidQ[i] = qRestore[i];
        }
    }

    if (Array.isArray(window.humanoidQ)) {
        for (let i = 0; i < window.humanoidQ.length && i < qRestore.length; i++) {
            window.humanoidQ[i] = qRestore[i];
        }
    }

    vis.setHumanoidJoints?.(qRestore);
    callbacks.updateRobotPose?.();
    callbacks.refreshJointControlDisplay?.();
}

/**
 * 仅腰部预对齐求解（只允许腰部关节变化）
 * - 输入目标为世界坐标系下的 body_link 位姿
 * - 内部会转为底盘局部目标并调用 visualizer 的腰部专用 IK
 */
function solveWaistTargetForBasePose(
    DQ,
    vis,
    worldTargetDQArray,
    basePose,
    params,
    seedQ = null
) {
    if (!vis || !worldTargetDQArray) {
        return { ok: false, message: '可视化器或目标丢失' };
    }
    if (typeof vis.updateHumanoidWaistPoseTarget !== 'function') {
        return { ok: false, message: '当前模型不支持腰部专用求解' };
    }

    const localTargetArray = computeLocalTargetForBasePose(DQ, vis, worldTargetDQArray, basePose);
    if (!Array.isArray(localTargetArray) || localTargetArray.length !== 8) {
        return { ok: false, message: '无法计算腰部局部目标' };
    }

    const qGlobalBefore = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qGlobalBefore) || qGlobalBefore.length === 0) {
        return { ok: false, message: '人形关节未初始化' };
    }

    const initialQ =
        Array.isArray(seedQ) && seedQ.length === qGlobalBefore.length
            ? seedQ.slice()
            : qGlobalBefore.slice();

    let bestQ = null;
    let bestErr = null;

    try {
        const maxPasses = Math.max(1, Math.floor(Number(params?.armRefineMaxPasses) || 1));
        const posTolM = Math.max(0, (Number(params?.finalPosTolMm) || 0.5) / 1000);
        const rotTolRad = Math.max(0, (Number(params?.finalRotTolDeg) || 0.2) * Math.PI / 180);

        restoreHumanoidQ(vis, initialQ);
        let workQ = initialQ.slice();

        for (let pass = 0; pass < maxPasses; pass++) {
            const ok = vis.updateHumanoidWaistPoseTarget(localTargetArray, workQ);
            if (!ok) break;

            const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
            if (!Array.isArray(qNow) || qNow.length === 0) break;

            const errNow = evaluateHumanoidWaistWorldError(DQ, vis, qNow, basePose, worldTargetDQArray);
            if (!errNow) break;

            if (
                !bestErr ||
                errNow.posErr3D < bestErr.posErr3D - 1e-9 ||
                (Math.abs(errNow.posErr3D - bestErr.posErr3D) <= 1e-9 && errNow.rotErrRad < bestErr.rotErrRad)
            ) {
                bestErr = errNow;
                bestQ = qNow.slice();
            }

            if (errNow.posErr3D <= posTolM && errNow.rotErrRad <= rotTolRad) {
                break;
            }

            workQ = qNow;
        }

        restoreHumanoidQ(vis, qGlobalBefore);
        if (!bestQ) {
            return { ok: false, message: '腰部 IK 未收敛' };
        }

        return {
            ok: true,
            qStart: qGlobalBefore,
            qTarget: bestQ,
            solvedError: bestErr,
        };
    } catch (e) {
        console.error('规划运动 - 腰部 IK 规划失败:', e);
        restoreHumanoidQ(vis, qGlobalBefore);
        return { ok: false, message: `腰部 IK 失败: ${e.message}` };
    }
}

/**
 * 仅用于“规划阶段”的 IK 求解：
 * - 求出目标关节 qTarget
 * - 规划结束后恢复当前真实姿态
 */
function solveArmTargetForBasePose(
    DQ,
    vis,
    worldTargetDQArray,
    basePose,
    params,
    relativePoseConstraint = null,
    seedQ = null
) {
    if (!vis || !worldTargetDQArray) {
        return { ok: false, message: '可视化器或目标丢失' };
    }

    const baseDQ = vis._makeHumanoidBaseDQ?.(basePose);
    if (!baseDQ) {
        return { ok: false, message: '无法构建底盘 DQ' };
    }

    const baseDQInv = DQ.inverse(baseDQ);
    const normalizedWorldTarget = normalizeDualQuaternionArray(worldTargetDQArray);
    if (!normalizedWorldTarget) {
        return { ok: false, message: '目标 DQ 无法单位化' };
    }
    const worldTargetDQ = DQ.createFromArray(normalizedWorldTarget);
    const localTargetDQ = DQ.multiply(baseDQInv, worldTargetDQ);
    const localTargetArray = DQ.toArray(localTargetDQ);

    const qRef = window.humanoidQ || appState.humanoidQ;
    if (!qRef || qRef.length === 0) {
        return { ok: false, message: '人形关节未初始化' };
    }

    const qGlobalBefore = qRef.slice();
    const initialQ =
        Array.isArray(seedQ) && seedQ.length === qGlobalBefore.length
            ? seedQ.slice()
            : qGlobalBefore.slice();
    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
    const fixedRelativeConstraint =
        Array.isArray(relativePoseConstraint) && relativePoseConstraint.length === 8
            ? relativePoseConstraint.slice()
            : getHumanoidRelativePoseConstraint(vis, initialQ);

    let bestQ = null;
    let bestErr = null;

    try {
        const maxPasses = Math.max(1, Math.floor(Number(params.armRefineMaxPasses) || 1));
        const posTolM = Math.max(0, (Number(params.finalPosTolMm) || 0.5) / 1000);
        const rotTolRad = Math.max(0, (Number(params.finalRotTolDeg) || 0.2) * Math.PI / 180);

        restoreHumanoidQ(vis, initialQ);
        let workQ = initialQ.slice();

        for (let pass = 0; pass < maxPasses; pass++) {
            const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                localTargetArray,
                workQ,
                enableLimitAvoidance,
                params.armIKIncludeWaist,
                !!params.armIKKeepBodyZVertical,
                fixedRelativeConstraint
            );
            if (!ok) break;

            const qNow = (appState.humanoidQ || window.humanoidQ)?.slice();
            if (!qNow || qNow.length === 0) break;

            const errNow = evaluateHumanoidWorldError(DQ, vis, qNow, basePose, worldTargetDQArray);
            if (!errNow) break;

            if (
                !bestErr ||
                errNow.posErr3D < bestErr.posErr3D - 1e-9 ||
                (Math.abs(errNow.posErr3D - bestErr.posErr3D) <= 1e-9 && errNow.rotErrRad < bestErr.rotErrRad)
            ) {
                bestErr = errNow;
                bestQ = qNow.slice();
            }

            if (errNow.posErr3D <= posTolM && errNow.rotErrRad <= rotTolRad) {
                break;
            }

            workQ = qNow;
        }

        if (!bestQ) {
            restoreHumanoidQ(vis, qGlobalBefore);
            return { ok: false, message: '手臂 IK 未收敛' };
        }

        restoreHumanoidQ(vis, qGlobalBefore);
        return {
            ok: true,
            qStart: qGlobalBefore,
            qTarget: bestQ,
            solvedError: bestErr,
        };
    } catch (e) {
        console.error('规划运动 - 手臂 IK 规划失败:', e);
        restoreHumanoidQ(vis, qGlobalBefore);
        return { ok: false, message: `手臂 IK 失败: ${e.message}` };
    }
}

function getHumanoidArmLocalPose(vis, q, armIndex) {
    if (!vis || !Array.isArray(q) || q.length === 0) return null;
    if (armIndex === 1) return vis._getHumanoidPose1?.(q) || null;
    if (armIndex === 2) return vis._getHumanoidPose2?.(q) || null;
    return null;
}

function getHumanoidBodyLocalPose(vis, q) {
    if (!vis || !Array.isArray(q) || q.length === 0) return null;
    const bodyPose = vis?.humanoidModel?.fkmEffector?.(q, 0);
    return normalizeDualQuaternionArray(bodyPose) || bodyPose || null;
}

function computeArmPoseRelativeToBody(DQ, vis, q, armIndex) {
    if (!DQ || !vis || !Array.isArray(q) || q.length === 0) return null;
    const bodyLocal = getHumanoidBodyLocalPose(vis, q);
    const armLocal = getHumanoidArmLocalPose(vis, q, armIndex);
    const bodyN = normalizeDualQuaternionArray(bodyLocal);
    const armN = normalizeDualQuaternionArray(armLocal);
    if (!bodyN || !armN) return null;
    try {
        const dqBody = DQ.createFromArray(bodyN);
        const dqArm = DQ.createFromArray(armN);
        const rel = DQ.multiply(DQ.inverse(dqBody), dqArm);
        return normalizeDualQuaternionArray(DQ.toArray(rel));
    } catch {
        return null;
    }
}

function composeArmLocalPoseFromBodyRelative(DQ, vis, q, armBodyRelativeDQArray) {
    if (!DQ || !vis || !Array.isArray(q) || q.length === 0) return null;
    const bodyLocal = getHumanoidBodyLocalPose(vis, q);
    const bodyN = normalizeDualQuaternionArray(bodyLocal);
    const relN = normalizeDualQuaternionArray(armBodyRelativeDQArray);
    if (!bodyN || !relN) return null;
    try {
        const dqBody = DQ.createFromArray(bodyN);
        const dqRel = DQ.createFromArray(relN);
        const targetLocal = DQ.multiply(dqBody, dqRel);
        return normalizeDualQuaternionArray(DQ.toArray(targetLocal));
    } catch {
        return null;
    }
}

function evaluateArmBodyRelativeError(DQ, vis, q, armIndex, armBodyRelativeTarget) {
    const relNow = computeArmPoseRelativeToBody(DQ, vis, q, armIndex);
    if (!relNow || !Array.isArray(armBodyRelativeTarget) || armBodyRelativeTarget.length !== 8) return null;
    return computePoseError(DQ, relNow, armBodyRelativeTarget);
}

function getHumanoidArmWorldPose(DQ, vis, q, basePose, armIndex) {
    const localPose = getHumanoidArmLocalPose(vis, q, armIndex);
    if (!localPose) return null;
    return vis?._composeHumanoidWorldPose?.(basePose, localPose) || localPose;
}

function evaluateHumanoidArmWorldError(DQ, vis, q, basePose, worldTargetDQArray, armIndex) {
    const worldPose = getHumanoidArmWorldPose(DQ, vis, q, basePose, armIndex);
    if (!worldPose) return null;
    return computePoseError(DQ, worldPose, worldTargetDQArray);
}

function solveSingleArmTargetForBasePose(
    DQ,
    vis,
    worldTargetDQArray,
    basePose,
    params,
    armIndex,
    seedQ = null
) {
    if (!vis || !worldTargetDQArray) {
        return { ok: false, message: '可视化器或目标丢失' };
    }

    if (typeof vis.updateHumanoidSingleArmPoseTarget !== 'function') {
        return { ok: false, message: '当前模型不支持单臂目标求解' };
    }

    const localTargetArray = computeLocalTargetForBasePose(DQ, vis, worldTargetDQArray, basePose);
    if (!localTargetArray) {
        return { ok: false, message: '无法计算单臂局部目标' };
    }

    const qGlobalBefore = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qGlobalBefore) || qGlobalBefore.length === 0) {
        return { ok: false, message: '人形关节未初始化' };
    }

    const initialQ =
        Array.isArray(seedQ) && seedQ.length === qGlobalBefore.length
            ? seedQ.slice()
            : qGlobalBefore.slice();

    let bestQ = null;
    let bestErr = null;

    try {
        const maxPasses = Math.max(1, Math.floor(Number(params.armRefineMaxPasses) || 1));
        const posTolM = Math.max(0, (Number(params.finalPosTolMm) || 0.5) / 1000);
        const rotTolRad = Math.max(0, (Number(params.finalRotTolDeg) || 0.2) * Math.PI / 180);

        restoreHumanoidQ(vis, initialQ);
        let workQ = initialQ.slice();

        for (let pass = 0; pass < maxPasses; pass++) {
            const ok = vis.updateHumanoidSingleArmPoseTarget(
                localTargetArray,
                workQ,
                armIndex,
                params.singleArmIKIncludeWaist !== false,
                params.singleArmIKKeepBodyZVertical !== false
            );
            if (!ok) break;

            const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
            if (!qNow || qNow.length === 0) break;

            const errNow = evaluateHumanoidArmWorldError(DQ, vis, qNow, basePose, worldTargetDQArray, armIndex);
            if (!errNow) break;

            if (
                !bestErr ||
                errNow.posErr3D < bestErr.posErr3D - 1e-9 ||
                (Math.abs(errNow.posErr3D - bestErr.posErr3D) <= 1e-9 && errNow.rotErrRad < bestErr.rotErrRad)
            ) {
                bestErr = errNow;
                bestQ = qNow.slice();
            }

            if (errNow.posErr3D <= posTolM && errNow.rotErrRad <= rotTolRad) {
                break;
            }

            workQ = qNow;
        }

        restoreHumanoidQ(vis, qGlobalBefore);

        if (!bestQ) {
            return { ok: false, message: `单臂 IK 未收敛（arm=${armIndex}）` };
        }

        return {
            ok: true,
            // qStart 必须是“求解前真实状态”，不能是 seed（否则会把起点提前到预对齐后）
            qStart: qGlobalBefore,
            qTarget: bestQ,
            solvedError: bestErr,
        };
    } catch (e) {
        console.error('规划运动 - 单臂 IK 规划失败:', e);
        restoreHumanoidQ(vis, qGlobalBefore);
        return { ok: false, message: `单臂 IK 失败: ${e.message}` };
    }
}

function buildDualArmSequentialPlaceTaskPlan(DQ, vis, leftTargetDQArray, rightTargetDQArray, params, forcedBaseYaw = 0) {
    const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qNow) || qNow.length === 0) {
        updateStatus(STATE.ERROR, '人形关节未初始化');
        return null;
    }

    const baseStartPose = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });

    const leftPlan = buildSingleArmMotionPlan(
        DQ,
        vis,
        leftTargetDQArray,
        params,
        1,
        '左臂',
        'A',
        forcedBaseYaw
    );
    if (!leftPlan) {
        updateStatus(STATE.ERROR, '左臂放置规划失败');
        return null;
    }

    const qRestore = qNow.slice();
    const baseRestore = cloneBasePose(baseStartPose);
    applyHumanoidBasePoseToState(vis, leftPlan.baseTargetPose);
    restoreHumanoidQ(vis, leftPlan.qTarget);
    const rightPlan = buildSingleArmMotionPlan(
        DQ,
        vis,
        rightTargetDQArray,
        params,
        2,
        '右臂',
        'B',
        forcedBaseYaw
    );
    restoreHumanoidQ(vis, qRestore);
    applyHumanoidBasePoseToState(vis, baseRestore);
    if (!rightPlan) {
        updateStatus(STATE.ERROR, '右臂放置规划失败');
        return null;
    }

    const plan = {
        type: TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE,
        params: { ...params },
        baseTargetYaw: Number.isFinite(forcedBaseYaw) ? wrapAngle(forcedBaseYaw) : 0,
        baseStartPose,
        qStart: qNow.slice(),
        steps: [
            {
                ...leftPlan,
                stepIndex: 0,
            },
            {
                ...rightPlan,
                stepIndex: 1,
            },
        ],
        createdAt: Date.now(),
    };

    console.log(
        `顺序放置任务规划完成: ` +
        `baseYaw=${(plan.baseTargetYaw * 180 / Math.PI).toFixed(1)}deg, ` +
        `Step1(左臂->A, 底盘=${leftPlan.needBaseMove ? '是' : '否'}) jointCost=${computeJointMotionCost(leftPlan.qStart, leftPlan.qTarget).toFixed(3)}, ` +
        `Step2(右臂->B, 底盘=${rightPlan.needBaseMove ? '是' : '否'}) jointCost=${computeJointMotionCost(rightPlan.qStart, rightPlan.qTarget).toFixed(3)}`
    );

    return plan;
}

function executePreparedPlan() {
    if (!appState.humanoidMode || !appState.visualizer) {
        updateStatus(STATE.ERROR, '仅在人形模式下可用');
        return false;
    }

    if (isPlannedMotionRunning()) {
        updateStatus(STATE.ERROR, '当前有规划正在进行中，请先取消');
        return false;
    }

    if (!motionPreparedPlan && !motionPreparedTaskPlan) {
        updateStatus(STATE.ERROR, '请先执行“规划预览”或“规划任务”');
        return false;
    }

    const prepared = motionPreparedTaskPlan || motionPreparedPlan;
    if (!isPreparedPlanStillValid(prepared)) {
        motionPreparedPlan = null;
        motionPreparedTaskPlan = null;
        clearPreviewVisualization();
        updateStatus(STATE.ERROR, '当前姿态已变化，请先重新规划预览');
        return false;
    }

    clearPreviewVisualization();
    stopAllRuntimeTimers();

    if (motionPreparedTaskPlan) {
        motionActiveTaskPlan = motionPreparedTaskPlan;
        motionPreparedTaskPlan = null;
        motionActivePlan = null;
        motionPreparedPlan = null;
        motionTarget = null;
        motionParams = resolvePlannedMotionParams(motionActiveTaskPlan.params || {});
        startDualArmSequentialPlaceTaskExecution(motionActiveTaskPlan);
        return true;
    }

    motionActivePlan = motionPreparedPlan;
    motionPreparedPlan = null;
    motionActiveTaskPlan = null;
    motionTarget = [...motionActivePlan.targetDQ];
    motionParams = resolvePlannedMotionParams(motionActivePlan.params || {});

    startPlanExecutionWithOptionalWaistPreAlign(motionActivePlan);

    return true;
}

function isPreparedPlanStillValid(plan) {
    if (!plan) return false;

    const qNow = appState.humanoidQ || window.humanoidQ;
    if (!Array.isArray(qNow) || qNow.length !== plan.qStart.length) return false;

    const baseNow = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });
    const baseThen = plan.baseStartPose;

    const basePosDiff = Math.hypot(baseNow.x - baseThen.x, baseNow.y - baseThen.y);
    const baseYawDiff = Math.abs(wrapAngle(baseNow.yaw - baseThen.yaw));

    let maxJointDiff = 0;
    for (let i = 0; i < qNow.length; i++) {
        const d = Math.abs(wrapAngle((qNow[i] || 0) - (plan.qStart[i] || 0)));
        if (d > maxJointDiff) maxJointDiff = d;
    }

    // 宽松阈值：允许小抖动，但避免“规划与当前状态明显不一致”
    return basePosDiff <= 0.02 && baseYawDiff <= (5 * Math.PI / 180) && maxJointDiff <= 0.05;
}

function rebuildStepPlanFromCurrentState(taskPlan, stepPlan) {
    if (!stepPlan || stepPlan.planKind !== 'single-arm') {
        return stepPlan ? { ...stepPlan } : null;
    }

    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;
    if (!DQ || !vis) {
        return { ...stepPlan };
    }

    const params = resolvePlannedMotionParams(taskPlan?.params || stepPlan.params || motionParams || {});
    const forcedBaseYaw = Number.isFinite(taskPlan?.baseTargetYaw)
        ? wrapAngle(Number(taskPlan.baseTargetYaw))
        : (
            Number.isFinite(stepPlan?.requestedBaseYaw)
                ? wrapAngle(Number(stepPlan.requestedBaseYaw))
                : 0
        );
    const rebuilt = buildSingleArmMotionPlan(
        DQ,
        vis,
        stepPlan.targetDQ,
        params,
        stepPlan.armIndex,
        stepPlan.armLabel,
        stepPlan.pointLabel,
        forcedBaseYaw
    );
    if (!rebuilt) return null;

    return {
        ...stepPlan,
        ...rebuilt,
        stepIndex: stepPlan.stepIndex,
    };
}

function startDualArmSequentialPlaceTaskExecution(taskPlan) {
    if (!taskPlan || !Array.isArray(taskPlan.steps) || taskPlan.steps.length === 0) {
        updateStatus(STATE.ERROR, '顺序放置任务执行失败：任务无效');
        motionActiveTaskPlan = null;
        return;
    }

    const stepResults = [];
    const steps = taskPlan.steps;

    const runStep = (stepIndex) => {
        if (!motionActiveTaskPlan) return;
        if (stepIndex >= steps.length) {
            const left = stepResults.find(v => v?.armIndex === 1)?.error;
            const right = stepResults.find(v => v?.armIndex === 2)?.error;
            const leftMsg = left
                ? `${(left.posErr3D * 1000).toFixed(2)} mm / ${left.rotErrDeg.toFixed(3)} deg`
                : '未知';
            const rightMsg = right
                ? `${(right.posErr3D * 1000).toFixed(2)} mm / ${right.rotErrDeg.toFixed(3)} deg`
                : '未知';
            updateStatus(STATE.DONE, `任务完成！左臂误差 ${leftMsg}；右臂误差 ${rightMsg}`);
            motionActiveTaskPlan = null;
            motionTarget = null;
            return;
        }

        const rebuiltStepPlan = rebuildStepPlanFromCurrentState(taskPlan, steps[stepIndex]);
        if (!rebuiltStepPlan) {
            updateStatus(STATE.ERROR, `顺序放置任务执行失败：步骤 ${stepIndex + 1} 重规划失败`);
            motionActiveTaskPlan = null;
            motionActivePlan = null;
            motionTarget = null;
            return;
        }
        steps[stepIndex] = rebuiltStepPlan;
        const stepPlan = rebuiltStepPlan;
        const statusPrefix = `步骤 ${stepIndex + 1}/${steps.length}：${stepPlan.armLabel}放置到 ${stepPlan.pointLabel} 点`;
        motionActivePlan = {
            ...stepPlan,
            statusPrefix,
        };
        motionTarget = [...stepPlan.targetDQ];

        const onStepDone = (result) => {
            if (!motionActiveTaskPlan) return;
            if (!result?.ok) {
                updateStatus(STATE.ERROR, result?.message || `顺序放置任务执行失败：步骤 ${stepIndex + 1} 失败`);
                motionActiveTaskPlan = null;
                motionActivePlan = null;
                motionTarget = null;
                return;
            }
            stepResults.push({
                armIndex: stepPlan.armIndex,
                error: result.finalError || stepPlan.solvedError || null,
            });
            runStep(stepIndex + 1);
        };

        startPlanExecutionWithOptionalWaistPreAlign(motionActivePlan, onStepDone);
    };

    runStep(0);
}

function shouldRunWaistPreAlignBeforeExecution(plan) {
    if (!plan) return false;
    const preAlign = plan.waistYawPreAlign;
    return !!preAlign?.applied && Array.isArray(preAlign?.qSeed) && preAlign.qSeed.length > 0;
}

function startWaistPreAlignExecution(plan, onDone = null) {
    const done = typeof onDone === 'function' ? onDone : (() => {});
    const vis = appState.visualizer;
    const qRef = appState.humanoidQ || window.humanoidQ;
    if (!vis || !Array.isArray(qRef) || qRef.length === 0) {
        done();
        return;
    }

    const qFrom = qRef.slice();
    const qTo = buildWaistPreAlignTargetQ(plan, qFrom);
    if (!Array.isArray(qTo) || qTo.length !== qFrom.length) {
        done();
        return;
    }

    const waistCount = getWaistJointCount(plan);
    let maxWaistDelta = 0;
    for (let i = 0; i < waistCount; i++) {
        const d = Math.abs(wrapAngle((qTo[i] || 0) - (qFrom[i] || 0)));
        if (d > maxWaistDelta) maxWaistDelta = d;
    }
    if (maxWaistDelta < 1e-4) {
        done();
        return;
    }

    const prefix = plan?.statusPrefix ? `${plan.statusPrefix} | ` : '';
    updateStatus(STATE.WAIST_ALIGNING, `${prefix}腰部预对齐中…`);

    const durationMs = Math.max(120, (Number(motionParams.waistPreAlignDuration) || 0.45) * 1000);
    const startTs = performance.now();

    const tick = (now) => {
        if (motionState !== STATE.WAIST_ALIGNING) {
            motionWaistAnimationId = null;
            return;
        }

        const t = clampValue((now - startTs) / durationMs, 0, 1);
        const s = easeInOutCubic(t);

        for (let i = 0; i < qRef.length; i++) {
            qRef[i] = lerpJoint(qFrom[i] || 0, qTo[i] || 0, s);
        }
        if (Array.isArray(appState.humanoidQ)) {
            for (let i = 0; i < appState.humanoidQ.length && i < qRef.length; i++) appState.humanoidQ[i] = qRef[i];
        }
        if (Array.isArray(window.humanoidQ)) {
            for (let i = 0; i < window.humanoidQ.length && i < qRef.length; i++) window.humanoidQ[i] = qRef[i];
        }
        vis.setHumanoidJoints?.(qRef);
        callbacks.updateRobotPose?.();
        callbacks.refreshJointControlDisplay?.();

        if (t >= 1) {
            motionWaistAnimationId = null;
            done();
            return;
        }

        motionWaistAnimationId = requestAnimationFrame(tick);
    };

    motionWaistAnimationId = requestAnimationFrame(tick);
}

function startPlanExecutionWithOptionalWaistPreAlign(plan, onDone = null) {
    const runMainStages = () => {
        if (!plan) return;
        if (plan.needBaseMove) startBaseMovement(plan, onDone);
        else startArmCompensation(plan, onDone);
    };

    if (!shouldRunWaistPreAlignBeforeExecution(plan)) {
        runMainStages();
        return;
    }

    const mode = motionParams?.waistPreAlignExecutionMode === 'pre-stage' ? 'pre-stage' : 'during-base';
    if (mode === 'pre-stage') {
        startWaistPreAlignExecution(plan, runMainStages);
        return;
    }

    // 默认策略：腰部随底盘阶段渐进到预对齐目标，避免开头瞬间转动。
    runMainStages();
}

function getExecutionBaseStages(plan) {
    if (!plan) return [];
    if (Array.isArray(plan.baseStages) && plan.baseStages.length > 0) {
        return plan.baseStages.map((stage, idx) => ({
            key: stage?.key || `stage-${idx + 1}`,
            label: stage?.label || `阶段 ${idx + 1}`,
            targetPose: cloneBasePose(stage?.targetPose || stage),
        }));
    }
    if (plan.baseTargetPose) {
        return [{
            key: 'base-adjust',
            label: '底盘调整',
            targetPose: cloneBasePose(plan.baseTargetPose),
        }];
    }
    return [];
}

function getBaseStageProgressText(plan) {
    const stages = Array.isArray(plan?._baseStages) ? plan._baseStages : [];
    if (stages.length === 0) return '底盘移动';
    const idx = Math.max(0, Math.min(stages.length - 1, Number(plan?._baseStageIndex) || 0));
    if (stages.length === 1) return stages[idx]?.label || '底盘调整';
    return `阶段 ${idx + 1}/${stages.length} ${stages[idx]?.label || '底盘调整'}`;
}

function activateBaseStageTarget(plan, stageIndex) {
    if (!plan || !Array.isArray(plan._baseStages)) return null;
    if (stageIndex < 0 || stageIndex >= plan._baseStages.length) return null;

    const stage = plan._baseStages[stageIndex];
    const target = cloneBasePose(stage?.targetPose || {});
    if (!appState.humanoidBaseTargetPose || typeof appState.humanoidBaseTargetPose !== 'object') {
        appState.humanoidBaseTargetPose = { x: 0, y: 0, yaw: 0 };
    }
    appState.humanoidBaseTargetPose.x = target.x;
    appState.humanoidBaseTargetPose.y = target.y;
    appState.humanoidBaseTargetPose.yaw = target.yaw;
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidBaseReverseDrive = false;

    plan._baseStageIndex = stageIndex;
    return stage;
}

/**
 * 阶段 1：启动底盘移动
 */
function startBaseMovement(plan, onDone = null) {
    if (typeof onDone === 'function') {
        plan._onDone = onDone;
    }
    const vis = appState.visualizer;
    if (plan.planKind !== 'single-arm') {
        if (!Array.isArray(plan.syncRelativePoseConstraint) || plan.syncRelativePoseConstraint.length !== 8) {
            plan.syncRelativePoseConstraint = getHumanoidRelativePoseConstraint(
                appState.visualizer,
                plan.qSyncTarget || plan.qTarget
            );
        }
        if (!Array.isArray(plan.relativePoseConstraint) || plan.relativePoseConstraint.length !== 8) {
            plan.relativePoseConstraint = getHumanoidRelativePoseConstraint(
                appState.visualizer,
                plan.qTarget || plan.qSyncTarget
            );
        }

        const relativeMode = motionParams?.syncRelativeConstraintMode === 'planned-sync'
            ? 'planned-sync'
            : 'runtime-lock';
        plan._syncRelativeConstraintMode = relativeMode;

        const qNowForRel = (window.humanoidQ || appState.humanoidQ)?.slice();
        if (relativeMode === 'runtime-lock' && Array.isArray(qNowForRel) && qNowForRel.length > 0) {
            plan._syncRelativeConstraintRuntime = getHumanoidRelativePoseConstraint(vis, qNowForRel);
        } else {
            plan._syncRelativeConstraintRuntime = null;
        }
    } else {
        plan._syncRelativeConstraintMode = null;
        plan._syncRelativeConstraintRuntime = null;
    }

    plan._baseStages = getExecutionBaseStages(plan);
    plan._baseStageIndex = 0;
    if (plan._baseStages.length === 0) {
        startArmCompensation(plan, plan?._onDone || null);
        return;
    }
    plan.baseTargetPose = cloneBasePose(plan._baseStages[plan._baseStages.length - 1].targetPose);
    activateBaseStageTarget(plan, 0);

    // 确保底盘控制模式为 point-to-point
    appState.humanoidBaseControlMode = 'point';
    plan.baseMoveTrackingTick = 0;
    plan.baseMoveTrackingStartTs = performance.now();
    plan._syncConvergeWaitCycles = 0;
    plan._syncRelErrDeg = null;
    plan._syncRelErrMm = null;

    // 默认执行策略下：腰部在底盘阶段渐进到预对齐目标，而不是开头瞬转。
    const shouldBlendWaist =
        shouldRunWaistPreAlignBeforeExecution(plan) &&
        (motionParams?.waistPreAlignExecutionMode !== 'pre-stage');
    if (shouldBlendWaist) {
        const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
        if (Array.isArray(qNow) && qNow.length > 0) {
            plan._waistBlendTargetQ = adaptJointVectorToReference(plan.qWaistVirtualTarget, qNow);
            const waistCount = getWaistJointCount(plan);
            const tolDeg = Math.max(0.01, Number(motionParams.syncWaistConvergeTolDeg) || 0.10);
            let maxRemain = 0;
            for (let i = 0; i < waistCount; i++) {
                const d = Math.abs(wrapAngle((plan._waistBlendTargetQ[i] || 0) - (qNow[i] || 0)));
                if (d > maxRemain) maxRemain = d;
            }
            plan._waistBlendDone = (maxRemain * 180 / Math.PI) <= tolDeg;
        } else {
            plan._waistBlendTargetQ = null;
            plan._waistBlendDone = true;
        }
    } else {
        plan._waistBlendTargetQ = null;
        plan._waistBlendDone = true;
    }

    // 同步阶段目标：默认收敛到 qSyncTarget（没有则回退 qTarget）。
    if (!motionParams.baseMoveTrackingEnabled) {
        plan._syncArmTargetQ = null;
        plan._syncArmRemainingDeg = null;
        plan._syncArmDone = true;
    } else {
        const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
        if (Array.isArray(qNow) && qNow.length > 0) {
            const syncTargetCandidate = plan.qSyncTarget || plan.qTarget;
            plan._syncArmTargetQ = adaptJointVectorToReference(syncTargetCandidate, qNow);
            const waistCount = getWaistJointCount(plan);
            const remainingDeg = computeNonWaistRemainingDeg(plan._syncArmTargetQ, qNow, waistCount);
            const tolDeg = Math.max(0.01, Number(motionParams.syncArmConvergeTolDeg) || 0.35);
            plan._syncArmRemainingDeg = remainingDeg;
            plan._syncArmDone = Number.isFinite(remainingDeg) ? (remainingDeg <= tolDeg) : true;
        } else {
            plan._syncArmTargetQ = null;
            plan._syncArmRemainingDeg = null;
            plan._syncArmDone = true;
        }
    }

    // 确保底盘控制循环在运行
    if (!appState.humanoidBaseControlTimer) {
        callbacks.startHumanoidBaseControlLoop?.();
    }

    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    const prefix = plan?.statusPrefix ? `${plan.statusPrefix} | ` : '';
    const stageText = getBaseStageProgressText(plan);
    updateStatus(STATE.BASE_MOVING, `${prefix}${stageText}… 偏差 ${(Math.hypot(dx, dy) * 100).toFixed(1)} cm`);

    motionPollTimer = setInterval(() => {
        pollBaseMovement();
    }, motionParams.pollInterval);
}

function blendWaistPreAlignDuringBaseMove(plan) {
    if (!plan || !Array.isArray(plan._waistBlendTargetQ) || plan._waistBlendDone) return null;

    const vis = appState.visualizer;
    const qRef = window.humanoidQ || appState.humanoidQ;
    if (!vis || !Array.isArray(qRef) || qRef.length === 0) return null;

    const waistCount = getWaistJointCount(plan);
    if (waistCount <= 0) {
        plan._waistBlendDone = true;
        return { remainingDeg: 0 };
    }

    const qCurrent = qRef.slice();
    const qGoal = adaptJointVectorToReference(plan._waistBlendTargetQ, qCurrent);
    const blend = clampValue(Number(motionParams.waistBlendDuringBaseBlend) || 0.34, 0.01, 1.0);
    const maxStepRad = Math.max(0.1, Number(motionParams.waistBlendDuringBaseMaxStepDeg) || 2.8) * Math.PI / 180;
    const minStepRad = Math.max(0, Number(motionParams.waistBlendDuringBaseMinStepDeg) || 0.22) * Math.PI / 180;

    const qNext = qCurrent.slice();
    let maxRemain = 0;
    for (let i = 0; i < waistCount; i++) {
        const cur = qCurrent[i] || 0;
        const goal = qGoal[i] || cur;
        const d = wrapAngle(goal - cur);
        const absD = Math.abs(d);
        if (absD <= 1e-12) {
            qNext[i] = goal;
            continue;
        }
        const stepMag = clampValue(absD * blend, minStepRad, maxStepRad);
        const step = Math.sign(d) * Math.min(absD, stepMag);
        qNext[i] = cur + step;
        const remain = Math.abs(wrapAngle(goal - qNext[i]));
        if (remain > maxRemain) maxRemain = remain;
    }

    const tolDeg = Math.max(0.01, Number(motionParams.syncWaistConvergeTolDeg) || 0.10);
    if ((maxRemain * 180 / Math.PI) < tolDeg) {
        for (let i = 0; i < waistCount; i++) {
            qNext[i] = qGoal[i] || qNext[i];
        }
        plan._waistBlendDone = true;
    }
    restoreHumanoidQ(vis, qNext);
    return { remainingDeg: maxRemain * 180 / Math.PI };
}

/**
 * 轮询底盘移动状态
 */
function pollBaseMovement() {
    if (motionState !== STATE.BASE_MOVING) {
        if (motionPollTimer) {
            clearInterval(motionPollTimer);
            motionPollTimer = null;
        }
        return;
    }

    const converged = appState.humanoidBaseConvergedCount >= motionParams.baseSettleCycles;

    let trackingErr = null;
    if (motionParams.baseMoveTrackingEnabled && motionActivePlan) {
        trackingErr = trackUpperBodyDuringBaseMove(motionActivePlan);
    }
    if (motionActivePlan && !motionParams.baseMoveTrackingEnabled) {
        const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
        if (Array.isArray(qNow) && qNow.length > 0) {
            const targetQ = adaptJointVectorToReference(motionActivePlan._syncArmTargetQ, qNow);
            const remainDeg = computeNonWaistRemainingDeg(targetQ, qNow, getWaistJointCount(motionActivePlan));
            motionActivePlan._syncArmRemainingDeg = remainDeg;
            motionActivePlan._syncArmDone = true;
        } else {
            motionActivePlan._syncArmRemainingDeg = null;
            motionActivePlan._syncArmDone = true;
        }
        motionActivePlan._syncRelErrDeg = null;
        motionActivePlan._syncRelErrMm = null;
    }
    const waistBlend = motionActivePlan ? blendWaistPreAlignDuringBaseMove(motionActivePlan) : null;

    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    const posErr = Math.hypot(dx, dy);
    const prefix = motionActivePlan?.statusPrefix ? `${motionActivePlan.statusPrefix} | ` : '';
    const stageText = getBaseStageProgressText(motionActivePlan);
    let status = `${prefix}${stageText}… 剩余 ${(posErr * 100).toFixed(1)} cm`;
    if (trackingErr) {
        status += ` | 跟踪误差 ${(trackingErr.posErr3D * 1000).toFixed(1)} mm / ${trackingErr.rotErrDeg.toFixed(2)} deg`;
    }
    if (waistBlend && Number.isFinite(waistBlend.remainingDeg)) {
        status += ` | 腰部剩余 ${waistBlend.remainingDeg.toFixed(2)} deg`;
    }
    if (Number.isFinite(motionActivePlan?._syncArmRemainingDeg)) {
        status += ` | 双臂剩余 ${motionActivePlan._syncArmRemainingDeg.toFixed(2)} deg`;
    }
    if (Number.isFinite(motionActivePlan?._syncRelErrDeg)) {
        const relMm = Number.isFinite(motionActivePlan?._syncRelErrMm)
            ? motionActivePlan._syncRelErrMm.toFixed(1)
            : '-';
        status += ` | 相对误差 ${relMm} mm / ${motionActivePlan._syncRelErrDeg.toFixed(2)} deg`;
    }
    updateStatus(STATE.BASE_MOVING, status);

    if (converged) {
        const stages = Array.isArray(motionActivePlan?._baseStages) ? motionActivePlan._baseStages : [];
        const stageIndex = Math.max(0, Math.floor(Number(motionActivePlan?._baseStageIndex) || 0));
        if (stages.length > 0 && stageIndex < stages.length - 1) {
            activateBaseStageTarget(motionActivePlan, stageIndex + 1);
            motionActivePlan.baseMoveTrackingTick = 0;
            return;
        }

        clearInterval(motionPollTimer);
        motionPollTimer = null;
        console.log('规划运动：底盘到位，直接进入补偿阶段');
        startArmCompensation(motionActivePlan, motionActivePlan?._onDone || null);
    }
}

/**
 * 底盘移动中在线跟踪：
 * - 优先按“相对 body_link 的左右臂目标”跟踪（来自虚拟状态3）
 * - 只控制非腰部关节（腰部由虚拟状态1独立收敛）
 * - 若 body_link 相对目标不可用，则回退到双臂绝对 + 臂间相对约束
 * - 底盘停下后再进入世界坐标终修（startArmCompensation 内）
 */
function trackUpperBodyDuringBaseMove(plan) {
    const vis = appState.visualizer;
    const DQ = getDQModule()?.DQWrapper;
    if (!vis || !DQ || !plan) return null;
    const isSingleArmPlan = plan.planKind === 'single-arm';
    const singleArmIndex = Number(plan.armIndex);
    const hasSingleArmIndex = singleArmIndex === 1 || singleArmIndex === 2;

    const qRef = window.humanoidQ || appState.humanoidQ;
    if (!qRef || qRef.length === 0) return null;
    const waistCount = getWaistJointCount(plan);
    const relTolDeg = Math.max(0.01, Number(motionParams.syncRelativeTolDeg) || 2.00);
    const syncTargetQ = Array.isArray(plan._syncArmTargetQ)
        ? adaptJointVectorToReference(plan._syncArmTargetQ, qRef)
        : null;
    const fixedConstraint = Array.isArray(plan._syncRelativeConstraintRuntime) && plan._syncRelativeConstraintRuntime.length === 8
        ? plan._syncRelativeConstraintRuntime
        : (
            Array.isArray(plan.syncRelativePoseConstraint) && plan.syncRelativePoseConstraint.length === 8
                ? plan.syncRelativePoseConstraint
                : plan.relativePoseConstraint
        );
    const bodyRelTargets = plan.syncArmBodyRelativeTargets || {};
    const singleArmBodyRelativeTarget =
        hasSingleArmIndex &&
        Array.isArray(bodyRelTargets?.[`arm${singleArmIndex}`]) &&
        bodyRelTargets[`arm${singleArmIndex}`].length === 8
            ? bodyRelTargets[`arm${singleArmIndex}`]
            : null;
    const hasBodyRelativeTargets =
        !isSingleArmPlan &&
        typeof vis.updateHumanoidSingleArmPoseTarget === 'function' &&
        Array.isArray(bodyRelTargets.arm1) &&
        bodyRelTargets.arm1.length === 8 &&
        Array.isArray(bodyRelTargets.arm2) &&
        bodyRelTargets.arm2.length === 8;

    const updateSyncStatus = (qCurrent) => {
        if (!Array.isArray(syncTargetQ) || syncTargetQ.length === 0 || !Array.isArray(qCurrent)) {
            plan._syncArmRemainingDeg = null;
            plan._syncArmDone = true;
            plan._syncRelErrDeg = null;
            plan._syncRelErrMm = null;
            return;
        }
        const remainingDeg = computeNonWaistRemainingDeg(syncTargetQ, qCurrent, waistCount);
        plan._syncArmRemainingDeg = remainingDeg;
        // 仅用于显示“关节剩余角度”；同步完成判据以任务空间误差为主，避免冗余关节导致长时间等待。
        plan._syncArmDone = true;
        if (!Number.isFinite(remainingDeg)) {
            plan._syncArmDone = false;
        }

        if (isSingleArmPlan && Array.isArray(singleArmBodyRelativeTarget)) {
            const armErr = evaluateArmBodyRelativeError(DQ, vis, qCurrent, singleArmIndex, singleArmBodyRelativeTarget);
            if (armErr) {
                plan._syncRelErrDeg = armErr.rotErrDeg;
                plan._syncRelErrMm = armErr.posErr3D * 1000;
                if (Number.isFinite(armErr.rotErrDeg) && armErr.rotErrDeg > relTolDeg) {
                    plan._syncArmDone = false;
                }
            } else {
                plan._syncRelErrDeg = null;
                plan._syncRelErrMm = null;
            }
        } else if (hasBodyRelativeTargets) {
            const arm1Err = evaluateArmBodyRelativeError(DQ, vis, qCurrent, 1, bodyRelTargets.arm1);
            const arm2Err = evaluateArmBodyRelativeError(DQ, vis, qCurrent, 2, bodyRelTargets.arm2);
            const maxPosMm = Math.max(
                Number.isFinite(arm1Err?.posErr3D) ? arm1Err.posErr3D * 1000 : 0,
                Number.isFinite(arm2Err?.posErr3D) ? arm2Err.posErr3D * 1000 : 0
            );
            const maxDeg = Math.max(
                Number.isFinite(arm1Err?.rotErrDeg) ? arm1Err.rotErrDeg : 0,
                Number.isFinite(arm2Err?.rotErrDeg) ? arm2Err.rotErrDeg : 0
            );
            plan._syncRelErrMm = Number.isFinite(maxPosMm) ? maxPosMm : null;
            plan._syncRelErrDeg = Number.isFinite(maxDeg) ? maxDeg : null;
            if (Number.isFinite(maxDeg) && maxDeg > relTolDeg) {
                plan._syncArmDone = false;
            }
        } else {
            const relErr = evaluateHumanoidRelativeError(DQ, vis, qCurrent, fixedConstraint);
            if (relErr) {
                plan._syncRelErrDeg = relErr.rotErrDeg;
                plan._syncRelErrMm = relErr.posErr3D * 1000;
                if (Number.isFinite(relErr.rotErrDeg) && relErr.rotErrDeg > relTolDeg) {
                    plan._syncArmDone = false;
                }
            } else {
                plan._syncRelErrDeg = null;
                plan._syncRelErrMm = null;
            }
        }
    };
    updateSyncStatus(qRef.slice());

    const worldTarget = motionTarget || plan.targetDQ;
    if (!Array.isArray(worldTarget) || worldTarget.length !== 8) return null;

    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
    const passes = Math.max(1, Math.floor(Number(motionParams.baseMoveTrackingPasses) || 1));
    const qBefore = qRef.slice();
    let okAny = false;

    const trackingHoldMs = Math.max(0, Number(motionParams.baseMoveTrackingStartupHoldMs) || 0);
    if (trackingHoldMs > 0) {
        const startTs = Number(plan.baseMoveTrackingStartTs) || performance.now();
        const elapsedMs = performance.now() - startTs;
        if (elapsedMs < trackingHoldMs) {
            plan.baseMoveTrackingTick = Math.max(0, Math.floor(Number(plan.baseMoveTrackingTick) || 0)) + 1;
            updateSyncStatus(qBefore);
            const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
            const errHold = isSingleArmPlan
                ? evaluateHumanoidArmWorldError(DQ, vis, qBefore, basePoseNow, worldTarget, singleArmIndex)
                : evaluateHumanoidWorldError(DQ, vis, qBefore, basePoseNow, worldTarget);
            if (errHold && Number.isFinite(plan._syncArmRemainingDeg)) {
                errHold.syncArmRemainingDeg = plan._syncArmRemainingDeg;
            }
            return errHold;
        }
    }

    // 单臂任务：优先按该臂相对 body_link 目标跟踪（不含腰部）。
    if (isSingleArmPlan) {
        if (!hasSingleArmIndex || typeof vis.updateHumanoidSingleArmPoseTarget !== 'function') {
            updateSyncStatus(qBefore);
            return null;
        }
        let localTargetArray = Array.isArray(singleArmBodyRelativeTarget)
            ? composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, singleArmBodyRelativeTarget)
            : null;
        if (!Array.isArray(localTargetArray) || localTargetArray.length !== 8) {
            const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
            localTargetArray = computeLocalTargetForBasePose(DQ, vis, worldTarget, basePoseNow);
        }
        if (!Array.isArray(localTargetArray) || localTargetArray.length !== 8) {
            updateSyncStatus(qBefore);
            return null;
        }
        for (let i = 0; i < passes; i++) {
            const ok = vis.updateHumanoidSingleArmPoseTarget(
                localTargetArray,
                qRef,
                singleArmIndex,
                false,
                false
            );
            if (!ok) break;
            okAny = true;
        }
    } else if (hasBodyRelativeTargets) {
        // 优先路径：按 body_link->双臂 的相对目标进行同步跟踪（不含腰部）。
        for (let i = 0; i < passes; i++) {
            const arm1TargetLocal = composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, bodyRelTargets.arm1);
            const arm2TargetLocal = composeArmLocalPoseFromBodyRelative(DQ, vis, qRef, bodyRelTargets.arm2);
            if (!arm1TargetLocal || !arm2TargetLocal) break;

            const ok1 = vis.updateHumanoidSingleArmPoseTarget(
                arm1TargetLocal,
                qRef,
                1,
                false,
                false
            );
            const ok2 = vis.updateHumanoidSingleArmPoseTarget(
                arm2TargetLocal,
                qRef,
                2,
                false,
                false
            );

            if (!ok1 && !ok2) break;
            okAny = true;
        }
    } else {
        // 兜底路径：按双臂绝对位姿 + 臂间相对位姿约束跟踪。
        let localTargetArray = Array.isArray(plan.baseMoveLocalTarget) && plan.baseMoveLocalTarget.length === 8
            ? plan.baseMoveLocalTarget
            : null;
        if (!localTargetArray) {
            const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
            localTargetArray = computeLocalTargetForBasePose(DQ, vis, worldTarget, basePoseNow);
            if (!localTargetArray) return null;
        }

        for (let i = 0; i < passes; i++) {
            const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                localTargetArray,
                qRef,
                enableLimitAvoidance,
                false, // 同步阶段不允许 IK 改腰部
                !!motionParams.armIKKeepBodyZVertical,
                fixedConstraint
            );
            if (!ok) break;
            okAny = true;
        }
    }

    if (!okAny) {
        updateSyncStatus(qBefore);
        return null;
    }

    const qSolved = (window.humanoidQ || appState.humanoidQ || qRef)?.slice();
    if (!qSolved || qSolved.length === 0) return null;

    // 平滑逼近，避免在线跟踪出现“瞬移”。
    const blend = clampValue(Number(motionParams.baseMoveTrackingBlend) || 0.25, 0.01, 1.0);
    const maxStepRad = Math.max(0.1, Number(motionParams.baseMoveTrackingMaxJointStepDeg) || 2.0) * Math.PI / 180;
    const trackingTick = Math.max(0, Math.floor(Number(plan.baseMoveTrackingTick) || 0));
    const warmupScale = computeTrackingWarmupScale(
        trackingTick,
        motionParams.pollInterval,
        motionParams.baseMoveTrackingWarmupMs,
        motionParams.baseMoveTrackingWarmupMinScale
    );
    plan.baseMoveTrackingTick = trackingTick + 1;
    let qSmooth = stepQTowardsGoal(qBefore, qSolved, blend, maxStepRad, warmupScale);
    const jitterDeadbandRad = Math.max(0, Number(motionParams.baseMoveTrackingJitterDeadbandDeg) || 0.08) * Math.PI / 180;
    if (jitterDeadbandRad > 0 && Array.isArray(qSmooth) && Array.isArray(qBefore)) {
        for (let i = waistCount; i < qSmooth.length && i < qBefore.length; i++) {
            const step = Math.abs(wrapAngle((qSmooth[i] || 0) - (qBefore[i] || 0)));
            if (step < jitterDeadbandRad) qSmooth[i] = qBefore[i] || 0;
        }
    }
    restoreHumanoidQ(vis, qSmooth);
    updateSyncStatus(qSmooth);

    const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
    const err = isSingleArmPlan
        ? evaluateHumanoidArmWorldError(DQ, vis, qSmooth, basePoseNow, worldTarget, singleArmIndex)
        : evaluateHumanoidWorldError(DQ, vis, qSmooth, basePoseNow, worldTarget);
    if (err && Number.isFinite(plan._syncArmRemainingDeg)) {
        err.syncArmRemainingDeg = plan._syncArmRemainingDeg;
    }
    return err;
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

function buildJointCompensationWaypoints(initialQ, targetQ, waypointCount = 7) {
    const q0 = Array.isArray(initialQ) ? initialQ.slice() : [];
    const q1 = Array.isArray(targetQ) ? targetQ.slice() : [];
    if (q0.length === 0 || q1.length !== q0.length) return null;

    const points = Math.max(2, Math.floor(Number(waypointCount) || 7));
    const deltas = q0.map((v, i) => wrapAngle((q1[i] || 0) - (v || 0)));
    const waypoints = [];
    for (let k = 0; k < points; k++) {
        const alpha = points <= 1 ? 1 : (k / (points - 1));
        waypoints.push(q0.map((v, i) => (v || 0) + deltas[i] * alpha));
    }
    return waypoints;
}

function buildJointCompensationTrajectory(initialQ, targetQ, params, useToppra) {
    const q0 = Array.isArray(initialQ) ? initialQ.slice() : [];
    const q1 = Array.isArray(targetQ) ? targetQ.slice() : [];
    if (q0.length === 0 || q1.length !== q0.length) return null;

    const fallbackDurationSec = Math.max(0.05, Number(params?.armCompensationDuration) || 0.5);
    const buildFallback = (message = null) => ({
        source: 'fallback',
        waypoints: [q0, q1],
        waypointTimes: [0, fallbackDurationSec],
        durationSec: fallbackDurationSec,
        message: message ? String(message) : null,
    });
    const fallback = buildFallback();

    if (!useToppra) return fallback;

    const waypoints = buildJointCompensationWaypoints(
        q0,
        q1,
        params?.armCompensationToppraWaypoints
    );
    if (!Array.isArray(waypoints) || waypoints.length < 2) {
        return buildFallback('TOPPRA 路点构建失败');
    }

    const maxVelDeg = Math.max(1e-3, Number(params?.armCompensationToppraMaxVelDeg) || 120.0);
    const maxAccDeg = Math.max(1e-3, Number(params?.armCompensationToppraMaxAccDeg) || 240.0);
    const maxVelRad = maxVelDeg * Math.PI / 180;
    const maxAccRad = maxAccDeg * Math.PI / 180;
    const gridpoints = Number.isFinite(Number(params?.armCompensationToppraGridpoints))
        ? Math.max(0, Math.floor(Number(params.armCompensationToppraGridpoints)))
        : 0;

    const retimeResult = retimeWaypointsWithTOPPRA(waypoints, {
        maxVel: maxVelRad,
        maxAcc: maxAccRad,
        gridpoints,
    });
    if (!retimeResult?.ok || !Array.isArray(retimeResult.waypointTimes) || retimeResult.waypointTimes.length !== waypoints.length) {
        return buildFallback(retimeResult?.message || 'TOPPRA 不可用或返回异常');
    }

    const waypointTimes = [];
    let prev = 0;
    for (let i = 0; i < retimeResult.waypointTimes.length; i++) {
        const raw = Number(retimeResult.waypointTimes[i]);
        const t = Number.isFinite(raw) ? Math.max(prev, raw) : prev;
        waypointTimes.push(t);
        prev = t;
    }
    if (prev <= 1e-6) return buildFallback('TOPPRA 输出时长无效');

    return {
        source: 'toppra',
        waypoints,
        waypointTimes,
        durationSec: prev,
    };
}

function sampleJointCompensationTrajectory(trajectory, tSec) {
    if (!trajectory || !Array.isArray(trajectory.waypoints) || trajectory.waypoints.length === 0) return null;
    const waypoints = trajectory.waypoints;
    const times = Array.isArray(trajectory.waypointTimes) ? trajectory.waypointTimes : [];
    if (times.length !== waypoints.length) return waypoints[waypoints.length - 1].slice();

    const t = Math.max(0, Number(tSec) || 0);
    if (t <= times[0]) return waypoints[0].slice();
    const tEnd = times[times.length - 1];
    if (t >= tEnd) return waypoints[waypoints.length - 1].slice();

    let idx = 0;
    while (idx < times.length - 1 && times[idx + 1] < t) idx++;
    const t0 = times[idx];
    const t1 = times[idx + 1];
    const a = waypoints[idx];
    const b = waypoints[idx + 1];
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return waypoints[waypoints.length - 1].slice();

    const span = Math.max(1e-9, t1 - t0);
    const r = clampValue((t - t0) / span, 0, 1);
    return a.map((v, i) => lerpJoint(v || 0, b[i] || 0, r));
}

/**
 * 阶段 2：按规划结果做手臂补偿动画
 */
function startArmCompensation(plan, onDone = null) {
    const doneCb = typeof onDone === 'function' ? onDone : (typeof plan?._onDone === 'function' ? plan._onDone : null);
    const statusPrefix = plan?.statusPrefix ? `${plan.statusPrefix} | ` : '';
    updateStatus(STATE.ARM_COMPENSATING, `${statusPrefix}手臂补偿中…`);

    const fail = (message) => {
        motionActivePlan = null;
        motionTarget = null;
        if (doneCb) {
            doneCb({ ok: false, message });
            return;
        }
        updateStatus(STATE.ERROR, message);
    };

    const vis = appState.visualizer;
    if (!vis || !plan || !plan.qTarget || !motionTarget) {
        fail('可视化器或规划目标丢失');
        return;
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        fail('DQ 模块不可用');
        return;
    }
    const runParams = resolvePlannedMotionParams(plan?.params || motionParams || {});

    const qRef = appState.humanoidQ || window.humanoidQ;
    if (!qRef || qRef.length === 0) {
        fail('人形关节未初始化');
        return;
    }

    const initialQ = qRef.slice();
    const plannedTargetQ = adaptJointVectorToReference(plan.qTarget, initialQ);
    if (!Array.isArray(plannedTargetQ) || plannedTargetQ.length !== initialQ.length) {
        fail('规划关节维度与当前机器人不匹配，请重新规划');
        return;
    }
    let targetQ = plannedTargetQ.slice();
    let solvedError = plan.solvedError || null;

    const executionBasePose = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
    // 进入手臂补偿时锁定底盘目标，避免底盘继续微动影响末端绝对位姿。
    alignBaseTargetToPose(executionBasePose);

    // 执行前精修：根据计划类型选对应 IK。
    if (plan.planKind === 'single-arm') {
        // 单臂顺序任务：执行阶段保持“规划得到的腰部目标”不变，仅修正目标臂。
        const refineParams = {
            ...runParams,
            singleArmIKIncludeWaist: false,
        };
        const refineResult = solveSingleArmTargetForBasePose(
            DQ,
            vis,
            motionTarget,
            executionBasePose,
            refineParams,
            plan.armIndex,
            plannedTargetQ
        );
        if (refineResult.ok && Array.isArray(refineResult.qTarget) && refineResult.qTarget.length === initialQ.length) {
            targetQ = buildSingleArmSyncJointTarget(
                plannedTargetQ,
                refineResult.qTarget,
                plan.armIndex,
                { qStart: plannedTargetQ }
            );
            const waistCount = getWaistJointCount(plan);
            for (let i = 0; i < waistCount && i < targetQ.length && i < plannedTargetQ.length; i++) {
                targetQ[i] = plannedTargetQ[i];
            }
            solvedError = refineResult.solvedError || solvedError;
            plan.qTarget = targetQ.slice();
            plan.solvedError = solvedError;
            plan.baseTargetPose = executionBasePose;
        } else if (refineResult.message) {
            targetQ = plannedTargetQ.slice();
            console.warn('规划运动：单臂执行前 IK 精修失败，保持规划阶段腰部+目标臂目标。原因:', refineResult.message);
        }
    } else {
        if (!Array.isArray(plan.relativePoseConstraint) || plan.relativePoseConstraint.length !== 8) {
            plan.relativePoseConstraint = getHumanoidRelativePoseConstraint(vis, initialQ);
        }
        const refineResult = solveArmTargetForBasePose(
            DQ,
            vis,
            motionTarget,
            executionBasePose,
            runParams,
            plan.relativePoseConstraint
        );
        if (refineResult.ok && Array.isArray(refineResult.qTarget) && refineResult.qTarget.length === initialQ.length) {
            targetQ = refineResult.qTarget.slice();
            solvedError = refineResult.solvedError || solvedError;
            plan.qTarget = targetQ.slice();
            plan.solvedError = solvedError;
            plan.baseTargetPose = executionBasePose;
        } else if (refineResult.message) {
            console.warn('规划运动：执行前 IK 精修失败，回退使用规划阶段目标。原因:', refineResult.message);
        }
    }

    const useToppraForArmCompensation =
        runParams?.armCompensationUseToppra !== false &&
        (
            !runParams?.armCompensationToppraSingleArmOnly ||
            plan.planKind === 'single-arm'
        );
    const compensationTrajectory = buildJointCompensationTrajectory(
        initialQ,
        targetQ,
        runParams,
        useToppraForArmCompensation
    ) || {
        source: 'fallback',
        waypoints: [initialQ.slice(), targetQ.slice()],
        waypointTimes: [0, Math.max(0.05, Number(runParams.armCompensationDuration) || 0.5)],
        durationSec: Math.max(0.05, Number(runParams.armCompensationDuration) || 0.5),
        message: null,
    };
    const trajectoryModeText = compensationTrajectory.source === 'toppra'
        ? `TOPPRA ${Number(compensationTrajectory.durationSec || 0).toFixed(3)}s`
        : `固定时长 ${Number(compensationTrajectory.durationSec || 0).toFixed(3)}s`;
    plan._armCompensationTrajectory = {
        source: compensationTrajectory.source || 'fallback',
        durationSec: Number(compensationTrajectory.durationSec) || 0,
        message: compensationTrajectory.message || null,
        useToppraRequested: !!useToppraForArmCompensation,
    };
    updateStatus(STATE.ARM_COMPENSATING, `${statusPrefix}手臂补偿中… (${trajectoryModeText})`);
    if (useToppraForArmCompensation && compensationTrajectory.source !== 'toppra') {
        console.warn(
            `规划运动：${plan.armLabel || (plan.planKind === 'single-arm' ? '单臂' : '全身')}补偿未使用 TOPPRA，已回退固定时长。原因: ${compensationTrajectory.message || '未知'}`
        );
    }
    const durationMs = Math.max(1, Number(compensationTrajectory.durationSec) * 1000);
    const easeName = runParams.armCompensationEase || 'easeInOutCubic';
    const easeFn = easeName === 'easeInOutCubic' ? easeInOutCubic : (t) => t;
    if (compensationTrajectory.source === 'toppra' && plan.planKind === 'single-arm') {
        console.log(
            `规划运动：${plan.armLabel || '单臂'}补偿使用 TOPPRA，耗时 ${compensationTrajectory.durationSec.toFixed(3)} s`
        );
    }

    const startTime = performance.now();

    function animateArm(now) {
        if (motionState !== STATE.ARM_COMPENSATING) {
            motionArmAnimationId = null;
            return;
        }

        const elapsed = now - startTime;
        const t = Math.min(1, elapsed / durationMs);
        const elapsedSec = Math.max(0, elapsed / 1000);
        const qSample =
            compensationTrajectory.source === 'toppra'
                ? sampleJointCompensationTrajectory(compensationTrajectory, elapsedSec)
                : null;

        if (Array.isArray(qSample) && qSample.length === qRef.length) {
            for (let i = 0; i < qRef.length; i++) {
                qRef[i] = qSample[i];
            }
        } else {
            const s = easeFn(t);
            for (let i = 0; i < qRef.length; i++) {
                qRef[i] = lerpJoint(initialQ[i], targetQ[i], s);
            }
        }

        if (appState.humanoidQ) {
            for (let i = 0; i < appState.humanoidQ.length; i++) {
                appState.humanoidQ[i] = qRef[i];
            }
        }

        if (window.humanoidQ) {
            for (let i = 0; i < window.humanoidQ.length; i++) {
                window.humanoidQ[i] = qRef[i];
            }
        }

        vis.setHumanoidJoints?.(qRef);
        callbacks.updateRobotPose?.();
        callbacks.refreshJointControlDisplay?.();

        if (t >= 1) {
            motionArmAnimationId = null;
            // TOPPRA 路径在 ±pi 处可能使用等价角分支，结束时强制落到 IK 目标关节。
            for (let i = 0; i < qRef.length; i++) {
                qRef[i] = targetQ[i];
            }
            if (Array.isArray(appState.humanoidQ)) {
                for (let i = 0; i < appState.humanoidQ.length && i < qRef.length; i++) {
                    appState.humanoidQ[i] = qRef[i];
                }
            }
            if (Array.isArray(window.humanoidQ)) {
                for (let i = 0; i < window.humanoidQ.length && i < qRef.length; i++) {
                    window.humanoidQ[i] = qRef[i];
                }
            }
            vis.setHumanoidJoints?.(qRef);
            callbacks.updateRobotPose?.();
            callbacks.refreshJointControlDisplay?.();

            let finalErr = null;
            if (plan.planKind === 'single-arm') {
                finalErr = evaluateHumanoidArmWorldError(
                    DQ,
                    vis,
                    qRef.slice(),
                    executionBasePose,
                    motionTarget,
                    plan.armIndex
                );
            } else {
                const finalWorldAbs = getCurrentWorldAbsolutePose(DQ);
                if (finalWorldAbs && motionTarget) {
                    finalErr = computePoseError(DQ, finalWorldAbs, motionTarget);
                }
            }
            finalErr = finalErr || solvedError;

            motionActivePlan = null;
            motionTarget = null;

            if (doneCb) {
                doneCb({
                    ok: true,
                    finalError: finalErr || null,
                });
                return;
            }

            if (finalErr) {
                updateStatus(
                    STATE.DONE,
                    `完成！残余误差: 平移 ${(finalErr.posErr3D * 1000).toFixed(2)} mm, ` +
                    `旋转 ${finalErr.rotErrDeg.toFixed(3)} deg`
                );
            } else {
                updateStatus(STATE.DONE, '手臂补偿完成');
            }
            return;
        }

        motionArmAnimationId = requestAnimationFrame(animateArm);
    }

    motionArmAnimationId = requestAnimationFrame(animateArm);
}

function getWorldAbsolutePoseForSample(DQ, vis, q, basePose) {
    return getWorldAbsolutePoseFromBaseAndJoints(vis, q, basePose);
}

function extractPositionFromDQ(DQ, dqArray) {
    const pose = safeCreatePoseDQ(DQ, dqArray);
    const t = pose?.t || [1, 0, 0, 0];
    return new THREE.Vector3(t[1] || 0, t[2] || 0, t[3] || 0);
}

function assignUniformTimingToPreviewSamples(samples, durationSec = 1.2) {
    if (!Array.isArray(samples) || samples.length === 0) return 0;
    const total = Math.max(0.1, Number(durationSec) || 1.2);
    if (samples.length === 1) {
        samples[0].timeSec = 0;
        return 0;
    }

    const denom = Math.max(1, samples.length - 1);
    for (let i = 0; i < samples.length; i++) {
        samples[i].timeSec = (i / denom) * total;
    }
    return total;
}

function applyToppraTimingToPreviewSamples(samples, params) {
    if (!Array.isArray(samples) || samples.length < 2) return assignUniformTimingToPreviewSamples(samples, params?.previewDuration);
    if (params?.previewUseToppraTiming === false) {
        return assignUniformTimingToPreviewSamples(samples, params?.previewDuration);
    }

    const waypoints = samples.map((s) => [
        Number(s?.absPosition?.x) || 0,
        Number(s?.absPosition?.y) || 0,
        Number(s?.absPosition?.z) || 0,
    ]);

    const retimeResult = retimeWaypointsWithTOPPRA(waypoints, {
        maxVel: Number(params?.previewToppraMaxVel) || 0.45,
        maxAcc: Number(params?.previewToppraMaxAcc) || 0.90,
        gridpoints: Number.isFinite(Number(params?.previewToppraGridpoints))
            ? Math.max(0, Math.floor(Number(params.previewToppraGridpoints)))
            : 0,
    });

    if (!retimeResult?.ok || !Array.isArray(retimeResult.waypointTimes) || retimeResult.waypointTimes.length !== samples.length) {
        if (retimeResult?.message) {
            console.warn('规划预览：TOPPRA 时间参数化失败，回退固定时长。原因:', retimeResult.message);
        }
        return assignUniformTimingToPreviewSamples(samples, params?.previewDuration);
    }

    let prev = 0;
    for (let i = 0; i < samples.length; i++) {
        const rawT = Number(retimeResult.waypointTimes[i]);
        const t = Number.isFinite(rawT) ? Math.max(prev, rawT) : prev;
        samples[i].timeSec = t;
        prev = t;
    }
    if (prev <= 1e-6) {
        return assignUniformTimingToPreviewSamples(samples, params?.previewDuration);
    }
    return prev;
}

function buildPreviewSamples(DQ, vis, plan) {
    const samples = [];

    const addSample = (basePose, q, phase) => {
        const worldAbs = getWorldAbsolutePoseForSample(DQ, vis, q, basePose);
        if (!worldAbs) return;
        samples.push({
            phase,
            basePose: cloneBasePose(basePose),
            q: Array.isArray(q) ? q.slice() : [],
            absPosition: extractPositionFromDQ(DQ, worldAbs),
        });
    };

    const baseSteps = plan.needBaseMove ? Math.max(2, Math.floor(plan.params.previewBaseSamples || 16)) : 0;
    const armSteps = Math.max(4, Math.floor(plan.params.previewArmSamples || 24));
    const trackingBlend = clampValue(Number(plan.params.baseMoveTrackingBlend) || 0.25, 0.01, 1.0);
    const trackingMaxStepRad = Math.max(0.1, Number(plan.params.baseMoveTrackingMaxJointStepDeg) || 2.0) * Math.PI / 180;
    const trackingPollMs = Math.max(1, Number(plan.params.pollInterval) || 50);
    const baseSimStepMs = 20;
    let previewTrackingTick = 0;
    let qTracked = Array.isArray(plan.qStart) ? plan.qStart.slice() : [];
    const previewWaistCount = getWaistJointCount(plan);
    const previewWaistTarget = adaptJointVectorToReference(plan.qWaistVirtualTarget, qTracked);
    const baseStages = Array.isArray(plan.baseStages) && plan.baseStages.length > 0
        ? plan.baseStages
        : [{ key: 'base-adjust', label: '底盘调整', targetPose: plan.baseTargetPose }];
    const previewSingleArmIndex = plan.planKind === 'single-arm'
        ? Number(plan.armIndex)
        : null;
    const previewLocalTarget = (previewSingleArmIndex === 1 || previewSingleArmIndex === 2)
        ? null
        : (
            (Array.isArray(plan.baseMoveLocalTarget) && plan.baseMoveLocalTarget.length === 8)
                ? plan.baseMoveLocalTarget
                : computeLocalTargetForBasePose(DQ, vis, plan.targetDQ, plan.baseTargetPose)
        );
    const previewBodyRelativeTargets = plan.syncArmBodyRelativeTargets || null;
    const hasPreviewSingleArmBodyRelativeTarget =
        (previewSingleArmIndex === 1 || previewSingleArmIndex === 2) &&
        Array.isArray(previewBodyRelativeTargets?.[`arm${previewSingleArmIndex}`]) &&
        previewBodyRelativeTargets[`arm${previewSingleArmIndex}`].length === 8;
    const hasPreviewBodyRelativeTargets =
        !(previewSingleArmIndex === 1 || previewSingleArmIndex === 2) &&
        Array.isArray(previewBodyRelativeTargets?.arm1) &&
        previewBodyRelativeTargets.arm1.length === 8 &&
        Array.isArray(previewBodyRelativeTargets?.arm2) &&
        previewBodyRelativeTargets.arm2.length === 8;

    if (baseSteps > 0) {
        // 用与真实底盘控制器一致的分段轨迹做预览，确保与执行阶段一致。
        const simTrajectory = simulateBaseTrajectoryStagesForPreview(
            plan.baseStartPose,
            baseStages,
            plan.params.baseSettleCycles
        );
        const sampledTrajectory = sampleTrajectoryByCountWithIndices(simTrajectory, baseSteps + 1);
        let prevRawIdx = sampledTrajectory[0]?.idx ?? 0;
        for (let i = 0; i < sampledTrajectory.length; i++) {
            const b = sampledTrajectory[i]?.pose;
            const rawIdx = sampledTrajectory[i]?.idx ?? prevRawIdx;
            if (!b) continue;
            let previewLocalTargetNow = previewLocalTarget;
            if (previewSingleArmIndex === 1 || previewSingleArmIndex === 2) {
                previewLocalTargetNow = computeLocalTargetForBasePose(DQ, vis, plan.targetDQ, b);
            }
            // 预览中模拟“底盘运动时上肢在线跟踪”，避免与真实执行差异过大。
            if (
                i > 0 &&
                qTracked.length === plan.qTarget.length &&
                (
                    Array.isArray(previewLocalTargetNow) ||
                    hasPreviewBodyRelativeTargets ||
                    hasPreviewSingleArmBodyRelativeTarget
                )
            ) {
                const qGoalNow = computeTrackingGoalQForPreview(
                    vis,
                    previewLocalTargetNow,
                    qTracked,
                    plan.params,
                    b,
                    plan.qSyncTarget || plan.qTarget,
                    plan.syncRelativePoseConstraint || plan.relativePoseConstraint,
                    false,
                    previewBodyRelativeTargets,
                    previewSingleArmIndex
                );
                // 下采样会跳过很多底盘控制周期，这里按跳过步数重复步进，贴近真实执行中的上肢变化节奏。
                const skippedSimSteps = Math.max(1, rawIdx - prevRawIdx);
                const repeat = Math.max(1, Math.round((skippedSimSteps * baseSimStepMs) / trackingPollMs));
                for (let s = 0; s < repeat; s++) {
                    const warmupScale = computeTrackingWarmupScale(
                        previewTrackingTick,
                        trackingPollMs,
                        plan.params.baseMoveTrackingWarmupMs,
                        plan.params.baseMoveTrackingWarmupMinScale
                    );
                    qTracked = stepQTowardsGoal(qTracked, qGoalNow, trackingBlend, trackingMaxStepRad, warmupScale);
                    qTracked = blendWaistPreAlignForPreviewStep(
                        qTracked,
                        previewWaistTarget,
                        previewWaistCount,
                        plan.params
                    );
                    previewTrackingTick += 1;
                }
            }
            addSample(b, qTracked.length ? qTracked : plan.qStart, 'base');
            prevRawIdx = rawIdx;
        }
    }

    const armStartIdx = baseSteps > 0 ? 1 : 0;
    const qArmStart = qTracked.length === plan.qTarget.length ? qTracked.slice() : plan.qStart.slice();
    const previewUseToppraForArmCompensation =
        plan.params?.armCompensationUseToppra !== false &&
        (
            !plan.params?.armCompensationToppraSingleArmOnly ||
            plan.planKind === 'single-arm'
        );
    const previewArmTrajectory = buildJointCompensationTrajectory(
        qArmStart,
        plan.qTarget,
        plan.params,
        previewUseToppraForArmCompensation
    );
    if (previewArmTrajectory?.source === 'toppra') {
        const dur = Math.max(1e-6, Number(previewArmTrajectory.durationSec) || 0);
        for (let i = armStartIdx; i <= armSteps; i++) {
            const tNorm = i / armSteps;
            let qInterp = sampleJointCompensationTrajectory(previewArmTrajectory, tNorm * dur)
                || qArmStart.map((v, idx) => lerpJoint(v, plan.qTarget[idx], tNorm));
            if (i === armSteps) {
                qInterp = plan.qTarget.slice();
            }
            addSample(plan.baseTargetPose, qInterp, 'arm');
        }
    } else {
        for (let i = armStartIdx; i <= armSteps; i++) {
            const t = i / armSteps;
            const s = easeInOutCubic(t);
            const qInterp = qArmStart.map((v, idx) => lerpJoint(v, plan.qTarget[idx], s));
            addSample(plan.baseTargetPose, qInterp, 'arm');
        }
    }

    const totalTimeSec = applyToppraTimingToPreviewSamples(samples, plan.params);
    for (let i = 0; i < samples.length; i++) {
        samples[i].totalTimeSec = totalTimeSec;
    }

    return samples;
}

function disposeObjectTree(root) {
    if (!root) return;
    root.traverse((obj) => {
        if (
            obj.geometry &&
            obj.geometry.userData?.previewOwnedGeometry &&
            typeof obj.geometry.dispose === 'function'
        ) {
            obj.geometry.dispose();
        }

        if (obj.material) {
            if (Array.isArray(obj.material)) {
                obj.material.forEach((m) => {
                    if (m?.userData?.previewOwnedMaterial && typeof m.dispose === 'function') {
                        m.dispose();
                    }
                });
            } else if (
                obj.material.userData?.previewOwnedMaterial &&
                typeof obj.material.dispose === 'function'
            ) {
                obj.material.dispose();
            }
        }
    });
}

function flushDeferredPreviewDisposal() {
    motionPreviewDeferredDisposeScheduled = false;
    if (!Array.isArray(motionPreviewDeferredDisposeQueue) || motionPreviewDeferredDisposeQueue.length === 0) {
        return;
    }

    const startTs = performance.now();
    while (motionPreviewDeferredDisposeQueue.length > 0) {
        const root = motionPreviewDeferredDisposeQueue.shift();
        disposeObjectTree(root);
        if (performance.now() - startTs >= 8) break;
    }

    if (motionPreviewDeferredDisposeQueue.length > 0) {
        scheduleDeferredPreviewDisposal();
    }
}

function scheduleDeferredPreviewDisposal() {
    if (motionPreviewDeferredDisposeScheduled) return;
    motionPreviewDeferredDisposeScheduled = true;

    if (typeof requestIdleCallback === 'function') {
        requestIdleCallback(flushDeferredPreviewDisposal, { timeout: 120 });
    } else {
        setTimeout(flushDeferredPreviewDisposal, 16);
    }
}

function clearPreviewVisualization(options = {}) {
    const deferDispose = options?.deferDispose !== false;
    if (motionPreviewGroup && appState.visualizer?.scene) {
        appState.visualizer.scene.remove(motionPreviewGroup);
    }

    if (motionPreviewGroup) {
        if (deferDispose) {
            motionPreviewDeferredDisposeQueue.push(motionPreviewGroup);
            scheduleDeferredPreviewDisposal();
        } else {
            disposeObjectTree(motionPreviewGroup);
        }
    }

    motionPreviewGroup = null;
    motionPreviewItems = null;
}

function markPreviewOwnedGeometry(geometry) {
    if (geometry) {
        geometry.userData = { ...(geometry.userData || {}), previewOwnedGeometry: true };
    }
    return geometry;
}

function markPreviewOwnedMaterial(material) {
    if (material) {
        material.userData = { ...(material.userData || {}), previewOwnedMaterial: true };
    }
    return material;
}

function selectIndicesUniform(indices, count, forceLast = false) {
    if (!Array.isArray(indices) || indices.length === 0 || count <= 0) return [];
    if (indices.length <= count) {
        const all = indices.slice();
        if (forceLast && all[all.length - 1] !== indices[indices.length - 1]) {
            all[all.length - 1] = indices[indices.length - 1];
        }
        return all;
    }

    const selected = [];
    if (count === 1) {
        return [forceLast ? indices[indices.length - 1] : indices[0]];
    }

    for (let i = 0; i < count; i++) {
        const idx = Math.round((i * (indices.length - 1)) / (count - 1));
        selected.push(indices[Math.max(0, Math.min(indices.length - 1, idx))]);
    }

    // 去重后若数量不足，按顺序补齐
    const uniq = [];
    const used = new Set();
    for (const v of selected) {
        if (!used.has(v)) {
            used.add(v);
            uniq.push(v);
        }
    }
    if (uniq.length < count) {
        for (const v of indices) {
            if (uniq.length >= count) break;
            if (!used.has(v)) {
                used.add(v);
                uniq.push(v);
            }
        }
    }

    if (forceLast && uniq.length > 0) {
        uniq[uniq.length - 1] = indices[indices.length - 1];
    }
    return uniq.slice(0, count);
}

function selectPreviewFrameSamples(samples, maxCount) {
    if (!Array.isArray(samples) || samples.length === 0) return [];
    if (!Number.isFinite(maxCount) || maxCount <= 0 || samples.length <= maxCount) {
        return samples.slice();
    }

    const baseIndices = [];
    const armIndices = [];
    for (let i = 0; i < samples.length; i++) {
        if (samples[i]?.phase === 'base') baseIndices.push(i);
        else armIndices.push(i);
    }

    // 没有分阶段信息时，退回全局均匀采样
    if (baseIndices.length === 0 || armIndices.length === 0) {
        const globalSel = selectIndicesUniform(
            Array.from({ length: samples.length }, (_, i) => i),
            maxCount,
            true
        );
        return globalSel.sort((a, b) => a - b).map((idx) => samples[idx]);
    }

    // 分阶段配额：底盘阶段优先，手臂阶段只保留少量关键帧（含最后一帧）
    let armQuota = maxCount >= 8 ? 2 : 1;
    armQuota = Math.min(armQuota, armIndices.length);
    let baseQuota = Math.max(1, maxCount - armQuota);
    if (baseQuota > baseIndices.length) {
        const extra = baseQuota - baseIndices.length;
        baseQuota = baseIndices.length;
        armQuota = Math.min(armIndices.length, armQuota + extra);
    }

    const selectedBase = selectIndicesUniform(baseIndices, baseQuota, false);
    const selectedArm = selectIndicesUniform(armIndices, armQuota, true);

    const merged = Array.from(new Set([...selectedBase, ...selectedArm])).sort((a, b) => a - b);

    // 去重导致数量不足时，用全局均匀点补齐
    if (merged.length < maxCount) {
        const fallback = selectIndicesUniform(
            Array.from({ length: samples.length }, (_, i) => i),
            maxCount,
            true
        );
        for (const idx of fallback) {
            if (merged.length >= maxCount) break;
            if (!merged.includes(idx)) merged.push(idx);
        }
        merged.sort((a, b) => a - b);
    }

    return merged.slice(0, maxCount).map((idx) => samples[idx]);
}

function cloneGhostMaterial(sourceMat, phase, index, total) {
    if (!sourceMat || typeof sourceMat.clone !== 'function') return sourceMat;

    const mat = sourceMat.clone();
    const phaseTint = new THREE.Color(phase === 'base' ? 0x60a5fa : 0x34d399);
    const alpha = Math.min(0.45, 0.10 + ((index + 1) / Math.max(total, 1)) * 0.35);

    if (mat.color?.isColor) {
        mat.color = mat.color.clone().lerp(phaseTint, 0.35);
    }
    if (mat.emissive?.isColor) {
        mat.emissive = mat.emissive.clone().lerp(phaseTint, 0.15);
        if ('emissiveIntensity' in mat) mat.emissiveIntensity = Math.max(mat.emissiveIntensity || 0, 0.2);
    }

    mat.transparent = true;
    mat.opacity = alpha;
    mat.depthWrite = false;
    mat.side = THREE.DoubleSide;
    markPreviewOwnedMaterial(mat);
    return mat;
}

function setGhostRobotPoseAndJoints(ghostRobot, sample, vis) {
    if (!ghostRobot || !sample || !vis) return;

    const names = vis.humanoidJointNames || [];
    const q = sample.q || [];
    for (let i = 0; i < names.length; i++) {
        const joint = ghostRobot.joints?.[names[i]];
        if (joint?.setJointValue) {
            joint.setJointValue(q[i] ?? 0);
        }
    }

    const z = Number(vis.humanoidBaseHeight) || 0;
    const yaw = Number(sample.basePose?.yaw) || 0;
    ghostRobot.position.set(
        Number(sample.basePose?.x) || 0,
        Number(sample.basePose?.y) || 0,
        z
    );
    ghostRobot.quaternion.set(0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2));
    ghostRobot.updateMatrixWorld(true);
}

function createRobotGhostClone(vis, sample, index, total) {
    const sourceRobot = vis?.humanoidURDF;
    if (!sourceRobot?.clone) return null;

    const ghost = sourceRobot.clone(true);
    ghost.name = `PlannedMotionGhostRobot_${index}`;

    ghost.traverse((obj) => {
        if (!obj?.isMesh) return;
        if (obj.material) {
            if (Array.isArray(obj.material)) {
                obj.material = obj.material.map((m) => cloneGhostMaterial(m, sample.phase, index, total));
            } else {
                obj.material = cloneGhostMaterial(obj.material, sample.phase, index, total);
            }
        }
        obj.castShadow = false;
        obj.receiveShadow = false;
        obj.renderOrder = 900 + index;
    });

    setGhostRobotPoseAndJoints(ghost, sample, vis);
    ghost.visible = false;
    return ghost;
}

function createPreviewVisualization(samples, params = {}) {
    const vis = appState.visualizer;
    const scene = vis?.scene;
    if (!scene || !vis?.humanoidURDF || !Array.isArray(samples) || samples.length < 2) return false;

    const group = new THREE.Group();
    group.name = 'PlannedMotionGhostPreview';

    // 绝对位姿路径线（常显）
    const pathPoints = samples.map(s => s.absPosition.clone());
    const pathGeom = markPreviewOwnedGeometry(new THREE.BufferGeometry().setFromPoints(pathPoints));
    const pathMat = markPreviewOwnedMaterial(new THREE.LineBasicMaterial({
        color: 0x94a3b8,
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
    }));
    const pathLine = new THREE.Line(pathGeom, pathMat);
    pathLine.renderOrder = 880;
    group.add(pathLine);

    // 机器人残影关键帧（初始隐藏，按时间逐步显现）
    const robotGhosts = [];
    const sampledFrames = selectPreviewFrameSamples(
        samples,
        Math.max(4, Math.floor(params.previewRobotSamples || 10))
    );

    for (let i = 0; i < sampledFrames.length; i++) {
        const ghost = createRobotGhostClone(vis, sampledFrames[i], i, sampledFrames.length);
        if (!ghost) continue;
        group.add(ghost);
        robotGhosts.push(ghost);
    }

    if (robotGhosts.length === 0) {
        disposeObjectTree(group);
        return false;
    }

    // 目标点强调
    const last = samples[samples.length - 1];
    const targetGeom = markPreviewOwnedGeometry(new THREE.SphereGeometry(0.02, 20, 14));
    const targetMat = markPreviewOwnedMaterial(new THREE.MeshBasicMaterial({
        color: 0xf59e0b,
        transparent: true,
        opacity: 0.85,
        depthWrite: false,
    }));
    const target = new THREE.Mesh(targetGeom, targetMat);
    target.position.copy(last.absPosition);
    target.renderOrder = 950;
    group.add(target);

    motionPreviewGroup = group;
    motionPreviewItems = {
        robotGhosts,
        target,
        sampledFrames,
    };

    scene.add(group);
    return true;
}

function startPreviewAnimation(plan) {
    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;
    if (!DQ || !vis) return false;

    const samples = buildPreviewSamples(DQ, vis, plan);
    if (!samples || samples.length < 2) return false;

    clearPreviewVisualization({ deferDispose: false });
    const ok = createPreviewVisualization(samples, plan.params);
    if (!ok || !motionPreviewItems) return false;

    updateStatus(STATE.PREVIEWING, '规划中… 正在播放机器人残影预览');

    const total = motionPreviewItems.robotGhosts.length;
    const totalTimeSecRaw = Number(samples[samples.length - 1]?.timeSec);
    const totalTimeSec = Number.isFinite(totalTimeSecRaw) && totalTimeSecRaw > 0
        ? totalTimeSecRaw
        : Math.max(0.1, Number(plan.params.previewDuration) || 1.2);
    const durationMs = Math.max(1, totalTimeSec * 1000);
    const startTs = performance.now();
    const frameTimes = (motionPreviewItems.sampledFrames || []).map((s, idx) => {
        const t = Number(s?.timeSec);
        if (Number.isFinite(t)) return Math.max(0, t);
        if (total <= 1) return 0;
        return (idx / Math.max(1, total - 1)) * totalTimeSec;
    });
    if (frameTimes.length !== total) {
        frameTimes.length = 0;
        for (let i = 0; i < total; i++) {
            frameTimes.push((i / Math.max(1, total - 1)) * totalTimeSec);
        }
    }

    function tick(now) {
        if (motionState !== STATE.PREVIEWING) {
            motionPreviewAnimationId = null;
            return;
        }

        const t = Math.min(1, (now - startTs) / durationMs);
        const curTimeSec = t * totalTimeSec;
        let revealCount = -1;
        for (let i = 0; i < frameTimes.length; i++) {
            if (frameTimes[i] <= curTimeSec + 1e-6) revealCount = i;
            else break;
        }

        for (let i = 0; i < total; i++) {
            const visible = i <= revealCount;
            const ghostRobot = motionPreviewItems.robotGhosts[i];
            if (!ghostRobot) continue;
            ghostRobot.visible = visible;
        }

        if (motionPreviewItems.target) {
            motionPreviewItems.target.material.opacity = 0.65 + 0.20 * Math.sin(now * 0.01);
        }

        if (t >= 1) {
            motionPreviewAnimationId = null;
            updateStatus(STATE.PLANNED, '规划完成，点击“执行已规划运动”开始真实执行');
            return;
        }

        motionPreviewAnimationId = requestAnimationFrame(tick);
    }

    motionPreviewAnimationId = requestAnimationFrame(tick);
    return true;
}

function invalidatePreparedPlan(message = '目标已修改，请重新规划预览') {
    if (!motionPreparedPlan && !motionPreparedTaskPlan) return;
    motionPreparedPlan = null;
    motionPreparedTaskPlan = null;
    motionDebugSelectedTaskStepIndex = 0;
    clearPreviewVisualization();
    setPlannedMotionDebugInfo('-');

    if (motionState === STATE.PLANNED || motionState === STATE.DONE || motionState === STATE.ERROR || motionState === STATE.IDLE) {
        updateStatus(STATE.IDLE, message);
    }
}

function isAbsolutePosePlan(plan) {
    return !!plan &&
        !plan.planKind &&
        Array.isArray(plan.targetDQ) &&
        plan.targetDQ.length === 8 &&
        Array.isArray(plan.qStart) &&
        Array.isArray(plan.qTarget);
}

function getDebugAbsolutePlan() {
    if (isAbsolutePosePlan(motionPreparedPlan)) return motionPreparedPlan;
    if (isAbsolutePosePlan(motionActivePlan) && !motionActiveTaskPlan) return motionActivePlan;
    return null;
}

function isSingleArmDebugPlan(plan) {
    return !!plan &&
        plan.planKind === 'single-arm' &&
        Array.isArray(plan.targetDQ) &&
        plan.targetDQ.length === 8 &&
        Array.isArray(plan.qStart) &&
        Array.isArray(plan.qTarget);
}

function getDebugTaskPlan() {
    if (motionPreparedTaskPlan?.type === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE) return motionPreparedTaskPlan;
    if (motionActiveTaskPlan?.type === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE) return motionActiveTaskPlan;
    return null;
}

function clampDebugTaskStepIndex(taskPlan, preferredIndex = 0) {
    const steps = Array.isArray(taskPlan?.steps) ? taskPlan.steps : [];
    if (steps.length <= 0) return -1;
    const idx = Math.floor(Number(preferredIndex) || 0);
    return Math.max(0, Math.min(steps.length - 1, idx));
}

function getDebugPlanSelection() {
    const taskPlan = getDebugTaskPlan();
    if (taskPlan) {
        const idx = clampDebugTaskStepIndex(taskPlan, motionDebugSelectedTaskStepIndex);
        const step = idx >= 0 ? taskPlan.steps[idx] : null;
        if (isSingleArmDebugPlan(step)) {
            return {
                plan: step,
                taskPlan,
                stepIndex: idx,
            };
        }
    }

    const absolute = getDebugAbsolutePlan();
    if (absolute) {
        return {
            plan: absolute,
            taskPlan: null,
            stepIndex: -1,
        };
    }

    return null;
}

function getDebugPlanDisplayPrefix(selection) {
    const stepIndex = Number(selection?.stepIndex);
    const steps = Array.isArray(selection?.taskPlan?.steps) ? selection.taskPlan.steps : null;
    if (!steps || steps.length <= 0 || stepIndex < 0) return '';
    const stepPlan = selection?.plan;
    const armLabel = stepPlan?.armLabel || (stepPlan?.armIndex === 1 ? '左臂' : stepPlan?.armIndex === 2 ? '右臂' : '步骤');
    return `步骤 ${stepIndex + 1}/${steps.length}（${armLabel}）`;
}

function formatDebugErrorText(err) {
    if (!err) return '';
    const posMm = (Number(err.posErr3D) || 0) * 1000;
    const rotDeg = Number(err.rotErrDeg) || 0;
    return `误差 ${posMm.toFixed(2)} mm / ${rotDeg.toFixed(3)} deg`;
}

function setPlannedMotionDebugInfo(text = '-') {
    const el = typeof document !== 'undefined'
        ? document.getElementById('planned-motion-debug-info')
        : null;
    if (!el) return;
    el.textContent = text || '-';
}

function formatRadToDeg(rad, digits = 2) {
    const deg = (Number(rad) || 0) * 180 / Math.PI;
    return `${deg.toFixed(digits)}deg`;
}

function adaptJointVectorToReference(candidateQ, referenceQ) {
    const ref = Array.isArray(referenceQ) ? referenceQ : [];
    if (ref.length === 0) {
        return Array.isArray(candidateQ) ? candidateQ.slice() : [];
    }

    if (!Array.isArray(candidateQ) || candidateQ.length === 0) {
        return ref.slice();
    }

    if (candidateQ.length === ref.length) {
        return candidateQ.slice();
    }

    const out = ref.slice();
    const n = Math.min(candidateQ.length, ref.length);
    for (let i = 0; i < n; i++) {
        out[i] = Number.isFinite(candidateQ[i]) ? candidateQ[i] : out[i];
    }
    return out;
}

function getWaistJointCount(plan = null) {
    const visWaistLen = Math.max(0, Math.floor(Number(appState.visualizer?.humanoidWaistLength) || 0));
    const qLen = Array.isArray(plan?.qStart)
        ? plan.qStart.length
        : (Array.isArray(appState.humanoidQ) ? appState.humanoidQ.length : 0);
    if (qLen <= 0) return 0;
    return Math.max(0, Math.min(visWaistLen, qLen));
}

function getHumanoidArmJointRanges(plan = null) {
    const qLen = Array.isArray(plan?.qStart)
        ? plan.qStart.length
        : (Array.isArray(appState.humanoidQ) ? appState.humanoidQ.length : 0);
    const waistCount = getWaistJointCount(plan);
    const rawLeftLen = Math.max(0, Math.floor(Number(appState.visualizer?.humanoidLeftArmLength) || 0));
    const rawRightLen = Math.max(0, Math.floor(Number(appState.visualizer?.humanoidRightArmLength) || 0));
    const leftStart = waistCount;
    const leftLen = Math.max(0, Math.min(rawLeftLen, Math.max(0, qLen - leftStart)));
    const rightStart = leftStart + leftLen;
    const rightLen = Math.max(0, Math.min(rawRightLen, Math.max(0, qLen - rightStart)));
    return {
        waistCount,
        leftStart,
        leftEnd: leftStart + leftLen,
        rightStart,
        rightEnd: rightStart + rightLen,
    };
}

function getHumanoidWaistJointLimits(vis, qRef = null, waistCountOverride = null) {
    if (!vis || typeof vis._getHumanoidJointLimits !== 'function') return null;
    const all = vis._getHumanoidJointLimits();
    if (!Array.isArray(all) || all.length === 0) return null;

    const qLen = Array.isArray(qRef) ? qRef.length : all.length;
    const rawWaistCount = Number.isFinite(Number(waistCountOverride))
        ? Math.floor(Number(waistCountOverride))
        : Math.floor(Number(vis.humanoidWaistLength) || 0);
    const waistCount = Math.max(0, Math.min(rawWaistCount, all.length, qLen));
    if (waistCount <= 0) return null;

    const limits = [];
    for (let i = 0; i < waistCount; i++) {
        const lim = all[i] || {};
        let lower = Number(lim.lower);
        let upper = Number(lim.upper);
        if (!Number.isFinite(lower)) lower = -Math.PI;
        if (!Number.isFinite(upper)) upper = Math.PI;
        if (upper < lower) {
            const t = lower;
            lower = upper;
            upper = t;
        }
        limits.push({ lower, upper });
    }
    return limits;
}

function evaluateWaistLimitPenalty(qCandidate, waistLimits, params = null) {
    const out = {
        penalty: 0,
        hardHit: false,
        outside: false,
        minRemainDeg: null,
    };
    if (!Array.isArray(qCandidate) || qCandidate.length === 0) return out;
    if (!Array.isArray(waistLimits) || waistLimits.length === 0) return out;

    const softMarginDeg = Math.max(0.25, Number(params?.waistLimitSoftMarginDeg) || 4.0);
    const hardMarginDeg = Math.max(0.05, Number(params?.waistLimitHardMarginDeg) || 1.0);
    const softMarginRad = softMarginDeg * Math.PI / 180;
    const hardMarginRad = Math.min(softMarginRad, hardMarginDeg * Math.PI / 180);

    let minRemain = Number.POSITIVE_INFINITY;
    for (let i = 0; i < waistLimits.length && i < qCandidate.length; i++) {
        const lim = waistLimits[i];
        if (!lim) continue;

        const q = Number(qCandidate[i]) || 0;
        const dLow = q - lim.lower;
        const dUp = lim.upper - q;
        const remain = Math.min(dLow, dUp);
        if (Number.isFinite(remain)) {
            minRemain = Math.min(minRemain, remain);
        }

        if (remain < 0) out.outside = true;
        if (remain < hardMarginRad) out.hardHit = true;

        if (remain < softMarginRad) {
            const r = (softMarginRad - remain) / Math.max(softMarginRad, 1e-9);
            out.penalty += r * r;
        }
    }

    out.minRemainDeg = Number.isFinite(minRemain) ? (minRemain * 180 / Math.PI) : null;
    return out;
}

function buildSingleArmSyncJointTarget(qStart, qFinal, armIndex, plan = null) {
    const startQ = adaptJointVectorToReference(qStart, qStart);
    const finalQ = adaptJointVectorToReference(qFinal, startQ);
    if (!Array.isArray(startQ) || startQ.length === 0) return [];

    const out = startQ.slice();
    const ranges = getHumanoidArmJointRanges(plan || { qStart: startQ });
    let begin = 0;
    let end = 0;
    if (armIndex === 1) {
        begin = ranges.leftStart;
        end = ranges.leftEnd;
    } else if (armIndex === 2) {
        begin = ranges.rightStart;
        end = ranges.rightEnd;
    }
    begin = Math.max(0, Math.min(begin, out.length));
    end = Math.max(begin, Math.min(end, out.length));
    for (let i = begin; i < end; i++) out[i] = finalQ[i];
    return out;
}

function buildSingleArmWaistAndArmJointTarget(qStart, qFinal, armIndex, plan = null) {
    const startQ = adaptJointVectorToReference(qStart, qStart);
    const finalQ = adaptJointVectorToReference(qFinal, startQ);
    if (!Array.isArray(startQ) || startQ.length === 0) return [];

    const out = startQ.slice();
    const ranges = getHumanoidArmJointRanges(plan || { qStart: startQ });
    const waistCount = Math.max(0, Math.min(ranges.waistCount || 0, out.length));
    for (let i = 0; i < waistCount; i++) out[i] = finalQ[i];

    let begin = 0;
    let end = 0;
    if (armIndex === 1) {
        begin = ranges.leftStart;
        end = ranges.leftEnd;
    } else if (armIndex === 2) {
        begin = ranges.rightStart;
        end = ranges.rightEnd;
    }
    begin = Math.max(0, Math.min(begin, out.length));
    end = Math.max(begin, Math.min(end, out.length));
    for (let i = begin; i < end; i++) out[i] = finalQ[i];
    return out;
}

function mergeWaistAndNonWaistTargets(waistSourceQ, nonWaistSourceQ, waistCount = null) {
    const ref = adaptJointVectorToReference(nonWaistSourceQ, nonWaistSourceQ);
    if (!Array.isArray(ref) || ref.length === 0) return [];
    const waistQ = adaptJointVectorToReference(waistSourceQ, ref);
    const nWaist = Number.isFinite(Number(waistCount))
        ? Math.max(0, Math.min(Math.floor(Number(waistCount)), ref.length))
        : getWaistJointCount({ qStart: ref });
    const out = ref.slice();
    for (let i = 0; i < nWaist; i++) out[i] = waistQ[i];
    return out;
}

function computeNonWaistRemainingDeg(targetQ, currentQ, waistCount) {
    const qTar = adaptJointVectorToReference(targetQ, currentQ);
    const qCur = adaptJointVectorToReference(currentQ, qTar);
    if (!Array.isArray(qTar) || !Array.isArray(qCur) || qTar.length === 0 || qCur.length === 0) {
        return null;
    }
    const start = Math.max(0, Math.min(Number(waistCount) || 0, qTar.length, qCur.length));
    if (start >= qTar.length) return 0;

    let maxAbs = 0;
    for (let i = start; i < qTar.length; i++) {
        const d = Math.abs(wrapAngle((qTar[i] || 0) - (qCur[i] || 0)));
        if (d > maxAbs) maxAbs = d;
    }
    return maxAbs * 180 / Math.PI;
}

function buildWaistDeltaSummary(qStart, qWaistAligned, waistCount, jointNames = []) {
    const q0 = adaptJointVectorToReference(qStart, qStart);
    const q1 = adaptJointVectorToReference(qWaistAligned, q0);
    const n = Math.max(0, Math.min(Number(waistCount) || 0, q0.length, q1.length));
    if (n <= 0) return '未检测到腰部关节（waistLength=0）';

    const deltas = [];
    let maxAbs = 0;
    for (let i = 0; i < n; i++) {
        const d = wrapAngle((q1[i] || 0) - (q0[i] || 0));
        const absD = Math.abs(d);
        if (absD > maxAbs) maxAbs = absD;
        deltas.push({
            idx: i,
            name: jointNames[i] || `q${i}`,
            delta: d,
            abs: absD,
        });
    }

    deltas.sort((a, b) => b.abs - a.abs);
    const top = deltas.slice(0, Math.min(4, deltas.length));
    const parts = top.map(v => `${v.name}:${formatRadToDeg(v.delta, 2)}`);
    return `腰部变化(max ${formatRadToDeg(maxAbs, 2)}): ${parts.join(' | ')}`;
}

function buildNonWaistDeltaSummary(qStart, qWaistAligned, waistCount, jointNames = []) {
    const q0 = adaptJointVectorToReference(qStart, qStart);
    const q1 = adaptJointVectorToReference(qWaistAligned, q0);
    const start = Math.max(0, Math.min(Number(waistCount) || 0, q0.length, q1.length));
    if (start >= q0.length) return '非腰部变化: 无';

    let maxAbs = 0;
    let maxIdx = -1;
    let maxDelta = 0;
    for (let i = start; i < q0.length; i++) {
        const d = wrapAngle((q1[i] || 0) - (q0[i] || 0));
        const absD = Math.abs(d);
        if (absD > maxAbs) {
            maxAbs = absD;
            maxIdx = i;
            maxDelta = d;
        }
    }

    if (maxIdx < 0) return '非腰部变化: 无';
    const name = jointNames[maxIdx] || `q${maxIdx}`;
    return `非腰部max ${name}:${formatRadToDeg(maxDelta, 2)}`;
}

function buildWaistPreAlignTargetQ(plan, referenceQ) {
    const ref = adaptJointVectorToReference(referenceQ, referenceQ);
    if (!Array.isArray(ref) || ref.length === 0) return [];

    const seed = adaptJointVectorToReference(plan?.waistYawPreAlign?.qSeed, ref);
    const waistCount = getWaistJointCount(plan);
    const out = ref.slice();
    const n = Math.max(0, Math.min(waistCount, out.length, seed.length));
    for (let i = 0; i < n; i++) out[i] = seed[i];
    return out;
}

function animateDebugJointTransition(vis, qFrom, qTo, durationMs = 700) {
    const from = adaptJointVectorToReference(qFrom, qTo);
    const to = adaptJointVectorToReference(qTo, from);
    if (!Array.isArray(from) || !Array.isArray(to) || from.length === 0 || to.length === 0) return;

    if (motionDebugAnimationId != null) {
        cancelAnimationFrame(motionDebugAnimationId);
        motionDebugAnimationId = null;
    }

    const qRef = appState.humanoidQ || window.humanoidQ;
    if (!Array.isArray(qRef) || qRef.length === 0) {
        restoreHumanoidQ(vis, to.slice());
        return;
    }

    const startTs = performance.now();
    const totalMs = Math.max(120, Number(durationMs) || 700);

    const tick = (now) => {
        const t = clampValue((now - startTs) / totalMs, 0, 1);
        const s = easeInOutCubic(t);

        for (let i = 0; i < qRef.length; i++) {
            const a = from[i] ?? 0;
            const b = to[i] ?? a;
            qRef[i] = lerpJoint(a, b, s);
        }

        if (Array.isArray(appState.humanoidQ)) {
            for (let i = 0; i < appState.humanoidQ.length && i < qRef.length; i++) {
                appState.humanoidQ[i] = qRef[i];
            }
        }
        if (Array.isArray(window.humanoidQ)) {
            for (let i = 0; i < window.humanoidQ.length && i < qRef.length; i++) {
                window.humanoidQ[i] = qRef[i];
            }
        }

        vis?.setHumanoidJoints?.(qRef);
        callbacks.updateRobotPose?.();
        callbacks.refreshJointControlDisplay?.();

        if (t >= 1) {
            motionDebugAnimationId = null;
            return;
        }
        motionDebugAnimationId = requestAnimationFrame(tick);
    };

    motionDebugAnimationId = requestAnimationFrame(tick);
}

function alignBaseTargetToPose(basePose) {
    if (!appState.humanoidBaseTargetPose || typeof appState.humanoidBaseTargetPose !== 'object') {
        appState.humanoidBaseTargetPose = { x: 0, y: 0, yaw: 0 };
    }
    appState.humanoidBaseTargetPose.x = Number(basePose?.x) || 0;
    appState.humanoidBaseTargetPose.y = Number(basePose?.y) || 0;
    appState.humanoidBaseTargetPose.yaw = Number(basePose?.yaw) || 0;
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidBaseReverseDrive = false;
}

function isBasePoseNearlyEqual(a, b, posTol = 1e-6, yawTol = 1e-6) {
    if (!a || !b) return false;
    return (
        Math.hypot((a.x || 0) - (b.x || 0), (a.y || 0) - (b.y || 0)) <= posTol &&
        Math.abs(wrapAngle((a.yaw || 0) - (b.yaw || 0))) <= yawTol
    );
}

function isJointVectorNearlyEqual(a, b, tolRad = 1e-6) {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length || a.length === 0) return false;
    for (let i = 0; i < a.length; i++) {
        if (Math.abs(wrapAngle((a[i] || 0) - (b[i] || 0))) > tolRad) return false;
    }
    return true;
}

function getDebugWorldPoseForPlan(DQ, vis, plan, q, basePose) {
    if (plan?.planKind === 'single-arm') {
        return getHumanoidArmWorldPose(DQ, vis, q, basePose, plan.armIndex);
    }
    return getWorldAbsolutePoseFromBaseAndJoints(vis, q, basePose);
}

function buildDebugFlowNodesForPlan(DQ, vis, plan) {
    if (!DQ || !vis || !plan) return [];
    const qStartRef = adaptJointVectorToReference(plan.qStart, plan.qStart);
    if (!Array.isArray(qStartRef) || qStartRef.length === 0) return [];

    const baseStart = cloneBasePose(plan.baseStartPose || appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });
    const baseTarget = cloneBasePose(plan.baseTargetPose || baseStart);

    const qWaist = adaptJointVectorToReference(
        plan.qVirtualState1 || plan.qWaistVirtualTarget || buildWaistPreAlignTargetQ(plan, qStartRef),
        qStartRef
    );
    const qBase = adaptJointVectorToReference(plan.qVirtualState2 || qWaist, qStartRef);
    const qSync = adaptJointVectorToReference(plan.qSyncTarget || plan.qVirtualState3FullBody || plan.qTarget, qStartRef);
    const qFinal = adaptJointVectorToReference(plan.qTarget || qSync, qStartRef);

    const rawNodes = [
        { key: 'start', label: '起点', basePose: baseStart, q: qStartRef },
        { key: 'waist', label: '腰部预对齐', basePose: baseStart, q: qWaist },
        { key: 'base', label: '底盘XY后', basePose: baseTarget, q: qBase },
        { key: 'sync', label: '同步目标', basePose: baseTarget, q: qSync },
        { key: 'final', label: '最终IK', basePose: baseTarget, q: qFinal },
    ];

    const compactNodes = [];
    for (const node of rawNodes) {
        const prev = compactNodes[compactNodes.length - 1];
        if (
            prev &&
            isJointVectorNearlyEqual(prev.q, node.q) &&
            isBasePoseNearlyEqual(prev.basePose, node.basePose)
        ) {
            continue;
        }

        const worldPose = getDebugWorldPoseForPlan(DQ, vis, plan, node.q, node.basePose);
        if (!worldPose) continue;
        const absPosition = extractPositionFromDQ(DQ, worldPose);
        compactNodes.push({
            ...node,
            worldPose,
            absPosition,
        });
    }

    return compactNodes;
}

function createDebugFlowVisualization(flowNodes, activeStageKey = 'start') {
    const vis = appState.visualizer;
    const scene = vis?.scene;
    if (!scene || !Array.isArray(flowNodes) || flowNodes.length === 0) return false;

    clearPreviewVisualization();

    const group = new THREE.Group();
    group.name = 'PlannedMotionDebugSolveFlow';

    if (flowNodes.length >= 2) {
        const pathPoints = flowNodes.map(n => n.absPosition.clone());
        const pathGeom = markPreviewOwnedGeometry(new THREE.BufferGeometry().setFromPoints(pathPoints));
        const pathMat = markPreviewOwnedMaterial(new THREE.LineBasicMaterial({
            color: 0x38bdf8,
            transparent: true,
            opacity: 0.82,
            depthWrite: false,
        }));
        const pathLine = new THREE.Line(pathGeom, pathMat);
        pathLine.renderOrder = 930;
        group.add(pathLine);
    }

    const colorByKey = {
        start: 0x94a3b8,
        waist: 0xf59e0b,
        base: 0x60a5fa,
        sync: 0x22d3ee,
        final: 0x34d399,
    };
    const activeColor = 0xf43f5e;

    for (let i = 0; i < flowNodes.length; i++) {
        const node = flowNodes[i];
        const isActive = node.key === activeStageKey || (activeStageKey === 'final' && node.key === 'sync' && !flowNodes.find(v => v.key === 'final'));
        const radius = isActive ? 0.025 : 0.016;
        const nodeGeom = markPreviewOwnedGeometry(new THREE.SphereGeometry(radius, 18, 14));
        const nodeMat = markPreviewOwnedMaterial(new THREE.MeshBasicMaterial({
            color: isActive ? activeColor : (colorByKey[node.key] || 0x94a3b8),
            transparent: true,
            opacity: isActive ? 0.96 : 0.88,
            depthWrite: false,
        }));
        const marker = new THREE.Mesh(nodeGeom, nodeMat);
        marker.position.copy(node.absPosition);
        marker.renderOrder = 940 + i;
        group.add(marker);
    }

    motionPreviewGroup = group;
    motionPreviewItems = null;
    scene.add(group);
    return true;
}

function applyDebugPlanStage(stageKey = 'start') {
    motionDebugLastStageKey = stageKey || 'start';
    if (isPlannedMotionRunning()) {
        updateStatus(STATE.ERROR, '当前正在执行/预览，请先取消后再使用 Debug 分步');
        return false;
    }

    const selection = getDebugPlanSelection();
    const plan = selection?.plan || null;
    if (!selection || !plan) {
        updateStatus(STATE.ERROR, '请先执行“规划预览/规划任务”生成可调试计划');
        return false;
    }
    const debugPrefix = getDebugPlanDisplayPrefix(selection);

    const vis = appState.visualizer;
    if (!vis) {
        updateStatus(STATE.ERROR, '可视化器不可用');
        return false;
    }

    const preAlign = plan.waistYawPreAlign || {};
    const qStartRef = Array.isArray(plan.qStart) ? plan.qStart : [];
    const qWaistOnly = adaptJointVectorToReference(
        plan.qVirtualState1 || plan.qWaistVirtualTarget || buildWaistPreAlignTargetQ(plan, qStartRef),
        qStartRef
    );
    const qBaseAfterXY = adaptJointVectorToReference(plan.qVirtualState2 || qWaistOnly, qStartRef);
    const qSyncFinal = adaptJointVectorToReference(plan.qSyncTarget || plan.qVirtualState3FullBody || plan.qTarget, qStartRef);
    const qFinalIK = adaptJointVectorToReference(plan.qTarget || qSyncFinal, qStartRef);
    const waistCount = getWaistJointCount(plan);
    const jointNames = appState.humanoidJointNames || vis.humanoidJointNames || [];
    const waistDeltaSummary = buildWaistDeltaSummary(qStartRef, qWaistOnly, waistCount, jointNames);
    const nonWaistDeltaSummary = buildNonWaistDeltaSummary(qStartRef, qWaistOnly, waistCount, jointNames);
    const baseAtWaist = cloneBasePose(plan.baseStartPose || appState.humanoidBasePose);

    let waistLabel = '腰部 yaw 预对齐后';
    if (!preAlign.applied) {
        if (preAlign.reason === 'small_delta') {
            const deltaDeg = Math.abs((preAlign.yawDeltaRad || 0) * 180 / Math.PI);
            waistLabel = `腰部 yaw 预对齐跳过(Δyaw=${deltaDeg.toFixed(2)}deg)`;
        } else if (preAlign.reason === 'disabled') {
            waistLabel = '腰部 yaw 预对齐关闭(回退起点)';
        } else if (preAlign.reason === 'solve_failed') {
            waistLabel = '腰部 yaw 预对齐失败(回退起点)';
        } else if (preAlign.reason) {
            waistLabel = `腰部 yaw 预对齐未执行(${preAlign.reason})`;
        } else if (!Array.isArray(preAlign.qSeed) || preAlign.qSeed.length === 0) {
            waistLabel = '腰部 yaw 预对齐后(回退起点)';
        }
    }
    waistLabel += '（仅腰部）';

    const stageMap = {
        start: {
            label: '起点',
            basePose: cloneBasePose(plan.baseStartPose || appState.humanoidBasePose),
            q: qStartRef,
        },
        waist: {
            label: waistLabel,
            basePose: baseAtWaist,
            q: qWaistOnly,
            animateFrom: qStartRef,
        },
        base: {
            label: '底盘 XY 收敛后',
            basePose: cloneBasePose(plan.baseTargetPose || plan.baseStartPose || appState.humanoidBasePose),
            q: qBaseAfterXY,
        },
        sync: {
            label: '同步阶段目标(虚拟状态3)',
            basePose: cloneBasePose(plan.baseTargetPose || plan.baseStartPose || appState.humanoidBasePose),
            q: qSyncFinal,
        },
        final: {
            label: plan.planKind === 'single-arm' ? '最终IK(腰部+目标臂)' : '最终IK',
            basePose: cloneBasePose(plan.baseTargetPose || plan.baseStartPose || appState.humanoidBasePose),
            q: qFinalIK,
        },
    };

    const stage = stageMap[stageKey] || stageMap.start;
    if (!Array.isArray(stage.q) || stage.q.length === 0) {
        updateStatus(STATE.ERROR, `Debug 阶段“${stage.label}”无有效关节数据`);
        return false;
    }

    stopAllRuntimeTimers();
    if (!motionDebugFlowEnabled) {
        clearPreviewVisualization();
    }
    applyHumanoidBasePoseToState(vis, stage.basePose);
    alignBaseTargetToPose(stage.basePose);

    if (Array.isArray(stage.animateFrom) && stage.animateFrom.length > 0) {
        restoreHumanoidQ(vis, stage.animateFrom.slice());
        animateDebugJointTransition(vis, stage.animateFrom, stage.q, 850);
    } else {
        restoreHumanoidQ(vis, stage.q.slice());
    }

    const DQ = getDQModule()?.DQWrapper;
    const qEval = stage.q.slice();
    const err = (DQ && Array.isArray(qEval) && qEval.length > 0)
        ? (
            plan.planKind === 'single-arm'
                ? evaluateHumanoidArmWorldError(DQ, vis, qEval, stage.basePose, plan.targetDQ, plan.armIndex)
                : evaluateHumanoidWorldError(DQ, vis, qEval, stage.basePose, plan.targetDQ)
        )
        : null;

    let waistHeightInfo = '';
    if (DQ && stageKey === 'waist' && Array.isArray(qStartRef) && qStartRef.length > 0) {
        const worldStart = getDebugWorldPoseForPlan(DQ, vis, plan, qStartRef, baseAtWaist);
        const worldWaist = getDebugWorldPoseForPlan(DQ, vis, plan, qWaistOnly, baseAtWaist);
        const poseStart = safeCreatePoseDQ(DQ, worldStart);
        const poseWaist = safeCreatePoseDQ(DQ, worldWaist);
        if (poseStart && poseWaist) {
            const dzMm = ((poseWaist.t[3] || 0) - (poseStart.t[3] || 0)) * 1000;
            waistHeightInfo = ` | 参考点高度Δ ${dzMm.toFixed(1)} mm`;
        }
    }

    let flowSummary = '';
    if (motionDebugFlowEnabled && DQ) {
        const flowNodes = buildDebugFlowNodesForPlan(DQ, vis, plan);
        if (flowNodes.length > 0) {
            createDebugFlowVisualization(flowNodes, stageKey);
            flowSummary = flowNodes.map(v => v.label).join(' -> ');
        } else {
            clearPreviewVisualization();
        }
    }

    const errText = formatDebugErrorText(err);
    const prefixText = debugPrefix ? `${debugPrefix} | ` : '';
    if (stageKey === 'waist') {
        setPlannedMotionDebugInfo(
            `${prefixText}${waistDeltaSummary} | ${nonWaistDeltaSummary}${waistHeightInfo}${flowSummary ? ` | 流程: ${flowSummary}` : ''}`
        );
    } else if (stageKey === 'start') {
        setPlannedMotionDebugInfo(
            `${prefixText}腰部关节数 ${waistCount} | ${waistDeltaSummary} | ${nonWaistDeltaSummary}${flowSummary ? ` | 流程: ${flowSummary}` : ''}`
        );
    } else {
        setPlannedMotionDebugInfo(
            `${prefixText}${waistDeltaSummary} | ${nonWaistDeltaSummary}${flowSummary ? ` | 流程: ${flowSummary}` : ''}`
        );
    }
    updateStatus(
        STATE.PLANNED,
        `Debug：${prefixText}已切换到 ${stage.label}${errText ? `，${errText}` : ''}`
    );
    return true;
}

function dumpDebugPlanSummary() {
    const selection = getDebugPlanSelection();
    const plan = selection?.plan || null;
    if (!selection || !plan) {
        updateStatus(STATE.ERROR, '请先执行“规划预览/规划任务”生成可调试计划');
        return false;
    }
    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;
    const debugPrefix = getDebugPlanDisplayPrefix(selection);

    const baseStart = cloneBasePose(plan.baseStartPose);
    const baseTarget = cloneBasePose(plan.baseTargetPose);
    const preAlign = plan.waistYawPreAlign || {};
    const flowNodes = (DQ && vis) ? buildDebugFlowNodesForPlan(DQ, vis, plan) : [];
    const summary = {
        debugPlanLabel: debugPrefix || '绝对位姿计划',
        planKind: plan.planKind || 'absolute',
        createdAt: Number.isFinite(Number(plan.createdAt)) ? new Date(plan.createdAt).toISOString() : null,
        poseErrorBeforeWaist: plan.poseErrorBeforeWaist || null,
        poseErrorAfterWaist: plan.poseError || null,
        waistYawPreAlign: {
            applied: !!preAlign.applied,
            reason: preAlign.reason || null,
            yawCurrentDeg: Number.isFinite(preAlign.yawCurrent) ? preAlign.yawCurrent * 180 / Math.PI : null,
            yawTargetDeg: Number.isFinite(preAlign.yawTarget) ? preAlign.yawTarget * 180 / Math.PI : null,
            yawAfterDeg: Number.isFinite(preAlign.yawAfter) ? preAlign.yawAfter * 180 / Math.PI : null,
            yawDeltaDeg: Number.isFinite(preAlign.yawDeltaRad) ? preAlign.yawDeltaRad * 180 / Math.PI : null,
        },
        baseStartPose: baseStart,
        baseTargetPose: baseTarget,
        baseDelta: {
            x: baseTarget.x - baseStart.x,
            y: baseTarget.y - baseStart.y,
            yawDeg: wrapAngle(baseTarget.yaw - baseStart.yaw) * 180 / Math.PI,
        },
        armCompensationTrajectory: plan._armCompensationTrajectory || null,
        syncRelativeConstraintMode: plan._syncRelativeConstraintMode || motionParams.syncRelativeConstraintMode || null,
        hasSyncRelativeConstraint: Array.isArray(plan.syncRelativePoseConstraint),
        hasRuntimeRelativeConstraint: Array.isArray(plan._syncRelativeConstraintRuntime),
        hasSyncArmBodyRelativeTargets: !!(
            Array.isArray(plan.syncArmBodyRelativeTargets?.arm1) &&
            Array.isArray(plan.syncArmBodyRelativeTargets?.arm2)
        ),
        hasWaistSeed: Array.isArray(preAlign.qSeed),
        hasVirtualState1: Array.isArray(plan.qVirtualState1),
        hasVirtualState2: Array.isArray(plan.qVirtualState2),
        hasVirtualState3FullBody: Array.isArray(plan.qVirtualState3FullBody),
        hasSyncTarget: Array.isArray(plan.qSyncTarget),
        qStartLength: Array.isArray(plan.qStart) ? plan.qStart.length : 0,
        qSyncLength: Array.isArray(plan.qSyncTarget) ? plan.qSyncTarget.length : 0,
        qTargetLength: Array.isArray(plan.qTarget) ? plan.qTarget.length : 0,
        flowNodes: flowNodes.map(v => ({
            key: v.key,
            label: v.label,
            x: Number(v.absPosition?.x) || 0,
            y: Number(v.absPosition?.y) || 0,
            z: Number(v.absPosition?.z) || 0,
        })),
    };

    console.group('[planned-motion][debug] 规划摘要');
    console.table(summary.baseDelta);
    console.log('summary', summary);
    if (selection?.taskPlan) {
        console.log('taskPlan', selection.taskPlan);
    }
    console.log('full plan', plan);
    console.groupEnd();
    updateStatus(STATE.PLANNED, `Debug：${debugPrefix ? `${debugPrefix} | ` : ''}规划摘要已输出到浏览器控制台`);
    return true;
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

function getPlannedMotionJointMonitorElements() {
    if (typeof document === 'undefined') return null;
    const root = document.getElementById('planned-motion-joint-monitor');
    if (!root) return null;
    return {
        root,
        state: document.getElementById('planned-motion-joint-monitor-state'),
        groups: {
            waist: {
                canvas: document.getElementById('planned-motion-joint-waist-canvas'),
                meta: document.getElementById('planned-motion-joint-waist-meta'),
                values: document.getElementById('planned-motion-joint-waist-values'),
            },
            left: {
                canvas: document.getElementById('planned-motion-joint-left-canvas'),
                meta: document.getElementById('planned-motion-joint-left-meta'),
                values: document.getElementById('planned-motion-joint-left-values'),
            },
            right: {
                canvas: document.getElementById('planned-motion-joint-right-canvas'),
                meta: document.getElementById('planned-motion-joint-right-meta'),
                values: document.getElementById('planned-motion-joint-right-values'),
            },
            head: {
                canvas: document.getElementById('planned-motion-joint-head-canvas'),
                meta: document.getElementById('planned-motion-joint-head-meta'),
                values: document.getElementById('planned-motion-joint-head-values'),
            },
        },
    };
}

function getPlannedMotionJointMonitorChannel() {
    if (motionJointMonitorChannel) return motionJointMonitorChannel;
    if (motionJointMonitorChannelUnsupported) return null;
    if (typeof window === 'undefined' || typeof window.BroadcastChannel !== 'function') {
        motionJointMonitorChannelUnsupported = true;
        return null;
    }
    try {
        motionJointMonitorChannel = new BroadcastChannel(MOTION_JOINT_MONITOR_CHANNEL_NAME);
        return motionJointMonitorChannel;
    } catch (error) {
        motionJointMonitorChannelUnsupported = true;
        console.warn('规划运动：Joint monitor BroadcastChannel 不可用:', error);
        return null;
    }
}

function postPlannedMotionJointMonitorMessage(payload) {
    const channel = getPlannedMotionJointMonitorChannel();
    if (!channel) return;
    try {
        channel.postMessage({
            channel: MOTION_JOINT_MONITOR_CHANNEL_NAME,
            ...payload,
        });
    } catch (error) {
        console.warn('规划运动：发送 Joint monitor 数据失败:', error);
    }
}

function buildIndexRange(start, end, totalLen) {
    const s = Math.max(0, Math.min(Number(start) || 0, totalLen));
    const e = Math.max(s, Math.min(Number(end) || 0, totalLen));
    const out = [];
    for (let i = s; i < e; i++) out.push(i);
    return out;
}

function getPlannedMotionJointMonitorGroups(qNow) {
    const qRef = Array.isArray(qNow) ? qNow : [];
    const qLen = qRef.length;
    const ranges = getHumanoidArmJointRanges({ qStart: qRef });
    const headStart = Math.max(0, Math.min(ranges.rightEnd || 0, qLen));
    return {
        waist: {
            label: '腰部',
            indices: buildIndexRange(0, ranges.waistCount || 0, qLen),
        },
        left: {
            label: '左臂',
            indices: buildIndexRange(ranges.leftStart || 0, ranges.leftEnd || 0, qLen),
        },
        right: {
            label: '右臂',
            indices: buildIndexRange(ranges.rightStart || 0, ranges.rightEnd || 0, qLen),
        },
        head: {
            label: '头部',
            indices: buildIndexRange(headStart, qLen, qLen),
        },
    };
}

function preparePlannedMotionJointCanvas(canvas) {
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const cssWidth = Math.max(1, Math.floor(rect.width || canvas.clientWidth || 1));
    const cssHeight = Math.max(1, Math.floor(rect.height || canvas.clientHeight || 1));
    const dpr = Math.max(1, Math.min(2, Number(window.devicePixelRatio) || 1));
    const widthPx = Math.max(1, Math.floor(cssWidth * dpr));
    const heightPx = Math.max(1, Math.floor(cssHeight * dpr));
    if (canvas.width !== widthPx || canvas.height !== heightPx) {
        canvas.width = widthPx;
        canvas.height = heightPx;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.scale(dpr, dpr);
    return {
        ctx,
        width: cssWidth,
        height: cssHeight,
    };
}

function drawPlannedMotionJointChart(canvas, history, indices) {
    const view = preparePlannedMotionJointCanvas(canvas);
    if (!view) return { minDeg: null, maxDeg: null };
    const { ctx, width, height } = view;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = 'rgba(15, 23, 42, 0.86)';
    ctx.fillRect(0, 0, width, height);

    if (!Array.isArray(indices) || indices.length === 0) {
        ctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
        ctx.font = '11px monospace';
        ctx.fillText('无关节', 8, Math.max(16, Math.round(height / 2)));
        return { minDeg: null, maxDeg: null };
    }
    if (!Array.isArray(history) || history.length === 0) {
        ctx.fillStyle = 'rgba(148, 163, 184, 0.9)';
        ctx.font = '11px monospace';
        ctx.fillText('等待数据', 8, Math.max(16, Math.round(height / 2)));
        return { minDeg: null, maxDeg: null };
    }

    const sampleCount = history.length;
    const series = indices.map(() => new Array(sampleCount));
    let minDeg = Number.POSITIVE_INFINITY;
    let maxDeg = Number.NEGATIVE_INFINITY;

    for (let s = 0; s < sampleCount; s++) {
        const qSample = Array.isArray(history[s]?.q) ? history[s].q : [];
        for (let j = 0; j < indices.length; j++) {
            const idx = indices[j];
            const deg = ((Number(qSample[idx]) || 0) * 180) / Math.PI;
            series[j][s] = deg;
            if (deg < minDeg) minDeg = deg;
            if (deg > maxDeg) maxDeg = deg;
        }
    }

    if (!Number.isFinite(minDeg) || !Number.isFinite(maxDeg)) {
        return { minDeg: null, maxDeg: null };
    }
    if (maxDeg - minDeg < 0.5) {
        const center = 0.5 * (maxDeg + minDeg);
        minDeg = center - 0.25;
        maxDeg = center + 0.25;
    }

    const padX = 6;
    const padY = 6;
    const plotW = Math.max(1, width - 2 * padX);
    const plotH = Math.max(1, height - 2 * padY);
    const xDen = Math.max(1, sampleCount - 1);
    const yDen = Math.max(1e-9, maxDeg - minDeg);
    const toX = (i) => padX + (i / xDen) * plotW;
    const toY = (deg) => padY + (1 - ((deg - minDeg) / yDen)) * plotH;

    ctx.strokeStyle = 'rgba(100, 116, 139, 0.32)';
    ctx.lineWidth = 1;
    for (let g = 1; g <= 3; g++) {
        const y = padY + (g / 4) * plotH;
        ctx.beginPath();
        ctx.moveTo(padX, y);
        ctx.lineTo(padX + plotW, y);
        ctx.stroke();
    }
    if (minDeg < 0 && maxDeg > 0) {
        const y0 = toY(0);
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.44)';
        ctx.beginPath();
        ctx.moveTo(padX, y0);
        ctx.lineTo(padX + plotW, y0);
        ctx.stroke();
    }

    for (let j = 0; j < series.length; j++) {
        const line = series[j];
        if (!Array.isArray(line) || line.length === 0) continue;
        ctx.strokeStyle = MOTION_JOINT_MONITOR_COLORS[j % MOTION_JOINT_MONITOR_COLORS.length];
        ctx.lineWidth = 1.25;
        ctx.beginPath();
        ctx.moveTo(toX(0), toY(line[0]));
        for (let i = 1; i < line.length; i++) {
            ctx.lineTo(toX(i), toY(line[i]));
        }
        ctx.stroke();

        const yLast = toY(line[line.length - 1]);
        const xLast = toX(line.length - 1);
        ctx.fillStyle = ctx.strokeStyle;
        ctx.beginPath();
        ctx.arc(xLast, yLast, 2.1, 0, 2 * Math.PI);
        ctx.fill();
    }

    return { minDeg, maxDeg };
}

function formatPlannedMotionJointValues(indices, qNow, jointNames) {
    if (!Array.isArray(indices) || indices.length === 0) return '-';
    const names = Array.isArray(jointNames) ? jointNames : [];
    const qRef = Array.isArray(qNow) ? qNow : [];
    const maxItems = 10;
    const n = Math.min(indices.length, maxItems);
    const chunks = [];
    for (let i = 0; i < n; i++) {
        const idx = indices[i];
        const name = names[idx] || `q${idx}`;
        const deg = ((Number(qRef[idx]) || 0) * 180 / Math.PI).toFixed(2);
        chunks.push(`${name}: ${deg}°`);
    }
    if (indices.length > n) chunks.push(`+${indices.length - n} joints`);
    return chunks.join(' | ');
}

function updatePlannedMotionJointMonitor() {
    const els = getPlannedMotionJointMonitorElements();
    const localVisible = !!els && els.root.offsetParent !== null;

    const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
    const validHumanoid = appState.humanoidMode && Array.isArray(qNow) && qNow.length > 0;
    if (!validHumanoid) {
        motionJointMonitorHistory = [];
        motionJointMonitorLastQDim = 0;
        motionJointMonitorLastRenderTs = 0;
        motionJointMonitorLastStateText = '';
        postPlannedMotionJointMonitorMessage({
            type: 'status',
            ts: performance.now(),
            available: false,
            message: '等待人形数据',
        });
        if (localVisible) {
            if (els.state) els.state.textContent = '等待人形数据';
            const emptyKeys = ['waist', 'left', 'right', 'head'];
            for (const key of emptyKeys) {
                const panel = els.groups[key];
                if (!panel) continue;
                if (panel.meta) panel.meta.textContent = '0 joints';
                if (panel.values) panel.values.textContent = '-';
                drawPlannedMotionJointChart(panel.canvas, [], []);
            }
        }
        return;
    }

    if (motionJointMonitorLastQDim !== qNow.length) {
        motionJointMonitorHistory = [];
        motionJointMonitorLastQDim = qNow.length;
    }

    motionJointMonitorHistory.push({
        t: performance.now(),
        q: qNow,
    });
    if (motionJointMonitorHistory.length > MOTION_JOINT_MONITOR_HISTORY_LIMIT) {
        motionJointMonitorHistory.splice(0, motionJointMonitorHistory.length - MOTION_JOINT_MONITOR_HISTORY_LIMIT);
    }

    const groups = getPlannedMotionJointMonitorGroups(qNow);
    const jointNames = Array.isArray(appState.humanoidJointNames) ? appState.humanoidJointNames : [];
    const executing = (
        motionState === STATE.WAIST_ALIGNING ||
        motionState === STATE.BASE_MOVING ||
        motionState === STATE.ARM_COMPENSATING
    );
    const stateText = `${executing ? '执行中' : '监控中'} | ${motionJointMonitorHistory.length}帧`;
    if (localVisible && els.state && stateText !== motionJointMonitorLastStateText) {
        els.state.textContent = stateText;
        motionJointMonitorLastStateText = stateText;
    }
    const nowTs = performance.now();

    postPlannedMotionJointMonitorMessage({
        type: 'snapshot',
        ts: nowTs,
        stateKey: motionState,
        stateText,
        executing,
        qNow,
        groups,
        jointNames,
        historyFrames: motionJointMonitorHistory.length,
    });

    if (!localVisible) {
        return;
    }

    const minRenderInterval = executing
        ? MOTION_JOINT_MONITOR_RENDER_INTERVAL_EXEC_MS
        : MOTION_JOINT_MONITOR_RENDER_INTERVAL_IDLE_MS;
    if ((nowTs - motionJointMonitorLastRenderTs) < minRenderInterval) {
        return;
    }
    motionJointMonitorLastRenderTs = nowTs;

    const keys = ['waist', 'left', 'right', 'head'];
    for (const key of keys) {
        const panel = els.groups[key];
        const group = groups[key];
        if (!panel || !group) continue;

        const chartInfo = drawPlannedMotionJointChart(
            panel.canvas,
            motionJointMonitorHistory,
            group.indices
        );
        const rangeText = (Number.isFinite(chartInfo.minDeg) && Number.isFinite(chartInfo.maxDeg))
            ? `${chartInfo.minDeg.toFixed(1)}~${chartInfo.maxDeg.toFixed(1)}°`
            : '-';
        if (panel.meta) panel.meta.textContent = `${group.indices.length} joints | ${rangeText}`;
        if (panel.values) {
            panel.values.textContent = formatPlannedMotionJointValues(group.indices, qNow, jointNames);
        }
    }
}

function startPlannedMotionJointMonitorLoop() {
    if (motionJointMonitorTimer != null) return;
    motionJointMonitorLastRenderTs = 0;
    motionJointMonitorLastStateText = '';
    updatePlannedMotionJointMonitor();
    motionJointMonitorTimer = setInterval(() => {
        updatePlannedMotionJointMonitor();
    }, MOTION_JOINT_MONITOR_INTERVAL_MS);
}

// ── UI 事件绑定 ──────────────────────────────────

function parseTargetInput(inputEl, label = '目标双四元数', options = {}) {
    const allowBaseYaw = !!options.allowBaseYaw;
    if (!inputEl) {
        return { ok: false, message: `未找到${label}输入框` };
    }

    const text = inputEl.value.trim();
    if (!text) {
        return { ok: false, message: `请输入${label}` };
    }

    let baseYaw = null;
    let textForDQ = text;

    if (allowBaseYaw) {
        const yawMatch = text.match(/(?:^|[\s,;])yaw\s*[:=]\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*(deg|rad|°)?(?=$|[\s,;])/i);
        if (yawMatch) {
            const yawRaw = Number(yawMatch[1]);
            if (!Number.isFinite(yawRaw)) {
                return { ok: false, message: `${label}里的底盘 yaw 非法` };
            }
            const unit = (yawMatch[2] || 'deg').toLowerCase();
            baseYaw = unit === 'rad' ? yawRaw : yawRaw * Math.PI / 180;
            baseYaw = wrapAngle(baseYaw);
            const idx = yawMatch.index ?? 0;
            textForDQ = `${text.slice(0, idx)} ${text.slice(idx + yawMatch[0].length)}`.trim();
        }
    }

    const parts = textForDQ.split(/[\s,;]+/).filter(Boolean).map(Number);
    if (parts.some(Number.isNaN)) {
        const yawHint = allowBaseYaw ? '，可选追加 yaw（例如：yaw=30deg）' : '';
        return { ok: false, message: `${label}请输入 8 个数值（DQ 格式）${yawHint}` };
    }

    let dqParts = parts;
    if (allowBaseYaw && baseYaw == null && parts.length === 9) {
        // 无显式单位时，第 9 个数默认按角度处理
        baseYaw = wrapAngle((parts[8] || 0) * Math.PI / 180);
        dqParts = parts.slice(0, 8);
    }

    if (dqParts.length !== 8) {
        const yawHint = allowBaseYaw ? '，支持“8个DQ + yaw=30deg”或“9个数(最后一个为yaw角度)”' : '';
        return { ok: false, message: `${label}请输入 8 个数值（DQ 格式）${yawHint}` };
    }

    const normalized = normalizeDualQuaternionArray(dqParts);
    if (!normalized) {
        return { ok: false, message: `${label}无法单位化，请检查数值` };
    }

    return {
        ok: true,
        value: normalized,
        baseYaw,
    };
}

function parseOptionalBaseYawInput(inputEl, label = '底盘目标 yaw') {
    if (!inputEl) return { ok: true, value: null };
    const text = String(inputEl.value || '').trim();
    if (!text) return { ok: true, value: 0 };

    const match = text.match(/^\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*(deg|rad|°)?\s*$/i);
    if (!match) {
        return { ok: false, message: `${label}格式非法，请使用 30deg / 0.52rad / 30` };
    }

    const raw = Number(match[1]);
    if (!Number.isFinite(raw)) {
        return { ok: false, message: `${label}不是有效数字` };
    }

    const unit = (match[2] || 'deg').toLowerCase();
    const yawRad = unit === 'rad' ? raw : raw * Math.PI / 180;
    return { ok: true, value: wrapAngle(yawRad) };
}

/**
 * 初始化规划运动 UI 事件监听
 */
export function setupPlannedMotionListeners() {
    const planBtn = document.getElementById('planned-motion-plan-btn');
    const executeBtn = document.getElementById('planned-motion-execute-btn');
    const cancelBtn = document.getElementById('planned-motion-cancel-btn');
    const taskTypeSelect = document.getElementById('planned-motion-task-type-select');
    const absoluteTargetRow = document.getElementById('planned-motion-absolute-target-row');
    const baseYawRow = document.getElementById('planned-motion-base-yaw-row');
    const placementTargetsRow = document.getElementById('planned-motion-placement-targets');
    const thresholdRow = document.getElementById('planned-motion-threshold-row');
    const controlModeRow = document.getElementById('planned-motion-control-mode-row');
    const debugRow = document.getElementById('planned-motion-debug-row');
    const debugStepRow = document.getElementById('planned-motion-debug-step-row');
    const input = document.getElementById('planned-motion-dq-input');
    const baseYawInput = document.getElementById('planned-motion-base-yaw-input');
    const leftInput = document.getElementById('planned-motion-left-dq-input');
    const rightInput = document.getElementById('planned-motion-right-dq-input');
    const statusEl = document.getElementById('planned-motion-status');
    const thresholdInput = document.getElementById('planned-motion-threshold');
    const thresholdValue = document.getElementById('planned-motion-threshold-value');
    const copyCurrentBtn = document.getElementById('planned-motion-copy-current');
    const copyLeftCurrentBtn = document.getElementById('planned-motion-copy-left-current');
    const copyRightCurrentBtn = document.getElementById('planned-motion-copy-right-current');
    const debugStartBtn = document.getElementById('planned-motion-debug-start-btn');
    const debugWaistBtn = document.getElementById('planned-motion-debug-waist-btn');
    const debugBaseBtn = document.getElementById('planned-motion-debug-base-btn');
    const debugFinalBtn = document.getElementById('planned-motion-debug-final-btn');
    const debugDumpBtn = document.getElementById('planned-motion-debug-dump-btn');
    const debugFlowBtn = document.getElementById('planned-motion-debug-flow-btn');
    const debugStepLeftBtn = document.getElementById('planned-motion-debug-step-left-btn');
    const debugStepRightBtn = document.getElementById('planned-motion-debug-step-right-btn');
    const openMonitorWindowBtn = document.getElementById('planned-motion-joint-monitor-open-window');
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const plannedControlModeSelect = document.getElementById('planned-motion-control-mode-select');

    if (plannedControlModeSelect) {
        const mode = controlModeSelect?.value || (typeof window !== 'undefined' ? window.currentDualArmControlMode : null);
        plannedControlModeSelect.value = normalizePlannedMotionControlMode(mode);
    }

    const refreshTaskTypeUI = () => {
        const taskType = getPlannedMotionTaskType();
        const isSequenceTask = taskType === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE;
        if (absoluteTargetRow) absoluteTargetRow.style.display = isSequenceTask ? 'none' : '';
        if (baseYawRow) baseYawRow.style.display = '';
        if (placementTargetsRow) placementTargetsRow.style.display = isSequenceTask ? '' : 'none';
        if (thresholdRow) thresholdRow.style.display = '';
        if (controlModeRow) controlModeRow.style.display = isSequenceTask ? 'none' : '';
        if (debugRow) debugRow.style.display = '';
        if (debugStepRow) debugStepRow.style.display = isSequenceTask ? 'flex' : 'none';
        if (planBtn) {
            planBtn.textContent = isSequenceTask ? '规划任务' : '规划预览';
        }
    };

    refreshTaskTypeUI();

    const refreshDebugStepButtons = () => {
        const taskPlan = getDebugTaskPlan();
        const hasTaskPlan = !!taskPlan && Array.isArray(taskPlan.steps) && taskPlan.steps.length > 0;
        const taskType = getPlannedMotionTaskType();
        const shouldShowStepRow = hasTaskPlan || taskType === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE;
        if (debugStepRow) debugStepRow.style.display = shouldShowStepRow ? 'flex' : 'none';

        const idx = clampDebugTaskStepIndex(taskPlan, motionDebugSelectedTaskStepIndex);
        const leftStep = hasTaskPlan ? taskPlan.steps.find(s => s?.armIndex === 1) : null;
        const rightStep = hasTaskPlan ? taskPlan.steps.find(s => s?.armIndex === 2) : null;

        if (debugStepLeftBtn) {
            debugStepLeftBtn.textContent = leftStep?.armLabel ? `${leftStep.armLabel}流程` : '左臂流程';
            debugStepLeftBtn.disabled = !hasTaskPlan || !leftStep;
            debugStepLeftBtn.style.borderColor = (idx >= 0 && taskPlan.steps[idx]?.armIndex === 1) ? '#38bdf8' : '';
            debugStepLeftBtn.style.color = (idx >= 0 && taskPlan.steps[idx]?.armIndex === 1) ? '#e0f2fe' : '';
        }
        if (debugStepRightBtn) {
            debugStepRightBtn.textContent = rightStep?.armLabel ? `${rightStep.armLabel}流程` : '右臂流程';
            debugStepRightBtn.disabled = !hasTaskPlan || !rightStep;
            debugStepRightBtn.style.borderColor = (idx >= 0 && taskPlan.steps[idx]?.armIndex === 2) ? '#38bdf8' : '';
            debugStepRightBtn.style.color = (idx >= 0 && taskPlan.steps[idx]?.armIndex === 2) ? '#e0f2fe' : '';
        }
        if (debugFlowBtn) {
            debugFlowBtn.textContent = motionDebugFlowEnabled ? '流程可视化:开' : '流程可视化:关';
        }
    };

    let lastButtonSignature = null;
    let lastStatusClassState = null;
    let lastStatusText = null;

    const buildButtonRefreshSignature = () => {
        const running = isPlannedMotionRunning();
        const planned = hasPreparedPlan();
        const cancelable = running || planned || motionState === STATE.PLANNED;
        const debugSelection = getDebugPlanSelection();
        const hasDebugPlan = !!debugSelection;
        const debugPlanToken = debugSelection?.taskPlan?.createdAt
            || debugSelection?.plan?.createdAt
            || 0;
        const taskType = getPlannedMotionTaskType();
        return [
            running ? 1 : 0,
            planned ? 1 : 0,
            cancelable ? 1 : 0,
            hasDebugPlan ? 1 : 0,
            taskType,
            motionDebugSelectedTaskStepIndex,
            motionDebugFlowEnabled ? 1 : 0,
            debugPlanToken,
        ].join('|');
    };

    const refreshButtons = (force = false) => {
        const signature = buildButtonRefreshSignature();
        if (!force && signature === lastButtonSignature) {
            return;
        }
        lastButtonSignature = signature;

        const running = isPlannedMotionRunning();
        const planned = hasPreparedPlan();
        const cancelable = running || planned || motionState === STATE.PLANNED;
        const hasDebugPlan = !!getDebugPlanSelection();
        const debugDisabled = running || !hasDebugPlan;

        if (planBtn) planBtn.disabled = running;
        if (executeBtn) executeBtn.disabled = running || !planned;
        if (cancelBtn) cancelBtn.disabled = !cancelable;
        if (debugStartBtn) debugStartBtn.disabled = debugDisabled;
        if (debugWaistBtn) debugWaistBtn.disabled = debugDisabled;
        if (debugBaseBtn) debugBaseBtn.disabled = debugDisabled;
        if (debugFinalBtn) debugFinalBtn.disabled = debugDisabled;
        if (debugDumpBtn) debugDumpBtn.disabled = debugDisabled;
        if (debugFlowBtn) debugFlowBtn.disabled = running || !hasDebugPlan;
        refreshDebugStepButtons();
    };

    if (openMonitorWindowBtn) {
        openMonitorWindowBtn.addEventListener('click', () => {
            try {
                const monitorUrl = new URL('./joint-monitor.html', window.location.href);
                const popup = window.open(
                    monitorUrl.toString(),
                    'dq-vizlab-joint-monitor',
                    'popup=yes,width=1360,height=920,resizable=yes,scrollbars=yes'
                );
                if (popup && typeof popup.focus === 'function') {
                    popup.focus();
                    return;
                }
            } catch (error) {
                console.warn('规划运动：打开独立关节监控页失败:', error);
            }
            window.alert('未能打开独立监控窗口，请检查浏览器弹窗拦截设置。');
        });
    }

    // 状态回调
    setPlannedMotionStatusCallback((state, msg) => {
        if (statusEl) {
            const statusText = msg || state;
            if (statusText !== lastStatusText) {
                statusEl.textContent = statusText;
                lastStatusText = statusText;
            }

            if (state !== lastStatusClassState) {
                statusEl.className = 'planned-motion-status';
                if (state === STATE.ERROR) statusEl.classList.add('status-error');
                else if (state === STATE.DONE || state === STATE.PLANNED) statusEl.classList.add('status-done');
                else if (
                    state === STATE.PREVIEWING ||
                    state === STATE.WAIST_ALIGNING ||
                    state === STATE.BASE_MOVING ||
                    state === STATE.ARM_COMPENSATING
                ) {
                    statusEl.classList.add('status-running');
                }
                lastStatusClassState = state;
            }
        }

        refreshButtons();
    });

    // 阈值滑块
    if (thresholdInput && thresholdValue) {
        thresholdInput.addEventListener('input', () => {
            thresholdValue.textContent = thresholdInput.value;
            invalidatePreparedPlan('底盘阈值已修改，请重新规划预览');
        });
    }

    if (taskTypeSelect) {
        taskTypeSelect.addEventListener('change', () => {
            motionDebugSelectedTaskStepIndex = 0;
            refreshTaskTypeUI();
            invalidatePreparedPlan('任务类型已修改，请重新规划');
        });
    }

    // 输入变化：使已规划结果失效
    if (input) {
        input.addEventListener('input', () => {
            invalidatePreparedPlan('目标位姿已修改，请重新规划预览');
        });
    }
    if (baseYawInput) {
        baseYawInput.addEventListener('input', () => {
            invalidatePreparedPlan('底盘目标 yaw 已修改，请重新规划预览');
        });
    }
    if (leftInput) {
        leftInput.addEventListener('input', () => {
            invalidatePreparedPlan('左臂目标已修改，请重新规划任务');
        });
    }
    if (rightInput) {
        rightInput.addEventListener('input', () => {
            invalidatePreparedPlan('右臂目标已修改，请重新规划任务');
        });
    }

    if (plannedControlModeSelect) {
        plannedControlModeSelect.addEventListener('change', () => {
            invalidatePreparedPlan('规划运动手臂补偿模式已修改，请重新规划预览');
        });
    } else if (controlModeSelect) {
        controlModeSelect.addEventListener('change', () => {
            invalidatePreparedPlan('控制模式已修改，请重新规划预览');
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
                invalidatePreparedPlan('已填入当前位姿，请重新规划预览');
            }
        });
    }

    const fillArmCurrentPoseInput = (armIndex, targetInput, message) => {
        const DQ = getDQModule()?.DQWrapper;
        const vis = appState.visualizer;
        const qNow = (window.humanoidQ || appState.humanoidQ)?.slice();
        if (!DQ || !vis || !Array.isArray(qNow) || qNow.length === 0 || !targetInput) return;
        const basePose = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });
        const armWorldPose = getHumanoidArmWorldPose(DQ, vis, qNow, basePose, armIndex);
        if (!armWorldPose) return;
        targetInput.value = armWorldPose.map(v => v.toFixed(6)).join(' ');
        invalidatePreparedPlan(message);
    };

    if (copyLeftCurrentBtn) {
        copyLeftCurrentBtn.addEventListener('click', () => {
            fillArmCurrentPoseInput(1, leftInput, '已填入左臂当前位姿，请重新规划任务');
        });
    }

    if (copyRightCurrentBtn) {
        copyRightCurrentBtn.addEventListener('click', () => {
            fillArmCurrentPoseInput(2, rightInput, '已填入右臂当前位姿，请重新规划任务');
        });
    }

    // 规划预览按钮
    if (planBtn) {
        planBtn.addEventListener('click', () => {
            const threshold = thresholdInput ? Number(thresholdInput.value) / 100 : DEFAULT_PARAMS.baseThreshold;
            const uprightMode = detectUprightControlMode();
            const taskType = getPlannedMotionTaskType();
            const parsedBaseYaw = parseOptionalBaseYawInput(baseYawInput);
            if (!parsedBaseYaw.ok) {
                updateStatus(STATE.ERROR, parsedBaseYaw.message);
                return;
            }
            const requestedBaseYaw = parsedBaseYaw.value;

            if (taskType === TASK_TYPE.DUAL_ARM_SEQUENCE_PLACE) {
                motionDebugSelectedTaskStepIndex = 0;
                const parsedLeft = parseTargetInput(leftInput, '左臂放置点 A');
                if (!parsedLeft.ok) {
                    updateStatus(STATE.ERROR, parsedLeft.message);
                    return;
                }
                const parsedRight = parseTargetInput(rightInput, '右臂放置点 B');
                if (!parsedRight.ok) {
                    updateStatus(STATE.ERROR, parsedRight.message);
                    return;
                }

                planDualArmSequentialPlaceTask(parsedLeft.value, parsedRight.value, {
                    baseThreshold: threshold,
                    armIKKeepBodyZVertical: uprightMode,
                    baseTargetYaw: requestedBaseYaw,
                    playPreview: false,
                });
                return;
            }

            const parsed = parseTargetInput(input, '目标双四元数', { allowBaseYaw: true });
            if (!parsed.ok) {
                updateStatus(STATE.ERROR, parsed.message);
                return;
            }
            const requestedBaseYawAbsolute = requestedBaseYaw ?? parsed.baseYaw ?? null;

            planPlannedMotion(parsed.value, {
                baseThreshold: threshold,
                armIKKeepBodyZVertical: uprightMode,
                baseTargetYaw: requestedBaseYawAbsolute,
                playPreview: true
            });
        });
    }

    // 执行按钮：仅执行已规划结果
    if (executeBtn) {
        executeBtn.addEventListener('click', () => {
            executePlannedMotion();
        });
    }

    // 取消按钮
    if (cancelBtn) {
        cancelBtn.addEventListener('click', () => {
            cancelPlannedMotion();
        });
    }

    if (debugStartBtn) {
        debugStartBtn.addEventListener('click', () => {
            applyDebugPlanStage('start');
        });
    }

    if (debugWaistBtn) {
        debugWaistBtn.addEventListener('click', () => {
            applyDebugPlanStage('waist');
        });
    }

    if (debugBaseBtn) {
        debugBaseBtn.addEventListener('click', () => {
            applyDebugPlanStage('base');
        });
    }

    if (debugFinalBtn) {
        debugFinalBtn.addEventListener('click', () => {
            applyDebugPlanStage('final');
        });
    }

    if (debugDumpBtn) {
        debugDumpBtn.addEventListener('click', () => {
            dumpDebugPlanSummary();
        });
    }

    if (debugStepLeftBtn) {
        debugStepLeftBtn.addEventListener('click', () => {
            const taskPlan = getDebugTaskPlan();
            const idx = clampDebugTaskStepIndex(taskPlan, 0);
            if (idx < 0) return;
            motionDebugSelectedTaskStepIndex = idx;
            applyDebugPlanStage(motionDebugLastStageKey || 'start');
        });
    }

    if (debugStepRightBtn) {
        debugStepRightBtn.addEventListener('click', () => {
            const taskPlan = getDebugTaskPlan();
            if (!taskPlan || !Array.isArray(taskPlan.steps) || taskPlan.steps.length <= 0) return;
            const preferred = taskPlan.steps.findIndex(v => v?.armIndex === 2);
            const idx = clampDebugTaskStepIndex(taskPlan, preferred >= 0 ? preferred : (taskPlan.steps.length - 1));
            if (idx < 0) return;
            motionDebugSelectedTaskStepIndex = idx;
            applyDebugPlanStage(motionDebugLastStageKey || 'start');
        });
    }

    if (debugFlowBtn) {
        debugFlowBtn.addEventListener('click', () => {
            motionDebugFlowEnabled = !motionDebugFlowEnabled;
            if (!motionDebugFlowEnabled) {
                clearPreviewVisualization();
                updateStatus(STATE.PLANNED, 'Debug：已关闭求解流程可视化');
            } else if (!applyDebugPlanStage(motionDebugLastStageKey || 'start')) {
                updateStatus(STATE.ERROR, 'Debug：无可视化流程，请先规划');
            }
            refreshButtons(true);
        });
    }

    startPlannedMotionJointMonitorLoop();
    setPlannedMotionDebugInfo('-');
    updateStatus(STATE.IDLE, '就绪');
    refreshButtons(true);
}

/**
 * 注册回调
 */
export function registerPlannedMotionCallbacks() {
    callbacks.planPlannedMotion = planPlannedMotion;
    callbacks.executePlannedMotion = executePlannedMotion;
    callbacks.cancelPlannedMotion = cancelPlannedMotion;
    callbacks.setupPlannedMotionListeners = setupPlannedMotionListeners;
}
