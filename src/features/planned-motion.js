/**
 * 规划运动模块
 *
 * 新流程（人形模式）：
 *  1) 先规划并播放残影预览（不执行真实底盘/关节动作）
 *  2) 用户确认后再执行真实运动（底盘阶段 + 手臂补偿阶段）
 */

import * as THREE from 'three';
import { appState, callbacks } from '../core/state.js';
import { getDQModule } from '../dqrobotics.js';

// ── 状态机状态 ──────────────────────────────────
const STATE = {
    IDLE: 'idle',
    PREVIEWING: 'previewing',
    PLANNED: 'planned',
    BASE_MOVING: 'base_moving',
    ARM_COMPENSATING: 'arm_compensating',
    DONE: 'done',
    ERROR: 'error',
};

// ── 默认参数 ──────────────────────────────────
const DEFAULT_PARAMS = {
    baseThreshold: 0.40,           // 底盘启动阈值（米）
    baseSettleCycles: 10,          // 底盘收敛判定次数
    armIKIncludeWaist: true,       // 手臂 IK 是否包含腰部关节
    armIKKeepBodyZVertical: false, // 绝对位姿控制时是否保持 body_link z 轴竖直
    pollInterval: 50,              // 状态轮询间隔（ms）
    armCompensationDuration: 0.5,  // 手臂补偿动画时长（秒）
    armCompensationEase: 'easeInOutCubic',
    armRefineMaxPasses: 4,         // 手臂 IK 自动精修轮次
    finalPosTolMm: 0.5,            // 期望最终平移误差阈值（毫米）
    finalRotTolDeg: 0.2,           // 期望最终旋转误差阈值（度）
    previewDuration: 1.2,          // 残影动画总时长（秒）
    previewBaseSamples: 16,        // 预览底盘阶段采样数
    previewArmSamples: 24,         // 预览手臂阶段采样数
    previewRobotSamples: 8,        // 机器人残影关键帧数量（越小越稀疏）
    baseMoveTrackingEnabled: true, // 底盘移动中是否做局部目标在线跟踪
    baseMoveTrackingPasses: 1,     // 每个轮询周期的在线跟踪迭代次数
    baseMoveTrackingBlend: 0.25,   // 在线跟踪平滑系数（0~1，越大越快）
    baseMoveTrackingMaxJointStepDeg: 2.0, // 在线跟踪每周期最大关节步长（度）
    baseMoveTrackingWarmupMs: 1200, // 在线跟踪暖启动时长（毫秒）
    baseMoveTrackingWarmupMinScale: 0.30, // 在线跟踪初始速度比例（0~1）
};

// ── 模块状态 ──────────────────────────────────
let motionState = STATE.IDLE;
let motionTarget = null;             // 当前执行目标 DQ (8 元素数组)
let motionParams = { ...DEFAULT_PARAMS };

let motionPreparedPlan = null;       // 已规划未执行
let motionActivePlan = null;         // 当前正在执行的计划

let motionPollTimer = null;
let motionArmAnimationId = null;
let motionPreviewAnimationId = null;

let motionPreviewGroup = null;
let motionPreviewItems = null;

let motionStatusCallback = null;     // UI 状态更新回调

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
    return !!motionPreparedPlan;
}

function detectUprightControlMode() {
    const modeSelect = document.getElementById('dual-arm-control-mode-select');
    const mode = modeSelect?.value || (typeof window !== 'undefined' ? window.currentDualArmControlMode : null);
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
 */
export function planPlannedMotion(targetDQArray, options = {}) {
    const validation = validatePlanInput(targetDQArray);
    if (!validation.ok) {
        updateStatus(STATE.ERROR, validation.message);
        return false;
    }

    // 开始新规划前，先清理当前运行态和旧残影
    stopAllRuntimeTimers();
    clearPreviewVisualization();
    motionPreparedPlan = null;
    motionActivePlan = null;
    motionTarget = null;

    motionParams = resolvePlannedMotionParams(options);

    const DQ = getDQModule()?.DQWrapper;
    const vis = appState.visualizer;

    const plan = buildMotionPlan(DQ, vis, targetDQArray, motionParams);
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
        motionTarget
    );

    stopAllRuntimeTimers();
    clearPreviewVisualization();

    motionPreparedPlan = null;
    motionActivePlan = null;
    motionTarget = null;

    if (hadAnyState) {
        updateStatus(STATE.IDLE, '已取消');
    } else {
        updateStatus(STATE.IDLE, '就绪');
    }
}

// ── 内部逻辑 ──────────────────────────────────

function validatePlanInput(targetDQArray) {
    if (!appState.humanoidMode || !appState.visualizer) {
        return { ok: false, message: '仅在人形模式下可用' };
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        return { ok: false, message: 'DQ 模块未加载' };
    }

    if (!Array.isArray(targetDQArray) || targetDQArray.length !== 8) {
        return { ok: false, message: '目标 DQ 格式无效（需要 8 个数）' };
    }

    if (targetDQArray.some(v => Number.isNaN(Number(v)))) {
        return { ok: false, message: '目标 DQ 包含非法数字' };
    }

    return { ok: true };
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
}

function buildMotionPlan(DQ, vis, targetDQArray, params) {
    const currentWorldAbs = getCurrentWorldAbsolutePose(DQ);
    if (!currentWorldAbs) {
        updateStatus(STATE.ERROR, '无法获取当前绝对位姿');
        return null;
    }

    const poseError = computePoseError(DQ, currentWorldAbs, targetDQArray);
    const baseStartPose = cloneBasePose(appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 });

    // 底盘阶段判定只看 XY 偏差
    const needBaseMove = poseError.posErrXY > params.baseThreshold;

    // 基于当前绝对位姿与目标绝对位姿，先计算底盘平移目标（x/y）
    const baseTargetXY = computeBaseTargetXY(DQ, currentWorldAbs, targetDQArray, baseStartPose, needBaseMove);

    // 底盘 yaw 多候选：自动挑选“机械臂转动更小”的方案
    const yawCandidates = buildBaseYawCandidates(DQ, currentWorldAbs, targetDQArray, baseStartPose, needBaseMove);
    let chosen = null;
    const candidateLogs = [];

    for (const yaw of yawCandidates) {
        const baseCandidate = {
            x: baseTargetXY.x,
            y: baseTargetXY.y,
            yaw,
        };

        const solveResult = solveArmTargetForBasePose(DQ, vis, targetDQArray, baseCandidate, params);
        if (!solveResult.ok) {
            candidateLogs.push(`yaw=${(yaw * 180 / Math.PI).toFixed(1)}deg -> IK失败`);
            continue;
        }

        const jointCost = computeJointMotionCost(solveResult.qStart, solveResult.qTarget);
        const yawDelta = Math.abs(wrapAngle(yaw - baseStartPose.yaw));
        const err = solveResult.solvedError || { posErr3D: Infinity, rotErrRad: Infinity, rotErrDeg: Infinity };
        const score =
            jointCost +
            0.02 * yawDelta +
            20 * (err.posErr3D || 0) +
            0.2 * (err.rotErrRad || 0);

        candidateLogs.push(
            `yaw=${(yaw * 180 / Math.PI).toFixed(1)}deg, ` +
            `joint=${jointCost.toFixed(3)}, ` +
            `err=${((err.posErr3D || 0) * 1000).toFixed(2)}mm/${(err.rotErrDeg || 0).toFixed(3)}deg, ` +
            `score=${score.toFixed(3)}`
        );

        if (!chosen || score < chosen.score) {
            chosen = {
                baseTargetPose: baseCandidate,
                solveResult,
                jointCost,
                score,
            };
        }
    }

    if (!chosen) {
        console.warn('规划运动 yaw 候选评估结果:\n' + candidateLogs.join('\n'));
        updateStatus(STATE.ERROR, '手臂 IK 规划失败');
        return null;
    }

    const baseTargetPose = chosen.baseTargetPose;
    const solveResult = chosen.solveResult;
    const baseMoveLocalTarget = computeLocalTargetForBasePose(DQ, vis, targetDQArray, baseTargetPose);

    const plan = {
        targetDQ: [...targetDQArray],
        params: { ...params },
        needBaseMove,
        poseError,
        baseStartPose,
        baseTargetPose,
        baseMoveLocalTarget,
        qStart: solveResult.qStart,
        qTarget: solveResult.qTarget,
        solvedError: solveResult.solvedError,
        createdAt: Date.now(),
    };

    console.log(
        `规划运动：偏差 XY=${(poseError.posErrXY * 100).toFixed(1)} cm, ` +
        `3D=${(poseError.posErr3D * 100).toFixed(1)} cm, ` +
        `rot=${poseError.rotErrDeg.toFixed(2)} deg, ` +
        `底盘阈值=${(params.baseThreshold * 100).toFixed(1)} cm, ` +
        `底盘阶段=${needBaseMove ? '需要' : '跳过'}, ` +
        `腰部竖直约束=${params.armIKKeepBodyZVertical ? '开' : '关'}, ` +
        `选中yaw=${(baseTargetPose.yaw * 180 / Math.PI).toFixed(1)} deg, ` +
        `jointCost=${chosen.jointCost.toFixed(3)}`
    );
    console.log('规划运动 yaw 候选评估:\n' + candidateLogs.join('\n'));

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

    const absLocal = vis._getHumanoidAbsolutePose(q);
    if (!absLocal) return null;

    const basePose = appState.humanoidBasePose || { x: 0, y: 0, yaw: 0 };
    return vis._composeHumanoidWorldPose(basePose, absLocal) || absLocal;
}

/**
 * 计算绝对位姿误差（平移 + 旋转）
 * @returns {{posErr3D: number, posErrXY: number, rotErrRad: number, rotErrDeg: number}}
 */
function computePoseError(DQ, currentDQArray, targetDQArray) {
    const dqCur = DQ.createFromArray(currentDQArray);
    const dqTgt = DQ.createFromArray(targetDQArray);
    const tCur = DQ.getTranslation(dqCur);
    const tTgt = DQ.getTranslation(dqTgt);

    const dx = tTgt[1] - tCur[1];
    const dy = tTgt[2] - tCur[2];
    const dz = tTgt[3] - tCur[3];
    const posErr3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const posErrXY = Math.hypot(dx, dy);

    const qCur = DQ.getRotation(dqCur);
    const qTgt = DQ.getRotation(dqTgt);
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

/**
 * 在给定底盘位姿下评估某组关节对应的世界绝对位姿误差
 */
function evaluateHumanoidWorldError(DQ, vis, q, basePose, worldTargetDQArray) {
    const localAbs = vis?._getHumanoidAbsolutePose?.(q);
    if (!localAbs) return null;
    const worldAbs = vis?._composeHumanoidWorldPose?.(basePose, localAbs) || localAbs;
    return computePoseError(DQ, worldAbs, worldTargetDQArray);
}

function computeLocalTargetForBasePose(DQ, vis, worldTargetDQArray, basePose) {
    if (!DQ || !vis || !Array.isArray(worldTargetDQArray) || worldTargetDQArray.length !== 8) return null;
    const baseDQ = vis._makeHumanoidBaseDQ?.(basePose);
    if (!baseDQ) return null;
    const localTargetDQ = DQ.multiply(DQ.inverse(baseDQ), DQ.createFromArray(worldTargetDQArray));
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

    const dqCur = DQ.createFromArray(currentWorldAbs);
    const dqTgt = DQ.createFromArray(targetWorldAbs);
    const tCur = DQ.getTranslation(dqCur);
    const tTgt = DQ.getTranslation(dqTgt);

    const deltaX = tTgt[1] - tCur[1];
    const deltaY = tTgt[2] - tCur[2];

    return {
        x: (basePose.x || 0) + deltaX,
        y: (basePose.y || 0) + deltaY,
    };
}

function extractYawFromDQArray(DQ, dqArray) {
    if (!Array.isArray(dqArray) || dqArray.length !== 8) return 0;
    const dq = DQ.createFromArray(dqArray);
    const q = DQ.getRotation(dq); // [w, x, y, z]
    const w = q[0] || 0;
    const x = q[1] || 0;
    const y = q[2] || 0;
    const z = q[3] || 0;
    const yaw = Math.atan2(
        2 * (w * z + x * y),
        1 - 2 * (y * y + z * z)
    );
    return wrapAngle(yaw);
}

function pushUniqueYaw(arr, yaw, eps = 1e-4) {
    const y = wrapAngle(yaw);
    const exists = arr.some(v => Math.abs(wrapAngle(v - y)) <= eps);
    if (!exists) arr.push(y);
}

function buildBaseYawCandidates(DQ, currentWorldAbs, targetWorldAbs, basePose, needBaseMove) {
    const candidates = [];
    const yawCurrent = basePose.yaw || 0;

    // 候选1：保持当前 yaw（通常会显著减少机械臂补偿角度）
    pushUniqueYaw(candidates, yawCurrent);

    if (!needBaseMove) return candidates;

    // 候选2：朝向平移方向（原有策略）
    const dqCur = DQ.createFromArray(currentWorldAbs);
    const dqTgt = DQ.createFromArray(targetWorldAbs);
    const tCur = DQ.getTranslation(dqCur);
    const tTgt = DQ.getTranslation(dqTgt);
    const dx = tTgt[1] - tCur[1];
    const dy = tTgt[2] - tCur[2];
    const xyDist = Math.hypot(dx, dy);
    const yawFaceMove = xyDist > 1e-6 ? Math.atan2(dy, dx) : yawCurrent;
    pushUniqueYaw(candidates, yawFaceMove);

    // 候选3：对齐目标绝对位姿的 yaw（让底盘承担更多朝向补偿）
    const yawTarget = extractYawFromDQArray(DQ, targetWorldAbs);
    pushUniqueYaw(candidates, yawTarget);

    // 候选4：当前与目标 yaw 中间值（折中）
    pushUniqueYaw(candidates, yawCurrent + 0.5 * wrapAngle(yawTarget - yawCurrent));

    return candidates;
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

function computeTrackingGoalQForPreview(vis, localTargetArray, qStart, params, previewBasePose = null, fallbackQ = null) {
    if (!vis || !Array.isArray(localTargetArray) || localTargetArray.length !== 8) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart?.length) return fallbackQ.slice();
        return Array.isArray(qStart) ? qStart.slice() : [];
    }
    if (!Array.isArray(qStart) || qStart.length === 0) return [];

    const qGlobalBefore = (window.humanoidQ || appState.humanoidQ)?.slice();
    if (!Array.isArray(qGlobalBefore) || qGlobalBefore.length === 0) {
        if (Array.isArray(fallbackQ) && fallbackQ.length === qStart.length) return fallbackQ.slice();
        return qStart.slice();
    }

    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
    const passes = Math.max(1, Math.floor(Number(params.baseMoveTrackingPasses) || 1));

    const solveTrackingGoal = () => {
        // 在不改变最终真实状态的前提下，临时求一帧在线跟踪目标。
        restoreHumanoidQ(vis, qStart);
        const qRef = window.humanoidQ || appState.humanoidQ;
        let okAny = false;
        for (let i = 0; i < passes; i++) {
            const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                localTargetArray,
                qRef,
                enableLimitAvoidance,
                params.armIKIncludeWaist,
                !!params.armIKKeepBodyZVertical
            );
            if (!ok) break;
            okAny = true;
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
 * 仅用于“规划阶段”的 IK 求解：
 * - 求出目标关节 qTarget
 * - 规划结束后恢复当前真实姿态
 */
function solveArmTargetForBasePose(DQ, vis, worldTargetDQArray, basePose, params) {
    if (!vis || !worldTargetDQArray) {
        return { ok: false, message: '可视化器或目标丢失' };
    }

    const baseDQ = vis._makeHumanoidBaseDQ?.(basePose);
    if (!baseDQ) {
        return { ok: false, message: '无法构建底盘 DQ' };
    }

    const baseDQInv = DQ.inverse(baseDQ);
    const worldTargetDQ = DQ.createFromArray(worldTargetDQArray);
    const localTargetDQ = DQ.multiply(baseDQInv, worldTargetDQ);
    const localTargetArray = DQ.toArray(localTargetDQ);

    const qRef = window.humanoidQ || appState.humanoidQ;
    if (!qRef || qRef.length === 0) {
        return { ok: false, message: '人形关节未初始化' };
    }

    const initialQ = qRef.slice();
    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

    let bestQ = null;
    let bestErr = null;

    try {
        const maxPasses = Math.max(1, Math.floor(Number(params.armRefineMaxPasses) || 1));
        const posTolM = Math.max(0, (Number(params.finalPosTolMm) || 0.5) / 1000);
        const rotTolRad = Math.max(0, (Number(params.finalRotTolDeg) || 0.2) * Math.PI / 180);

        let workQ = initialQ.slice();

        for (let pass = 0; pass < maxPasses; pass++) {
            const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                localTargetArray,
                workQ,
                enableLimitAvoidance,
                params.armIKIncludeWaist,
                !!params.armIKKeepBodyZVertical
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
            restoreHumanoidQ(vis, initialQ);
            return { ok: false, message: '手臂 IK 未收敛' };
        }

        restoreHumanoidQ(vis, initialQ);
        return {
            ok: true,
            qStart: initialQ,
            qTarget: bestQ,
            solvedError: bestErr,
        };
    } catch (e) {
        console.error('规划运动 - 手臂 IK 规划失败:', e);
        restoreHumanoidQ(vis, initialQ);
        return { ok: false, message: `手臂 IK 失败: ${e.message}` };
    }
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

    if (!motionPreparedPlan) {
        updateStatus(STATE.ERROR, '请先点击“规划预览”生成计划');
        return false;
    }

    if (!isPreparedPlanStillValid(motionPreparedPlan)) {
        motionPreparedPlan = null;
        clearPreviewVisualization();
        updateStatus(STATE.ERROR, '当前姿态已变化，请先重新规划预览');
        return false;
    }

    clearPreviewVisualization();
    stopAllRuntimeTimers();

    motionActivePlan = motionPreparedPlan;
    motionPreparedPlan = null;
    motionTarget = [...motionActivePlan.targetDQ];
    motionParams = resolvePlannedMotionParams(motionActivePlan.params || {});

    if (motionActivePlan.needBaseMove) {
        startBaseMovement(motionActivePlan);
    } else {
        startArmCompensation(motionActivePlan);
    }

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

/**
 * 阶段 1：启动底盘移动
 */
function startBaseMovement(plan) {
    const baseTarget = plan.baseTargetPose;

    appState.humanoidBaseTargetPose.x = baseTarget.x;
    appState.humanoidBaseTargetPose.y = baseTarget.y;
    appState.humanoidBaseTargetPose.yaw = baseTarget.yaw;
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidBaseReverseDrive = false;

    // 确保底盘控制模式为 point-to-point
    appState.humanoidBaseControlMode = 'point';
    plan.baseMoveTrackingTick = 0;

    // 确保底盘控制循环在运行
    if (!appState.humanoidBaseControlTimer) {
        callbacks.startHumanoidBaseControlLoop?.();
    }

    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    updateStatus(STATE.BASE_MOVING, `底盘移动中… 偏差 ${(Math.hypot(dx, dy) * 100).toFixed(1)} cm`);

    motionPollTimer = setInterval(() => {
        pollBaseMovement();
    }, motionParams.pollInterval);
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

    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    const posErr = Math.hypot(dx, dy);
    let status = `底盘移动中… 剩余 ${(posErr * 100).toFixed(1)} cm`;
    if (trackingErr) {
        status += ` | 跟踪误差 ${(trackingErr.posErr3D * 1000).toFixed(1)} mm / ${trackingErr.rotErrDeg.toFixed(2)} deg`;
    }
    updateStatus(STATE.BASE_MOVING, status);

    if (converged) {
        clearInterval(motionPollTimer);
        motionPollTimer = null;
        console.log('规划运动：底盘到位，开始手臂补偿');
        startArmCompensation(motionActivePlan);
    }
}

/**
 * 底盘移动中在线跟踪：
 * - 使用“相对底盘的固定局部目标”（规划时在底盘终点定义）
 * - 底盘停下后再进入世界坐标终修（startArmCompensation 内）
 */
function trackUpperBodyDuringBaseMove(plan) {
    const vis = appState.visualizer;
    const DQ = getDQModule()?.DQWrapper;
    if (!vis || !DQ || !plan) return null;

    const qRef = window.humanoidQ || appState.humanoidQ;
    if (!qRef || qRef.length === 0) return null;

    const worldTarget = motionTarget || plan.targetDQ;
    if (!Array.isArray(worldTarget) || worldTarget.length !== 8) return null;

    // 优先使用“规划时定义在底盘终点坐标系下”的局部目标（符合用户期望流程）
    let localTargetArray = Array.isArray(plan.baseMoveLocalTarget) && plan.baseMoveLocalTarget.length === 8
        ? plan.baseMoveLocalTarget
        : null;

    // 兜底：如果计划里没有，退化为当前底盘下的世界目标映射
    if (!localTargetArray) {
        const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
        localTargetArray = computeLocalTargetForBasePose(DQ, vis, worldTarget, basePoseNow);
        if (!localTargetArray) return null;
    }

    const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
    const passes = Math.max(1, Math.floor(Number(motionParams.baseMoveTrackingPasses) || 1));
    const qBefore = qRef.slice();
    let okAny = false;

    for (let i = 0; i < passes; i++) {
        const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
            localTargetArray,
            qRef,
            enableLimitAvoidance,
            motionParams.armIKIncludeWaist,
            !!motionParams.armIKKeepBodyZVertical
        );
        if (!ok) break;
        okAny = true;
    }

    if (!okAny) return null;

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
    const qSmooth = stepQTowardsGoal(qBefore, qSolved, blend, maxStepRad, warmupScale);
    restoreHumanoidQ(vis, qSmooth);

    const basePoseNow = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
    return evaluateHumanoidWorldError(DQ, vis, qSmooth, basePoseNow, worldTarget);
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
 * 阶段 2：按规划结果做手臂补偿动画
 */
function startArmCompensation(plan) {
    updateStatus(STATE.ARM_COMPENSATING, '手臂补偿中…');

    const vis = appState.visualizer;
    if (!vis || !plan || !plan.qTarget || !motionTarget) {
        updateStatus(STATE.ERROR, '可视化器或规划目标丢失');
        motionActivePlan = null;
        motionTarget = null;
        return;
    }

    const DQ = getDQModule()?.DQWrapper;
    if (!DQ) {
        updateStatus(STATE.ERROR, 'DQ 模块不可用');
        motionActivePlan = null;
        motionTarget = null;
        return;
    }

    const qRef = appState.humanoidQ || window.humanoidQ;
    if (!qRef || qRef.length === 0) {
        updateStatus(STATE.ERROR, '人形关节未初始化');
        motionActivePlan = null;
        motionTarget = null;
        return;
    }

    const initialQ = qRef.slice();
    let targetQ = plan.qTarget.slice();
    let solvedError = plan.solvedError || null;

    if (targetQ.length !== initialQ.length) {
        updateStatus(STATE.ERROR, '规划关节维度与当前机器人不匹配，请重新规划');
        motionActivePlan = null;
        motionTarget = null;
        return;
    }

    // 执行前精修：以“当前真实底盘姿态”重新求一次 IK，避免必须二次规划才能到位。
    const executionBasePose = cloneBasePose(appState.humanoidBasePose || plan.baseTargetPose || { x: 0, y: 0, yaw: 0 });
    const refineResult = solveArmTargetForBasePose(DQ, vis, motionTarget, executionBasePose, motionParams);
    if (refineResult.ok && Array.isArray(refineResult.qTarget) && refineResult.qTarget.length === initialQ.length) {
        targetQ = refineResult.qTarget.slice();
        solvedError = refineResult.solvedError || solvedError;
        plan.qTarget = targetQ.slice();
        plan.solvedError = solvedError;
        plan.baseTargetPose = executionBasePose;
    } else if (refineResult.message) {
        console.warn('规划运动：执行前 IK 精修失败，回退使用规划阶段目标。原因:', refineResult.message);
    }

    const durationMs = Math.max(1, (motionParams.armCompensationDuration || 0.5) * 1000);
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

            const finalWorldAbs = getCurrentWorldAbsolutePose(DQ);
            if (finalWorldAbs && motionTarget) {
                const finalErr = computePoseError(DQ, finalWorldAbs, motionTarget);
                updateStatus(
                    STATE.DONE,
                    `完成！残余误差: 平移 ${(finalErr.posErr3D * 1000).toFixed(2)} mm, ` +
                    `旋转 ${finalErr.rotErrDeg.toFixed(3)} deg`
                );
            } else if (solvedError) {
                updateStatus(
                    STATE.DONE,
                    `完成！残余误差: 平移 ${(solvedError.posErr3D * 1000).toFixed(2)} mm, ` +
                    `旋转 ${solvedError.rotErrDeg.toFixed(3)} deg`
                );
            } else {
                updateStatus(STATE.DONE, '手臂补偿完成');
            }

            motionActivePlan = null;
            motionTarget = null;
            return;
        }

        motionArmAnimationId = requestAnimationFrame(animateArm);
    }

    motionArmAnimationId = requestAnimationFrame(animateArm);
}

function getWorldAbsolutePoseForSample(DQ, vis, q, basePose) {
    const localAbs = vis?._getHumanoidAbsolutePose?.(q);
    if (!localAbs) return null;
    return vis?._composeHumanoidWorldPose?.(basePose, localAbs) || localAbs;
}

function extractPositionFromDQ(DQ, dqArray) {
    const dq = DQ.createFromArray(dqArray);
    const t = DQ.getTranslation(dq);
    return new THREE.Vector3(t[1] || 0, t[2] || 0, t[3] || 0);
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
    const previewLocalTarget = (Array.isArray(plan.baseMoveLocalTarget) && plan.baseMoveLocalTarget.length === 8)
        ? plan.baseMoveLocalTarget
        : computeLocalTargetForBasePose(DQ, vis, plan.targetDQ, plan.baseTargetPose);

    if (baseSteps > 0) {
        // 用与真实底盘控制器一致的近似轨迹做预览，避免 yaw 残影与真实执行不一致。
        const simTrajectory = simulateBaseTrajectoryForPreview(
            plan.baseStartPose,
            plan.baseTargetPose,
            plan.params.baseSettleCycles
        );
        const sampledTrajectory = sampleTrajectoryByCountWithIndices(simTrajectory, baseSteps + 1);
        let prevRawIdx = sampledTrajectory[0]?.idx ?? 0;
        for (let i = 0; i < sampledTrajectory.length; i++) {
            const b = sampledTrajectory[i]?.pose;
            const rawIdx = sampledTrajectory[i]?.idx ?? prevRawIdx;
            if (!b) continue;
            // 预览中模拟“底盘运动时上肢在线跟踪”，避免与真实执行差异过大。
            if (i > 0 && qTracked.length === plan.qTarget.length && Array.isArray(previewLocalTarget)) {
                const qGoalNow = computeTrackingGoalQForPreview(
                    vis,
                    previewLocalTarget,
                    qTracked,
                    plan.params,
                    b,
                    plan.qTarget
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
                    previewTrackingTick += 1;
                }
            }
            addSample(b, qTracked.length ? qTracked : plan.qStart, 'base');
            prevRawIdx = rawIdx;
        }
    }

    const armStartIdx = baseSteps > 0 ? 1 : 0;
    const qArmStart = qTracked.length === plan.qTarget.length ? qTracked.slice() : plan.qStart.slice();
    for (let i = armStartIdx; i <= armSteps; i++) {
        const t = i / armSteps;
        const s = easeInOutCubic(t);
        const qInterp = qArmStart.map((v, idx) => lerpJoint(v, plan.qTarget[idx], s));
        addSample(plan.baseTargetPose, qInterp, 'arm');
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

function clearPreviewVisualization() {
    if (motionPreviewGroup && appState.visualizer?.scene) {
        appState.visualizer.scene.remove(motionPreviewGroup);
    }

    if (motionPreviewGroup) {
        disposeObjectTree(motionPreviewGroup);
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

    clearPreviewVisualization();
    const ok = createPreviewVisualization(samples, plan.params);
    if (!ok || !motionPreviewItems) return false;

    updateStatus(STATE.PREVIEWING, '规划中… 正在播放机器人残影预览');

    const total = motionPreviewItems.robotGhosts.length;
    const durationMs = Math.max(1, (plan.params.previewDuration || 1.2) * 1000);
    const startTs = performance.now();

    function tick(now) {
        if (motionState !== STATE.PREVIEWING) {
            motionPreviewAnimationId = null;
            return;
        }

        const t = Math.min(1, (now - startTs) / durationMs);
        const revealT = easeInOutCubic(t);
        const revealCount = Math.max(0, Math.floor(revealT * (total - 1)));

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
    if (!motionPreparedPlan) return;

    motionPreparedPlan = null;
    clearPreviewVisualization();

    if (motionState === STATE.PLANNED || motionState === STATE.DONE || motionState === STATE.ERROR || motionState === STATE.IDLE) {
        updateStatus(STATE.IDLE, message);
    }
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

function parseTargetInput(inputEl) {
    if (!inputEl) {
        return { ok: false, message: '未找到目标输入框' };
    }

    const text = inputEl.value.trim();
    if (!text) {
        return { ok: false, message: '请输入目标双四元数' };
    }

    const parts = text.split(/[\s,]+/).filter(Boolean).map(Number);
    if (parts.length !== 8 || parts.some(Number.isNaN)) {
        return { ok: false, message: '请输入 8 个数值（DQ 格式）' };
    }

    return { ok: true, value: parts };
}

/**
 * 初始化规划运动 UI 事件监听
 */
export function setupPlannedMotionListeners() {
    const planBtn = document.getElementById('planned-motion-plan-btn');
    const executeBtn = document.getElementById('planned-motion-execute-btn');
    const cancelBtn = document.getElementById('planned-motion-cancel-btn');
    const input = document.getElementById('planned-motion-dq-input');
    const statusEl = document.getElementById('planned-motion-status');
    const thresholdInput = document.getElementById('planned-motion-threshold');
    const thresholdValue = document.getElementById('planned-motion-threshold-value');
    const copyCurrentBtn = document.getElementById('planned-motion-copy-current');
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');

    const refreshButtons = () => {
        const running = isPlannedMotionRunning();
        const planned = hasPreparedPlan();
        const cancelable = running || planned || motionState === STATE.PLANNED;

        if (planBtn) planBtn.disabled = running;
        if (executeBtn) executeBtn.disabled = running || !planned;
        if (cancelBtn) cancelBtn.disabled = !cancelable;
    };

    // 状态回调
    setPlannedMotionStatusCallback((state, msg) => {
        if (statusEl) {
            statusEl.textContent = msg || state;
            statusEl.className = 'planned-motion-status';
            if (state === STATE.ERROR) statusEl.classList.add('status-error');
            else if (state === STATE.DONE || state === STATE.PLANNED) statusEl.classList.add('status-done');
            else if (
                state === STATE.PREVIEWING ||
                state === STATE.BASE_MOVING ||
                state === STATE.ARM_COMPENSATING
            ) {
                statusEl.classList.add('status-running');
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

    // 输入变化：使已规划结果失效
    if (input) {
        input.addEventListener('input', () => {
            invalidatePreparedPlan('目标位姿已修改，请重新规划预览');
        });
    }

    if (controlModeSelect) {
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

    // 规划预览按钮
    if (planBtn) {
        planBtn.addEventListener('click', () => {
            const parsed = parseTargetInput(input);
            if (!parsed.ok) {
                updateStatus(STATE.ERROR, parsed.message);
                return;
            }

            const threshold = thresholdInput ? Number(thresholdInput.value) / 100 : DEFAULT_PARAMS.baseThreshold;
            const uprightMode = detectUprightControlMode();
            planPlannedMotion(parsed.value, {
                baseThreshold: threshold,
                armIKKeepBodyZVertical: uprightMode,
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

    updateStatus(STATE.IDLE, '就绪');
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
