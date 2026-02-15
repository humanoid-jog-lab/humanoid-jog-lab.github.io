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
    armRefineMaxPasses: 4,         // 手臂 IK 自动精修轮次（避免重复点击）
    finalPosTolMm: 0.5,            // 期望最终平移误差阈值（毫米）
    finalRotTolDeg: 0.2,           // 期望最终旋转误差阈值（度）
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

    // 底盘阶段判定只看 XY 偏差（底盘本身不可直接消除 Z 误差）
    const poseError = computePoseError(DQ, currentWorldAbs, motionTarget);
    console.log(
        `规划运动：偏差 XY=${(poseError.posErrXY * 100).toFixed(1)} cm, ` +
        `3D=${(poseError.posErr3D * 100).toFixed(1)} cm, ` +
        `rot=${poseError.rotErrDeg.toFixed(2)} deg, ` +
        `底盘阈值=${(motionParams.baseThreshold * 100).toFixed(1)} cm`
    );

    if (poseError.posErrXY > motionParams.baseThreshold) {
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
 * 计算绝对位姿误差（平移 + 旋转）
 * @returns {{posErr3D: number, posErrXY: number, rotErrRad: number, rotErrDeg: number}}
 */
function computePoseError(DQ, currentDQArray, targetDQArray) {
    const dqCur = DQ.createFromArray(currentDQArray);
    const dqTgt = DQ.createFromArray(targetDQArray);
    const tCur = DQ.getTranslation(dqCur); // [tw, tx, ty, tz]
    const tTgt = DQ.getTranslation(dqTgt);

    const dx = tTgt[1] - tCur[1];
    const dy = tTgt[2] - tCur[2];
    const dz = tTgt[3] - tCur[3];
    const posErr3D = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const posErrXY = Math.hypot(dx, dy);

    const qCur = DQ.getRotation(dqCur); // [w, x, y, z]
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

    // 底盘朝向：朝向目标方向（位移极小时保持当前朝向，避免 atan2(0,0) 导致抖动）
    const xyDist = Math.hypot(deltaX, deltaY);
    const baseTargetYaw = xyDist > 1e-6 ? Math.atan2(deltaY, deltaX) : (basePose.yaw || 0);

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
    let solvedError = null;
    try {
        // 自动多轮精修，避免“同一任务要点两次”
        const maxPasses = Math.max(1, Math.floor(Number(motionParams.armRefineMaxPasses) || 1));
        const posTolM = Math.max(0, (Number(motionParams.finalPosTolMm) || 0.5) / 1000);
        const rotTolRad = Math.max(0, (Number(motionParams.finalRotTolDeg) || 0.2) * Math.PI / 180);

        let workQ = initialQ.slice();
        let bestQ = null;
        let bestErr = null;

        for (let pass = 0; pass < maxPasses; pass++) {
            const ok = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                localTargetArray,
                workQ,
                enableLimitAvoidance,
                motionParams.armIKIncludeWaist
            );
            if (!ok) break;

            const qNow = (appState.humanoidQ || window.humanoidQ)?.slice();
            if (!qNow || qNow.length === 0) break;

            const errNow = evaluateHumanoidWorldError(DQ, vis, qNow, basePose, motionTarget);
            if (!errNow) break;

            if (
                !bestErr ||
                errNow.posErr3D < bestErr.posErr3D - 1e-9 ||
                (Math.abs(errNow.posErr3D - bestErr.posErr3D) <= 1e-9 && errNow.rotErrRad < bestErr.rotErrRad)
            ) {
                bestErr = errNow;
                bestQ = qNow.slice();
            }

            updateStatus(
                STATE.ARM_COMPENSATING,
                `手臂补偿中… 第${pass + 1}/${maxPasses}轮 | ` +
                `平移 ${(errNow.posErr3D * 1000).toFixed(2)} mm, 旋转 ${errNow.rotErrDeg.toFixed(3)} deg`
            );

            if (errNow.posErr3D <= posTolM && errNow.rotErrRad <= rotTolRad) {
                break;
            }

            workQ = qNow;
        }

        if (!bestQ) {
            updateStatus(STATE.ERROR, '手臂 IK 未收敛');
            motionTarget = null;
            return;
        }

        targetQ = bestQ;
        solvedError = bestErr;
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
