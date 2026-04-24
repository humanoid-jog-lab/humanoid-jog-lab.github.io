/**
 * 人形机器人底盘控制 - 差速驱动控制器
 */

import { DEG_TO_RAD } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { wrapAngle, clampValue, quaternionToYaw } from '../utils/math.js';
import { getDQModule } from '../dqrobotics.js';
import { computeRegulatedPurePursuitControl } from './pure-pursuit.js';
import { buildHumanoidBasePoseDQArray } from './base-pose-dq.js';

// ──────────────────────────────────────────────────────────────
// 配置读取
// ──────────────────────────────────────────────────────────────

export function getHumanoidBasePoseFromConfig(config) {
    const pose = { x: 0, y: 0, yaw: 0 };
    const mobileInitialQ = config?.mobile_base?.initial_q;
    if (Array.isArray(mobileInitialQ) && mobileInitialQ.length >= 3) {
        return {
            x: Number(mobileInitialQ[0]) || 0,
            y: Number(mobileInitialQ[1]) || 0,
            yaw: Number(mobileInitialQ[2]) || 0
        };
    }

    const basePose = config?.base_pose || config?.mdh_extraction?.base_pose;
    if (!Array.isArray(basePose) || basePose.length < 2) {
        return pose;
    }

    pose.x = Number(basePose[0]) || 0;
    pose.y = Number(basePose[1]) || 0;

    if (basePose.length === 7) {
        const [qw, qx, qy, qz] = basePose;
        pose.yaw = quaternionToYaw(
            Number(qw) || 1,
            Number(qx) || 0,
            Number(qy) || 0,
            Number(qz) || 0
        );
    } else if (basePose.length === 8) {
        const dqModule = getDQModule();
        const DQWrapper = dqModule?.DQWrapper;
        if (DQWrapper) {
            try {
                const dqObj = DQWrapper.normalize(DQWrapper.createFromArray(basePose));
                const t = DQWrapper.getTranslation(dqObj);
                const r = DQWrapper.getRotation(dqObj);
                pose.x = Number(t[1]) || pose.x;
                pose.y = Number(t[2]) || pose.y;
                pose.yaw = quaternionToYaw(
                    Number(r[0]) || 1,
                    Number(r[1]) || 0,
                    Number(r[2]) || 0,
                    Number(r[3]) || 0
                );
            } catch (error) {
                console.warn('解析 humanoid base_pose(DQ) 失败，回退到平移默认值:', error);
            }
        }
    }

    return pose;
}

export function getHumanoidDiffDriveParamsFromConfig(config) {
    const mobile = config?.mobile_base || {};
    const params = { ...appState.humanoidBaseControlParams };
    const readNumber = (v, fallback) => (Number.isFinite(Number(v)) ? Number(v) : fallback);

    params.wheelRadius = readNumber(
        mobile.wheel_radius ?? mobile.radius ?? mobile.r,
        params.wheelRadius
    );
    params.wheelDistance = readNumber(
        mobile.distance_between_wheels ?? mobile.wheel_distance ?? mobile.wheel_base ?? mobile.l,
        params.wheelDistance
    );

    const ctrl = mobile.controller || {};
    params.kRho = readNumber(ctrl.k_rho ?? ctrl.k_forward, params.kRho);
    params.kAlpha = readNumber(ctrl.k_alpha, params.kAlpha);
    params.kBeta = readNumber(ctrl.k_beta, params.kBeta);
    params.kYawFinal = readNumber(ctrl.k_yaw_final ?? ctrl.k_yaw, params.kYawFinal);
    params.maxLinear = readNumber(ctrl.max_linear, params.maxLinear);
    params.maxAngular = readNumber(ctrl.max_angular, params.maxAngular);
    params.maxWheelSpeed = readNumber(ctrl.max_wheel_speed, params.maxWheelSpeed);
    params.minWheelSpeed = readNumber(ctrl.min_wheel_speed, params.minWheelSpeed);
    params.tolPos = readNumber(ctrl.tol_pos, params.tolPos);
    params.tolYaw = readNumber(ctrl.tol_yaw, params.tolYaw);
    params.settleCycles = Math.max(1, Math.floor(readNumber(ctrl.settle_cycles, params.settleCycles)));

    // 如有 rpp 子项，读取 Regulated Pure Pursuit 参数
    const rpp = mobile.rpp || mobile.regulated_pure_pursuit || ctrl.rpp || {};
    const rppP = appState.humanoidBaseRPPParams;
    rppP.lookahead_dist = readNumber(rpp.lookahead_dist, rppP.lookahead_dist);
    rppP.min_lookahead_dist = readNumber(rpp.min_lookahead_dist, rppP.min_lookahead_dist);
    rppP.max_lookahead_dist = readNumber(rpp.max_lookahead_dist, rppP.max_lookahead_dist);
    rppP.lookahead_time = readNumber(rpp.lookahead_time, rppP.lookahead_time);
    rppP.max_linear_vel = readNumber(rpp.max_linear_vel ?? rpp.max_linear, rppP.max_linear_vel);
    rppP.min_linear_vel = readNumber(rpp.min_linear_vel ?? rpp.min_linear, rppP.min_linear_vel);
    rppP.max_angular_vel = readNumber(rpp.max_angular_vel ?? rpp.max_angular, rppP.max_angular_vel);
    rppP.max_angular_accel = readNumber(rpp.max_angular_accel, rppP.max_angular_accel);
    rppP.regulated_linear_scaling_min_radius = readNumber(rpp.regulated_linear_scaling_min_radius ?? rpp.min_radius, rppP.regulated_linear_scaling_min_radius);
    rppP.regulated_linear_scaling_min_speed = readNumber(rpp.regulated_linear_scaling_min_speed ?? rpp.min_speed, rppP.regulated_linear_scaling_min_speed);
    rppP.rotate_to_heading_min_angle = readNumber(rpp.rotate_to_heading_min_angle, rppP.rotate_to_heading_min_angle);
    rppP.rotate_to_heading_angular_vel = readNumber(rpp.rotate_to_heading_angular_vel, rppP.rotate_to_heading_angular_vel);
    rppP.goal_dist_tol = readNumber(rpp.goal_dist_tol ?? rpp.goal_tolerance, rppP.goal_dist_tol);
    rppP.approach_velocity_scaling_dist = readNumber(rpp.approach_velocity_scaling_dist, rppP.approach_velocity_scaling_dist);
    rppP.min_approach_linear_velocity = readNumber(rpp.min_approach_linear_velocity, rppP.min_approach_linear_velocity);
    if (rpp.use_velocity_scaled_lookahead !== undefined) rppP.use_velocity_scaled_lookahead = !!rpp.use_velocity_scaled_lookahead;
    if (rpp.use_regulated_linear_velocity_scaling !== undefined) rppP.use_regulated_linear_velocity_scaling = !!rpp.use_regulated_linear_velocity_scaling;
    if (rpp.use_rotate_to_heading !== undefined) rppP.use_rotate_to_heading = !!rpp.use_rotate_to_heading;
    if (rpp.allow_reversing !== undefined) rppP.allow_reversing = !!rpp.allow_reversing;

    return params;
}

// ──────────────────────────────────────────────────────────────
// 差速驱动 rho-alpha-beta 控制器
// ──────────────────────────────────────────────────────────────

function computeHumanoidDiffDriveControl() {
    const p = appState.humanoidBaseControlParams;
    const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
    const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
    const theta = appState.humanoidBasePose.yaw || 0;
    const thetaTarget = appState.humanoidBaseTargetPose.yaw || 0;
    const posErr = Math.hypot(dx, dy);
    const eYaw = wrapAngle(thetaTarget - theta);

    let v = 0;
    let omega = 0;
    let goalReached = false;

    if (posErr < p.tolPos) {
        appState.humanoidBaseReverseDrive = false;
        v = 0;

        if (Math.abs(eYaw) < p.tolYaw) {
            omega = 0;
            goalReached = true;
        } else {
            omega = p.kYawFinal * eYaw;
            omega = clampValue(omega, -p.maxAngular, p.maxAngular);
        }
    } else {
        const headingToTarget = Math.atan2(dy, dx);
        const alphaForward = wrapAngle(headingToTarget - theta);
        const alphaReverse = wrapAngle(alphaForward - Math.PI);

        const reverseEnter = 100 * DEG_TO_RAD;
        const reverseExit = 70 * DEG_TO_RAD;
        if (appState.humanoidBaseReverseDrive) {
            if (Math.abs(alphaReverse) < reverseExit) {
                // keep reverse
            } else {
                appState.humanoidBaseReverseDrive = false;
            }
        } else if (Math.abs(alphaForward) > reverseEnter) {
            appState.humanoidBaseReverseDrive = true;
        }

        const alpha = appState.humanoidBaseReverseDrive ? alphaReverse : alphaForward;
        const rhoSigned = appState.humanoidBaseReverseDrive ? -posErr : posErr;
        const beta = wrapAngle(thetaTarget - theta - alpha);

        const yawBlendDist = Math.max(0.15, 5 * p.tolPos);
        const yawBlend = clampValue((yawBlendDist - posErr) / yawBlendDist, 0, 1);

        v = p.kRho * rhoSigned;
        omega = p.kAlpha * alpha + yawBlend * p.kBeta * beta;

        v = clampValue(v, -p.maxLinear, p.maxLinear);
        omega = clampValue(omega, -p.maxAngular, p.maxAngular);
    }

    let wr = (v / p.wheelRadius) + (p.wheelDistance / (2 * p.wheelRadius)) * omega;
    let wl = (v / p.wheelRadius) - (p.wheelDistance / (2 * p.wheelRadius)) * omega;
    wr = clampValue(wr, -p.maxWheelSpeed, p.maxWheelSpeed);
    wl = clampValue(wl, -p.maxWheelSpeed, p.maxWheelSpeed);

    return { wr, wl, converged: goalReached, posErr, eYaw };
}

// ──────────────────────────────────────────────────────────────
// 底盘积分与控制循环
// ──────────────────────────────────────────────────────────────

function integrateHumanoidDiffDrive(dt) {
    const p = appState.humanoidBaseControlParams;
    let wr, wl, converged, posErr, eYaw;

    if (appState.humanoidBaseControlMode === 'path') {
        if (appState.humanoidBasePath.length >= 2 && appState.humanoidBaseRPPState.isRunning) {
            const rpp = computeRegulatedPurePursuitControl();
            const v = rpp.v;
            const omega = rpp.omega;
            wr = (v / p.wheelRadius) + (p.wheelDistance / (2 * p.wheelRadius)) * omega;
            wl = (v / p.wheelRadius) - (p.wheelDistance / (2 * p.wheelRadius)) * omega;
            wr = clampValue(wr, -p.maxWheelSpeed, p.maxWheelSpeed);
            wl = clampValue(wl, -p.maxWheelSpeed, p.maxWheelSpeed);
            converged = rpp.converged;
            posErr = rpp.posErr;
            eYaw = rpp.eYaw;
        } else {
            wr = 0; wl = 0; converged = true; posErr = 0; eYaw = 0;
        }
    } else {
        const ctrl = computeHumanoidDiffDriveControl();
        wr = ctrl.wr; wl = ctrl.wl;
        converged = ctrl.converged;
        posErr = ctrl.posErr; eYaw = ctrl.eYaw;
    }

    appState.humanoidBaseWheelVelocity.right = wr;
    appState.humanoidBaseWheelVelocity.left = wl;

    if (appState.humanoidDiffDriveModel && typeof appState.humanoidDiffDriveModel.constraintJacobian === 'function') {
        const Jc = appState.humanoidDiffDriveModel.constraintJacobian(appState.humanoidBasePose.yaw || 0);
        const qdotX = (Jc?.[0]?.[0] || 0) * wr + (Jc?.[0]?.[1] || 0) * wl;
        const qdotY = (Jc?.[1]?.[0] || 0) * wr + (Jc?.[1]?.[1] || 0) * wl;
        const qdotYaw = (Jc?.[2]?.[0] || 0) * wr + (Jc?.[2]?.[1] || 0) * wl;
        appState.humanoidBasePose.x += dt * qdotX;
        appState.humanoidBasePose.y += dt * qdotY;
        appState.humanoidBasePose.yaw = wrapAngle((appState.humanoidBasePose.yaw || 0) + dt * qdotYaw);
    } else {
        const v = (p.wheelRadius / 2) * (wr + wl);
        const omega = (p.wheelRadius / p.wheelDistance) * (wr - wl);
        const yaw = appState.humanoidBasePose.yaw || 0;
        const c = Math.cos(yaw);
        const s = Math.sin(yaw);
        appState.humanoidBasePose.x += dt * v * c;
        appState.humanoidBasePose.y += dt * v * s;
        appState.humanoidBasePose.yaw = wrapAngle(yaw + dt * omega);
    }

    return { converged, posErr, eYaw };
}

function stepHumanoidBaseControl(nowMs = performance.now()) {
    if (!appState.humanoidMode || !appState.humanoidModel) return;
    if (!appState.humanoidBaseControlLastTs) {
        appState.humanoidBaseControlLastTs = nowMs;
        return;
    }
    const dt = clampValue((nowMs - appState.humanoidBaseControlLastTs) / 1000, 0.001, 0.05);
    appState.humanoidBaseControlLastTs = nowMs;

    const controlState = integrateHumanoidDiffDrive(dt);

    if (controlState.converged) {
        appState.humanoidBaseConvergedCount += 1;
    } else {
        appState.humanoidBaseConvergedCount = 0;
    }

    if (appState.humanoidBaseControlMode === 'path' && appState.humanoidBaseRPPState.isRunning
        && appState.humanoidBaseConvergedCount >= appState.humanoidBaseControlParams.settleCycles) {
        appState.humanoidBaseRPPState.isRunning = false;
        if (appState.visualizer) appState.visualizer.setAddWaypointMode(true);
        if (typeof window._updateRPPRunBtn === 'function') window._updateRPPRunBtn();
    }

    if (appState.humanoidBaseControlMode === 'path' && appState.visualizer) {
        if (typeof appState.visualizer.setLookaheadPoint === 'function') {
            appState.visualizer.setLookaheadPoint(appState.humanoidBaseRPPState.carrotWorld);
        }
    }

    applyHumanoidBasePose();
    callbacks.updateRobotPose?.();
}

export function stopHumanoidBaseControlLoop() {
    if (appState.humanoidBaseControlTimer) {
        clearInterval(appState.humanoidBaseControlTimer);
        appState.humanoidBaseControlTimer = null;
    }
    appState.humanoidBaseControlLastTs = 0;
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidDiffDriveModel = null;
}

export function startHumanoidBaseControlLoop() {
    stopHumanoidBaseControlLoop();
    appState.humanoidBaseControlTimer = setInterval(() => {
        stepHumanoidBaseControl(performance.now());
    }, 20);
}

export function applyHumanoidBasePose() {
    const x = Number(appState.humanoidBasePose.x) || 0;
    const y = Number(appState.humanoidBasePose.y) || 0;
    const yaw = Number(appState.humanoidBasePose.yaw) || 0;

    if (appState.humanoidModel && typeof appState.humanoidModel.setReferenceFrame === 'function') {
        const dqBase = buildHumanoidBasePoseDQArray({ x, y, yaw });
        try {
            appState.humanoidModel.setReferenceFrame(dqBase);
        } catch (error) {
            console.warn('设置 humanoid base reference frame 失败:', error);
        }
    }

    if (appState.visualizer && typeof appState.visualizer.setHumanoidBasePose === 'function') {
        appState.visualizer.setHumanoidBasePose({ x, y, yaw });
    }
}

// ──────────────────────────────────────────────────────────────
// 注册回调到全局 callbacks
// ──────────────────────────────────────────────────────────────

export function registerDiffDriveCallbacks() {
    callbacks.stopHumanoidBaseControlLoop = stopHumanoidBaseControlLoop;
    callbacks.startHumanoidBaseControlLoop = startHumanoidBaseControlLoop;
    callbacks.applyHumanoidBasePose = applyHumanoidBasePose;
    callbacks.getHumanoidDiffDriveParamsFromConfig = getHumanoidDiffDriveParamsFromConfig;
    callbacks.getHumanoidBasePoseFromConfig = getHumanoidBasePoseFromConfig;
}
