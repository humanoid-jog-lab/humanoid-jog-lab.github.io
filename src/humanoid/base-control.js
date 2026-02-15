/**
 * 人形机器人底盘控制
 * - 差速驱动 rho-alpha-beta 控制器
 * - Regulated Pure Pursuit 路径跟踪
 */

import { DEG_TO_RAD } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { wrapAngle, clampValue, quaternionToYaw } from '../utils/math-utils.js';
import { DQ, getDQModule } from '../dqrobotics.js';

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
// Regulated Pure Pursuit Controller
// ──────────────────────────────────────────────────────────────

function rppTransformPathToRobotFrame(path, robotPose) {
    const cx = robotPose.x || 0;
    const cy = robotPose.y || 0;
    const theta = robotPose.yaw || 0;
    const cosT = Math.cos(-theta);
    const sinT = Math.sin(-theta);
    return path.map(p => {
        const dx = p.x - cx;
        const dy = p.y - cy;
        return {
            x: cosT * dx - sinT * dy,
            y: sinT * dx + cosT * dy,
            yaw: p.yaw !== undefined ? wrapAngle(p.yaw - theta) : undefined
        };
    });
}

function rppFindClosestProjection(pathLocal, startSegIdx = 0) {
    let bestDistSq = Infinity;
    let bestSeg = Math.max(0, startSegIdx);
    let bestT = 0;
    let bestPX = 0, bestPY = 0;

    const searchFrom = Math.max(0, startSegIdx - 1);
    for (let i = searchFrom; i < pathLocal.length - 1; i++) {
        const ax = pathLocal[i].x, ay = pathLocal[i].y;
        const bx = pathLocal[i + 1].x, by = pathLocal[i + 1].y;
        const dx = bx - ax, dy = by - ay;
        const lenSq = dx * dx + dy * dy;

        let t = 0;
        if (lenSq > 1e-12) {
            t = -(ax * dx + ay * dy) / lenSq;
            t = Math.max(0, Math.min(1, t));
        }

        const px = ax + t * dx;
        const py = ay + t * dy;
        const distSq = px * px + py * py;

        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestSeg = i;
            bestT = t;
            bestPX = px;
            bestPY = py;
        }
    }

    if (bestSeg < startSegIdx) {
        bestSeg = startSegIdx;
    }

    return { segIdx: bestSeg, t: bestT, projX: bestPX, projY: bestPY, distSq: bestDistSq };
}

function rppGetLookAheadFromProjection(lookaheadDist, pathLocal, segIdx, t) {
    if (pathLocal.length < 2) {
        const p = pathLocal[pathLocal.length - 1] || { x: 0, y: 0 };
        return { x: p.x, y: p.y };
    }

    const p0 = pathLocal[segIdx];
    const p1 = pathLocal[segIdx + 1];

    const projX = p0.x + t * (p1.x - p0.x);
    const projY = p0.y + t * (p1.y - p0.y);

    const remainOnSeg = Math.hypot(p1.x - projX, p1.y - projY);

    if (remainOnSeg >= lookaheadDist) {
        const ratio = remainOnSeg > 1e-9 ? lookaheadDist / remainOnSeg : 0;
        return {
            x: projX + ratio * (p1.x - projX),
            y: projY + ratio * (p1.y - projY)
        };
    }

    let accumulated = remainOnSeg;
    for (let i = segIdx + 1; i < pathLocal.length - 1; i++) {
        const a = pathLocal[i];
        const b = pathLocal[i + 1];
        const segLen = Math.hypot(b.x - a.x, b.y - a.y);
        if (accumulated + segLen >= lookaheadDist) {
            const remaining = lookaheadDist - accumulated;
            const ratio = segLen > 1e-9 ? remaining / segLen : 0;
            return {
                x: a.x + ratio * (b.x - a.x),
                y: a.y + ratio * (b.y - a.y)
            };
        }
        accumulated += segLen;
    }

    const last = pathLocal[pathLocal.length - 1];
    return { x: last.x, y: last.y };
}

function rppCalculateCurvature(lookaheadPoint) {
    const L2 = lookaheadPoint.x * lookaheadPoint.x + lookaheadPoint.y * lookaheadPoint.y;
    if (L2 > 0.001) {
        return 2.0 * lookaheadPoint.y / L2;
    }
    return 0.0;
}

function rppCurvatureConstraint(rawVel, curvature, minRadius) {
    if (Math.abs(curvature) < 1e-6) return rawVel;
    const radius = Math.abs(1.0 / curvature);
    if (radius < minRadius) {
        return rawVel * (1.0 - Math.abs(radius - minRadius) / minRadius);
    }
    return rawVel;
}

function rppRemainingPathLength(pathLocal) {
    let len = 0;
    for (let i = 0; i < pathLocal.length - 1; i++) {
        len += Math.hypot(
            pathLocal[i + 1].x - pathLocal[i].x,
            pathLocal[i + 1].y - pathLocal[i].y
        );
    }
    return len;
}

function rppApproachVelocityConstraint(constrainedVel, pathLocal, minApproachVel, approachDist) {
    const remaining = rppRemainingPathLength(pathLocal);
    if (remaining < approachDist && pathLocal.length > 0) {
        const last = pathLocal[pathLocal.length - 1];
        const distToGoal = Math.hypot(last.x, last.y);
        const scale = approachDist > 1e-6 ? distToGoal / approachDist : 1.0;
        let approachVel = constrainedVel * scale;
        if (approachVel < minApproachVel) approachVel = minApproachVel;
        return Math.min(constrainedVel, approachVel);
    }
    return constrainedVel;
}

export function resetRPPState() {
    const st = appState.humanoidBaseRPPState;
    st.isRotatingToHeading = false;
    st.hasReachedGoalXY = false;
    st.lastCmdLinear = 0;
    st.lastCmdAngular = 0;
    st.isFinished = false;
    st.isRunning = false;
    st.carrotWorld = null;
    st.progressSegIdx = 0;
}

function computeRegulatedPurePursuitControl() {
    const p = appState.humanoidBaseRPPParams;
    const state = appState.humanoidBaseRPPState;
    const bp = appState.humanoidBasePose;

    if (!appState.humanoidBasePath || appState.humanoidBasePath.length < 2) {
        state.isFinished = true;
        state.carrotWorld = null;
        return { v: 0, omega: 0, converged: true, posErr: 0, eYaw: 0 };
    }

    const pathLocal = rppTransformPathToRobotFrame(appState.humanoidBasePath, bp);
    const proj = rppFindClosestProjection(pathLocal, state.progressSegIdx);
    state.progressSegIdx = proj.segIdx;

    const goalLocal = pathLocal[pathLocal.length - 1];
    const distToGoal = Math.hypot(goalLocal.x, goalLocal.y);
    const theta = bp.yaw || 0;
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);

    const finalTolPos = appState.humanoidBaseControlParams.tolPos;
    const finalTolYaw = appState.humanoidBaseControlParams.tolYaw;

    if (distToGoal < finalTolPos) {
        const goalWorld = appState.humanoidBasePath[appState.humanoidBasePath.length - 1];
        if (goalWorld.yaw !== undefined) {
            const eYaw = wrapAngle(goalWorld.yaw - (bp.yaw || 0));
            if (Math.abs(eYaw) > finalTolYaw) {
                const sign = eYaw > 0 ? 1 : -1;
                let omega = sign * p.rotate_to_heading_angular_vel;
                const maxVelToStop = Math.sqrt(2 * p.max_angular_accel * Math.abs(eYaw));
                if (Math.abs(omega) > maxVelToStop) omega = sign * maxVelToStop;
                state.carrotWorld = null;
                return { v: 0, omega, converged: false, posErr: distToGoal, eYaw };
            }
        }
        state.isFinished = true;
        state.carrotWorld = null;
        return { v: 0, omega: 0, converged: true, posErr: distToGoal, eYaw: 0 };
    }

    if (distToGoal < p.goal_dist_tol) {
        if (!state.hasReachedGoalXY) state.hasReachedGoalXY = true;

        const goalHeading = Math.atan2(goalLocal.y, goalLocal.x);
        const kFinal = appState.humanoidBaseControlParams.kRho;
        let vFinal = kFinal * distToGoal;
        vFinal = clampValue(vFinal, -p.max_linear_vel, p.max_linear_vel);
        let omegaFinal = appState.humanoidBaseControlParams.kAlpha * goalHeading;
        omegaFinal = clampValue(omegaFinal, -p.max_angular_vel, p.max_angular_vel);

        state.lastCmdLinear = vFinal;
        state.lastCmdAngular = omegaFinal;
        state.carrotWorld = {
            x: cosT * goalLocal.x - sinT * goalLocal.y + (bp.x || 0),
            y: sinT * goalLocal.x + cosT * goalLocal.y + (bp.y || 0)
        };
        return { v: vFinal, omega: omegaFinal, converged: false, posErr: distToGoal, eYaw: 0 };
    }

    let lookaheadDist = p.lookahead_dist;
    if (p.use_velocity_scaled_lookahead) {
        lookaheadDist = Math.abs(state.lastCmdLinear) * p.lookahead_time;
        lookaheadDist = Math.max(p.min_lookahead_dist, Math.min(p.max_lookahead_dist, lookaheadDist));
    }

    const carrot = rppGetLookAheadFromProjection(lookaheadDist, pathLocal, proj.segIdx, proj.t);

    state.carrotWorld = {
        x: cosT * carrot.x - sinT * carrot.y + (bp.x || 0),
        y: sinT * carrot.x + cosT * carrot.y + (bp.y || 0)
    };

    const curvature = rppCalculateCurvature(carrot);

    let xVelSign = 1.0;
    if (p.allow_reversing) {
        xVelSign = carrot.x >= 0 ? 1.0 : -1.0;
    }

    let linearVel, angularVel;

    let angleToHeading = Math.atan2(carrot.y, carrot.x);
    if (xVelSign < 0) angleToHeading = wrapAngle(angleToHeading + Math.PI);

    if (p.use_rotate_to_heading && Math.abs(angleToHeading) > p.rotate_to_heading_min_angle) {
        state.isRotatingToHeading = true;
        linearVel = 0;
        const sign = angleToHeading > 0 ? 1 : -1;
        angularVel = sign * p.rotate_to_heading_angular_vel;

        const dt = 0.02;
        const minFeasible = state.lastCmdAngular - p.max_angular_accel * dt;
        const maxFeasible = state.lastCmdAngular + p.max_angular_accel * dt;
        angularVel = clampValue(angularVel, minFeasible, maxFeasible);

        const maxVelToStop = Math.sqrt(2 * p.max_angular_accel * Math.abs(angleToHeading));
        if (Math.abs(angularVel) > maxVelToStop) {
            angularVel = sign * maxVelToStop;
        }
    } else {
        state.isRotatingToHeading = false;
        linearVel = p.max_linear_vel;

        if (p.use_regulated_linear_velocity_scaling) {
            linearVel = rppCurvatureConstraint(
                linearVel, curvature, p.regulated_linear_scaling_min_radius
            );
        }

        const tailPath = [{ x: proj.projX, y: proj.projY }, ...pathLocal.slice(proj.segIdx + 1)];
        linearVel = rppApproachVelocityConstraint(
            linearVel, tailPath,
            p.min_approach_linear_velocity,
            p.approach_velocity_scaling_dist
        );

        linearVel = Math.max(p.regulated_linear_scaling_min_speed, Math.min(p.max_linear_vel, Math.abs(linearVel)));
        linearVel = xVelSign * linearVel;

        angularVel = linearVel * curvature;
    }

    angularVel = clampValue(angularVel, -p.max_angular_vel, p.max_angular_vel);

    state.lastCmdLinear = linearVel;
    state.lastCmdAngular = angularVel;

    return { v: linearVel, omega: angularVel, converged: false, posErr: distToGoal, eYaw: 0 };
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
        const dqBase = DQ.pose(0, 0, 1, yaw, x, y, 0).toArray();
        appState.humanoidModel.setReferenceFrame(dqBase);
    }

    if (appState.visualizer && typeof appState.visualizer.setHumanoidBasePose === 'function') {
        appState.visualizer.setHumanoidBasePose({ x, y, yaw });
    }
}

/**
 * 注册回调到全局 callbacks
 */
export function registerHumanoidBaseControlCallbacks() {
    callbacks.stopHumanoidBaseControlLoop = stopHumanoidBaseControlLoop;
    callbacks.startHumanoidBaseControlLoop = startHumanoidBaseControlLoop;
    callbacks.applyHumanoidBasePose = applyHumanoidBasePose;
    callbacks.getHumanoidDiffDriveParamsFromConfig = getHumanoidDiffDriveParamsFromConfig;
    callbacks.getHumanoidBasePoseFromConfig = getHumanoidBasePoseFromConfig;
}
