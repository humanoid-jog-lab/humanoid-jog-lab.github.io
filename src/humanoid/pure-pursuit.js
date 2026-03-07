/**
 * Regulated Pure Pursuit Controller
 */

import { appState, callbacks } from '../core/state.js';
import { wrapAngle, clampValue } from '../utils/math.js';

// ──────────────────────────────────────────────────────────────
// Local helpers
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

// ──────────────────────────────────────────────────────────────
// Exported functions
// ──────────────────────────────────────────────────────────────

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

export function computeRegulatedPurePursuitControl() {
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
