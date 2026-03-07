/**
 * 数学工具函数
 * 角度转换、格式化、限幅等
 */

import { RAD_TO_DEG, DEG_TO_RAD } from '../core/constants.js';
import { appState } from '../core/state.js';

export function angleToDisplay(angleRad) {
    if (!Number.isFinite(angleRad)) {
        return 0;
    }
    return appState.angleDisplayMode === 'deg' ? angleRad * RAD_TO_DEG : angleRad;
}

export function displayToAngle(displayValue) {
    if (!Number.isFinite(displayValue)) {
        return 0;
    }
    return appState.angleDisplayMode === 'deg' ? displayValue * DEG_TO_RAD : displayValue;
}

export function getSliderPrecision() {
    return appState.angleDisplayMode === 'deg' ? 1 : 3;
}

export function getInputPrecision() {
    return appState.angleDisplayMode === 'deg' ? 2 : 4;
}

export function displayStep() {
    return appState.angleDisplayMode === 'deg' ? 0.1 : 0.01;
}

export function getAnglePlaceholder() {
    return appState.angleDisplayMode === 'deg' ? '角度(°)' : '角度(rad)';
}

export function formatDisplayValue(value, digits) {
    const safeValue = Number.isFinite(value) ? value : 0;
    return safeValue.toFixed(digits);
}

export function formatDisplayAngle(angleRad, digits = getInputPrecision()) {
    return formatDisplayValue(angleToDisplay(angleRad), digits);
}

export function getActiveJointConfig() {
    return appState.customRobotConfig || appState.frankaConfigCache;
}

export function quaternionToYaw(qw, qx, qy, qz) {
    const siny = 2 * (qw * qz + qx * qy);
    const cosy = 1 - 2 * (qy * qy + qz * qz);
    return Math.atan2(siny, cosy);
}

export function wrapAngle(angle) {
    let a = angle;
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}

export function clampValue(value, minValue, maxValue) {
    return Math.max(minValue, Math.min(maxValue, value));
}
