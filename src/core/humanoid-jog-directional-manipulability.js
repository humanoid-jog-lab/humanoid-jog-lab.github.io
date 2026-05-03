import { directionalCCTSM } from './cctsm.js';

const AXIS_TO_ABSOLUTE_INDEX = Object.freeze({
    rx: 0,
    ry: 1,
    rz: 2,
    x: 3,
    y: 4,
    z: 5
});

export function buildHumanoidJogDirectionalAxisVector({
    axis = '',
    gramSize = 12
} = {}) {
    const index = AXIS_TO_ABSOLUTE_INDEX[String(axis || '').toLowerCase()];
    if (!Number.isInteger(index)) return null;
    if (!Number.isFinite(gramSize) || gramSize <= index) return null;
    const vector = new Array(Math.max(0, Math.floor(gramSize))).fill(0);
    vector[index] = 1;
    return vector;
}

export function evaluateHumanoidJogDirectionalManipulability({
    axis = '',
    cctsmResult = null
} = {}) {
    const gram = Array.isArray(cctsmResult?.Gram) ? cctsmResult.Gram : null;
    const gramSize = Array.isArray(gram) ? gram.length : 0;
    const vHat = buildHumanoidJogDirectionalAxisVector({ axis, gramSize });
    if (!gram || !vHat) return Number.NaN;
    return directionalCCTSM(gram, vHat);
}

export function evaluateHumanoidJogProjectedManipulability(cctsmResult = null) {
    const projected = Number.isFinite(cctsmResult?.w_cmd) ? cctsmResult.w_cmd : Number.NaN;
    if (Number.isFinite(projected)) return projected;
    return Number.isFinite(cctsmResult?.w) ? cctsmResult.w : Number.NaN;
}

export function evaluateHumanoidJogAbsoluteManipulability(cctsmResult = null) {
    const absolute = Number.isFinite(cctsmResult?.w_abs) ? cctsmResult.w_abs : Number.NaN;
    if (Number.isFinite(absolute)) return absolute;
    return Number.isFinite(cctsmResult?.w) ? cctsmResult.w : Number.NaN;
}

export function evaluateHumanoidJogTaskManipulability(cctsmResult = null) {
    const task = Number.isFinite(cctsmResult?.w_task) ? cctsmResult.w_task : Number.NaN;
    if (Number.isFinite(task)) return task;
    return evaluateHumanoidJogAbsoluteManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteHierarchicalManipulability(cctsmResult = null) {
    const hierarchical = Number.isFinite(cctsmResult?.w_abs_hier) ? cctsmResult.w_abs_hier : Number.NaN;
    if (Number.isFinite(hierarchical)) return hierarchical;
    return evaluateHumanoidJogAbsoluteManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteHierarchicalExtendedManipulability(cctsmResult = null) {
    const extended = Number.isFinite(cctsmResult?.w_abs_hier_ext) ? cctsmResult.w_abs_hier_ext : Number.NaN;
    if (Number.isFinite(extended)) return extended;
    return evaluateHumanoidJogAbsoluteHierarchicalManipulability(cctsmResult);
}

export function evaluateHumanoidJogTaskHierarchicalManipulability(cctsmResult = null) {
    const hierarchical = Number.isFinite(cctsmResult?.w_task_hier) ? cctsmResult.w_task_hier : Number.NaN;
    if (Number.isFinite(hierarchical)) return hierarchical;
    return evaluateHumanoidJogAbsoluteHierarchicalManipulability(cctsmResult);
}

export function evaluateHumanoidJogTaskHierarchicalExtendedManipulability(cctsmResult = null) {
    const extended = Number.isFinite(cctsmResult?.w_task_hier_ext) ? cctsmResult.w_task_hier_ext : Number.NaN;
    if (Number.isFinite(extended)) return extended;
    return evaluateHumanoidJogAbsoluteHierarchicalExtendedManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteArmManipulability(cctsmResult = null) {
    const absoluteArm = Number.isFinite(cctsmResult?.w_abs_arm) ? cctsmResult.w_abs_arm : Number.NaN;
    if (Number.isFinite(absoluteArm)) return absoluteArm;
    const absolute = Number.isFinite(cctsmResult?.w_abs) ? cctsmResult.w_abs : Number.NaN;
    if (Number.isFinite(absolute)) return absolute;
    return Number.isFinite(cctsmResult?.w) ? cctsmResult.w : Number.NaN;
}

export function evaluateHumanoidJogTaskArmManipulability(cctsmResult = null) {
    const taskArm = Number.isFinite(cctsmResult?.w_task_arm) ? cctsmResult.w_task_arm : Number.NaN;
    if (Number.isFinite(taskArm)) return taskArm;
    return evaluateHumanoidJogTaskManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteArmVFIManipulability(cctsmResult = null) {
    const absoluteArmVFI = Number.isFinite(cctsmResult?.w_abs_arm_vfi) ? cctsmResult.w_abs_arm_vfi : Number.NaN;
    if (Number.isFinite(absoluteArmVFI)) return absoluteArmVFI;
    const absoluteVFI = Number.isFinite(cctsmResult?.w_abs_vfi) ? cctsmResult.w_abs_vfi : Number.NaN;
    if (Number.isFinite(absoluteVFI)) return absoluteVFI;
    return evaluateHumanoidJogAbsoluteArmManipulability(cctsmResult);
}

export function evaluateHumanoidJogTaskArmVFIManipulability(cctsmResult = null) {
    const taskArmVFI = Number.isFinite(cctsmResult?.w_task_arm_vfi) ? cctsmResult.w_task_arm_vfi : Number.NaN;
    if (Number.isFinite(taskArmVFI)) return taskArmVFI;
    return evaluateHumanoidJogTaskVFIManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteVFIManipulability(cctsmResult = null) {
    const absoluteVFI = Number.isFinite(cctsmResult?.w_abs_vfi) ? cctsmResult.w_abs_vfi : Number.NaN;
    if (Number.isFinite(absoluteVFI)) return absoluteVFI;
    if (Number.isFinite(cctsmResult?.w_vfi)) return cctsmResult.w_vfi;
    return Number.isFinite(cctsmResult?.w) ? cctsmResult.w : Number.NaN;
}

export function evaluateHumanoidJogTaskVFIManipulability(cctsmResult = null) {
    const taskVFI = Number.isFinite(cctsmResult?.w_task_vfi) ? cctsmResult.w_task_vfi : Number.NaN;
    if (Number.isFinite(taskVFI)) return taskVFI;
    return evaluateHumanoidJogAbsoluteVFIManipulability(cctsmResult);
}

export function evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability(cctsmResult = null, options = {}) {
    const hierarchicalVfi = Number.isFinite(cctsmResult?.w_abs_hier_vfi) ? cctsmResult.w_abs_hier_vfi : Number.NaN;
    const baselineHierarchical = evaluateHumanoidJogAbsoluteHierarchicalManipulability(cctsmResult);
    if (Number.isFinite(hierarchicalVfi)) {
        return Number.isFinite(baselineHierarchical)
            ? Math.min(baselineHierarchical, hierarchicalVfi)
            : hierarchicalVfi;
    }
    const hierarchicalBlend = Number.isFinite(options?.hierarchicalVfiBlend)
        ? Math.max(0, Math.min(1, options.hierarchicalVfiBlend))
        : 0.5;
    const wAbs = baselineHierarchical;
    const wAbsVfi = Number.isFinite(cctsmResult?.w_abs_vfi)
        ? cctsmResult.w_abs_vfi
        : evaluateHumanoidJogAbsoluteVFIManipulability(cctsmResult);
    if (!Number.isFinite(wAbs) && !Number.isFinite(wAbsVfi)) {
        return Number.NaN;
    }
    if (!Number.isFinite(wAbs)) return wAbsVfi;
    if (!Number.isFinite(wAbsVfi)) return wAbs;
    return Math.min(wAbs, wAbs + hierarchicalBlend * (wAbsVfi - wAbs));
}

export function evaluateHumanoidJogTaskHierarchicalVFIManipulability(cctsmResult = null, options = {}) {
    const hierarchicalVfi = Number.isFinite(cctsmResult?.w_task_hier_vfi) ? cctsmResult.w_task_hier_vfi : Number.NaN;
    const baselineHierarchical = evaluateHumanoidJogTaskHierarchicalManipulability(cctsmResult);
    if (Number.isFinite(hierarchicalVfi)) {
        return Number.isFinite(baselineHierarchical)
            ? Math.min(baselineHierarchical, hierarchicalVfi)
            : hierarchicalVfi;
    }
    const hierarchicalBlend = Number.isFinite(options?.hierarchicalVfiBlend)
        ? Math.max(0, Math.min(1, options.hierarchicalVfiBlend))
        : 0.5;
    const wTask = baselineHierarchical;
    const wTaskVfi = Number.isFinite(cctsmResult?.w_task_vfi)
        ? cctsmResult.w_task_vfi
        : evaluateHumanoidJogTaskVFIManipulability(cctsmResult);
    if (!Number.isFinite(wTask) && !Number.isFinite(wTaskVfi)) {
        return Number.NaN;
    }
    if (!Number.isFinite(wTask)) return wTaskVfi;
    if (!Number.isFinite(wTaskVfi)) return wTask;
    return Math.min(wTask, wTask + hierarchicalBlend * (wTaskVfi - wTask));
}
