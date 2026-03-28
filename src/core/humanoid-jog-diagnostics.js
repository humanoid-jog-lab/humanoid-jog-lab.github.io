import { getHumanoidJogCurrentCCTSMMessage } from './humanoid-jog-solver-presentation.js';

export function buildCurrentCCTSMPayload({
    solverMode = 'priority',
    modeTag,
    includeWaist = false,
    keepBodyZVertical = false,
    jointLimitAvoidance = true,
    message = getHumanoidJogCurrentCCTSMMessage(solverMode),
    cctsmResult = null
} = {}) {
    return {
        solverMode,
        modeTag,
        active: true,
        message,
        includeWaist,
        keepBodyZVertical,
        jointLimitAvoidance,
        cctsmW: Number.isFinite(cctsmResult?.w) ? cctsmResult.w : Number.NaN,
        cctsmWCmd: Number.isFinite(cctsmResult?.w_cmd) ? cctsmResult.w_cmd : Number.NaN,
        cctsmWAbs: Number.isFinite(cctsmResult?.w_abs) ? cctsmResult.w_abs : Number.NaN,
        cctsmWAbsArm: Number.isFinite(cctsmResult?.w_abs_arm) ? cctsmResult.w_abs_arm : Number.NaN,
        cctsmWAbsArmVfi: Number.isFinite(cctsmResult?.w_abs_arm_vfi) ? cctsmResult.w_abs_arm_vfi : Number.NaN,
        cctsmWAbsVfi: Number.isFinite(cctsmResult?.w_abs_vfi) ? cctsmResult.w_abs_vfi : Number.NaN,
        cctsmWvfi: Number.isFinite(cctsmResult?.w_vfi) ? cctsmResult.w_vfi : Number.NaN
    };
}

export function buildPriorityCurrentCCTSMPayload(options = {}) {
    return buildCurrentCCTSMPayload({
        solverMode: 'priority',
        ...options
    });
}
