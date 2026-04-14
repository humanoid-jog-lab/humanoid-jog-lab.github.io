export function normalizeHumanoidJogSolverMode(modeRaw) {
    return modeRaw === 'priority'
        || modeRaw === 'mj_restore'
        || modeRaw === 'mj_restore_tr'
        || modeRaw === 'mj_restore_pre_w'
        ? modeRaw
        : 'vfi';
}

export function getHumanoidJogSolverDisplayName(mode) {
    if (mode === 'priority') return '零空间投影';
    if (mode === 'mj_restore') return 'MuJoCo 风格恢复';
    if (mode === 'mj_restore_tr') return '恢复优化(TR)';
    if (mode === 'mj_restore_pre_w') return 'MuJoCo 风格恢复 + w预优化';
    return 'new qpJ';
}

export function getHumanoidJogSolverReadyMessage(mode) {
    return `${getHumanoidJogSolverDisplayName(mode)} 已启用，等待点动输入。`;
}

export function getHumanoidJogCurrentCCTSMMessage(mode) {
    return `当前为${getHumanoidJogSolverDisplayName(mode)}求解器，显示当前 CCTSM。`;
}
