import { t } from '../i18n.js';

export function normalizeHumanoidJogSolverMode(modeRaw) {
    return modeRaw === 'priority'
        || modeRaw === 'mj_restore'
        || modeRaw === 'mj_restore_tr'
        || modeRaw === 'mj_restore_tr_safe'
        || modeRaw === 'mj_restore_pre_w'
        ? modeRaw
        : 'vfi';
}

export function getHumanoidJogSolverDisplayName(mode) {
    if (mode === 'priority') return t('Null-space Projection');
    if (mode === 'mj_restore') return t('Restore (LS)');
    if (mode === 'mj_restore_tr') return t('Recovery Optimization (TR)');
    if (mode === 'mj_restore_tr_safe') return t('Recovery Optimization (TR+Safety Bias)');
    if (mode === 'mj_restore_pre_w') return t('MuJoCo-style Recovery + w Pre-optimization');
    return 'new qpJ';
}

export function getHumanoidJogSolverReadyMessage(mode) {
    return t('{{solver}} is enabled and waiting for jog input.', {
        solver: getHumanoidJogSolverDisplayName(mode)
    });
}

export function getHumanoidJogCurrentCCTSMMessage(mode) {
    return t('Current solver: {{solver}}. Displaying the current CCTSM.', {
        solver: getHumanoidJogSolverDisplayName(mode)
    });
}
