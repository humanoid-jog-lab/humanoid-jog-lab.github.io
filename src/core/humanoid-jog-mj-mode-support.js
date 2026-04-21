export function supportsHumanoidJogMJRestoreMode(modeTag = '') {
    return modeTag === 'absolute-target'
        || modeTag === 'relative-target'
        || modeTag === 'distance-target'
        || modeTag === 'robot1-target'
        || modeTag === 'robot2-target'
        || modeTag === 'robot1-waist-target'
        || modeTag === 'robot2-waist-target'
        || modeTag === 'waist-body-link-xz-target'
        || modeTag === 'left-ruled-surface-polishing-target'
        || modeTag === 'cooperative-ruled-surface-polishing-target'
        || modeTag === 'cooperative-ruled-surface-polishing-dq-setpoint-target'
        || modeTag === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only-target';
}

export default supportsHumanoidJogMJRestoreMode;
