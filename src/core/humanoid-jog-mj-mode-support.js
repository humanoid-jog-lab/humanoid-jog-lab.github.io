export function supportsHumanoidJogMJRestoreMode(modeTag = '') {
    return modeTag === 'absolute-target'
        || modeTag === 'relative-target'
        || modeTag === 'distance-target'
        || modeTag === 'robot1-target'
        || modeTag === 'robot2-target'
        || modeTag === 'robot1-waist-target'
        || modeTag === 'robot2-waist-target'
        || modeTag === 'waist-body-link-xz-target';
}

export default supportsHumanoidJogMJRestoreMode;
