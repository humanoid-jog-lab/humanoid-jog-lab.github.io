export const DEFAULT_REAL_JOINT_STATE_TOPIC = '/whole/joint_states';

export const DEFAULT_HUMANOID_IGNORED_JOINTS = new Set([
    'left_wheel_joint',
    'right_wheel_joint'
]);

export function mapHumanoidJointPositions({
    messageNames,
    positions,
    jointNames,
    currentQ,
    ignoredJointNames = DEFAULT_HUMANOID_IGNORED_JOINTS
} = {}) {
    if (!Array.isArray(messageNames) || !Array.isArray(positions) || !Array.isArray(jointNames) || !Array.isArray(currentQ)) {
        return { updated: false, nextQ: Array.isArray(currentQ) ? [...currentQ] : [], updatedCount: 0 };
    }

    if (jointNames.length === 0 || jointNames.length !== currentQ.length) {
        return { updated: false, nextQ: [...currentQ], updatedCount: 0 };
    }

    const ignored = ignoredJointNames instanceof Set
        ? ignoredJointNames
        : new Set(Array.isArray(ignoredJointNames) ? ignoredJointNames : []);
    const nameToPosition = new Map();
    const count = Math.min(messageNames.length, positions.length);

    for (let i = 0; i < count; i++) {
        const jointName = typeof messageNames[i] === 'string' ? messageNames[i] : '';
        if (!jointName || ignored.has(jointName)) continue;
        const value = Number(positions[i]);
        if (!Number.isFinite(value)) continue;
        nameToPosition.set(jointName, value);
    }

    const nextQ = [...currentQ];
    let updatedCount = 0;
    for (let i = 0; i < jointNames.length; i++) {
        const jointName = jointNames[i];
        if (!nameToPosition.has(jointName)) continue;
        nextQ[i] = nameToPosition.get(jointName);
        updatedCount += 1;
    }

    return {
        updated: updatedCount > 0,
        nextQ,
        updatedCount
    };
}
