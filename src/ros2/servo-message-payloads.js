export const HUMANOID_WAIST_JOINT_NAMES = [
    'ankle_joint',
    'knee_joint',
    'hip_pitch_joint',
    'hip_yaw_joint'
];

export const HUMANOID_LEFT_ARM_JOINT_NAMES = [
    'ljoint1',
    'ljoint2',
    'ljoint3',
    'ljoint4',
    'ljoint5',
    'ljoint6',
    'ljoint7'
];

export const HUMANOID_RIGHT_ARM_JOINT_NAMES = [
    'rjoint1',
    'rjoint2',
    'rjoint3',
    'rjoint4',
    'rjoint5',
    'rjoint6',
    'rjoint7'
];

const HUMANOID_LEFT_ARM_START = 4;
const HUMANOID_RIGHT_ARM_START = HUMANOID_LEFT_ARM_START + HUMANOID_LEFT_ARM_JOINT_NAMES.length;

function toFiniteNumber(value, fallback = 0) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : fallback;
}

function fillJointPositions(messageData, jointNames, values = []) {
    for (let i = 0; i < jointNames.length; i++) {
        messageData[`${jointNames[i]}_position`] = toFiniteNumber(values[i], 0);
    }
}

function readHumanoidSlice(humanoidQ, start, count) {
    const values = Array.isArray(humanoidQ) ? humanoidQ : [];
    const out = new Array(count).fill(0);
    for (let i = 0; i < count; i++) {
        out[i] = toFiniteNumber(values[start + i], 0);
    }
    return out;
}

export function buildRobotArmServoMessageData({
    humanoidMode = false,
    humanoidQ = [],
    dualArmMode = false,
    jointAngles = [],
    jointAngles1 = [],
    jointAngles2 = [],
    runMode = 1,
    robotPower = true,
    runTime = 0
} = {}) {
    const messageData = {
        run_mode: toFiniteNumber(runMode, 1),
        robot_power: Boolean(robotPower),
        run_time: toFiniteNumber(runTime, 0)
    };

    let source = 'single-arm';

    if (humanoidMode) {
        source = 'humanoid';
        fillJointPositions(
            messageData,
            HUMANOID_LEFT_ARM_JOINT_NAMES,
            readHumanoidSlice(humanoidQ, HUMANOID_LEFT_ARM_START, HUMANOID_LEFT_ARM_JOINT_NAMES.length)
        );
        fillJointPositions(
            messageData,
            HUMANOID_RIGHT_ARM_JOINT_NAMES,
            readHumanoidSlice(humanoidQ, HUMANOID_RIGHT_ARM_START, HUMANOID_RIGHT_ARM_JOINT_NAMES.length)
        );
        return { messageData, source };
    }

    if (dualArmMode) {
        source = 'dual-arm';
        fillJointPositions(messageData, HUMANOID_LEFT_ARM_JOINT_NAMES, jointAngles2);
        fillJointPositions(messageData, HUMANOID_RIGHT_ARM_JOINT_NAMES, jointAngles1);
        return { messageData, source };
    }

    fillJointPositions(messageData, HUMANOID_LEFT_ARM_JOINT_NAMES, []);
    fillJointPositions(messageData, HUMANOID_RIGHT_ARM_JOINT_NAMES, jointAngles);
    return { messageData, source };
}

export function buildRobotServoMessageData({
    humanoidMode = false,
    humanoidQ = [],
    runMode = 0,
    x = 0,
    y = 0,
    phi = 0,
    modeLegSelect = 0,
    vel = 0.1
} = {}) {
    const waistValues = readHumanoidSlice(humanoidQ, 0, HUMANOID_WAIST_JOINT_NAMES.length);

    return {
        valid: humanoidMode && Array.isArray(humanoidQ) && humanoidQ.length >= HUMANOID_WAIST_JOINT_NAMES.length,
        messageData: {
            run_mode: toFiniteNumber(runMode, 0),
            ankle_position: waistValues[0],
            knee_position: waistValues[1],
            hip_pitch_position: waistValues[2],
            hip_yaw_position: waistValues[3],
            x: toFiniteNumber(x, 0),
            y: toFiniteNumber(y, 0),
            phi: toFiniteNumber(phi, 0),
            mode_leg_select: toFiniteNumber(modeLegSelect, 0),
            vel: toFiniteNumber(vel, 0.1)
        }
    };
}
