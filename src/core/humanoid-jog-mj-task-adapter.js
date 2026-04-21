function createInvariant(kind, target = null, weight = 1, priorityEquivalent = 100) {
    return {
        kind,
        target,
        weight,
        priorityEquivalent,
        diagnosticLabel: `invariant:${kind}`
    };
}

function createExtraConstraint(kind, target = null, weight = 1, priorityEquivalent = 100, diagnosticLabelSuffix = kind) {
    return {
        kind,
        target,
        weight,
        priorityEquivalent,
        diagnosticLabel: `extra-constraint:${diagnosticLabelSuffix}`
    };
}

function buildUnsupportedDescriptor(commandTarget) {
    return {
        valid: false,
        modeTag: 'unsupported-target',
        commandKind: null,
        commandTarget,
        commandDiagnosticLabel: 'command:unsupported',
        activeJointScope: null,
        invariants: [],
        extraConstraints: [],
        diagnosticLabel: 'mode_unsupported'
    };
}

const MODE_CONFIGS = {
    absolute: {
        modeTag: 'absolute-target',
        commandKind: 'absolute_pose',
        activeJointScope: 'arms',
        invariants: () => [
            createInvariant('relative_pose')
        ]
    },
    decoupled: {
        modeTag: 'absolute-target',
        commandKind: 'absolute_pose',
        activeJointScope: 'waist+arms',
        invariants: () => [
            createInvariant('relative_pose')
        ]
    },
    'decoupled-upright': {
        modeTag: 'absolute-target',
        commandKind: 'absolute_pose',
        activeJointScope: 'waist+arms',
        invariants: () => [
            createInvariant('relative_pose')
        ],
        extraConstraints: () => [
            createExtraConstraint('body_z', { upright: true }, 1, 100, 'body_z/upright')
        ]
    },
    relative: {
        modeTag: 'relative-target',
        commandKind: 'relative_pose',
        activeJointScope: 'arms',
        invariants: () => [
            createInvariant('absolute_pose')
        ]
    },
    'relative-fullbody': {
        modeTag: 'relative-target',
        commandKind: 'relative_pose',
        activeJointScope: 'waist+arms',
        invariants: () => [
            createInvariant('absolute_pose')
        ]
    },
    distance: {
        modeTag: 'distance-target',
        commandKind: 'relative_distance',
        activeJointScope: 'arms',
        invariants: () => [
            createInvariant('absolute_pose')
        ]
    },
    'distance-fullbody': {
        modeTag: 'distance-target',
        commandKind: 'relative_distance',
        activeJointScope: 'waist+arms',
        invariants: () => [
            createInvariant('absolute_pose')
        ]
    },
    robot1: {
        modeTag: 'robot1-target',
        commandKind: 'single_arm_absolute_pose',
        activeJointScope: 'left-arm',
        invariants: () => [
            createInvariant('right_arm_fixed', null, 1, 100),
            createInvariant('waist_fixed', null, 0.5, 50)
        ]
    },
    robot2: {
        modeTag: 'robot2-target',
        commandKind: 'single_arm_absolute_pose',
        activeJointScope: 'right-arm',
        invariants: () => [
            createInvariant('left_arm_fixed', null, 1, 100),
            createInvariant('waist_fixed', null, 0.5, 50)
        ]
    },
    'robot1-waist': {
        modeTag: 'robot1-waist-target',
        commandKind: 'single_arm_absolute_pose',
        activeJointScope: 'waist+left-arm',
        invariants: () => [
            createInvariant('right_arm_fixed', null, 1, 100)
        ]
    },
    'robot2-waist': {
        modeTag: 'robot2-waist-target',
        commandKind: 'single_arm_absolute_pose',
        activeJointScope: 'waist+right-arm',
        invariants: () => [
            createInvariant('left_arm_fixed', null, 1, 100)
        ]
    },
    'waist-body-link-xz': {
        modeTag: 'waist-body-link-xz-target',
        commandKind: 'body_link_xz',
        activeJointScope: 'waist',
        invariants: () => [
            createInvariant('left_arm_fixed', null, 1, 100),
            createInvariant('right_arm_fixed', null, 1, 100)
        ]
    },
    'left-ruled-surface-polishing': {
        modeTag: 'left-ruled-surface-polishing-target',
        commandKind: 'single_arm_surface_polishing_pose',
        activeJointScope: 'left-arm',
        invariants: () => [
            createInvariant('right_arm_fixed', null, 1, 100),
            createInvariant('waist_fixed', null, 0.5, 50)
        ]
    },
    'cooperative-ruled-surface-polishing': {
        modeTag: 'cooperative-ruled-surface-polishing-target',
        commandKind: 'bimanual_surface_polishing_pose',
        activeJointScope: 'waist+arms',
        invariants: () => [],
        extraConstraints: () => [
            createExtraConstraint('body_z', { upright: true }, 1, 100, 'body_z/upright')
        ]
    },
    'cooperative-ruled-surface-polishing-dq-setpoint': {
        modeTag: 'cooperative-ruled-surface-polishing-dq-setpoint-target',
        commandKind: 'bimanual_surface_polishing_pose_dq_setpoint',
        activeJointScope: 'waist+arms',
        invariants: () => [],
        extraConstraints: () => [
            createExtraConstraint('body_z', { upright: true }, 1, 100, 'body_z/upright')
        ]
    },
    'cooperative-ruled-surface-polishing-dq-setpoint-arm-only': {
        modeTag: 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only-target',
        commandKind: 'bimanual_surface_polishing_pose_dq_setpoint',
        activeJointScope: 'arms',
        invariants: () => [],
        extraConstraints: () => []
    }
};

export function buildHumanoidJogMJTaskDescriptor(input = {}) {
    const { controlMode = '', commandTarget = null } = input;
    const config = MODE_CONFIGS[controlMode];

    if (!config) {
        return buildUnsupportedDescriptor(commandTarget);
    }

    const invariants = typeof config.invariants === 'function' ? config.invariants(input) : [];
    const extraConstraints = typeof config.extraConstraints === 'function' ? config.extraConstraints(input) : [];

    return {
        valid: true,
        modeTag: config.modeTag,
        commandKind: config.commandKind,
        commandTarget,
        commandDiagnosticLabel: `command:${config.commandKind}`,
        activeJointScope: config.activeJointScope,
        invariants,
        extraConstraints,
        diagnosticLabel: `mode:${config.modeTag}`
    };
}

export default buildHumanoidJogMJTaskDescriptor;
