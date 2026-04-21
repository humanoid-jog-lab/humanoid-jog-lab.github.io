import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogMJTaskDescriptor } from './humanoid-jog-mj-task-adapter.js';

function assertInvariant(resultInvariant, expected) {
    assert.equal(resultInvariant.kind, expected.kind);
    assert.equal(resultInvariant.weight, expected.weight);
    assert.equal(resultInvariant.priorityEquivalent, expected.priorityEquivalent);
    assert.equal(resultInvariant.diagnosticLabel, expected.diagnosticLabel);
}

function assertDescriptor(input, expected) {
    const result = buildHumanoidJogMJTaskDescriptor(input);

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, expected.modeTag);
    assert.equal(result.commandKind, expected.commandKind);
    assert.deepEqual(result.commandTarget, expected.commandTarget);
    assert.equal(result.commandDiagnosticLabel, expected.commandDiagnosticLabel);
    assert.equal(result.activeJointScope, expected.activeJointScope);
    assert.equal(result.diagnosticLabel, expected.diagnosticLabel);
    assert.deepEqual(result.extraConstraints, expected.extraConstraints ?? []);
    assert.equal(result.invariants.length, expected.invariants.length);
    for (let i = 0; i < expected.invariants.length; i++) {
        assertInvariant(result.invariants[i], expected.invariants[i]);
    }
}

test('buildHumanoidJogMJTaskDescriptor builds canonical absolute and decoupled descriptors', () => {
    assertDescriptor(
        {
            controlMode: 'absolute',
            commandTarget: { pose: 'absolute-target-pose' }
        },
        {
            modeTag: 'absolute-target',
            commandKind: 'absolute_pose',
            commandTarget: { pose: 'absolute-target-pose' },
            commandDiagnosticLabel: 'command:absolute_pose',
            activeJointScope: 'arms',
            diagnosticLabel: 'mode:absolute-target',
            invariants: [
                {
                    kind: 'relative_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:relative_pose'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'decoupled',
            commandTarget: { pose: 'decoupled-target-pose' }
        },
        {
            modeTag: 'absolute-target',
            commandKind: 'absolute_pose',
            commandTarget: { pose: 'decoupled-target-pose' },
            commandDiagnosticLabel: 'command:absolute_pose',
            activeJointScope: 'waist+arms',
            diagnosticLabel: 'mode:absolute-target',
            invariants: [
                {
                    kind: 'relative_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:relative_pose'
                }
            ]
        }
    );
});

test('buildHumanoidJogMJTaskDescriptor adds body-z upright metadata for decoupled-upright', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'decoupled-upright',
        commandTarget: { pose: 'upright-target-pose' }
    });

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'absolute-target');
    assert.equal(result.commandKind, 'absolute_pose');
    assert.deepEqual(result.commandTarget, { pose: 'upright-target-pose' });
    assert.equal(result.commandDiagnosticLabel, 'command:absolute_pose');
    assert.equal(result.activeJointScope, 'waist+arms');
    assert.equal(result.diagnosticLabel, 'mode:absolute-target');
    assert.equal(result.invariants.length, 1);
    assertInvariant(result.invariants[0], {
        kind: 'relative_pose',
        weight: 1,
        priorityEquivalent: 100,
        diagnosticLabel: 'invariant:relative_pose'
    });
    assert.deepEqual(result.extraConstraints, [
        {
            kind: 'body_z',
            target: { upright: true },
            diagnosticLabel: 'extra-constraint:body_z/upright',
            weight: 1,
            priorityEquivalent: 100
        }
    ]);
});

test('buildHumanoidJogMJTaskDescriptor builds canonical relative and distance family descriptors', () => {
    assertDescriptor(
        {
            controlMode: 'relative',
            commandTarget: { pose: 'relative-target-pose' }
        },
        {
            modeTag: 'relative-target',
            commandKind: 'relative_pose',
            commandTarget: { pose: 'relative-target-pose' },
            commandDiagnosticLabel: 'command:relative_pose',
            activeJointScope: 'arms',
            diagnosticLabel: 'mode:relative-target',
            invariants: [
                {
                    kind: 'absolute_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:absolute_pose'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'relative-fullbody',
            commandTarget: { pose: 'relative-fullbody-target-pose' }
        },
        {
            modeTag: 'relative-target',
            commandKind: 'relative_pose',
            commandTarget: { pose: 'relative-fullbody-target-pose' },
            commandDiagnosticLabel: 'command:relative_pose',
            activeJointScope: 'waist+arms',
            diagnosticLabel: 'mode:relative-target',
            invariants: [
                {
                    kind: 'absolute_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:absolute_pose'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'distance',
            commandTarget: { distance: 0.45 }
        },
        {
            modeTag: 'distance-target',
            commandKind: 'relative_distance',
            commandTarget: { distance: 0.45 },
            commandDiagnosticLabel: 'command:relative_distance',
            activeJointScope: 'arms',
            diagnosticLabel: 'mode:distance-target',
            invariants: [
                {
                    kind: 'absolute_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:absolute_pose'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'distance-fullbody',
            commandTarget: { distance: 0.82 }
        },
        {
            modeTag: 'distance-target',
            commandKind: 'relative_distance',
            commandTarget: { distance: 0.82 },
            commandDiagnosticLabel: 'command:relative_distance',
            activeJointScope: 'waist+arms',
            diagnosticLabel: 'mode:distance-target',
            invariants: [
                {
                    kind: 'absolute_pose',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:absolute_pose'
                }
            ]
        }
    );
});

test('buildHumanoidJogMJTaskDescriptor builds canonical special-mode descriptors', () => {
    assertDescriptor(
        {
            controlMode: 'robot1',
            commandTarget: { pose: 'left-arm-pose' }
        },
        {
            modeTag: 'robot1-target',
            commandKind: 'single_arm_absolute_pose',
            commandTarget: { pose: 'left-arm-pose' },
            commandDiagnosticLabel: 'command:single_arm_absolute_pose',
            activeJointScope: 'left-arm',
            diagnosticLabel: 'mode:robot1-target',
            invariants: [
                {
                    kind: 'right_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:right_arm_fixed'
                },
                {
                    kind: 'waist_fixed',
                    weight: 0.5,
                    priorityEquivalent: 50,
                    diagnosticLabel: 'invariant:waist_fixed'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'robot2',
            commandTarget: { pose: 'right-arm-pose' }
        },
        {
            modeTag: 'robot2-target',
            commandKind: 'single_arm_absolute_pose',
            commandTarget: { pose: 'right-arm-pose' },
            commandDiagnosticLabel: 'command:single_arm_absolute_pose',
            activeJointScope: 'right-arm',
            diagnosticLabel: 'mode:robot2-target',
            invariants: [
                {
                    kind: 'left_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:left_arm_fixed'
                },
                {
                    kind: 'waist_fixed',
                    weight: 0.5,
                    priorityEquivalent: 50,
                    diagnosticLabel: 'invariant:waist_fixed'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'robot1-waist',
            commandTarget: { pose: 'left-arm-waist-pose' }
        },
        {
            modeTag: 'robot1-waist-target',
            commandKind: 'single_arm_absolute_pose',
            commandTarget: { pose: 'left-arm-waist-pose' },
            commandDiagnosticLabel: 'command:single_arm_absolute_pose',
            activeJointScope: 'waist+left-arm',
            diagnosticLabel: 'mode:robot1-waist-target',
            invariants: [
                {
                    kind: 'right_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:right_arm_fixed'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'robot2-waist',
            commandTarget: { pose: 'right-arm-waist-pose' }
        },
        {
            modeTag: 'robot2-waist-target',
            commandKind: 'single_arm_absolute_pose',
            commandTarget: { pose: 'right-arm-waist-pose' },
            commandDiagnosticLabel: 'command:single_arm_absolute_pose',
            activeJointScope: 'waist+right-arm',
            diagnosticLabel: 'mode:robot2-waist-target',
            invariants: [
                {
                    kind: 'left_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:left_arm_fixed'
                }
            ]
        }
    );

    assertDescriptor(
        {
            controlMode: 'waist-body-link-xz',
            commandTarget: { x: 0.15, z: -0.07 }
        },
        {
            modeTag: 'waist-body-link-xz-target',
            commandKind: 'body_link_xz',
            commandTarget: { x: 0.15, z: -0.07 },
            commandDiagnosticLabel: 'command:body_link_xz',
            activeJointScope: 'waist',
            diagnosticLabel: 'mode:waist-body-link-xz-target',
            invariants: [
                {
                    kind: 'left_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:left_arm_fixed'
                },
                {
                    kind: 'right_arm_fixed',
                    weight: 1,
                    priorityEquivalent: 100,
                    diagnosticLabel: 'invariant:right_arm_fixed'
                }
            ]
        }
    );
});

test('buildHumanoidJogMJTaskDescriptor rejects removed surface-slide control modes', () => {
    const removedModes = [
        'surface-slide',
        'surface-slide-absolute',
        'dual-independent-surface-slide'
    ];

    removedModes.forEach((controlMode) => {
        const result = buildHumanoidJogMJTaskDescriptor({
            controlMode,
            commandTarget: { surface: 'demo-surface' }
        });

        assert.equal(result.valid, false);
        assert.equal(result.modeTag, 'unsupported-target');
        assert.equal(result.commandKind, null);
        assert.equal(result.commandDiagnosticLabel, 'command:unsupported');
        assert.equal(result.activeJointScope, null);
        assert.deepEqual(result.invariants, []);
        assert.deepEqual(result.extraConstraints, []);
        assert.equal(result.diagnosticLabel, 'mode_unsupported');
    });
});

test('buildHumanoidJogMJTaskDescriptor builds a cooperative ruled-surface polishing descriptor', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'cooperative-ruled-surface-polishing',
        commandTarget: {
            surface: 'demo-surface',
            u: 0.35,
            vCenter: 0.5,
            stripSpacing: 0.24,
            normalOffset: 0.06
        }
    });

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'cooperative-ruled-surface-polishing-target');
    assert.equal(result.commandKind, 'bimanual_surface_polishing_pose');
    assert.deepEqual(result.commandTarget, {
        surface: 'demo-surface',
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06
    });
    assert.equal(result.commandDiagnosticLabel, 'command:bimanual_surface_polishing_pose');
    assert.equal(result.activeJointScope, 'waist+arms');
    assert.equal(result.diagnosticLabel, 'mode:cooperative-ruled-surface-polishing-target');
    assert.deepEqual(result.invariants, []);
    assert.equal(result.extraConstraints.length, 1);
    assert.deepEqual(result.extraConstraints[0], {
        kind: 'body_z',
        target: { upright: true },
        weight: 1,
        priorityEquivalent: 100,
        diagnosticLabel: 'extra-constraint:body_z/upright'
    });
});

test('buildHumanoidJogMJTaskDescriptor builds a cooperative ruled-surface polishing DQ set-point descriptor', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'cooperative-ruled-surface-polishing-dq-setpoint',
        commandTarget: {
            left: { position: [0.1, 0.2, 0.3], quaternion: [1, 0, 0, 0] },
            right: { position: [0.4, 0.5, 0.6], quaternion: [1, 0, 0, 0] }
        }
    });

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'cooperative-ruled-surface-polishing-dq-setpoint-target');
    assert.equal(result.commandKind, 'bimanual_surface_polishing_pose_dq_setpoint');
    assert.equal(result.activeJointScope, 'waist+arms');
    assert.equal(result.commandDiagnosticLabel, 'command:bimanual_surface_polishing_pose_dq_setpoint');
    assert.equal(result.diagnosticLabel, 'mode:cooperative-ruled-surface-polishing-dq-setpoint-target');
    assert.deepEqual(result.invariants, []);
    assert.equal(result.extraConstraints.length, 1);
    assert.deepEqual(result.extraConstraints[0], {
        kind: 'body_z',
        target: { upright: true },
        weight: 1,
        priorityEquivalent: 100,
        diagnosticLabel: 'extra-constraint:body_z/upright'
    });
});

test('buildHumanoidJogMJTaskDescriptor builds an arm-only cooperative ruled-surface polishing DQ set-point descriptor', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only',
        commandTarget: {
            left: { position: [0.1, 0.2, 0.3], quaternion: [1, 0, 0, 0] },
            right: { position: [0.4, 0.5, 0.6], quaternion: [1, 0, 0, 0] }
        }
    });

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only-target');
    assert.equal(result.commandKind, 'bimanual_surface_polishing_pose_dq_setpoint');
    assert.equal(result.activeJointScope, 'arms');
    assert.equal(result.commandDiagnosticLabel, 'command:bimanual_surface_polishing_pose_dq_setpoint');
    assert.equal(result.diagnosticLabel, 'mode:cooperative-ruled-surface-polishing-dq-setpoint-arm-only-target');
    assert.deepEqual(result.invariants, []);
    assert.deepEqual(result.extraConstraints, []);
});

test('buildHumanoidJogMJTaskDescriptor builds a left ruled-surface polishing descriptor', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'left-ruled-surface-polishing',
        commandTarget: {
            surface: 'demo-surface',
            u: 0.3,
            v: 0.5,
            normalOffset: 0.05
        }
    });

    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'left-ruled-surface-polishing-target');
    assert.equal(result.commandKind, 'single_arm_surface_polishing_pose');
    assert.deepEqual(result.commandTarget, {
        surface: 'demo-surface',
        u: 0.3,
        v: 0.5,
        normalOffset: 0.05
    });
    assert.equal(result.commandDiagnosticLabel, 'command:single_arm_surface_polishing_pose');
    assert.equal(result.activeJointScope, 'left-arm');
    assert.equal(result.diagnosticLabel, 'mode:left-ruled-surface-polishing-target');
    assert.deepEqual(
        result.invariants.map((item) => item.kind),
        ['right_arm_fixed', 'waist_fixed']
    );
    assert.deepEqual(result.extraConstraints, []);
});

test('buildHumanoidJogMJTaskDescriptor returns a valid:false result for unsupported modes', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'unsupported-mode',
        commandTarget: { any: 'thing' }
    });

    assert.equal(result.valid, false);
    assert.equal(result.modeTag, 'unsupported-target');
    assert.equal(result.commandKind, null);
    assert.deepEqual(result.commandTarget, { any: 'thing' });
    assert.equal(result.commandDiagnosticLabel, 'command:unsupported');
    assert.equal(result.activeJointScope, null);
    assert.equal(result.diagnosticLabel, 'mode_unsupported');
    assert.deepEqual(result.invariants, []);
    assert.deepEqual(result.extraConstraints, []);
});
