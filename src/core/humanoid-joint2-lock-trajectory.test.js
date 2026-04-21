import test from 'node:test';
import assert from 'node:assert/strict';

import {
    JOINT2_LOCK_TRAJECTORY_CASE,
    buildJoint2LockTrajectoryPlotPayload
} from './humanoid-joint2-lock-trajectory.js';

test('JOINT2_LOCK_TRAJECTORY_CASE defines the absolute dual-arm joint2-lock scenario', () => {
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.taskKind, 'absolute');
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.structure, 'dual-arm');
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.controlMode, 'absolute');
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.includeWaist, false);
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.keepBodyZVertical, false);
    assert.equal(JOINT2_LOCK_TRAJECTORY_CASE.joints.length, 2);
    assert.deepEqual(
        JOINT2_LOCK_TRAJECTORY_CASE.joints.map((entry) => ({
            name: entry.name,
            index: entry.index,
            lower: entry.lower,
            upper: entry.upper
        })),
        [
            { name: 'ljoint2', index: 5, lower: -0.1, upper: 0.2 },
            { name: 'rjoint2', index: 12, lower: -0.2, upper: 0.1 }
        ]
    );
});

test('buildJoint2LockTrajectoryPlotPayload reshapes metric trajectories for both joint2 bands', () => {
    const payload = buildJoint2LockTrajectoryPlotPayload({
        repeatCount: 2,
        samplesByMetric: {
            w_abs_hier: [
                { step: 0, q: [0, 0, 0, 0, 0, 0.19, 0, 0, 0, 0, 0, 0, -0.19] },
                { step: 1, q: [0, 0, 0, 0, 0, 0.17, 0, 0, 0, 0, 0, 0, -0.17] },
                { step: 2, q: [0, 0, 0, 0, 0, 0.15, 0, 0, 0, 0, 0, 0, -0.15] }
            ],
            w_abs_hier_vfi: [
                { step: 0, q: [0, 0, 0, 0, 0, 0.19, 0, 0, 0, 0, 0, 0, -0.19] },
                { step: 1, q: [0, 0, 0, 0, 0, 0.16, 0, 0, 0, 0, 0, 0, -0.16] },
                { step: 2, q: [0, 0, 0, 0, 0, 0.14, 0, 0, 0, 0, 0, 0, -0.14] }
            ]
        }
    });

    assert.equal(payload.repeatCount, 2);
    assert.equal(payload.title, 'Absolute | dual-arm | joint2 lock');
    assert.deepEqual(payload.metrics.map((entry) => entry.label), ['w_hier+', 'w_hier_vfi+']);
    assert.equal(payload.joints.length, 2);
    assert.deepEqual(
        payload.joints.map((entry) => ({
            name: entry.name,
            lower: entry.lower,
            upper: entry.upper,
            wHier: entry.series[0].values,
            wHierVfi: entry.series[1].values
        })),
        [
            {
                name: 'ljoint2',
                lower: -0.1,
                upper: 0.2,
                wHier: [0.19, 0.17, 0.15],
                wHierVfi: [0.19, 0.16, 0.14]
            },
            {
                name: 'rjoint2',
                lower: -0.2,
                upper: 0.1,
                wHier: [-0.19, -0.17, -0.15],
                wHierVfi: [-0.19, -0.16, -0.14]
            }
        ]
    );
});
