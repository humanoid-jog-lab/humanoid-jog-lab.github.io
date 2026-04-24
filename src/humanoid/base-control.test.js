import test from 'node:test';
import assert from 'node:assert/strict';

import { appState } from '../core/state.js';
import { applyHumanoidBasePose } from './base-control.js';

function expectedBaseDQ(x, y, yaw) {
    const halfYaw = 0.5 * yaw;
    const c = Math.cos(halfYaw);
    const s = Math.sin(halfYaw);
    return [
        c, 0, 0, s,
        0,
        0.5 * (x * c + y * s),
        0.5 * (-x * s + y * c),
        0
    ];
}

function assertCloseArray(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let i = 0; i < expected.length; i += 1) {
        assert.ok(
            Math.abs(actual[i] - expected[i]) <= tolerance,
            `index ${i}: expected ${expected[i]}, got ${actual[i]}`
        );
    }
}

test('applyHumanoidBasePose updates the reference frame without constructing a WASM DQ pose', () => {
    const previousModel = appState.humanoidModel;
    const previousPose = appState.humanoidBasePose;
    const previousVisualizer = appState.visualizer;
    const captured = {};

    appState.humanoidBasePose = { x: 0.42, y: -0.13, yaw: 0.7 };
    appState.humanoidModel = {
        setReferenceFrame: (dqBase) => {
            captured.dqBase = dqBase;
        }
    };
    appState.visualizer = {
        setHumanoidBasePose: (pose) => {
            captured.pose = pose;
        }
    };

    try {
        assert.doesNotThrow(() => applyHumanoidBasePose());
        assertCloseArray(captured.dqBase, expectedBaseDQ(0.42, -0.13, 0.7));
        assert.deepEqual(captured.pose, { x: 0.42, y: -0.13, yaw: 0.7 });
    } finally {
        appState.humanoidModel = previousModel;
        appState.humanoidBasePose = previousPose;
        appState.visualizer = previousVisualizer;
    }
});
