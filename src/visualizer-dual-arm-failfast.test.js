import test from 'node:test';
import assert from 'node:assert/strict';

import { RobotVisualizer, getHumanoidJogTranslationAxisGuard } from './visualizer.js';

function withWindow(value, fn) {
    const previous = globalThis.window;
    globalThis.window = value;
    try {
        return fn();
    } finally {
        globalThis.window = previous;
    }
}

test('RobotVisualizer cooperative dual-arm helpers fail fast without dualArmSystem fallback', () => {
    withWindow({ humanoidMode: true, dualArmSystem: null, DQModule: null }, () => {
        const visualizer = new RobotVisualizer(null);
        visualizer.humanoidModel = {
            fkmEffector: () => {
                throw new Error('local cooperative fallback should not be used');
            }
        };
        const q = new Array(20).fill(0);

        assert.equal(visualizer._getHumanoidAbsolutePose(q), null);
        assert.equal(visualizer._getHumanoidRelativePose(q), null);
        assert.equal(visualizer._getHumanoidAbsolutePoseJacobian(q), null);
        assert.equal(visualizer._getHumanoidRelativePoseJacobian(q), null);
    });
});

test('RobotVisualizer cooperative dual-arm helpers return null when adapter throws fail-fast errors', () => {
    withWindow({
        humanoidMode: true,
        dualArmSystem: {
            getAbsolutePose: () => {
                throw new Error('cooperative dual-arm WASM API unavailable');
            },
            getRelativePose: () => {
                throw new Error('cooperative dual-arm WASM API unavailable');
            },
            getAbsolutePoseJacobian: () => {
                throw new Error('cooperative dual-arm WASM API unavailable');
            },
            getRelativePoseJacobian: () => {
                throw new Error('cooperative dual-arm WASM API unavailable');
            }
        },
        DQModule: null
    }, () => {
        const visualizer = new RobotVisualizer(null);
        const q = new Array(20).fill(0);

        assert.equal(visualizer._getHumanoidAbsolutePose(q), null);
        assert.equal(visualizer._getHumanoidRelativePose(q), null);
        assert.equal(visualizer._getHumanoidAbsolutePoseJacobian(q), null);
        assert.equal(visualizer._getHumanoidRelativePoseJacobian(q), null);
    });
});

test('getHumanoidJogTranslationAxisGuard extracts the guard from internal options', () => {
    const guard = { commandDelta: [0.01, 0, 0] };
    assert.equal(getHumanoidJogTranslationAxisGuard({ translationAxisGuard: guard }), guard);
    assert.equal(getHumanoidJogTranslationAxisGuard(null), null);
});
