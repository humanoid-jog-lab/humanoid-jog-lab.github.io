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

test('RobotVisualizer skips humanoid DQ effector markers when FK throws', () => {
    const previousWarn = console.warn;
    const warnings = [];
    console.warn = (...args) => warnings.push(args);

    try {
        withWindow({
            humanoidMode: true,
            humanoidQ: new Array(20).fill(0),
            DQModule: {
                DQWrapper: {
                    createFromArray: () => ({}),
                    getTranslation: () => [0, 0, 0, 0],
                    getRotation: () => [1, 0, 0, 0]
                }
            }
        }, () => {
            const visualizer = new RobotVisualizer(null);
            const makeLink = () => ({
                getWorldPosition: (target) => target.set(0, 0, 0)
            });
            visualizer.humanoidURDF = {
                links: {
                    waist: makeLink(),
                    left: makeLink(),
                    right: makeLink(),
                    head: makeLink()
                }
            };
            visualizer.humanoidEffectorLinkNames = ['waist', 'left', 'right', 'head'];
            visualizer.humanoidEffectorSpheres = {
                children: Array.from({ length: 4 }, () => ({
                    position: { copy: () => {} }
                }))
            };
            visualizer.humanoidWaistEndEffector = {};
            visualizer.humanoidModel = {
                getDimConfigurationSpace: () => 20,
                fkmEffector: () => {
                    throw new Error('fk unavailable');
                }
            };
            visualizer._updateHumanoidSurfaceSlideVisualization = () => {};

            assert.doesNotThrow(() => visualizer._updateHumanoidEffectorSpheres(window.humanoidQ));
            assert.ok(warnings.some((args) => String(args[0]).includes('Humanoid effector FK unavailable')));
        });
    } finally {
        console.warn = previousWarn;
    }
});

test('getHumanoidJogTranslationAxisGuard extracts the guard from internal options', () => {
    const guard = { commandDelta: [0.01, 0, 0] };
    assert.equal(getHumanoidJogTranslationAxisGuard({ translationAxisGuard: guard }), guard);
    assert.equal(getHumanoidJogTranslationAxisGuard(null), null);
});

test('RobotVisualizer decoupled upright absolute translation returns a failed jog result to stop repeats', () => {
    const previousDocument = globalThis.document;
    globalThis.document = {
        getElementById(id) {
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    try {
        withWindow({
            humanoidQ: new Array(20).fill(0),
            DQModule: {
                DQWrapper: {
                    createFromArray: (value) => ({ value }),
                    getRotation: () => [1, 0, 0, 0],
                    translation: (x, y, z) => ({ x, y, z }),
                    multiply: (a, b) => ({ a, b }),
                    toArray: () => [1, 0, 0, 0, 0.005, 0, 0, 0]
                }
            }
        }, () => {
            const visualizer = new RobotVisualizer(null);
            visualizer.humanoidMode = true;
            visualizer.getTeachTranslationStep = () => 0.005;
            visualizer.getTranslationDirection = () => ({
                multiplyScalar(scale) {
                    return { x: scale, y: 0, z: 0 };
                }
            });
            visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
            visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst = () => false;

            assert.equal(
                visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright('x', 'negative'),
                false
            );
        });
    } finally {
        if (previousDocument === undefined) delete globalThis.document;
        else globalThis.document = previousDocument;
    }
});

test('RobotVisualizer decoupled absolute translation also returns a failed jog result to stop repeats', () => {
    const previousDocument = globalThis.document;
    globalThis.document = {
        getElementById(id) {
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    try {
        withWindow({
            humanoidQ: new Array(20).fill(0),
            DQModule: {
                DQWrapper: {
                    createFromArray: (value) => ({ value }),
                    getRotation: () => [1, 0, 0, 0],
                    translation: (x, y, z) => ({ x, y, z }),
                    multiply: (a, b) => ({ a, b }),
                    toArray: () => [1, 0, 0, 0, 0.005, 0, 0, 0]
                }
            }
        }, () => {
            const visualizer = new RobotVisualizer(null);
            visualizer.humanoidMode = true;
            visualizer.getTeachTranslationStep = () => 0.005;
            visualizer.getTranslationDirection = () => ({
                multiplyScalar(scale) {
                    return { x: scale, y: 0, z: 0 };
                }
            });
            visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
            visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst = () => false;

            assert.equal(
                visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupled('x', 'negative'),
                false
            );
        });
    } finally {
        if (previousDocument === undefined) delete globalThis.document;
        else globalThis.document = previousDocument;
    }
});

test('RobotVisualizer decoupled upright absolute translation propagates async TR safety-bias failures', async () => {
    const previousDocument = globalThis.document;
    globalThis.document = {
        getElementById(id) {
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    try {
        await withWindow({
            humanoidQ: new Array(20).fill(0),
            DQModule: {
                DQWrapper: {
                    createFromArray: (value) => ({ value }),
                    getRotation: () => [1, 0, 0, 0],
                    translation: (x, y, z) => ({ x, y, z }),
                    multiply: (a, b) => ({ a, b }),
                    toArray: () => [1, 0, 0, 0, 0.005, 0, 0, 0]
                }
            }
        }, async () => {
            const visualizer = new RobotVisualizer(null);
            visualizer.humanoidMode = true;
            visualizer.getTeachTranslationStep = () => 0.005;
            visualizer.getTranslationDirection = () => ({
                multiplyScalar(scale) {
                    return { x: scale, y: 0, z: 0 };
                }
            });
            visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr_safe';
            visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
            visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst = () => Promise.resolve(false);

            assert.equal(
                await visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright('x', 'negative'),
                false
            );
        });
    } finally {
        if (previousDocument === undefined) delete globalThis.document;
        else globalThis.document = previousDocument;
    }
});

test('RobotVisualizer absolute update propagates async mj_restore_tr_safe failures', async () => {
    const previousWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    try {
        const visualizer = new RobotVisualizer(null);
        visualizer.humanoidModel = {};
        visualizer.humanoidMode = true;
        visualizer.setHumanoidJogSolverMode('mj_restore_tr_safe');
        visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
        visualizer._getHumanoidRelativePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
        visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = () => Promise.resolve(false);

        const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
            [1, 0, 0, 0, 0.005, 0, 0, 0],
            globalThis.window.humanoidQ,
            true,
            true,
            true,
            null,
            { commandDelta: [-0.005, 0, 0] }
        );

        assert.equal(result, false);
    } finally {
        if (previousWindow === undefined) delete globalThis.window;
        else globalThis.window = previousWindow;
    }
});
