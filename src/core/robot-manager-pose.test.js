import test from 'node:test';
import assert from 'node:assert/strict';

test('updateHumanoidPose degrades gracefully when humanoid FK is unavailable', async () => {
    const previousWindow = globalThis.window;
    const dqWrapper = {
        createFromArray: (value) => ({ value }),
        getTranslation: () => [0, 0, 0, 0],
        getRotation: () => [1, 0, 0, 0],
        translation: (x, y, z) => ({ x, y, z }),
        rotation: (yaw, x, y, z) => ({ yaw, x, y, z }),
        multiply: (a, b) => ({ a, b }),
        toArray: () => [1, 0, 0, 0, 0, 0, 0, 0],
        inverse: (value) => ({ inverse: value })
    };
    globalThis.window = {
        createDQRoboticsModule: async () => ({ DQWrapper: dqWrapper })
    };

    const { initDQRobotics } = await import('../dqrobotics.js');
    const { appState, callbacks } = await import('./state.js');
    const { updateHumanoidPose } = await import('./robot-manager.js');
    await initDQRobotics();

    const previousState = {
        humanoidModel: appState.humanoidModel,
        humanoidQ: appState.humanoidQ,
        humanoidBasePose: appState.humanoidBasePose,
        visualizer: appState.visualizer,
        applyHumanoidBasePose: callbacks.applyHumanoidBasePose,
        updateHumanoidJointSlidersFromAngles: callbacks.updateHumanoidJointSlidersFromAngles
    };
    const previousError = console.error;
    const previousWarn = console.warn;
    const errors = [];
    const warnings = [];

    console.error = (...args) => errors.push(args);
    console.warn = (...args) => warnings.push(args);

    let sliderUpdates = 0;
    let setJointOptions = null;
    appState.humanoidBasePose = { x: 0, y: 0, yaw: 0 };
    appState.humanoidQ = new Array(20).fill(0);
    appState.humanoidModel = {
        fkmEffector: () => {
            throw new WebAssembly.RuntimeError('memory access out of bounds');
        }
    };
    appState.visualizer = {
        setHumanoidJoints: (_q, options) => {
            setJointOptions = options;
        }
    };
    callbacks.applyHumanoidBasePose = () => {};
    callbacks.updateHumanoidJointSlidersFromAngles = () => {
        sliderUpdates += 1;
    };

    try {
        assert.doesNotThrow(() => updateHumanoidPose());
        assert.equal(sliderUpdates, 1);
        assert.deepEqual(setJointOptions, { skipDQKinematics: true });
        assert.equal(errors.length, 0);
        assert.ok(warnings.some((args) => String(args[0]).includes('Humanoid FK display unavailable')));
    } finally {
        appState.humanoidModel = previousState.humanoidModel;
        appState.humanoidQ = previousState.humanoidQ;
        appState.humanoidBasePose = previousState.humanoidBasePose;
        appState.visualizer = previousState.visualizer;
        callbacks.applyHumanoidBasePose = previousState.applyHumanoidBasePose;
        callbacks.updateHumanoidJointSlidersFromAngles = previousState.updateHumanoidJointSlidersFromAngles;
        console.error = previousError;
        console.warn = previousWarn;
        globalThis.window = previousWindow;
    }
});
