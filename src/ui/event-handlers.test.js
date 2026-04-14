import test from 'node:test';
import assert from 'node:assert/strict';

import { appState, callbacks } from '../core/state.js';
import {
    setupHumanoidDefaultPoseControls,
    setupHumanoidJogSolverControls,
    setupHumanoidCooperativePolishingSurfaceControls,
    setupHumanoidCooperativePolishingObjectPoseControls
} from './event-handlers.js';

function createMockLocalStorage(initialEntries = []) {
    const store = new Map(initialEntries);
    return {
        getItem(key) {
            return store.has(key) ? store.get(key) : null;
        },
        setItem(key, value) {
            store.set(key, String(value));
        },
        removeItem(key) {
            store.delete(key);
        }
    };
}

function createMockElement(initialValue = '') {
    const listeners = new Map();
    return {
        value: initialValue,
        checked: false,
        textContent: '',
        dataset: {},
        addEventListener(eventName, handler) {
            listeners.set(eventName, handler);
        },
        trigger(eventName, nextValue) {
            const handler = listeners.get(eventName);
            if (typeof handler !== 'function') return;
            handler({
                target: {
                    value: nextValue,
                    checked: Boolean(nextValue)
                }
            });
        }
    };
}

function createMockSelect(initialValue = '', optionValues = []) {
    const element = createMockElement(initialValue);
    element.options = optionValues.map((value) => ({
        value,
        textContent: value
    }));
    return element;
}

function multiplyQuaternions(a, b) {
    const [aw, ax, ay, az] = a;
    const [bw, bx, by, bz] = b;
    return [
        (aw * bw) - (ax * bx) - (ay * by) - (az * bz),
        (aw * bx) + (ax * bw) + (ay * bz) - (az * by),
        (aw * by) - (ax * bz) + (ay * bw) + (az * bx),
        (aw * bz) + (ax * by) - (ay * bx) + (az * bw)
    ];
}

const NORMAL_FLIP_QUATERNION = [0, 0, 1, 0];

function extractYawDegrees(quaternionLike) {
    const [w, x, y, z] = quaternionLike;
    const sinyCosp = 2 * ((w * z) + (x * y));
    const cosyCosp = 1 - (2 * ((y * y) + (z * z)));
    return Math.atan2(sinyCosp, cosyCosp) * (180 / Math.PI);
}

test('setupHumanoidDefaultPoseControls restores the saved humanoid default profile and applies it to the current humanoid on click', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalHumanoidQ = appState.humanoidQ;
    const originalHumanoidJointNames = appState.humanoidJointNames;
    const originalHumanoidMode = appState.humanoidMode;
    const originalUpdateRobotPose = callbacks.updateRobotPose;
    const originalUpdateGlobalVariables = callbacks.updateGlobalVariables;

    const profile1Btn = createMockElement('');
    const profile2Btn = createMockElement('');
    const updateRobotPoseCalls = [];
    const resetSurfaceSlideCalls = [];
    const updateGlobalVariablesCalls = [];

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-default-q-profile-1-btn') return profile1Btn;
            if (id === 'humanoid-default-q-profile-2-btn') return profile2Btn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-default-q-profile', 'default2']
    ]);
    appState.visualizer = {
        resetHumanoidSurfaceSlideReference() {
            resetSurfaceSlideCalls.push('reset');
        }
    };
    appState.humanoidMode = true;
    appState.humanoidJointNames = Array.from({ length: 20 }, (_, index) => `joint_${index}`);
    appState.humanoidQ = new Array(20).fill(0);

    try {
        callbacks.updateGlobalVariables = () => {
            updateGlobalVariablesCalls.push(appState.humanoidQ.slice());
            globalThis.window.humanoidQ = appState.humanoidQ.slice();
        };
        callbacks.updateRobotPose = () => {
            updateRobotPoseCalls.push(appState.humanoidQ.slice());
        };
        appState.visualizer.resetHumanoidSurfaceSlideReference = () => {
            resetSurfaceSlideCalls.push(globalThis.window.humanoidQ.slice());
        };

        setupHumanoidDefaultPoseControls();

        assert.equal(globalThis.window.humanoidInitialQProfileId, 'default2');
        assert.equal(globalThis.localStorage.getItem('humanoid-default-q-profile'), 'default2');
        assert.equal(profile2Btn.dataset.active, 'true');
        assert.equal(profile1Btn.dataset.active, 'false');

        profile1Btn.trigger('click');

        assert.equal(globalThis.window.humanoidInitialQProfileId, 'default1');
        assert.equal(globalThis.localStorage.getItem('humanoid-default-q-profile'), 'default1');
        assert.equal(profile1Btn.dataset.active, 'true');
        assert.equal(profile2Btn.dataset.active, 'false');
        assert.equal(updateGlobalVariablesCalls.length, 1);
        assert.equal(updateRobotPoseCalls.length, 1);
        assert.equal(resetSurfaceSlideCalls.length, 1);
        assert.equal(appState.humanoidQ.length, 20);
        assert.notDeepEqual(appState.humanoidQ, new Array(20).fill(0));
        assert.deepEqual(globalThis.window.humanoidQ, appState.humanoidQ);
        assert.deepEqual(resetSurfaceSlideCalls[0], appState.humanoidQ);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
        appState.humanoidQ = originalHumanoidQ;
        appState.humanoidJointNames = originalHumanoidJointNames;
        appState.humanoidMode = originalHumanoidMode;
        callbacks.updateRobotPose = originalUpdateRobotPose;
        callbacks.updateGlobalVariables = originalUpdateGlobalVariables;
    }
});

test('setupHumanoidJogSolverControls restores mj_restore_pre_w from storage and propagates it to the UI and visualizer', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalSetInterval = globalThis.setInterval;
    const originalClearInterval = globalThis.clearInterval;

    const humanoidJogSolverSelect = createMockElement('vfi');
    const humanoidJogMetricSelect = createMockSelect('w_abs_hier', ['w_abs_hier', 'w_abs_hier_vfi']);
    const humanoidWMetricIncreaseBtn = createMockElement('');
    const humanoidWaistXCorridorToggle = createMockElement('');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const visualizerCalls = [];
    const metricOptimizationCalls = [];
    const waistXCorridorCalls = [];
    const testLockCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            if (id === 'humanoid-manifold-jog-metric-select') return humanoidJogMetricSelect;
            if (id === 'humanoid-w-metric-increase-btn') return humanoidWMetricIncreaseBtn;
            if (id === 'humanoid-waist-x-corridor-toggle') return humanoidWaistXCorridorToggle;
            if (id === 'humanoid-test-dual-arm-lock-toggle') return humanoidTestDualArmLockToggle;
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-jog-solver-mode', 'mj_restore_pre_w'],
        ['humanoid-manifold-jog-metric', 'w_abs_hier_vfi'],
        ['humanoid-waist-x-corridor-enabled', '1'],
        ['humanoid-test-dual-arm-lock-enabled', '1']
    ]);
    globalThis.setInterval = () => 1;
    globalThis.clearInterval = () => {};
    appState.visualizer = {
        setHumanoidJogSolverMode(mode) {
            visualizerCalls.push(mode);
        },
        setHumanoidManifoldJogSecondaryMetric(metric) {
            metricOptimizationCalls.push(`set:${metric}`);
        },
        setHumanoidWaistXCorridorEnabled(enabled) {
            waistXCorridorCalls.push(enabled);
        },
        optimizeHumanoidHierarchicalAbsoluteManipulability(direction) {
            metricOptimizationCalls.push(`w_abs_hier:${direction}`);
        },
        optimizeHumanoidHierarchicalAbsoluteVFIManipulability(direction) {
            metricOptimizationCalls.push(`w_abs_hier_vfi:${direction}`);
        },
        setHumanoidTestDualArmLockEnabled(enabled) {
            testLockCalls.push(enabled);
        }
    };

    try {
        setupHumanoidJogSolverControls();

        assert.equal(humanoidJogSolverSelect.value, 'mj_restore_pre_w');
        assert.equal(humanoidJogMetricSelect.value, 'w_abs_hier_vfi');
        assert.deepEqual(
            humanoidJogMetricSelect.options.map((option) => option.textContent),
            ['w_abs', 'w_abs_vfi']
        );
        assert.equal(humanoidWaistXCorridorToggle.checked, true);
        assert.equal(humanoidTestDualArmLockToggle.checked, true);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'mj_restore_pre_w');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier_vfi');
        assert.equal(globalThis.window.humanoidWaistXCorridorEnabled, true);
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w']);
        assert.deepEqual(metricOptimizationCalls, ['set:w_abs_hier_vfi']);
        assert.deepEqual(waistXCorridorCalls, [true]);
        assert.deepEqual(testLockCalls, [true]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'mj_restore_pre_w');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier_vfi');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-enabled'), '1');
        assert.equal(globalThis.localStorage.getItem('humanoid-test-dual-arm-lock-enabled'), '1');

        humanoidJogSolverSelect.trigger('change', 'priority');
        humanoidJogMetricSelect.trigger('change', 'w_abs_hier');
        controlModeSelect.trigger('change', 'relative');
        humanoidWMetricIncreaseBtn.trigger('mousedown');
        humanoidWaistXCorridorToggle.trigger('change', false);
        humanoidTestDualArmLockToggle.trigger('change', false);

        assert.equal(humanoidJogSolverSelect.value, 'priority');
        assert.equal(humanoidJogMetricSelect.value, 'w_abs_hier');
        assert.deepEqual(
            humanoidJogMetricSelect.options.map((option) => option.textContent),
            ['w_rel', 'w_rel_vfi']
        );
        assert.equal(humanoidWaistXCorridorToggle.checked, false);
        assert.equal(humanoidTestDualArmLockToggle.checked, false);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'priority');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier');
        assert.equal(globalThis.window.humanoidWaistXCorridorEnabled, false);
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w', 'priority']);
        assert.deepEqual(metricOptimizationCalls, [
            'set:w_abs_hier_vfi',
            'set:w_abs_hier',
            'w_abs_hier:increase'
        ]);
        assert.deepEqual(waistXCorridorCalls, [true, false]);
        assert.deepEqual(testLockCalls, [true, false]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'priority');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-enabled'), '0');
        assert.equal(globalThis.localStorage.getItem('humanoid-test-dual-arm-lock-enabled'), '0');
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
        if (originalDocument === undefined) {
            delete globalThis.document;
        } else {
            globalThis.document = originalDocument;
        }
        if (originalLocalStorage === undefined) {
            delete globalThis.localStorage;
        } else {
            globalThis.localStorage = originalLocalStorage;
        }
        if (originalSetInterval === undefined) {
            delete globalThis.setInterval;
        } else {
            globalThis.setInterval = originalSetInterval;
        }
        if (originalClearInterval === undefined) {
            delete globalThis.clearInterval;
        } else {
            globalThis.clearInterval = originalClearInterval;
        }
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidCooperativePolishingSurfaceControls falls back to analytic when storage contains the removed shell source', async () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const analyticBtn = createMockElement('');
    const cylinderBtn = createMockElement('');
    const sourceCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-cooperative-polishing-surface-source-analytic-btn') return analyticBtn;
            if (id === 'humanoid-cooperative-polishing-surface-source-cylinder-btn') return cylinderBtn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-cooperative-polishing-surface-source', 'strip-ruled-shell']
    ]);
    appState.visualizer = {
        async setHumanoidCooperativeRuledSurfaceSurfaceSource(source) {
            sourceCalls.push(source);
        }
    };

    try {
        const controls = setupHumanoidCooperativePolishingSurfaceControls();
        await Promise.resolve();

        assert.equal(globalThis.window.humanoidCooperativeRuledSurfaceSource, 'analytic');
        assert.equal(analyticBtn.dataset.active, 'true');
        assert.equal(cylinderBtn.dataset.active, 'false');

        analyticBtn.trigger('click');
        await Promise.resolve();

        assert.equal(globalThis.window.humanoidCooperativeRuledSurfaceSource, 'analytic');
        assert.equal(analyticBtn.dataset.active, 'true');
        assert.equal(cylinderBtn.dataset.active, 'false');
        assert.equal(globalThis.localStorage.getItem('humanoid-cooperative-polishing-surface-source'), 'analytic');
        assert.deepEqual(sourceCalls, ['analytic', 'analytic']);
        assert.equal(typeof controls.applyHumanoidCooperativePolishingSurfaceSource, 'function');
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidCooperativePolishingSurfaceControls can switch to the cylindrical patch source', async () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const analyticBtn = createMockElement('');
    const cylinderBtn = createMockElement('');
    const sourceCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-cooperative-polishing-surface-source-analytic-btn') return analyticBtn;
            if (id === 'humanoid-cooperative-polishing-surface-source-cylinder-btn') return cylinderBtn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    appState.visualizer = {
        async setHumanoidCooperativeRuledSurfaceSurfaceSource(source) {
            sourceCalls.push(source);
        }
    };

    try {
        setupHumanoidCooperativePolishingSurfaceControls();
        cylinderBtn.trigger('click');
        await Promise.resolve();

        assert.equal(globalThis.window.humanoidCooperativeRuledSurfaceSource, 'cylindrical-patch');
        assert.equal(cylinderBtn.dataset.active, 'true');
        assert.equal(analyticBtn.dataset.active, 'false');
        assert.equal(globalThis.localStorage.getItem('humanoid-cooperative-polishing-surface-source'), 'cylindrical-patch');
        assert.deepEqual(sourceCalls, ['analytic', 'cylindrical-patch']);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidCooperativePolishingObjectPoseControls applies a fixed object pose and can capture the current work-area pose', async () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const xInput = createMockElement('0.40');
    const yInput = createMockElement('0.10');
    const zInput = createMockElement('-0.08');
    const yawInput = createMockElement('30');
    const flipNormalToggle = createMockElement('');
    flipNormalToggle.checked = true;
    const applyBtn = createMockElement('');
    const captureBtn = createMockElement('');
    const poseCalls = [];
    const capturedPose = {
        position: [0.2, -0.15, -0.05],
        quaternion: [Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8)],
        flipNormal: false
    };

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-cooperative-polishing-object-x-input') return xInput;
            if (id === 'humanoid-cooperative-polishing-object-y-input') return yInput;
            if (id === 'humanoid-cooperative-polishing-object-z-input') return zInput;
            if (id === 'humanoid-cooperative-polishing-object-yaw-input') return yawInput;
            if (id === 'humanoid-cooperative-polishing-object-flip-normal-toggle') return flipNormalToggle;
            if (id === 'humanoid-cooperative-polishing-object-apply-btn') return applyBtn;
            if (id === 'humanoid-cooperative-polishing-object-capture-btn') return captureBtn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    appState.visualizer = {
        setHumanoidCooperativeRuledSurfaceObjectPose(pose) {
            poseCalls.push({ type: 'set', pose });
            return pose;
        },
        captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands(options) {
            poseCalls.push({ type: 'capture', options });
            return {
                ...capturedPose,
                quaternion: multiplyQuaternions(capturedPose.quaternion, NORMAL_FLIP_QUATERNION),
                flipNormal: true,
                normalFlipApplied: true
            };
        }
    };

    try {
        setupHumanoidCooperativePolishingObjectPoseControls();

        applyBtn.trigger('click');
        assert.equal(poseCalls.length, 1);
        assert.equal(poseCalls[0].type, 'set');
        assert.deepEqual(poseCalls[0].pose.position.map((value) => Number(value.toFixed(6))), [0.4, 0.1, -0.08]);
        assert.deepEqual(
            poseCalls[0].pose.quaternion.map((value) => Number(value.toFixed(12))),
            multiplyQuaternions(
                [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)],
                NORMAL_FLIP_QUATERNION
            )
                .map((value) => Number(value.toFixed(12)))
        );
        assert.equal(poseCalls[0].pose.flipNormal, true);
        assert.equal(poseCalls[0].pose.normalFlipApplied, true);
        assert.ok(globalThis.localStorage.getItem('humanoid-cooperative-polishing-object-pose'));

        captureBtn.trigger('click');
        await Promise.resolve();

        assert.equal(poseCalls.length, 3);
        assert.equal(poseCalls[1].type, 'capture');
        assert.equal(poseCalls[1].options?.flipNormal, true);
        assert.equal(poseCalls[2].type, 'set');
        assert.deepEqual(poseCalls[2].pose.position, capturedPose.position);
        assert.deepEqual(
            poseCalls[2].pose.quaternion.map((value) => Number(value.toFixed(12))),
            multiplyQuaternions(capturedPose.quaternion, NORMAL_FLIP_QUATERNION)
                .map((value) => Number(value.toFixed(12)))
        );
        assert.equal(poseCalls[2].pose.flipNormal, true);
        assert.equal(poseCalls[2].pose.normalFlipApplied, true);
        assert.equal(Number(xInput.value), 0.2);
        assert.equal(Number(yInput.value), -0.15);
        assert.equal(Number(zInput.value), -0.05);
        assert.ok(
            Math.abs(
                Number(yawInput.value) -
                extractYawDegrees(multiplyQuaternions(capturedPose.quaternion, NORMAL_FLIP_QUATERNION))
            ) < 1e-9
        );
        assert.equal(flipNormalToggle.checked, true);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidCooperativePolishingObjectPoseControls applies flip-normal changes immediately without requiring a second apply click', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const xInput = createMockElement('0.40');
    const yInput = createMockElement('0.10');
    const zInput = createMockElement('-0.08');
    const yawInput = createMockElement('30');
    const flipNormalToggle = createMockElement('');
    flipNormalToggle.checked = false;
    const applyBtn = createMockElement('');
    const captureBtn = createMockElement('');
    const poseCalls = [];

    globalThis.window = {
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.4, 0.1, -0.08],
            quaternion: [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)],
            flipNormal: false
        }
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-cooperative-polishing-object-x-input') return xInput;
            if (id === 'humanoid-cooperative-polishing-object-y-input') return yInput;
            if (id === 'humanoid-cooperative-polishing-object-z-input') return zInput;
            if (id === 'humanoid-cooperative-polishing-object-yaw-input') return yawInput;
            if (id === 'humanoid-cooperative-polishing-object-flip-normal-toggle') return flipNormalToggle;
            if (id === 'humanoid-cooperative-polishing-object-apply-btn') return applyBtn;
            if (id === 'humanoid-cooperative-polishing-object-capture-btn') return captureBtn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    appState.visualizer = {
        setHumanoidCooperativeRuledSurfaceObjectPose(pose) {
            poseCalls.push(pose);
            return pose;
        }
    };

    try {
        setupHumanoidCooperativePolishingObjectPoseControls();
        flipNormalToggle.trigger('change', true);

        assert.equal(poseCalls.length, 1);
        assert.equal(poseCalls[0].flipNormal, true);
        assert.equal(poseCalls[0].normalFlipApplied, true);
        assert.deepEqual(
            poseCalls[0].quaternion.map((value) => Number(value.toFixed(12))),
            multiplyQuaternions(
                [Math.cos(Math.PI / 12), 0, 0, Math.sin(Math.PI / 12)],
                NORMAL_FLIP_QUATERNION
            )
                .map((value) => Number(value.toFixed(12)))
        );
        assert.ok(globalThis.localStorage.getItem('humanoid-cooperative-polishing-object-pose'));
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidCooperativePolishingObjectPoseControls rotates the current object quaternion by 180 degrees about the local v-axis when flipping the normal', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const xInput = createMockElement('0.40');
    const yInput = createMockElement('0.10');
    const zInput = createMockElement('-0.08');
    const yawInput = createMockElement('0');
    const flipNormalToggle = createMockElement('');
    flipNormalToggle.checked = false;
    const applyBtn = createMockElement('');
    const captureBtn = createMockElement('');
    const poseCalls = [];
    const preservedQuaternion = [Math.cos(Math.PI / 8), Math.sin(Math.PI / 8), 0, 0];

    globalThis.window = {
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.4, 0.1, -0.08],
            quaternion: preservedQuaternion,
            flipNormal: false
        }
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-cooperative-polishing-object-x-input') return xInput;
            if (id === 'humanoid-cooperative-polishing-object-y-input') return yInput;
            if (id === 'humanoid-cooperative-polishing-object-z-input') return zInput;
            if (id === 'humanoid-cooperative-polishing-object-yaw-input') return yawInput;
            if (id === 'humanoid-cooperative-polishing-object-flip-normal-toggle') return flipNormalToggle;
            if (id === 'humanoid-cooperative-polishing-object-apply-btn') return applyBtn;
            if (id === 'humanoid-cooperative-polishing-object-capture-btn') return captureBtn;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    appState.visualizer = {
        setHumanoidCooperativeRuledSurfaceObjectPose(pose) {
            poseCalls.push(pose);
            return pose;
        }
    };

    try {
        setupHumanoidCooperativePolishingObjectPoseControls();
        flipNormalToggle.trigger('change', true);

        assert.equal(poseCalls.length, 1);
        assert.deepEqual(
            poseCalls[0].quaternion.map((value) => Number(value.toFixed(12))),
            multiplyQuaternions(preservedQuaternion, NORMAL_FLIP_QUATERNION)
                .map((value) => Number(value.toFixed(12)))
        );
        assert.equal(poseCalls[0].flipNormal, true);
        assert.equal(poseCalls[0].normalFlipApplied, true);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidJogSolverControls falls back when storage contains a removed torso-seed solver mode', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalSetInterval = globalThis.setInterval;
    const originalClearInterval = globalThis.clearInterval;

    const humanoidJogSolverSelect = createMockElement('vfi');
    const humanoidJogMetricSelect = createMockSelect('w_abs_hier', ['w_abs_hier', 'w_abs_hier_vfi']);
    const humanoidWMetricIncreaseBtn = createMockElement('');
    const humanoidWaistXCorridorToggle = createMockElement('');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('decoupled-upright');
    const visualizerCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            if (id === 'humanoid-manifold-jog-metric-select') return humanoidJogMetricSelect;
            if (id === 'humanoid-w-metric-increase-btn') return humanoidWMetricIncreaseBtn;
            if (id === 'humanoid-waist-x-corridor-toggle') return humanoidWaistXCorridorToggle;
            if (id === 'humanoid-test-dual-arm-lock-toggle') return humanoidTestDualArmLockToggle;
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-jog-solver-mode', 'torso_seed_upright_vfi']
    ]);
    globalThis.setInterval = () => 1;
    globalThis.clearInterval = () => {};
    appState.visualizer = {
        setHumanoidJogSolverMode(mode) {
            visualizerCalls.push(mode);
        }
    };

    try {
        setupHumanoidJogSolverControls();
        assert.equal(humanoidJogSolverSelect.value, 'vfi');
        assert.equal(globalThis.window.humanoidJogSolverMode, 'vfi');
        assert.deepEqual(visualizerCalls, ['vfi']);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'vfi');
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        if (originalSetInterval === undefined) delete globalThis.setInterval;
        else globalThis.setInterval = originalSetInterval;
        if (originalClearInterval === undefined) delete globalThis.clearInterval;
        else globalThis.clearInterval = originalClearInterval;
        appState.visualizer = originalVisualizer;
    }
});

test('setupHumanoidJogSolverControls triggers humanoid w+ as a single-shot action without repeat timers', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalSetInterval = globalThis.setInterval;
    const originalClearInterval = globalThis.clearInterval;

    const humanoidJogSolverSelect = createMockElement('mj_restore');
    const humanoidJogMetricSelect = createMockSelect('w_abs_hier_vfi', ['w_abs_hier', 'w_abs_hier_vfi']);
    const humanoidWMetricIncreaseBtn = createMockElement('');
    const humanoidWaistXCorridorToggle = createMockElement('');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const metricOptimizationCalls = [];
    let setIntervalCalls = 0;

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            if (id === 'humanoid-manifold-jog-metric-select') return humanoidJogMetricSelect;
            if (id === 'humanoid-w-metric-increase-btn') return humanoidWMetricIncreaseBtn;
            if (id === 'humanoid-waist-x-corridor-toggle') return humanoidWaistXCorridorToggle;
            if (id === 'humanoid-test-dual-arm-lock-toggle') return humanoidTestDualArmLockToggle;
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    globalThis.setInterval = () => {
        setIntervalCalls += 1;
        return 1;
    };
    globalThis.clearInterval = () => {};
    appState.visualizer = {
        setHumanoidJogSolverMode() {},
        setHumanoidManifoldJogSecondaryMetric(metric) {
            metricOptimizationCalls.push(`set:${metric}`);
        },
        setHumanoidWaistXCorridorEnabled() {},
        setHumanoidTestDualArmLockEnabled() {},
        optimizeHumanoidHierarchicalAbsoluteManipulability(direction) {
            metricOptimizationCalls.push(`w_abs_hier:${direction}`);
        },
        optimizeHumanoidHierarchicalAbsoluteVFIManipulability(direction) {
            metricOptimizationCalls.push(`w_abs_hier_vfi:${direction}`);
        }
    };

    try {
        setupHumanoidJogSolverControls();
        metricOptimizationCalls.length = 0;

        humanoidWMetricIncreaseBtn.trigger('mousedown');

        assert.equal(setIntervalCalls, 0);
        assert.deepEqual(metricOptimizationCalls, ['w_abs_hier_vfi:increase']);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        if (originalSetInterval === undefined) delete globalThis.setInterval;
        else globalThis.setInterval = originalSetInterval;
        if (originalClearInterval === undefined) delete globalThis.clearInterval;
        else globalThis.clearInterval = originalClearInterval;
        appState.visualizer = originalVisualizer;
    }
});
