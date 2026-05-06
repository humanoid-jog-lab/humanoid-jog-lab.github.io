import test from 'node:test';
import assert from 'node:assert/strict';

import { appState, callbacks } from '../core/state.js';
import { resolveHumanoidInitialQ } from '../core/humanoid-default-q-profiles.js';
import { setupHumanoidJogSolverControls } from './event-handlers.js';

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
        options: [],
        addEventListener(eventName, handler) {
            if (!listeners.has(eventName)) listeners.set(eventName, []);
            listeners.get(eventName).push(handler);
        },
        trigger(eventName, nextValue) {
            this.value = nextValue ?? this.value;
            const handlers = listeners.get(eventName);
            if (!Array.isArray(handlers)) return;
            const event = {
                target: {
                    value: this.value,
                    checked: Boolean(nextValue)
                }
            };
            handlers.forEach((handler) => handler(event));
        }
    };
}

function createMockSelect(initialValue = '', optionValues = []) {
    const element = createMockElement(initialValue);
    element.options = optionValues.map((value) => ({ value, textContent: value }));
    return element;
}

test('setupHumanoidJogSolverControls resets humanoid joints from control mode defaults', async () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalHumanoidMode = appState.humanoidMode;
    const originalHumanoidJointNames = appState.humanoidJointNames;
    const originalHumanoidInitialQProfileId = appState.humanoidInitialQProfileId;
    const originalHumanoidQ = appState.humanoidQ;
    const originalUpdateRobotPose = callbacks.updateRobotPose;
    const originalUpdateGlobalVariables = callbacks.updateGlobalVariables;

    const humanoidJogSolverSelect = createMockElement('proxqp_w_vfi');
    const humanoidJogMetricSelect = createMockSelect('w_abs_hier_vfi', ['w_abs_hier', 'w_abs_hier_ext', 'w_abs_hier_vfi']);
    const humanoidWMetricIncreaseBtn = createMockElement('');
    const humanoidWaistXCorridorToggle = createMockElement('');
    const humanoidWaistXCorridorMinInput = createMockElement('-0.10');
    const humanoidWaistXCorridorMaxInput = createMockElement('0.05');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const updateRobotPoseCalls = [];
    const updateGlobalVariablesCalls = [];
    const resetSurfaceSlideCalls = [];
    const polishingReferenceCaptureCalls = [];

    globalThis.window = {
        humanoidQ: new Array(20).fill(9)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            if (id === 'humanoid-manifold-jog-metric-select') return humanoidJogMetricSelect;
            if (id === 'humanoid-w-metric-increase-btn') return humanoidWMetricIncreaseBtn;
            if (id === 'humanoid-waist-x-corridor-toggle') return humanoidWaistXCorridorToggle;
            if (id === 'humanoid-waist-x-corridor-min-input') return humanoidWaistXCorridorMinInput;
            if (id === 'humanoid-waist-x-corridor-max-input') return humanoidWaistXCorridorMaxInput;
            if (id === 'humanoid-test-dual-arm-lock-toggle') return humanoidTestDualArmLockToggle;
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-jog-solver-mode', 'proxqp_w_vfi'],
        ['humanoid-manifold-jog-metric', 'w_abs_hier_vfi'],
        ['humanoid-jog-default-version', 'proxqp-three-solver']
    ]);
    appState.visualizer = {
        setHumanoidJogSolverMode() {},
        setHumanoidManifoldJogSecondaryMetric() {},
        setHumanoidWaistXCorridorEnabled() {},
        setHumanoidWaistXCorridorMin() {},
        setHumanoidWaistXCorridorMax() {},
        setHumanoidTestDualArmLockEnabled() {},
        resetHumanoidSurfaceSlideReference() {
            resetSurfaceSlideCalls.push(globalThis.window.humanoidQ.slice());
        },
        captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands(options) {
            polishingReferenceCaptureCalls.push({
                q: globalThis.window.humanoidQ.slice(),
                flipNormal: Boolean(options?.flipNormal)
            });
            return {
                position: [0, 0, 0],
                quaternion: [1, 0, 0, 0]
            };
        },
        refreshHumanoidSurfaceSlideVisualization() {
            polishingReferenceCaptureCalls.push({ refreshed: true });
        }
    };
    appState.humanoidMode = true;
    appState.humanoidJointNames = Array.from({ length: 20 }, (_, index) => `joint_${index}`);
    appState.humanoidInitialQProfileId = 'default2';
    appState.humanoidQ = new Array(20).fill(9);
    callbacks.updateGlobalVariables = () => {
        updateGlobalVariablesCalls.push(appState.humanoidQ.slice());
        globalThis.window.humanoidQ = appState.humanoidQ.slice();
    };
    callbacks.updateRobotPose = () => {
        updateRobotPoseCalls.push(appState.humanoidQ.slice());
    };

    try {
        setupHumanoidJogSolverControls();

        controlModeSelect.trigger('change', 'relative');
        await Promise.resolve();

        const default1 = resolveHumanoidInitialQ({ initial_q_profile: 'default1' }, 20, 'default1');
        assert.deepEqual(appState.humanoidQ, default1);
        assert.deepEqual(globalThis.window.humanoidQ, default1);
        assert.deepEqual(updateGlobalVariablesCalls, [default1]);
        assert.deepEqual(updateRobotPoseCalls, [default1]);
        assert.deepEqual(resetSurfaceSlideCalls, [default1]);
        assert.deepEqual(polishingReferenceCaptureCalls, []);

        appState.humanoidInitialQProfileId = 'default1';
        appState.humanoidQ = new Array(20).fill(8);
        globalThis.window.humanoidQ = appState.humanoidQ.slice();
        updateGlobalVariablesCalls.length = 0;
        updateRobotPoseCalls.length = 0;
        resetSurfaceSlideCalls.length = 0;
        polishingReferenceCaptureCalls.length = 0;

        controlModeSelect.trigger('change', 'cooperative-ruled-surface-polishing-dq-setpoint');
        await Promise.resolve();

        const default2 = resolveHumanoidInitialQ({ initial_q_profile: 'default2' }, 20, 'default2');
        assert.deepEqual(appState.humanoidQ, default2);
        assert.deepEqual(globalThis.window.humanoidQ, default2);
        assert.deepEqual(updateGlobalVariablesCalls, [default2]);
        assert.deepEqual(updateRobotPoseCalls, [default2]);
        assert.deepEqual(resetSurfaceSlideCalls, [default2]);
        assert.deepEqual(polishingReferenceCaptureCalls, [
            { q: default2, flipNormal: false },
            { refreshed: true }
        ]);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
        appState.humanoidMode = originalHumanoidMode;
        appState.humanoidJointNames = originalHumanoidJointNames;
        appState.humanoidInitialQProfileId = originalHumanoidInitialQProfileId;
        appState.humanoidQ = originalHumanoidQ;
        callbacks.updateRobotPose = originalUpdateRobotPose;
        callbacks.updateGlobalVariables = originalUpdateGlobalVariables;
    }
});
