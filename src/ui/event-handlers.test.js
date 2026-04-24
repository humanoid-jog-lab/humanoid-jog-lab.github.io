import test from 'node:test';
import assert from 'node:assert/strict';

import { appState, callbacks } from '../core/state.js';
import {
    setupHumanoidDefaultPoseControls,
    setupHumanoidParameterControls,
    setupHumanoidJogSolverControls,
    setupHumanoidCooperativePolishingSurfaceControls,
    setupHumanoidCooperativePolishingObjectPoseControls,
    setupTeachOrderControls,
    setupEventListeners
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
            if (!listeners.has(eventName)) listeners.set(eventName, []);
            listeners.get(eventName).push(handler);
        },
        removeEventListener(eventName, handler) {
            const handlers = listeners.get(eventName);
            if (!Array.isArray(handlers)) return;
            const nextHandlers = handlers.filter((candidate) => candidate !== handler);
            listeners.set(eventName, nextHandlers);
        },
        trigger(eventName, nextValue) {
            const handlers = listeners.get(eventName);
            if (!Array.isArray(handlers) || handlers.length === 0) return;
            const event = {
                target: {
                    value: nextValue,
                    checked: Boolean(nextValue)
                },
                currentTarget: this,
                preventDefault() {},
                stopPropagation() {}
            };
            handlers.forEach((handler) => {
                if (typeof handler === 'function') handler(event);
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

test('setupTeachOrderControls defaults teach jogging to world frame and persists later user changes', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const teachOrderToggle = createMockElement('');
    const teachOrderStatus = createMockElement('');
    const visualizerCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'teach-multiply-order-toggle') return teachOrderToggle;
            if (id === 'teach-multiply-order-status') return teachOrderStatus;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    appState.visualizer = {
        setTeachMultiplicationOrder(order) {
            visualizerCalls.push(order);
        }
    };

    try {
        setupTeachOrderControls();

        assert.equal(teachOrderToggle.checked, false);
        assert.equal(teachOrderStatus.textContent, 'Current Mode: world frame (Δ × current)');
        assert.equal(globalThis.localStorage.getItem('teach-multiply-order-delta-first'), '0');
        assert.deepEqual(visualizerCalls, ['current-first']);

        teachOrderToggle.trigger('change', true);

        assert.equal(teachOrderToggle.checked, true);
        assert.equal(teachOrderStatus.textContent, 'Current Mode: local frame (current × Δ)');
        assert.equal(globalThis.localStorage.getItem('teach-multiply-order-delta-first'), '1');
        assert.deepEqual(visualizerCalls, ['current-first', 'delta-first']);
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

test('setupHumanoidParameterControls restores, applies, and validates humanoid joint-limit eta', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalRos2Defaults = appState.humanoidRos2PublishDefaults;

    const humanoidJointLimitEtaInput = createMockElement('');
    const humanoidJointLimitEtaStatus = createMockElement('');
    const humanoidWaistXEtaInput = createMockElement('');
    const humanoidWaistXVFIStatus = createMockElement('');
    const humanoidTRSafetyPushWeightInput = createMockElement('');
    const humanoidTRSafetyPushStatus = createMockElement('');
    const humanoidRos2ArmVelInput = createMockElement('');
    const humanoidRos2ArmAccInput = createMockElement('');
    const humanoidRos2WaistVelInput = createMockElement('');
    const humanoidRos2WaistAccInput = createMockElement('');
    const humanoidRos2PublishDefaultsStatus = createMockElement('');
    const etaCalls = [];
    const waistXVFICalls = [];
    const safetyPushWeightCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-joint-limit-eta-input') return humanoidJointLimitEtaInput;
            if (id === 'humanoid-joint-limit-eta-status') return humanoidJointLimitEtaStatus;
            if (id === 'humanoid-waist-x-eta-input') return humanoidWaistXEtaInput;
            if (id === 'humanoid-waist-x-vfi-status') return humanoidWaistXVFIStatus;
            if (id === 'humanoid-tr-safety-push-weight-input') return humanoidTRSafetyPushWeightInput;
            if (id === 'humanoid-tr-safety-push-status') return humanoidTRSafetyPushStatus;
            if (id === 'humanoid-ros2-arm-vel-input') return humanoidRos2ArmVelInput;
            if (id === 'humanoid-ros2-arm-acc-input') return humanoidRos2ArmAccInput;
            if (id === 'humanoid-ros2-waist-vel-input') return humanoidRos2WaistVelInput;
            if (id === 'humanoid-ros2-waist-acc-input') return humanoidRos2WaistAccInput;
            if (id === 'humanoid-ros2-publish-defaults-status') return humanoidRos2PublishDefaultsStatus;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-joint-limit-eta', '4.5'],
        ['humanoid-waist-x-eta', '2.5'],
        ['humanoid-tr-safety-push-weight', '0.08'],
        ['humanoid-ros2-arm-vel', '0.6'],
        ['humanoid-ros2-arm-acc', '1.1'],
        ['humanoid-ros2-waist-vel', '0.7'],
        ['humanoid-ros2-waist-acc', '1.2']
    ]);
    appState.visualizer = {
        vfiConfig: {
            jointLimitEta: 5,
            waistXEta: 1,
            waistXActivationThreshold: 0.12,
            mjRestoreSafetyPushWeight: 0.05
        },
        setHumanoidJointLimitEta(value) {
            etaCalls.push(value);
            this.vfiConfig.jointLimitEta = value;
        },
        setHumanoidWaistXVFIParams(value) {
            waistXVFICalls.push(value);
            this.vfiConfig.waistXEta = value.eta;
        },
        setHumanoidTRSafetyPushWeight(value) {
            safetyPushWeightCalls.push(value);
            this.vfiConfig.mjRestoreSafetyPushWeight = value;
        }
    };

    try {
        setupHumanoidParameterControls();

        assert.equal(humanoidJointLimitEtaInput.value, '4.5');
        assert.equal(humanoidJointLimitEtaStatus.textContent, '当前值已应用：η_θ=4.5');
        assert.equal(globalThis.window.humanoidJointLimitEta, 4.5);
        assert.deepEqual(etaCalls, [4.5]);
        assert.equal(humanoidWaistXEtaInput.value, '2.5');
        assert.equal(humanoidWaistXVFIStatus.textContent, '当前值已应用：η_x=2.5');
        assert.equal(globalThis.window.humanoidWaistXEta, 2.5);
        assert.equal(globalThis.window.humanoidWaistXActivationThreshold, undefined);
        assert.deepEqual(waistXVFICalls, [{ eta: 2.5 }]);
        assert.equal(humanoidTRSafetyPushWeightInput.value, '0.08');
        assert.equal(humanoidTRSafetyPushStatus.textContent, '当前值已应用：β=0.08');
        assert.equal(globalThis.window.humanoidTRSafetyPushWeight, 0.08);
        assert.deepEqual(safetyPushWeightCalls, [0.08]);
        assert.equal(humanoidRos2ArmVelInput.value, '0.6');
        assert.equal(humanoidRos2ArmAccInput.value, '1.1');
        assert.equal(humanoidRos2WaistVelInput.value, '0.7');
        assert.equal(humanoidRos2WaistAccInput.value, '1.2');
        assert.equal(humanoidRos2PublishDefaultsStatus.textContent, '当前值已应用：双臂 vel/acc=0.6/1.1，腰部 vel/acc=0.7/1.2');
        assert.deepEqual(appState.humanoidRos2PublishDefaults, {
            armVel: 0.6,
            armAcc: 1.1,
            waistVel: 0.7,
            waistAcc: 1.2
        });

        humanoidJointLimitEtaInput.trigger('change', '6.2');

        assert.equal(humanoidJointLimitEtaInput.value, '6.2');
        assert.equal(humanoidJointLimitEtaStatus.textContent, '当前值已应用：η_θ=6.2');
        assert.equal(globalThis.window.humanoidJointLimitEta, 6.2);
        assert.equal(globalThis.localStorage.getItem('humanoid-joint-limit-eta'), '6.2');
        assert.deepEqual(etaCalls, [4.5, 6.2]);

        humanoidWaistXEtaInput.value = '3.5';
        humanoidWaistXEtaInput.trigger('change', '3.5');

        assert.equal(humanoidWaistXEtaInput.value, '3.5');
        assert.equal(humanoidWaistXVFIStatus.textContent, '当前值已应用：η_x=3.5');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-eta'), '3.5');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-activation-threshold'), null);
        assert.deepEqual(waistXVFICalls, [
            { eta: 2.5 },
            { eta: 3.5 }
        ]);

        humanoidTRSafetyPushWeightInput.value = '0.12';
        humanoidTRSafetyPushWeightInput.trigger('change', '0.12');

        assert.equal(humanoidTRSafetyPushWeightInput.value, '0.12');
        assert.equal(humanoidTRSafetyPushStatus.textContent, '当前值已应用：β=0.12');
        assert.equal(globalThis.localStorage.getItem('humanoid-tr-safety-push-weight'), '0.12');
        assert.deepEqual(safetyPushWeightCalls, [0.08, 0.12]);

        humanoidTRSafetyPushWeightInput.value = '0';
        humanoidTRSafetyPushWeightInput.trigger('change', '0');

        assert.equal(humanoidTRSafetyPushWeightInput.value, '0');
        assert.equal(humanoidTRSafetyPushStatus.textContent, '当前值已应用：β=0');
        assert.equal(globalThis.localStorage.getItem('humanoid-tr-safety-push-weight'), '0');
        assert.deepEqual(safetyPushWeightCalls, [0.08, 0.12, 0]);

        humanoidRos2ArmVelInput.value = '0.9';
        humanoidRos2ArmVelInput.trigger('change', '0.9');

        assert.equal(humanoidRos2PublishDefaultsStatus.textContent, '当前值已应用：双臂 vel/acc=0.9/1.1，腰部 vel/acc=0.7/1.2');
        assert.equal(globalThis.localStorage.getItem('humanoid-ros2-arm-vel'), '0.9');
        assert.deepEqual(appState.humanoidRos2PublishDefaults, {
            armVel: 0.9,
            armAcc: 1.1,
            waistVel: 0.7,
            waistAcc: 1.2
        });

        humanoidJointLimitEtaInput.trigger('change', 'bad');

        assert.equal(humanoidJointLimitEtaInput.value, '6.2');
        assert.equal(humanoidJointLimitEtaStatus.textContent, '输入无效，已恢复为 η_θ=6.2');
        assert.equal(globalThis.window.humanoidJointLimitEta, 6.2);
        assert.equal(globalThis.localStorage.getItem('humanoid-joint-limit-eta'), '6.2');
        assert.deepEqual(etaCalls, [4.5, 6.2]);

        humanoidWaistXEtaInput.value = 'bad';
        humanoidWaistXEtaInput.trigger('change', 'bad');

        assert.equal(humanoidWaistXEtaInput.value, '3.5');
        assert.equal(humanoidWaistXVFIStatus.textContent, '输入无效，已恢复为 η_x=3.5');
        assert.equal(globalThis.window.humanoidWaistXEta, 3.5);
        assert.equal(globalThis.window.humanoidWaistXActivationThreshold, undefined);

        humanoidTRSafetyPushWeightInput.value = 'bad';
        humanoidTRSafetyPushWeightInput.trigger('change', 'bad');

        assert.equal(humanoidTRSafetyPushWeightInput.value, '0');
        assert.equal(humanoidTRSafetyPushStatus.textContent, '输入无效，已恢复为 β=0');

        humanoidRos2WaistAccInput.value = 'bad';
        humanoidRos2WaistAccInput.trigger('change', 'bad');

        assert.equal(humanoidRos2WaistAccInput.value, '1.2');
        assert.equal(humanoidRos2PublishDefaultsStatus.textContent, '输入无效，已恢复为 双臂(0.9/1.1) 腰部(0.7/1.2)');
        assert.deepEqual(appState.humanoidRos2PublishDefaults, {
            armVel: 0.9,
            armAcc: 1.1,
            waistVel: 0.7,
            waistAcc: 1.2
        });
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        appState.visualizer = originalVisualizer;
        appState.humanoidRos2PublishDefaults = originalRos2Defaults;
    }
});

test('setupHumanoidJogSolverControls falls back stored mj_restore_pre_w selections to mj_restore_tr and propagates the fallback', () => {
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
    const humanoidWaistXCorridorMinInput = createMockElement('-0.10');
    const humanoidWaistXCorridorMaxInput = createMockElement('0.05');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const visualizerCalls = [];
    const metricOptimizationCalls = [];
    const waistXCorridorCalls = [];
    const waistXCorridorMinCalls = [];
    const waistXCorridorMaxCalls = [];
    const testLockCalls = [];

    globalThis.window = {};
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
        ['humanoid-jog-solver-mode', 'mj_restore_pre_w'],
        ['humanoid-manifold-jog-metric', 'w_abs_hier_vfi'],
        ['humanoid-waist-x-corridor-enabled', '1'],
        ['humanoid-waist-x-corridor-min', '-0.12'],
        ['humanoid-waist-x-corridor-max', '0.12'],
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
        setHumanoidWaistXCorridorMin(value) {
            waistXCorridorMinCalls.push(value);
        },
        setHumanoidWaistXCorridorMax(value) {
            waistXCorridorMaxCalls.push(value);
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

        assert.equal(humanoidJogSolverSelect.value, 'mj_restore_tr');
        assert.equal(humanoidJogMetricSelect.value, 'w_abs_hier_vfi');
        assert.deepEqual(
            humanoidJogMetricSelect.options.map((option) => option.textContent),
            ['w_abs', 'w_abs_vfi']
        );
        assert.equal(humanoidWaistXCorridorToggle.checked, true);
        assert.equal(humanoidWaistXCorridorMinInput.value, '-0.12');
        assert.equal(humanoidWaistXCorridorMaxInput.value, '0.12');
        assert.equal(humanoidTestDualArmLockToggle.checked, true);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'mj_restore_tr');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier_vfi');
        assert.equal(globalThis.window.humanoidWaistXCorridorEnabled, true);
        assert.equal(globalThis.window.humanoidWaistXCorridorMin, -0.12);
        assert.equal(globalThis.window.humanoidWaistXCorridorMax, 0.12);
        assert.deepEqual(visualizerCalls, ['mj_restore_tr']);
        assert.deepEqual(metricOptimizationCalls, ['set:w_abs_hier_vfi']);
        assert.deepEqual(waistXCorridorCalls, [true]);
        assert.deepEqual(waistXCorridorMinCalls, [-0.12]);
        assert.deepEqual(waistXCorridorMaxCalls, [0.12]);
        assert.deepEqual(testLockCalls, [true]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'mj_restore_tr');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier_vfi');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-enabled'), '1');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-min'), '-0.12');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-max'), '0.12');
        assert.equal(globalThis.localStorage.getItem('humanoid-test-dual-arm-lock-enabled'), '1');

        humanoidJogSolverSelect.trigger('change', 'priority');
        humanoidJogMetricSelect.trigger('change', 'w_abs_hier');
        controlModeSelect.trigger('change', 'relative');
        humanoidWMetricIncreaseBtn.trigger('pointerdown');
        humanoidWaistXCorridorMinInput.trigger('change', '-0.08');
        humanoidWaistXCorridorMaxInput.trigger('change', '0.08');
        humanoidWaistXCorridorToggle.trigger('change', false);
        humanoidTestDualArmLockToggle.trigger('change', false);

        assert.equal(humanoidJogSolverSelect.value, 'priority');
        assert.equal(humanoidJogMetricSelect.value, 'w_abs_hier');
        assert.deepEqual(
            humanoidJogMetricSelect.options.map((option) => option.textContent),
            ['w_rel', 'w_rel_vfi']
        );
        assert.equal(humanoidWaistXCorridorToggle.checked, false);
        assert.equal(humanoidWaistXCorridorMinInput.value, '-0.08');
        assert.equal(humanoidTestDualArmLockToggle.checked, false);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'priority');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier');
        assert.equal(globalThis.window.humanoidWaistXCorridorEnabled, false);
        assert.equal(globalThis.window.humanoidWaistXCorridorMin, -0.08);
        assert.equal(globalThis.window.humanoidWaistXCorridorMax, 0.08);
        assert.deepEqual(visualizerCalls, ['mj_restore_tr', 'priority']);
        assert.deepEqual(metricOptimizationCalls, [
            'set:w_abs_hier_vfi',
            'set:w_abs_hier',
            'w_abs_hier:increase'
        ]);
        assert.deepEqual(waistXCorridorCalls, [true, false]);
        assert.deepEqual(waistXCorridorMinCalls, [-0.12, -0.08]);
        assert.deepEqual(waistXCorridorMaxCalls, [0.12, 0.08]);
        assert.deepEqual(testLockCalls, [true, false]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'priority');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-enabled'), '0');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-min'), '-0.08');
        assert.equal(globalThis.localStorage.getItem('humanoid-waist-x-corridor-max'), '0.08');
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
    const exportBtn = createMockElement('');
    const poseCalls = [];
    const exportCalls = [];
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
            if (id === 'humanoid-cooperative-polishing-export-obj-btn') return exportBtn;
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
        },
        exportHumanoidCooperativeRuledSurfacePrintableObj(options) {
            exportCalls.push(options);
            return {
                filename: 'current.obj'
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

        exportBtn.trigger('click');
        await Promise.resolve();

        assert.equal(exportCalls.length, 1);
        assert.equal(exportCalls[0]?.thicknessMetres, 0.008);
        assert.equal(exportCalls[0]?.segmentsU, 96);
        assert.equal(exportCalls[0]?.segmentsV, 32);
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

test('setupHumanoidJogSolverControls auto-switches ruled-surface polishing modes onto mj_restore when the current solver is unsupported', () => {
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
    globalThis.setInterval = () => 1;
    globalThis.clearInterval = () => {};
    appState.visualizer = {
        setHumanoidJogSolverMode(mode) {
            visualizerCalls.push(mode);
        },
        setHumanoidManifoldJogSecondaryMetric() {},
        setHumanoidWaistXCorridorEnabled() {},
        setHumanoidTestDualArmLockEnabled() {}
    };

    try {
        setupHumanoidJogSolverControls();
        assert.equal(humanoidJogSolverSelect.value, 'vfi');
        assert.equal(globalThis.window.humanoidJogSolverMode, 'vfi');

        controlModeSelect.trigger('change', 'left-ruled-surface-polishing');

        assert.equal(humanoidJogSolverSelect.value, 'mj_restore');
        assert.equal(globalThis.window.humanoidJogSolverMode, 'mj_restore');
        assert.deepEqual(visualizerCalls, ['vfi', 'mj_restore']);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'mj_restore');
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

test('setupHumanoidJogSolverControls repeats humanoid w+ while the button is held and stops on release', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalSetTimeout = globalThis.setTimeout;
    const originalClearTimeout = globalThis.clearTimeout;
    const originalSetInterval = globalThis.setInterval;
    const originalClearInterval = globalThis.clearInterval;

    const humanoidJogSolverSelect = createMockElement('mj_restore');
    const humanoidJogMetricSelect = createMockSelect('w_abs_hier_vfi', ['w_abs_hier', 'w_abs_hier_vfi']);
    const humanoidWMetricIncreaseBtn = createMockElement('');
    const humanoidWaistXCorridorToggle = createMockElement('');
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const metricOptimizationCalls = [];
    const timeoutCallbacks = new Map();
    const intervalCallbacks = new Map();
    const clearedTimeouts = [];
    const clearedIntervals = [];
    let nextTimerId = 1;

    globalThis.window = {
        addEventListener() {},
        removeEventListener() {}
    };
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
    globalThis.setTimeout = (handler) => {
        const id = nextTimerId++;
        timeoutCallbacks.set(id, handler);
        return id;
    };
    globalThis.clearTimeout = (id) => {
        clearedTimeouts.push(id);
        timeoutCallbacks.delete(id);
    };
    globalThis.setInterval = (handler) => {
        const id = nextTimerId++;
        intervalCallbacks.set(id, handler);
        return id;
    };
    globalThis.clearInterval = (id) => {
        clearedIntervals.push(id);
        intervalCallbacks.delete(id);
    };
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

        humanoidWMetricIncreaseBtn.trigger('pointerdown');

        assert.deepEqual(metricOptimizationCalls, ['w_abs_hier_vfi:increase']);
        assert.equal(timeoutCallbacks.size, 1);
        assert.equal(intervalCallbacks.size, 0);

        const [repeatDelayId, repeatDelayHandler] = Array.from(timeoutCallbacks.entries())[0];
        timeoutCallbacks.delete(repeatDelayId);
        repeatDelayHandler();

        assert.equal(timeoutCallbacks.size, 0);
        assert.equal(intervalCallbacks.size, 1);

        const [repeatIntervalId, repeatIntervalHandler] = Array.from(intervalCallbacks.entries())[0];
        repeatIntervalHandler();
        repeatIntervalHandler();

        assert.deepEqual(metricOptimizationCalls, [
            'w_abs_hier_vfi:increase',
            'w_abs_hier_vfi:increase',
            'w_abs_hier_vfi:increase'
        ]);

        humanoidWMetricIncreaseBtn.trigger('pointerup');

        assert.deepEqual(clearedTimeouts, []);
        assert.deepEqual(clearedIntervals, [repeatIntervalId]);
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        if (originalLocalStorage === undefined) delete globalThis.localStorage;
        else globalThis.localStorage = originalLocalStorage;
        if (originalSetTimeout === undefined) delete globalThis.setTimeout;
        else globalThis.setTimeout = originalSetTimeout;
        if (originalClearTimeout === undefined) delete globalThis.clearTimeout;
        else globalThis.clearTimeout = originalClearTimeout;
        if (originalSetInterval === undefined) delete globalThis.setInterval;
        else globalThis.setInterval = originalSetInterval;
        if (originalClearInterval === undefined) delete globalThis.clearInterval;
        else globalThis.clearInterval = originalClearInterval;
        appState.visualizer = originalVisualizer;
    }
});

test('setupEventListeners stops repeating cooperative ruled-surface polishing commands after a failed step', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;
    const originalSetInterval = globalThis.setInterval;
    const originalClearInterval = globalThis.clearInterval;

    const teachButton = createMockElement('');
    teachButton.dataset = { axis: 'x', direction: 'positive', type: 'translation' };
    teachButton.classList = {
        contains(className) {
            return className === 'teach-btn';
        }
    };

    const controlModeSelect = createMockElement('cooperative-ruled-surface-polishing');
    const intervalCallbacks = new Map();
    const clearedIntervals = [];
    let nextTimerId = 1;
    let moveCalls = 0;

    globalThis.window = {
        addEventListener() {},
        removeEventListener() {}
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        },
        querySelectorAll(selector) {
            if (selector === '.teach-btn') return [teachButton];
            return [];
        }
    };
    globalThis.localStorage = createMockLocalStorage();
    globalThis.setInterval = (handler) => {
        const id = nextTimerId++;
        intervalCallbacks.set(id, handler);
        return id;
    };
    globalThis.clearInterval = (id) => {
        clearedIntervals.push(id);
        intervalCallbacks.delete(id);
    };
    appState.visualizer = {
        humanoidMode: true,
        recordHumanoidJogAxisIntent() {},
        moveHumanoidCooperativeRuledSurfacePolishingAlongParameter() {
            moveCalls += 1;
            return false;
        }
    };

    try {
        setupEventListeners();

        teachButton.trigger('mousedown');

        assert.equal(moveCalls, 1);
        assert.equal(intervalCallbacks.size, 0);
        assert.deepEqual(clearedIntervals, [1]);
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
