import test from 'node:test';
import assert from 'node:assert/strict';

import { appState } from '../core/state.js';
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
        textContent: '',
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
    const humanoidTestDualArmLockToggle = createMockElement('');
    const controlModeSelect = createMockElement('absolute');
    const visualizerCalls = [];
    const metricOptimizationCalls = [];
    const testLockCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            if (id === 'humanoid-manifold-jog-metric-select') return humanoidJogMetricSelect;
            if (id === 'humanoid-w-metric-increase-btn') return humanoidWMetricIncreaseBtn;
            if (id === 'humanoid-test-dual-arm-lock-toggle') return humanoidTestDualArmLockToggle;
            if (id === 'dual-arm-control-mode-select') return controlModeSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-jog-solver-mode', 'mj_restore_pre_w'],
        ['humanoid-manifold-jog-metric', 'w_abs_hier_vfi'],
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
            ['w_abs_hier', 'w_abs_hier_vfi']
        );
        assert.equal(humanoidTestDualArmLockToggle.checked, true);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'mj_restore_pre_w');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier_vfi');
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w']);
        assert.deepEqual(metricOptimizationCalls, ['set:w_abs_hier_vfi']);
        assert.deepEqual(testLockCalls, [true]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'mj_restore_pre_w');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier_vfi');
        assert.equal(globalThis.localStorage.getItem('humanoid-test-dual-arm-lock-enabled'), '1');

        humanoidJogSolverSelect.trigger('change', 'priority');
        humanoidJogMetricSelect.trigger('change', 'w_abs_hier');
        controlModeSelect.trigger('change', 'relative');
        humanoidWMetricIncreaseBtn.trigger('mousedown');
        humanoidTestDualArmLockToggle.trigger('change', false);

        assert.equal(humanoidJogSolverSelect.value, 'priority');
        assert.equal(humanoidJogMetricSelect.value, 'w_abs_hier');
        assert.deepEqual(
            humanoidJogMetricSelect.options.map((option) => option.textContent),
            ['w_rel_hier', 'w_rel_hier_vfi']
        );
        assert.equal(humanoidTestDualArmLockToggle.checked, false);
        assert.equal(globalThis.window.humanoidJogSolverMode, 'priority');
        assert.equal(globalThis.window.humanoidManifoldJogSecondaryMetric, 'w_abs_hier');
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w', 'priority']);
        assert.deepEqual(metricOptimizationCalls, [
            'set:w_abs_hier_vfi',
            'set:w_abs_hier',
            'w_abs_hier:increase'
        ]);
        assert.deepEqual(testLockCalls, [true, false]);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'priority');
        assert.equal(globalThis.localStorage.getItem('humanoid-manifold-jog-metric'), 'w_abs_hier');
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
