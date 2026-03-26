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
            handler({ target: { value: nextValue } });
        }
    };
}

test('setupHumanoidJogSolverControls restores mj_restore_pre_w from storage and propagates it to the UI and visualizer', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalLocalStorage = globalThis.localStorage;
    const originalVisualizer = appState.visualizer;

    const humanoidJogSolverSelect = createMockElement('vfi');
    const visualizerCalls = [];

    globalThis.window = {};
    globalThis.document = {
        getElementById(id) {
            if (id === 'humanoid-jog-solver-select') return humanoidJogSolverSelect;
            return null;
        }
    };
    globalThis.localStorage = createMockLocalStorage([
        ['humanoid-jog-solver-mode', 'mj_restore_pre_w']
    ]);
    appState.visualizer = {
        setHumanoidJogSolverMode(mode) {
            visualizerCalls.push(mode);
        }
    };

    try {
        setupHumanoidJogSolverControls();

        assert.equal(humanoidJogSolverSelect.value, 'mj_restore_pre_w');
        assert.equal(globalThis.window.humanoidJogSolverMode, 'mj_restore_pre_w');
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w']);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'mj_restore_pre_w');

        humanoidJogSolverSelect.trigger('change', 'priority');

        assert.equal(humanoidJogSolverSelect.value, 'priority');
        assert.equal(globalThis.window.humanoidJogSolverMode, 'priority');
        assert.deepEqual(visualizerCalls, ['mj_restore_pre_w', 'priority']);
        assert.equal(globalThis.localStorage.getItem('humanoid-jog-solver-mode'), 'priority');
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
        appState.visualizer = originalVisualizer;
    }
});
