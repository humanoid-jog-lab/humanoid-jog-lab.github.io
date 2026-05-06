import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

import { appState } from '../core/state.js';
import { setupDQCalculatorPanel } from './dq-calculator.js';

const ROOT = resolve(import.meta.dirname, '../..');

function createMockElement(initialValue = '') {
    const listeners = new Map();
    return {
        value: initialValue,
        textContent: '',
        disabled: false,
        checked: false,
        style: {},
        dataset: {},
        addEventListener(eventName, handler) {
            listeners.set(eventName, handler);
        },
        trigger(eventName) {
            const handler = listeners.get(eventName);
            if (typeof handler !== 'function') {
                return;
            }
            handler({ target: this });
        }
    };
}

function createMockSelect(initialValue = '') {
    const element = createMockElement(initialValue);
    let options = [];
    let innerHTML = '';

    Object.defineProperty(element, 'options', {
        get() {
            return options;
        }
    });

    Object.defineProperty(element, 'innerHTML', {
        get() {
            return innerHTML;
        },
        set(value) {
            innerHTML = value;
            options = [];
        }
    });

    element.appendChild = (option) => {
        options.push(option);
    };

    return element;
}

test('app.html exposes current quaternion and current position buttons in the DQ calculator creator section', () => {
    const html = readFileSync(resolve(ROOT, 'app.html'), 'utf8');

    assert.match(html, /id="dq-calculator-load-current-quaternion"/);
    assert.match(html, /id="dq-calculator-load-current-position"/);
    assert.match(html, />加载当前四元数</);
    assert.match(html, />加载当前位置</);
});

test('setupDQCalculatorPanel loads the current quaternion and current position into their respective inputs', () => {
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const originalCurrentRobot = appState.currentRobot;
    const originalJointAngles = appState.jointAngles;
    const originalDualArmMode = appState.dualArmMode;
    const originalHumanoidMode = appState.humanoidMode;

    const robotSelect = createMockSelect('robot1');
    const loadCurrentDQBtn = createMockElement('');
    const loadCurrentQuatBtn = createMockElement('');
    const loadCurrentPositionBtn = createMockElement('');
    const applyBtn = createMockElement('');
    const copyInputBtn = createMockElement('');
    const copyOutputBtn = createMockElement('');
    const statusEl = createMockElement('');
    const quatStatusEl = createMockElement('');
    const inputEl = createMockElement('');
    const outputEl = createMockElement('');
    const quatW = createMockElement('1');
    const quatX = createMockElement('0');
    const quatY = createMockElement('0');
    const quatZ = createMockElement('0');
    const posX = createMockElement('0');
    const posY = createMockElement('0');
    const posZ = createMockElement('0');
    const translateX = createMockElement('0');
    const translateY = createMockElement('0');
    const translateZ = createMockElement('0');
    const createFromQuatBtn = createMockElement('');

    const elements = new Map([
        ['dq-calculator-robot', robotSelect],
        ['dq-calculator-load-current', loadCurrentDQBtn],
        ['dq-calculator-load-current-quaternion', loadCurrentQuatBtn],
        ['dq-calculator-load-current-position', loadCurrentPositionBtn],
        ['dq-calculator-apply', applyBtn],
        ['dq-calculator-copy-input', copyInputBtn],
        ['dq-calculator-copy-output', copyOutputBtn],
        ['dq-calculator-status', statusEl],
        ['dq-calculator-input', inputEl],
        ['dq-calculator-output', outputEl],
        ['dq-calculator-quat-w', quatW],
        ['dq-calculator-quat-x', quatX],
        ['dq-calculator-quat-y', quatY],
        ['dq-calculator-quat-z', quatZ],
        ['dq-calculator-pos-x', posX],
        ['dq-calculator-pos-y', posY],
        ['dq-calculator-pos-z', posZ],
        ['dq-calculator-translate-x', translateX],
        ['dq-calculator-translate-y', translateY],
        ['dq-calculator-translate-z', translateZ],
        ['dq-calculator-create-from-quat', createFromQuatBtn],
        ['dq-calculator-quat-status', quatStatusEl]
    ]);

    globalThis.window = {
        DQModule: {
            DQWrapper: {
                createFromArray(values) {
                    return values;
                },
                getRotation() {
                    return [0.92388, 0, 0.382683, 0];
                },
                getTranslation() {
                    return [0, 0.456789, -0.123456, 0.8];
                }
            }
        }
    };
    globalThis.document = {
        getElementById(id) {
            return elements.get(id) || null;
        },
        querySelector() {
            return null;
        },
        createElement() {
            return {
                value: '',
                textContent: ''
            };
        }
    };

    appState.currentRobot = {
        fkm() {
            return [1, 0, 0, 0, 0, 0.1, 0.2, 0.3];
        }
    };
    appState.jointAngles = [0];
    appState.dualArmMode = false;
    appState.humanoidMode = false;

    try {
        setupDQCalculatorPanel();

        loadCurrentQuatBtn.trigger('click');

        assert.equal(quatW.value, '0.92388');
        assert.equal(quatX.value, '0');
        assert.equal(quatY.value, '0.382683');
        assert.equal(quatZ.value, '0');
        assert.equal(posX.value, '0');
        assert.equal(posY.value, '0');
        assert.equal(posZ.value, '0');
        assert.equal(quatStatusEl.textContent, 'Loaded the current quaternion into the quaternion inputs');

        loadCurrentPositionBtn.trigger('click');

        assert.equal(posX.value, '0.456789');
        assert.equal(posY.value, '-0.123456');
        assert.equal(posZ.value, '0.8');
        assert.equal(quatW.value, '0.92388');
        assert.equal(quatY.value, '0.382683');
        assert.equal(quatStatusEl.textContent, 'Loaded the current position into the translation inputs');
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
        appState.currentRobot = originalCurrentRobot;
        appState.jointAngles = originalJointAngles;
        appState.dualArmMode = originalDualArmMode;
        appState.humanoidMode = originalHumanoidMode;
    }
});
