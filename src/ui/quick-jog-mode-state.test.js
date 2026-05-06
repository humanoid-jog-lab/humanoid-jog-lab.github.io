import test from 'node:test';
import assert from 'node:assert/strict';

import * as modeSwitcher from './mode-switcher.js';

test('getHumanoidQuickJogControlState maps standard, distance, and polishing controls', () => {
    assert.equal(typeof modeSwitcher.getHumanoidQuickJogControlState, 'function');

    const standard = modeSwitcher.getHumanoidQuickJogControlState('relative');
    assert.equal(standard.showDistanceControls, false);
    assert.deepEqual(
        Object.entries(standard.axes)
            .filter(([, axisState]) => axisState.visible)
            .map(([axis, axisState]) => `${axis}:${axisState.label}`),
        ['x:X', 'y:Y', 'z:Z', 'rx:Rx', 'ry:Ry', 'rz:Rz']
    );

    const distance = modeSwitcher.getHumanoidQuickJogControlState('distance');
    assert.equal(distance.showDistanceControls, true);
    assert.deepEqual(
        Object.values(distance.axes).map((axisState) => axisState.visible),
        [false, false, false, false, false, false]
    );

    const polish = modeSwitcher.getHumanoidQuickJogControlState('cooperative-ruled-surface-polishing-dq-setpoint');
    assert.equal(polish.showDistanceControls, false);
    assert.deepEqual(
        Object.entries(polish.axes)
            .filter(([, axisState]) => axisState.visible)
            .map(([axis, axisState]) => `${axis}:${axisState.label}`),
        ['x:u', 'y:vc', 'z:n', 'rx:Δv']
    );
});
