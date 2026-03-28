import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildCurrentCCTSMPayload,
    buildPriorityCurrentCCTSMPayload
} from './humanoid-jog-diagnostics.js';

test('buildPriorityCurrentCCTSMPayload carries current CCTSM values into the diagnostics payload', () => {
    const payload = buildPriorityCurrentCCTSMPayload({
        modeTag: 'absolute-target',
        includeWaist: true,
        keepBodyZVertical: true,
        jointLimitAvoidance: true,
        cctsmResult: {
            w: 0.267,
            w_cmd: 0.154,
            w_abs: 0.201,
            w_abs_arm: 0.181,
            w_abs_vfi: 0.188,
            w_vfi: 0.267
        }
    });

    assert.equal(payload.solverMode, 'priority');
    assert.equal(payload.modeTag, 'absolute-target');
    assert.equal(payload.message, '当前为零空间投影求解器，显示当前 CCTSM。');
    assert.equal(payload.includeWaist, true);
    assert.equal(payload.keepBodyZVertical, true);
    assert.equal(payload.jointLimitAvoidance, true);
    assert.equal(payload.cctsmW, 0.267);
    assert.equal(payload.cctsmWCmd, 0.154);
    assert.equal(payload.cctsmWAbs, 0.201);
    assert.equal(payload.cctsmWAbsArm, 0.181);
    assert.equal(payload.cctsmWAbsVfi, 0.188);
    assert.equal(payload.cctsmWvfi, 0.267);
});

test('buildPriorityCurrentCCTSMPayload preserves NaN when the current CCTSM is unavailable', () => {
    const payload = buildPriorityCurrentCCTSMPayload({
        modeTag: 'relative-target',
        includeWaist: false,
        jointLimitAvoidance: false,
        cctsmResult: null
    });

    assert.equal(payload.solverMode, 'priority');
    assert.equal(payload.modeTag, 'relative-target');
    assert.ok(Number.isNaN(payload.cctsmW));
    assert.ok(Number.isNaN(payload.cctsmWCmd));
    assert.ok(Number.isNaN(payload.cctsmWAbs));
    assert.ok(Number.isNaN(payload.cctsmWAbsArm));
    assert.ok(Number.isNaN(payload.cctsmWAbsVfi));
    assert.ok(Number.isNaN(payload.cctsmWvfi));
});
