import test from 'node:test';
import assert from 'node:assert/strict';

import {
    runHumanoidJogOfflineSampler,
    serializeHumanoidJogOfflineSamplesToCsv
} from './humanoid-jog-offline-sampler.js';

const INITIAL_Q = [
    -0.20794, 0.90626, -0.69682, 0,
    -0.611, 0, 0.00586, -0.94966, -0.00966, 0.00069, -0.01189,
    0.611, 0, -0.00586, 0.94966, 0.00966, -0.00069, 0.01189,
    0, 0
];

test('runHumanoidJogOfflineSampler produces deterministic samples without browser state', () => {
    const result = runHumanoidJogOfflineSampler({
        initialQ: INITIAL_Q,
        mode: 'absolute-target',
        solverMode: 'mj_restore_tr_safe',
        axis: 'x',
        direction: 'positive',
        steps: 5,
        speedPercent: 50,
        includeWaist: true,
        generatedAt: '2026-04-26T00:00:00.000Z'
    });

    assert.equal(result.meta.source, 'offline-core');
    assert.equal(result.meta.usesBrowser, false);
    assert.equal(result.meta.command.steps, 5);
    assert.equal(result.samples.length, 5);

    for (const sample of result.samples) {
        assert.equal(sample.q.length, INITIAL_Q.length);
        assert.equal(Number.isFinite(sample.err.cmd), true);
        assert.equal(Number.isFinite(sample.err.inv), true);
        assert.equal(Number.isFinite(sample.step), true);
        assert.equal(Number.isFinite(sample.dmin), true);
        assert.equal(Number.isFinite(sample.waist_x.p0x), true);
        assert.equal(typeof sample.safety.tr_safety, 'string');
        assert.equal(Number.isFinite(sample.timingMs.solve), true);
        assert.equal(Number.isFinite(sample.iters.total), true);
    }

    assert.deepEqual(
        result.samples.map((sample) => sample.index),
        [1, 2, 3, 4, 5]
    );
});

test('serializeHumanoidJogOfflineSamplesToCsv exports high-signal plotting columns', () => {
    const result = runHumanoidJogOfflineSampler({
        initialQ: INITIAL_Q,
        steps: 2,
        speedPercent: 20,
        includeWaist: true,
        generatedAt: '2026-04-26T00:00:00.000Z'
    });

    const csv = serializeHumanoidJogOfflineSamplesToCsv(result);

    assert.match(csv, /^index,status,reason,err_cmd,err_inv,step,iters_total,dmin,waist_x,waist_d_plus,waist_d_minus,tr_safety,timing_solve_ms/);
    assert.equal(csv.trim().split('\n').length, 3);
    assert.doesNotMatch(csv, /Playwright|chromium|document|window/);
});
