import test from 'node:test';
import assert from 'node:assert/strict';

import { minimizeLBFGS } from './lbfgs-utils.js';

test('minimizeLBFGS converges on an ill-conditioned quadratic', () => {
    const f = (x) => 0.5 * (1000 * x[0] * x[0] + x[1] * x[1]);
    const grad = (x, out) => {
        out[0] = 1000 * x[0];
        out[1] = x[1];
    };

    const result = minimizeLBFGS(f, grad, [1, 1], {
        maxIter: 40,
        gradTol: 1e-8,
        costTol: 1e-16,
        m: 10
    });

    assert.equal(result.converged, true, `expected convergence, reason=${result.reason}`);
    assert.ok(result.cost < 1e-16, `expected low cost, got ${result.cost}`);
    assert.ok(Math.abs(result.x[0]) < 1e-8);
    assert.ok(Math.abs(result.x[1]) < 1e-8);
    assert.ok(result.iterations <= 10, `expected fast convergence, got ${result.iterations}`);
});

test('minimizeLBFGS accepts practical convergence via cost tolerance', () => {
    const f = (x) => 500 * x[0] * x[0];
    const grad = (x, out) => {
        out[0] = 1000 * x[0];
    };

    const result = minimizeLBFGS(f, grad, [1e-6], {
        gradTol: 1e-12,
        costTol: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'cost');
});
