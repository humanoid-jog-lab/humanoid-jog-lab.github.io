import test from 'node:test';
import assert from 'node:assert/strict';

import { selectBranch } from './predictive-cert.js';

test('selectBranch prefers larger immediate deltaW before lower cost when feasibility metrics tie', () => {
    const branches = [
        { branchIdx: 0, h: 2, mu: 0.01, sigma: 0.02, deltaW: 0.001, cost: 0.1 },
        { branchIdx: 1, h: 2, mu: 0.01, sigma: 0.02, deltaW: 0.004, cost: 0.2 }
    ];

    const { selected, label } = selectBranch(branches, null, { horizon: 3 });

    assert.equal(label, 'horizon-limited');
    assert.equal(selected?.branchIdx, 1);
});

test('selectBranch still uses lower cost when deltaW also ties', () => {
    const branches = [
        { branchIdx: 0, h: 2, mu: 0.01, sigma: 0.02, deltaW: 0.003, cost: 0.1 },
        { branchIdx: 1, h: 2, mu: 0.01, sigma: 0.02, deltaW: 0.003, cost: 0.2 }
    ];

    const { selected } = selectBranch(branches, null, { horizon: 3 });

    assert.equal(selected?.branchIdx, 0);
});
