import test from 'node:test';
import assert from 'node:assert/strict';

import { getTeachRepeatMs } from './teach-repeat.js';

test('getTeachRepeatMs returns the shared teach-button repeat cadence', () => {
    assert.equal(getTeachRepeatMs(), 20);
});
