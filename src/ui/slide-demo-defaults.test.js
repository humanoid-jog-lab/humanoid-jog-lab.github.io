import test from 'node:test';
import assert from 'node:assert/strict';
import {
    applySlideDemoStorageDefaults,
    isSlideDemoLaunch,
    SLIDE_DEMO_STORAGE_DEFAULTS
} from './slide-demo-defaults.js';

function createStorage(initial = {}) {
    const data = new Map(Object.entries(initial));
    return {
        getItem(key) {
            return data.has(key) ? data.get(key) : null;
        },
        setItem(key, value) {
            data.set(key, String(value));
        }
    };
}

test('isSlideDemoLaunch only enables the Slidev demo query', () => {
    assert.equal(isSlideDemoLaunch('?slidevDemo=1'), true);
    assert.equal(isSlideDemoLaunch('slidevDemo=1'), true);
    assert.equal(isSlideDemoLaunch('?slidevDemo=0'), false);
    assert.equal(isSlideDemoLaunch('?demo=slidev'), false);
});

test('applySlideDemoStorageDefaults overwrites stale solver and constraint settings', () => {
    const storage = createStorage({
        'humanoid-waist-x-corridor-enabled': '0',
        'humanoid-waist-x-corridor-min': '-0.01',
        'humanoid-waist-x-corridor-max': '0.01',
        'humanoid-test-dual-arm-lock-enabled': '1',
        'teach-speed-percent': '100'
    });

    assert.equal(applySlideDemoStorageDefaults({ search: '?slidevDemo=1', storage }), true);

    Object.entries(SLIDE_DEMO_STORAGE_DEFAULTS).forEach(([key, value]) => {
        assert.equal(storage.getItem(key), value, key);
    });
});

test('applySlideDemoStorageDefaults leaves normal app launches untouched', () => {
    const storage = createStorage({
        'humanoid-waist-x-corridor-enabled': '1'
    });

    assert.equal(applySlideDemoStorageDefaults({ search: '', storage }), false);
    assert.equal(storage.getItem('humanoid-waist-x-corridor-enabled'), '1');
});
