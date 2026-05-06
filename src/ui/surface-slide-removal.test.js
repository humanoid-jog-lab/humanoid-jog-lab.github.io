import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('old surface-slide and plain cooperative polishing control modes stay removed while DQ polishing modes remain available', () => {
    const appHtml = readFileSync(resolve(ROOT, 'app.html'), 'utf8');
    assert.doesNotMatch(appHtml, /value="surface-slide"/);
    assert.doesNotMatch(appHtml, /value="surface-slide-absolute"/);
    assert.doesNotMatch(appHtml, /value="dual-independent-surface-slide"/);
    assert.match(appHtml, /value="left-ruled-surface-polishing"/);
    assert.doesNotMatch(appHtml, /value="cooperative-ruled-surface-polishing"/);
    assert.match(appHtml, /value="cooperative-ruled-surface-polishing-dq-setpoint"/);
    assert.match(appHtml, /value="cooperative-ruled-surface-polishing-dq-setpoint-arm-only"/);

    const modeSwitcherJs = readFileSync(resolve(ROOT, 'src/ui/mode-switcher.js'), 'utf8');
    assert.doesNotMatch(modeSwitcherJs, /Surface Slide/);
    assert.doesNotMatch(modeSwitcherJs, /surface-slide/);
    assert.doesNotMatch(modeSwitcherJs, /dual-independent-surface-slide/);
    assert.match(modeSwitcherJs, /left-ruled-surface-polishing/);
    assert.doesNotMatch(modeSwitcherJs, /mode === 'cooperative-ruled-surface-polishing'/);
    assert.doesNotMatch(modeSwitcherJs, /opt\.value === 'cooperative-ruled-surface-polishing'/);
    assert.match(modeSwitcherJs, /cooperative-ruled-surface-polishing-dq-setpoint/);
    assert.match(modeSwitcherJs, /cooperative-ruled-surface-polishing-dq-setpoint-arm-only/);
    assert.doesNotMatch(modeSwitcherJs, /prepareHumanoidCooperativeRuledSurfacePolishingMode/);

    const eventHandlersJs = readFileSync(resolve(ROOT, 'src/ui/event-handlers.js'), 'utf8');
    assert.doesNotMatch(eventHandlersJs, /controlMode === 'surface-slide'/);
    assert.doesNotMatch(eventHandlersJs, /controlMode === 'surface-slide-absolute'/);
    assert.doesNotMatch(eventHandlersJs, /controlMode === 'dual-independent-surface-slide'/);
    assert.match(eventHandlersJs, /controlMode === 'left-ruled-surface-polishing'/);
    assert.doesNotMatch(eventHandlersJs, /controlMode === 'cooperative-ruled-surface-polishing'/);
    assert.match(eventHandlersJs, /controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint'/);
    assert.match(eventHandlersJs, /controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'/);

    const taskAdapterJs = readFileSync(resolve(ROOT, 'src/core/humanoid-jog-mj-task-adapter.js'), 'utf8');
    assert.doesNotMatch(taskAdapterJs, /'surface-slide'/);
    assert.doesNotMatch(taskAdapterJs, /'surface-slide-absolute'/);
    assert.doesNotMatch(taskAdapterJs, /'dual-independent-surface-slide'/);
    assert.match(taskAdapterJs, /'cooperative-ruled-surface-polishing'/);
    assert.match(taskAdapterJs, /'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'/);

    const profileJs = readFileSync(resolve(ROOT, 'src/core/humanoid-jog-mj-profile.js'), 'utf8');
    assert.doesNotMatch(profileJs, /surface-slide/);
    assert.match(profileJs, /cooperative-ruled-surface-polishing/);
    assert.match(profileJs, /cooperative-ruled-surface-polishing-dq-setpoint-arm-only/);

    const modeSupportJs = readFileSync(resolve(ROOT, 'src/core/humanoid-jog-mj-mode-support.js'), 'utf8');
    assert.doesNotMatch(modeSupportJs, /surface-slide-target/);
    assert.doesNotMatch(modeSupportJs, /dual-independent-surface-slide-target/);
    assert.match(modeSupportJs, /cooperative-ruled-surface-polishing-target/);
    assert.match(modeSupportJs, /cooperative-ruled-surface-polishing-dq-setpoint-arm-only-target/);
});
