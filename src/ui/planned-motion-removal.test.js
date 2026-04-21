import test from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const ROOT = resolve(import.meta.dirname, '../..');

test('planned motion UI and monitor assets are fully removed from the source tree', () => {
    const indexHtml = readFileSync(resolve(ROOT, 'index.html'), 'utf8');
    assert.doesNotMatch(indexHtml, /planned-motion-section/);
    assert.doesNotMatch(indexHtml, /planned-motion-joint-monitor/);
    assert.doesNotMatch(indexHtml, /规划运动/);
    assert.doesNotMatch(indexHtml, /执行关节实时监控/);

    const mainJs = readFileSync(resolve(ROOT, 'src/main.js'), 'utf8');
    assert.doesNotMatch(mainJs, /planned-motion/);

    const eventHandlersJs = readFileSync(resolve(ROOT, 'src/ui/event-handlers.js'), 'utf8');
    assert.doesNotMatch(eventHandlersJs, /setupPlannedMotionListeners/);

    const modeSwitcherJs = readFileSync(resolve(ROOT, 'src/ui/mode-switcher.js'), 'utf8');
    assert.doesNotMatch(modeSwitcherJs, /planned-motion-section/);

    const stateJs = readFileSync(resolve(ROOT, 'src/core/state.js'), 'utf8');
    assert.doesNotMatch(stateJs, /planned-motion/);

    const stylesCss = readFileSync(resolve(ROOT, 'src/styles.css'), 'utf8');
    assert.doesNotMatch(stylesCss, /\.planned-motion-/);

    assert.equal(existsSync(resolve(ROOT, 'joint-monitor.html')), false);
    assert.equal(existsSync(resolve(ROOT, 'src/features/planned-motion.js')), false);
    assert.equal(existsSync(resolve(ROOT, 'src/features/joint-monitor-window.js')), false);
});
