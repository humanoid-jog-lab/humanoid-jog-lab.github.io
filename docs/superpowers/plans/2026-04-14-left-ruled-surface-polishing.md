# Left Ruled-Surface Polishing Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a brand-new humanoid-only `left-ruled-surface-polishing` mode that exposes a selectable UI option and drives the left hand with an explicit single-strip ruled-surface Jacobian while keeping the right arm and waist fixed.

**Architecture:** Reuse the existing surface projection, surface visualization, and Plucker tube invariant helpers, but do not restore the removed `surface-slide` mode names or behavior contracts. The new mode should have its own UI option, task descriptor, and visualizer pipeline that builds an explicit `J_surf^(y)` command differential and sends it through the existing `mj_restore` shell as a left-arm-only differential restoration problem.

**Tech Stack:** Vite, vanilla ES modules, Node test runner (`node --test`), Three.js visualizer, DQ pose arrays via `DQModule`, existing `mj_restore` humanoid restoration helpers.

---

## File Map

- Modify: `index.html`
  - Add the humanoid-only mode option for the new left-hand ruled-surface polishing mode.
- Modify: `src/i18n.js`
  - Add the bilingual labels and mode descriptions used by the new mode.
- Modify: `src/ui/mode-switcher.js`
  - Expose the new mode in humanoid UI, map teach pendant labels to `u / v / n`, and disable rotation buttons.
- Modify: `src/ui/event-handlers.js`
  - Route teach pendant button presses into the new visualizer move handler.
- Modify: `src/ui/humanoid-manip-controls.test.js`
  - Assert that the new mode is visible while old removed surface-slide modes stay removed.
- Modify: `src/ui/surface-slide-removal.test.js`
  - Update removal assertions so they still reject the old mode names but allow the new ruled-surface polishing mode.
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
  - Add a descriptor for `left-ruled-surface-polishing`.
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
  - Allow the new mode tag through the `mj_restore` support gate.
- Modify: `src/core/humanoid-jog-mj-profile.js`
  - Reuse the cooperative polishing profile for the new single-arm mode.
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
  - Assert the new descriptor shape and preserved invariants.
- Modify: `src/core/humanoid-jog-mj-mode-support.test.js`
  - Assert the new mode tag is supported while old removed surface-slide tags remain unsupported.
- Modify: `src/core/humanoid-jog-mj-profile.test.js`
  - Assert the new mode resolves to the intended profile/tag.
- Modify: `src/visualizer.js`
  - Add the new left-hand ruled-surface state, explicit command builder, differential restore helper, and parameter motion entrypoint.
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
  - Add visualizer-level tests for state initialization, explicit Jacobian command generation, and differential restore dispatch.

## Chunk 1: UI And Mode Contracts

### Task 1: Add the new humanoid mode option without reviving removed surface-slide modes

**Files:**
- Modify: `src/ui/humanoid-manip-controls.test.js`
- Modify: `src/ui/surface-slide-removal.test.js`
- Modify: `index.html`

- [ ] **Step 1: Write the failing test**

```js
test('index.html exposes left ruled-surface polishing while keeping removed surface-slide modes hidden', () => {
    const html = readFileSync(resolve(ROOT, 'index.html'), 'utf8');
    assert.match(html, /<option value="left-ruled-surface-polishing">左臂擦面<\/option>/);
    assert.doesNotMatch(html, /<option value="surface-slide">/);
    assert.doesNotMatch(html, /<option value="surface-slide-absolute">/);
    assert.doesNotMatch(html, /<option value="dual-independent-surface-slide">/);
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `node --test src/ui/humanoid-manip-controls.test.js src/ui/surface-slide-removal.test.js`
Expected: FAIL because `left-ruled-surface-polishing` is not present yet.

- [ ] **Step 3: Write minimal implementation**

```html
<option value="left-ruled-surface-polishing">左臂擦面</option>
```

Add the option only in the humanoid mode list. Do not re-add `surface-slide`, `surface-slide-absolute`, or `dual-independent-surface-slide`.

- [ ] **Step 4: Run test to verify it passes**

Run: `node --test src/ui/humanoid-manip-controls.test.js src/ui/surface-slide-removal.test.js`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add index.html src/ui/humanoid-manip-controls.test.js src/ui/surface-slide-removal.test.js
git commit -m "feat: add left ruled-surface mode option"
```

### Task 2: Wire the new mode into humanoid UI labels and teach-pendant dispatch

**Files:**
- Modify: `src/i18n.js`
- Modify: `src/ui/mode-switcher.js`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/ui/surface-slide-removal.test.js`

- [ ] **Step 1: Write the failing test**

Add source-contract assertions that the new mode is explicitly recognized by `mode-switcher.js` and `event-handlers.js`, while the old removed mode names stay absent from the UI branching contract.

```js
assert.match(modeSwitcherJs, /left-ruled-surface-polishing/);
assert.match(eventHandlersJs, /controlMode === 'left-ruled-surface-polishing'/);
assert.doesNotMatch(modeSwitcherJs, /surface-slide-absolute/);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `node --test src/ui/surface-slide-removal.test.js`
Expected: FAIL because the new mode is not yet handled in the UI source files.

- [ ] **Step 3: Write minimal implementation**

Implement these behaviors:

```js
if (mode === 'left-ruled-surface-polishing') {
    return t('Left Ruled-Surface Polishing: control u, v, and n while keeping the right arm and waist fixed');
}
```

```js
if (appState.visualizer.humanoidMode && controlMode === 'left-ruled-surface-polishing') {
    appState.visualizer.moveHumanoidLeftRuledSurfacePolishingAlongParameter(parameterAxis, direction);
}
```

Update the teach-pendant schema so this mode uses:
- `x -> u`
- `y -> v`
- `z -> n`

Disable all rotation buttons in this mode.

- [ ] **Step 4: Run test to verify it passes**

Run: `node --test src/ui/surface-slide-removal.test.js`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/i18n.js src/ui/mode-switcher.js src/ui/event-handlers.js src/ui/surface-slide-removal.test.js
git commit -m "feat: wire left ruled-surface mode into humanoid UI"
```

### Task 3: Add the core mode descriptor and solver-profile contracts

**Files:**
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.test.js`
- Modify: `src/core/humanoid-jog-mj-profile.test.js`
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
- Modify: `src/core/humanoid-jog-mj-profile.js`

- [ ] **Step 1: Write the failing tests**

```js
test('buildHumanoidJogMJTaskDescriptor builds a left ruled-surface polishing descriptor', () => {
    const result = buildHumanoidJogMJTaskDescriptor({
        controlMode: 'left-ruled-surface-polishing',
        commandTarget: { u: 0.3, v: 0.5, normalOffset: 0.05 }
    });
    assert.equal(result.valid, true);
    assert.equal(result.modeTag, 'left-ruled-surface-polishing-target');
    assert.equal(result.commandKind, 'single_arm_surface_polishing_pose');
    assert.deepEqual(result.invariants.map((item) => item.kind), ['right_arm_fixed', 'waist_fixed']);
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-profile.test.js`
Expected: FAIL because the new mode is not described or supported yet.

- [ ] **Step 3: Write minimal implementation**

Add a new `MODE_CONFIGS` entry and matching support/profile branches:

```js
'left-ruled-surface-polishing': {
    modeTag: 'left-ruled-surface-polishing-target',
    commandKind: 'single_arm_surface_polishing_pose',
    activeJointScope: 'left-arm',
    invariants: () => [
        createInvariant('right_arm_fixed', null, 1, 100),
        createInvariant('waist_fixed', null, 0.5, 50)
    ]
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-profile.test.js`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-profile.js src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-profile.test.js
git commit -m "feat: add left ruled-surface mode contracts"
```

## Chunk 2: Visualizer State And Explicit Jacobian Path

### Task 4: Add the left-hand ruled-surface state and explicit command builder

**Files:**
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/visualizer.js`

- [ ] **Step 1: Write the failing tests**

Add focused tests near the existing surface/cooperative coverage:

```js
test('RobotVisualizer left ruled-surface polishing initializes from the current left-hand surface attachment state', () => {
    // expect _ensureHumanoidLeftRuledSurfacePolishingState() to project the left hand
});

test('RobotVisualizer left ruled-surface polishing builds an explicit single-strip differential target for [u, v, n]', () => {
    // expect an 8x3 Jacobian and 8-vector commandTargetArray
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test --test-name-pattern "left ruled-surface polishing" src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because the new visualizer state and explicit command builder do not exist yet.

- [ ] **Step 3: Write minimal implementation**

Add these visualizer methods:

```js
_getHumanoidLeftRuledSurfacePolishingControlMode()
prepareHumanoidLeftRuledSurfacePolishingMode()
_ensureHumanoidLeftRuledSurfacePolishingState()
_buildHumanoidLeftRuledSurfacePolishingExplicitCommand(currentState, nextState)
```

Implementation requirements:
- Reuse the existing demo-surface creation and projection helpers.
- Store only left-hand state: `u`, `v`, `normalOffset`, `normalOffsetSign`, `quaternionOffset`, `bounds`.
- Build explicit command arrays from `buildSurfaceAttachedPoseJacobian(...)`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test --test-name-pattern "left ruled-surface polishing" src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS for the state/init and explicit-command tests.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: add left ruled-surface explicit command builder"
```

### Task 5: Add the single-arm differential restore helper and button-driven parameter motion

**Files:**
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/visualizer.js`

- [ ] **Step 1: Write the failing tests**

Add visualizer-level tests for:

```js
test('RobotVisualizer left ruled-surface polishing forwards explicit differential commands and fixed-arm invariants into single-arm restoration', async () => {
    // expect moveHumanoidLeftRuledSurfacePolishingAlongParameter(...) to call a new differential MJ helper
});

test('RobotVisualizer left ruled-surface polishing keeps the previous state when the solve is not applied', async () => {
    // expect state rollback on applied:false
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test --test-name-pattern "left ruled-surface polishing" src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because the move handler and differential helper do not exist yet.

- [ ] **Step 3: Write minimal implementation**

Add:

```js
async _updateHumanoidLeftRuledSurfacePolishingDifferentialMJRestore(currentState, nextState, currentQ, enableJointLimitAvoidance, internalOptions)
moveHumanoidLeftRuledSurfacePolishingAlongParameter(axis, direction)
```

Implementation requirements:
- Accept only `x`, `y`, `z` parameter axes.
- Use the current left-hand pose as the differential base pose.
- Keep the right arm fixed through the descriptor invariants.
- Keep the waist fixed by using the left-arm-only active-joint slice.
- Preserve the Plucker tube invariant if surface-tube protection is enabled.
- Do not call the old `_moveHumanoidSurfaceAlongParameter(...)` or old `surface-slide` mode strings.

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test --test-name-pattern "left ruled-surface polishing" src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: add left ruled-surface differential restore path"
```

## Chunk 3: Verification And Final Integration

### Task 6: Run the full targeted regression set

**Files:**
- No code changes expected

- [ ] **Step 1: Run UI and core regression tests**

Run:

```bash
node --test src/ui/humanoid-manip-controls.test.js src/ui/surface-slide-removal.test.js src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-profile.test.js
```

Expected: PASS

- [ ] **Step 2: Run ruled-surface visualizer regression tests**

Run:

```bash
node --test src/core/surface-slide-primitive.test.js
node --test --test-name-pattern "cooperative ruled-surface polishing|left ruled-surface polishing" src/core/humanoid-jog-manifold-config.test.js
```

Expected: PASS

- [ ] **Step 3: Run the frontend build**

Run: `npm run build`
Expected: PASS with the existing `numeric.js` eval warning only.

- [ ] **Step 4: Review the final diff**

Run:

```bash
git status --short
git diff --stat
```

Expected: only the planned files are touched, plus any already-existing unrelated dirty files that must not be reverted.

- [ ] **Step 5: Commit**

```bash
git add index.html src/i18n.js src/ui/mode-switcher.js src/ui/event-handlers.js src/ui/humanoid-manip-controls.test.js src/ui/surface-slide-removal.test.js src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-profile.js src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-profile.test.js src/visualizer.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: add left ruled-surface polishing mode"
```
