# MuJoCo-Style Restore + Pre-w Controller Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new humanoid jog solver mode that runs one protected `w` pre-optimization step before ordinary `MuJoCo 风格恢复` for `absolute / decoupled / decoupled-upright`, while keeping plain `mj_restore` untouched.

**Architecture:** Extend the solver surface with a new solver id `mj_restore_pre_w`, add a small helper module that owns pre-w mode support, metric resolution, and outcome mapping, and then let `RobotVisualizer` orchestrate `pre-w -> mj_restore` only for the supported absolute-family modes. Unsupported modes keep their current rollback-baseline behavior and only report `pre_w: disabled_for_mode` in diagnostics.

**Tech Stack:** Vite frontend, ES modules, `src/visualizer.js`, Node test runner (`node:test`), existing humanoid `mj_restore` / metric-only `w` pipelines.

---

## File Map

### Create

- `src/core/humanoid-jog-mj-pre-w.js`
  - Pure helper for:
    - allowed control modes for pre-w
    - scoped metric resolution for `mj_restore_pre_w`
    - fixed `increase` direction policy
    - mapping optimizer outcomes to `pre_w` diagnostics and `q_pre` eligibility
- `src/core/humanoid-jog-mj-pre-w.test.js`
  - Unit tests for the helper above
- `src/ui/event-handlers.test.js`
  - Verify solver persistence/apply behavior for `mj_restore_pre_w`

### Modify

- `index.html`
  - Add solver option `mj_restore_pre_w`
- `src/core/humanoid-jog-solver-presentation.js`
  - Normalize/display the new solver mode
- `src/core/humanoid-jog-solver-presentation.test.js`
  - Extend solver normalization/display coverage
- `src/ui/event-handlers.js`
  - Persist/apply the new solver option without changing `w` selector behavior
- `src/ui/humanoid-manip-controls.test.js`
  - Verify the new solver appears in UI and existing `w` selector remains unchanged
- `src/visualizer.js`
  - Add orchestration for:
    - supported-mode pre-w stage
    - candidate-only `q_pre`
    - fallback to `q0`
    - diagnostic fields `pre_w`, `pre_w_metric`, `pre_w_start`
- `src/core/humanoid-jog-manifold-config.test.js`
  - Add integration-style tests for visualizer routing and fallback semantics
- `src/core/humanoid-jog-mj-diagnostics.js`
  - Extend MJ diagnostics payloads with explicit pre-w fields
- `src/core/humanoid-jog-mj-diagnostics.test.js`
  - Add coverage for the new `pre_w` diagnostic fields in the MJ diagnostics helper

## Chunk 1: Solver Surface And Pure Pre-w Policy

### Task 1: Expose `mj_restore_pre_w` In UI And Solver Presentation

**Files:**
- Modify: `index.html`
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Modify: `src/core/humanoid-jog-solver-presentation.test.js`
- Modify: `src/ui/humanoid-manip-controls.test.js`
- Create: `src/ui/event-handlers.test.js`
- Modify: `src/ui/event-handlers.js`

- [ ] **Step 1: Write the failing solver-presentation test**

Add assertions in `src/core/humanoid-jog-solver-presentation.test.js` for:
- `normalizeHumanoidJogSolverMode('mj_restore_pre_w') === 'mj_restore_pre_w'`
- `getHumanoidJogSolverDisplayName('mj_restore_pre_w') === 'MuJoCo 风格恢复 + w预优化'`
- `getHumanoidJogSolverReadyMessage('mj_restore_pre_w') === 'MuJoCo 风格恢复 + w预优化 已启用，等待点动输入。'`

- [ ] **Step 2: Run the test to verify it fails**

Run: `node --test src/core/humanoid-jog-solver-presentation.test.js`
Expected: FAIL because `mj_restore_pre_w` is not recognized yet.

- [ ] **Step 3: Write the failing UI test**

Add assertions in `src/ui/humanoid-manip-controls.test.js` that:
- the solver dropdown contains `<option value="mj_restore_pre_w">MuJoCo 风格恢复 + w预优化</option>`
- Task 1 does not add any new humanoid `w` selector surface

- [ ] **Step 4: Run the UI test to verify it fails**

Run: `node --test src/ui/humanoid-manip-controls.test.js`
Expected: FAIL because the new solver option is missing.

- [ ] **Step 5: Write the failing persistence/apply test**

Create `src/ui/event-handlers.test.js` with a focused test that:
- seeds the humanoid solver storage key with `mj_restore_pre_w`
- runs the relevant solver-setup portion of `event-handlers.js`
- verifies the select value, `window.humanoidJogSolverMode`, and the visualizer
  callback receive `mj_restore_pre_w`

- [ ] **Step 6: Run the persistence test to verify it fails**

Run: `node --test src/ui/event-handlers.test.js`
Expected: FAIL because the new solver mode is not yet wired through setup.

- [ ] **Step 7: Implement the minimal solver-surface changes**

Update:
- `src/core/humanoid-jog-solver-presentation.js`
- `index.html`
- `src/ui/event-handlers.js`

Do not touch `src/visualizer.js` yet.

- [ ] **Step 8: Run the solver/UI tests to verify they pass**

Run: `node --test src/core/humanoid-jog-solver-presentation.test.js src/ui/humanoid-manip-controls.test.js src/ui/event-handlers.test.js`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add index.html src/core/humanoid-jog-solver-presentation.js src/core/humanoid-jog-solver-presentation.test.js src/ui/humanoid-manip-controls.test.js src/ui/event-handlers.js src/ui/event-handlers.test.js
git commit -m "feat: expose mj restore pre-w solver option"
```

### Task 2: Add A Pure Helper For Pre-w Mode Support, Metric Resolution, And Outcome Mapping

**Files:**
- Create: `src/core/humanoid-jog-mj-pre-w.js`
- Create: `src/core/humanoid-jog-mj-pre-w.test.js`

- [ ] **Step 1: Write the failing helper tests**

In `src/core/humanoid-jog-mj-pre-w.test.js`, cover:
- supported control modes:
  - `absolute`, `decoupled`, `decoupled-upright` => enabled
  - `relative`, `relative-fullbody`, `distance`, `distance-fullbody` => disabled
- metric resolution:
  - only `w_abs_hier` / `w_abs_hier_vfi` are accepted
  - everything else falls back to `w_abs_hier`
- direction policy:
  - always returns `increase`
- outcome mapping:
  - `active`
  - `active(primary_only; ...)`
  - `active(primary_only; ...)` with accepted candidate
  - `active(primary_only; ...)` without accepted candidate
  - `skipped(no_progress)`
  - `skipped(no_gradient)`
  - `skipped(projection_only_no_progress)`
  - `rejected(hard_constraints)`
  - thrown/invalid cases
  - explicit `q_pre` eligibility for each mapped case

- [ ] **Step 2: Run the helper tests to verify they fail**

Run: `node --test src/core/humanoid-jog-mj-pre-w.test.js`
Expected: FAIL because the helper file does not exist yet.

- [ ] **Step 3: Implement the minimal pure helper**

In `src/core/humanoid-jog-mj-pre-w.js`, add pure functions such as:
- `supportsHumanoidJogMJPreWControlMode(controlMode)`
- `resolveHumanoidJogMJPreWMetric(metricRaw)`
- `getHumanoidJogMJPreWDirection()`
- `mapHumanoidJogMJPreWOutcome(metricResult)`

Keep the file pure and UI-free.

- [ ] **Step 4: Run the helper tests to verify they pass**

Run: `node --test src/core/humanoid-jog-mj-pre-w.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-pre-w.js src/core/humanoid-jog-mj-pre-w.test.js
git commit -m "feat: add mj restore pre-w policy helpers"
```

## Chunk 2: Visualizer Orchestration

### Task 3: Add `mj_restore_pre_w` Routing Without Polluting Plain `mj_restore`

**Files:**
- Modify: `src/ui/event-handlers.js`
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`

- [ ] **Step 1: Write the failing visualizer routing tests**

In `src/core/humanoid-jog-manifold-config.test.js`, add tests that verify:
- `setHumanoidJogSolverMode('mj_restore_pre_w')` is retained by `RobotVisualizer`
- in `absolute` mode, `mj_restore_pre_w`:
  - resolves the selected `w` metric through the dedicated pre-w resolver
  - runs pre-w first
  - then calls the existing `mj_restore` absolute path using `q_pre`
- in `decoupled` mode, `mj_restore_pre_w`:
  - runs pre-w first with waist joints included
  - keeps current absolute and relative pose protected
  - then calls the existing full-body absolute `mj_restore` path
- in `decoupled-upright` mode, `mj_restore_pre_w`:
  - runs pre-w first with waist joints included
  - keeps current absolute pose, relative pose, and upright body-z protection
  - then calls the existing full-body upright `mj_restore` path
- if pre-w is `skipped(...)`, `rejected(...)`, or `failed(...)`, stage 2 starts from `q0`
- plain `mj_restore` still ignores the selected `w` metric for jog commands

- [ ] **Step 2: Run the routing tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because `mj_restore_pre_w` orchestration does not exist yet.

- [ ] **Step 3: Implement solver persistence/application**

Update `src/ui/event-handlers.js` so:
- the new solver mode persists through the existing solver storage key
- `window.humanoidJogSolverMode` and `visualizer.setHumanoidJogSolverMode(...)` can receive `mj_restore_pre_w`
- no `w` selector value is ever pushed into plain `mj_restore` automatically

- [ ] **Step 4: Implement visualizer pre-w orchestration**

Update `src/visualizer.js` to:
- recognize `mj_restore_pre_w` separately from plain `mj_restore`
- add a small orchestration helper, for example:
  - capture `q0`
  - compute protected pose targets
  - run metric-only pre-w in candidate-only mode
  - map the outcome through `mapHumanoidJogMJPreWOutcome(...)`
  - choose `q_pre` or `q0` for stage 2
  - call the existing plain `mj_restore` absolute-family path

Keep these rules strict:
- pre-w only for `absolute / decoupled / decoupled-upright`
- plain `mj_restore` path must remain unchanged
- pre-w always uses `increase`

- [ ] **Step 5: Re-run the routing tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/ui/event-handlers.js src/visualizer.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: orchestrate mj restore pre-w absolute-family jogs"
```

### Task 4: Preserve Rollback-Baseline Behavior For Unsupported Modes

**Files:**
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`

- [ ] **Step 1: Write the failing unsupported-mode tests**

Extend `src/core/humanoid-jog-manifold-config.test.js` to verify:
- `relative` / `relative-fullbody` with selected solver `mj_restore_pre_w`
  - do not run pre-w
  - continue to use the rollback-baseline relative backend path
- `distance` / `distance-fullbody` with selected solver `mj_restore_pre_w`
  - do not run pre-w
  - continue to use the existing distance `mj_restore` path

- [ ] **Step 2: Run the unsupported-mode tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because unsupported-mode fallback is not explicit yet.

- [ ] **Step 3: Implement explicit unsupported-mode fallback**

Update `src/visualizer.js` so that:
- unsupported modes never attempt pre-w
- mode-family fallback is explicit rather than implied
- selected solver can remain `mj_restore_pre_w` while execution uses the proper rollback-baseline backend for that mode

- [ ] **Step 4: Re-run the unsupported-mode tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: preserve baseline fallback for unsupported pre-w modes"
```

## Chunk 3: Diagnostics And End-to-End Verification

### Task 5: Add Explicit `pre_w` Diagnostics

**Files:**
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-mj-diagnostics.js`
- Modify: `src/core/humanoid-jog-mj-diagnostics.test.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`

- [ ] **Step 1: Write the failing diagnostics tests**

Add coverage for:
- `pre_w`
- `pre_w_metric`
- `pre_w_start`

Cases to cover:
- `pre_w: applied` with `pre_w_start: q_pre`
- `pre_w: skipped(...)` with `pre_w_start: q0`
- `pre_w: disabled_for_mode`

- [ ] **Step 2: Run the diagnostics tests to verify they fail**

Run: `node --test src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because these diagnostics fields are not populated yet.

- [ ] **Step 3: Implement the diagnostics fields**

Update:
- `src/core/humanoid-jog-mj-diagnostics.js`
- `src/visualizer.js`

so the selected solver `mj_restore_pre_w` publishes:
- the mapped `pre_w` state
- the selected metric
- the actual stage-2 start source (`q0` or `q_pre`)

Keep plain `mj_restore` output unchanged.

- [ ] **Step 4: Re-run the diagnostics tests**

Run: `node --test src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-mj-diagnostics.js src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-manifold-config.test.js
git commit -m "feat: report mj restore pre-w diagnostics explicitly"
```

### Task 6: Final Verification

**Files:**
- Verify only

- [ ] **Step 1: Run focused unit/integration tests**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js \
  src/core/humanoid-jog-mj-pre-w.test.js \
  src/core/humanoid-jog-manifold-config.test.js \
  src/core/humanoid-jog-mj-diagnostics.test.js \
  src/ui/humanoid-manip-controls.test.js \
  src/ui/event-handlers.test.js
```

Expected: PASS.

- [ ] **Step 2: Run syntax checks on touched runtime files**

Run:

```bash
node --check src/core/humanoid-jog-solver-presentation.js
node --check src/core/humanoid-jog-mj-pre-w.js
node --check src/core/humanoid-jog-mj-diagnostics.js
node --check src/ui/event-handlers.js
node --check src/visualizer.js
```

Expected: PASS.

- [ ] **Step 3: Smoke-check the app build**

Run: `npm run build`
Expected: PASS with no new frontend build errors.

- [ ] **Step 4: Record manual verification notes**

Verify in browser:
- solver dropdown shows `MuJoCo 风格恢复 + w预优化`
- `absolute / decoupled / decoupled-upright` use pre-w before jog
- `relative / distance` do not use pre-w
- plain `MuJoCo 风格恢复` still behaves exactly as before

- [ ] **Step 5: Commit**

```bash
git add index.html src/core/humanoid-jog-solver-presentation.js src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-mj-pre-w.js src/core/humanoid-jog-mj-pre-w.test.js src/ui/event-handlers.js src/ui/event-handlers.test.js src/ui/humanoid-manip-controls.test.js src/visualizer.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-diagnostics.js src/core/humanoid-jog-mj-diagnostics.test.js docs/superpowers/specs/2026-03-26-mj-restore-pre-w-design.md docs/superpowers/plans/2026-03-26-mj-restore-pre-w-controller.md
git commit -m "feat: add mj restore pre-w humanoid jog controller"
```
