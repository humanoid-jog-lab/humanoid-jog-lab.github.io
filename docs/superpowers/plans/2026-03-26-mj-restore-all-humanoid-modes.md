# MuJoCo-Style Restore All Humanoid Modes Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend plain `MuJoCo 风格恢复` so it truly supports all humanoid task modes under one solver framework, while preserving the existing task semantics of each mode and eliminating silent fallback to `new qpJ`.

**Architecture:** Add a thin humanoid MJ task-adapter layer that converts each control mode into a unified task description (`command`, `invariants[]`, `extraConstraints`, `active joints`) and then route all supported modes through the existing `mj_restore` restore machinery. Land the work in three phases: adapter foundation, `relative` family integration, then special single-arm/waist modes, while keeping `mj_restore_pre_w` explicitly limited to its current absolute-family scope.

**Tech Stack:** Vite frontend, ES modules, `src/visualizer.js`, existing humanoid `mj_restore` restore path, Node test runner (`node:test`), current diagnostics helpers.

---

## File Map

### Create

- `src/core/humanoid-jog-mj-task-adapter.js`
  - Pure adapter that maps humanoid control modes to unified `mj_restore` task descriptors.
- `src/core/humanoid-jog-mj-task-adapter.test.js`
  - Unit tests for task adapter outputs across all humanoid modes.

### Modify

- `src/core/humanoid-jog-mj-mode-support.js`
  - Expand `mj_restore` mode support in stages as each new task family becomes actually wired.
- `src/core/humanoid-jog-mj-mode-support.test.js`
  - Update support matrix expectations alongside each staged wiring change.
- `src/visualizer.js`
  - Replace current per-mode fallback branches with adapter-driven `mj_restore` routing.
  - Keep `mj_restore_pre_w` explicitly disabled outside `absolute / decoupled / decoupled-upright`.
- `src/core/humanoid-jog-mj-diagnostics.js`
  - Support adapter-driven diagnostics with multi-invariant reporting and explicit hard no-commit failure reasons.
- `src/core/humanoid-jog-mj-diagnostics.test.js`
  - Add coverage for multi-invariant diagnostics and no-fallback failure states.
- `src/core/humanoid-jog-manifold-config.test.js`
  - Add integration-style tests for routing, hard no-commit behavior, and `mj_restore_pre_w` disabled-for-mode semantics.
- `src/core/humanoid-jog-diagnostics.test.js`
  - Extend generic diagnostics assertions if modeTag / invariant shape is surfaced there.

## Chunk 1: Adapter Foundation And Support Matrix

### Task 1: Add A Pure Humanoid MJ Task Adapter

**Files:**
- Create: `src/core/humanoid-jog-mj-task-adapter.js`
- Create: `src/core/humanoid-jog-mj-task-adapter.test.js`

- [ ] **Step 1: Write the failing adapter tests**

In `src/core/humanoid-jog-mj-task-adapter.test.js`, add focused tests for:
- `absolute` → `modeTag: 'absolute-target'`, `commandKind: 'absolute_pose'`, correct `commandTarget`, diagnostic label, invariant `relative_pose`, active joints `arms`
- `decoupled` → `commandKind: 'absolute_pose'`, correct `commandTarget`, diagnostic label, invariant `relative_pose`, active joints `waist+arms`
- `decoupled-upright` → same as `decoupled` plus `body_z` extra constraint metadata
- `relative` → `modeTag: 'relative-target'`, `commandKind: 'relative_pose'`, correct `commandTarget`, diagnostic label, invariant `absolute_pose`, active joints `arms`
- `relative-fullbody` → same command/invariant swap with active joints `waist+arms`
- `distance` / `distance-fullbody` → `modeTag: 'distance-target'`, `commandKind: 'relative_distance'`, correct distance target, invariant `absolute_pose`
- `robot1` → single-arm command with `commandTarget` for the left-arm pose and invariants holding right-arm and waist joint groups fixed, including invariant weights/priority-equivalents
- `robot2` → single-arm command with `commandTarget` for the right-arm pose and invariants holding left-arm and waist joint groups fixed, including invariant weights/priority-equivalents
- `waist-body-link-xz` → partial body command on `body_link x/z` with correct target payload and invariant weights for fixed arm groups
- unsupported/unknown mode → explicit invalid adapter result

- [ ] **Step 2: Run the adapter test to verify it fails**

Run: `node --test src/core/humanoid-jog-mj-task-adapter.test.js`
Expected: FAIL because the adapter file does not exist yet.

- [ ] **Step 3: Write the minimal pure adapter**

Create `src/core/humanoid-jog-mj-task-adapter.js` with pure functions such as:
- `buildHumanoidJogMJTaskDescriptor({ controlMode, currentQ, currentAbsolutePose, currentRelativePose, currentDistance, currentRobot1Pose, currentRobot2Pose, currentBodyLinkPose, targetAbsolutePose, targetRelativePose, targetDistance, targetBodyLinkXZ, includeWaist, keepBodyZVertical, ... })`
- small helpers for:
  - active joint scope
  - invariant array construction
  - special-mode command kinds
  - diagnostic labels
  - invariant weights/priority-equivalents

Do not reach into the DOM or `window` from this file.

- [ ] **Step 4: Run the adapter test to verify it passes**

Run: `node --test src/core/humanoid-jog-mj-task-adapter.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-task-adapter.test.js
git commit -m "feat: add humanoid mj task adapter"
```

## Chunk 2: Relative Family Through Plain `mj_restore`

### Task 3: Route `relative` And `relative-fullbody` Through The Unified Restore Path

**Files:**
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.test.js`

- [ ] **Step 1: Write the failing integration tests for relative-family routing**

In `src/core/humanoid-jog-manifold-config.test.js`, add tests that verify:
- selecting plain `mj_restore` in `relative` mode no longer publishes `fallback_to_new_qpJ`
- selecting plain `mj_restore` in `relative-fullbody` mode no longer publishes `fallback_to_new_qpJ`
- relative-family `mj_restore` diagnostics use `modeTag: 'relative-target'`
- `supportsHumanoidJogMJRestoreMode('relative-target') === true`
- when adapter/task construction fails, the result is a hard no-commit failure and no QP fallback is invoked
  - `window.humanoidQ` remains byte-for-byte unchanged from `q0`
  - `setHumanoidJoints` is not called with a new committed state
  - diagnostics include `diagnosticLabel: 'mode_adapter_failed'` or `diagnosticLabel: 'command_builder_failed'`
- `mj_restore_pre_w` remains `pre_w: disabled_for_mode` for `relative` / `relative-fullbody`

Also extend:
- `src/core/humanoid-jog-mj-mode-support.test.js` to flip the `relative-target` expectation from `false` to `true`
- `src/core/humanoid-jog-mj-task-adapter.test.js` to assert the runtime descriptor fields needed by the relative-family route

- [ ] **Step 2: Run the relative-family tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-task-adapter.test.js`
Expected: FAIL because `relative` still falls back to `new qpJ`.

- [ ] **Step 3: Implement adapter-driven relative-family routing**

Update `src/visualizer.js` to:
- extend `src/core/humanoid-jog-mj-task-adapter.js` so it can build runtime descriptors for `relative` / `relative-fullbody`
- add a dedicated helper in `src/visualizer.js`, for example `_updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore(...)`, that:
  - captures current absolute pose and target relative pose
  - asks the adapter for the unified task descriptor
  - forwards that descriptor into the existing `mj_restore` execution path
- route `updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(...)` into that new MJ helper instead of `_updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI(...)`
- keep `mj_restore_pre_w` explicitly disabled for these modes in this plan
- on adapter/build failure:
  - publish a hard failure diagnostic
  - do not modify joints
  - do not silently fall back to `new qpJ`

- update `src/core/humanoid-jog-mj-mode-support.js` so `relative-target` only becomes supported at the same time this routing is landed

- [ ] **Step 4: Re-run the relative-family integration tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-task-adapter.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-task-adapter.test.js src/visualizer.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-mode-support.test.js
git commit -m "feat: route humanoid relative modes through mj restore"
```

### Task 4: Add Multi-Invariant Diagnostics For Adapter-Driven Tasks

**Files:**
- Modify: `src/core/humanoid-jog-mj-diagnostics.js`
- Modify: `src/core/humanoid-jog-mj-diagnostics.test.js`
- Modify: `src/core/humanoid-jog-diagnostics.test.js`

- [ ] **Step 1: Write the failing diagnostics tests**

Add tests that verify MJ diagnostics can render:
- `commandKind`
- `invariants[]`
- `active_joints`
- hard failure labels such as `mode_adapter_failed` / `command_builder_failed`

Use a multi-invariant case so the tests fail if diagnostics still assume a single invariant.

- [ ] **Step 2: Run the diagnostics tests to verify they fail**

Run: `node --test src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js`
Expected: FAIL because diagnostics do not yet expose multi-invariant metadata.

- [ ] **Step 3: Implement the diagnostics extension**

Update diagnostics helpers so adapter-driven `mj_restore` payloads can carry and render:
- `commandKind`
- `invariants`
- `diagnosticLabel`
- hard-failure reasons for no-commit exits

Keep existing absolute/distance diagnostics backward-compatible.

- [ ] **Step 4: Re-run the diagnostics tests**

Run: `node --test src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-diagnostics.js src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js
git commit -m "feat: add adapter-driven mj diagnostics metadata"
```

## Chunk 3: Special Single-Arm And Waist Modes

### Task 5: Route `robot1` And `robot2` Through Plain `mj_restore`

**Files:**
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.test.js`

- [ ] **Step 1: Write the failing single-arm integration tests**

Extend `src/core/humanoid-jog-manifold-config.test.js` so it verifies:
- plain `mj_restore` in `robot1` mode does not call the old single-arm pseudo-inverse path
- plain `mj_restore` in `robot2` mode does not call the old single-arm pseudo-inverse path
- `supportsHumanoidJogMJRestoreMode('robot1-target') === true`
- `supportsHumanoidJogMJRestoreMode('robot2-target') === true`
- `robot1` task descriptors hold right-arm and waist joint groups fixed
- `robot2` task descriptors hold left-arm and waist joint groups fixed
- on success, diagnostics identify the correct special-mode adapter task

- [ ] **Step 2: Run the single-arm tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because those modes still use dedicated non-`mj_restore` paths.

- [ ] **Step 3: Implement adapter-driven single-arm routing**

Update `src/visualizer.js` so:
- `robot1` and `robot2` build adapter tasks via `src/core/humanoid-jog-mj-task-adapter.js`
- add or extend explicit MJ helpers for these modes rather than inlining all logic into the public event entrypoint
- they route through plain `mj_restore`
- `src/core/humanoid-jog-mj-mode-support.js` marks the new special single-arm tags as supported only when this routing exists
- existing mode semantics remain:
  - only the selected arm is active
  - non-target arm and waist remain fixed by invariant definition
- old direct pseudo-inverse / LBFGS paths are no longer used when solver mode is `mj_restore`

- [ ] **Step 4: Re-run the single-arm integration tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-mode-support.test.js
git commit -m "feat: route humanoid single-arm modes through mj restore"
```

### Task 6: Route `waist-body-link-xz` Through Plain `mj_restore`

**Files:**
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.test.js`

- [ ] **Step 1: Write the failing waist-mode integration tests**

Extend `src/core/humanoid-jog-manifold-config.test.js` to verify:
- plain `mj_restore` in `waist-body-link-xz` mode does not use the old dedicated waist path
- `supportsHumanoidJogMJRestoreMode('waist-body-link-xz-target') === true`
- only waist joints are active
- arm joint groups are held fixed by invariants
- the command kind is a partial body command on `body_link x/z`
- adapter/build failure causes hard no-commit failure with no silent fallback
  - `window.humanoidQ` remains unchanged from `q0`
  - `setHumanoidJoints` is not called with a committed new state
  - diagnostics include `diagnosticLabel: 'mode_adapter_failed'` or `diagnosticLabel: 'command_builder_failed'`

- [ ] **Step 2: Run the waist-mode tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL because this mode still uses the dedicated waist path.

- [ ] **Step 3: Implement adapter-driven waist-mode routing**

Update `src/visualizer.js` so:
- `waist-body-link-xz` builds a partial-body adapter task via `src/core/humanoid-jog-mj-task-adapter.js`
- add or extend an explicit MJ helper for waist partial-body tasks and route through plain `mj_restore`
- `src/core/humanoid-jog-mj-mode-support.js` marks the waist partial-body target as supported only when this routing exists
- only waist joints are active
- arm joint groups remain fixed

- [ ] **Step 4: Re-run the waist-mode integration tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.test.js`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-mode-support.test.js
git commit -m "feat: route humanoid waist xz mode through mj restore"
```

### Task 7: Verify Semantic Preservation And Remove Silent Fallback Assumptions

**Files:**
- Modify: `src/core/humanoid-jog-manifold-config.test.js`
- Modify: `src/visualizer.js`

- [ ] **Step 1: Write the failing semantic-preservation tests**

Add integration-style tests for:
- `relative`: absolute pose stays unchanged while relative pose command changes
- `relative-fullbody`: absolute pose stays unchanged while allowing waist participation
- `robot1`: right-arm and waist joint groups remain unchanged
- `robot2`: left-arm and waist joint groups remain unchanged
- `waist-body-link-xz`: arm joint groups remain unchanged
- any adapter/builder failure produces hard no-commit failure and a non-fallback diagnostic

- [ ] **Step 2: Run the semantic-preservation tests to verify they fail**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: FAIL until all special-mode routing and failure handling are correct.

- [ ] **Step 3: Implement the minimal fixes required to satisfy semantic preservation**

Adjust `src/visualizer.js` only where tests prove behavior still leaks old semantics or hidden fallback assumptions.

- [ ] **Step 4: Re-run the semantic-preservation tests**

Run: `node --test src/core/humanoid-jog-manifold-config.test.js`
Expected: PASS.

- [ ] **Step 5: Run the focused full regression suite**

Run:

```bash
node --test src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js
```

Expected: PASS.

- [ ] **Step 6: Run syntax/build verification**

Run:

```bash
node --check src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-diagnostics.js src/visualizer.js
npm run build
```

Expected: PASS, with only pre-existing build warnings allowed.

- [ ] **Step 7: Commit**

```bash
git add src/core/humanoid-jog-mj-task-adapter.js src/core/humanoid-jog-mj-task-adapter.test.js src/core/humanoid-jog-mj-mode-support.js src/core/humanoid-jog-mj-mode-support.test.js src/core/humanoid-jog-mj-diagnostics.js src/core/humanoid-jog-mj-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js src/core/humanoid-jog-manifold-config.test.js src/visualizer.js
git commit -m "feat: unify mj restore across humanoid task modes"
```
