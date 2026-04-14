# Surface Slide Primitive Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-stage parameter-surface sliding primitive to `dq_vizlab` by generating a surface-attached reference pose from a user-supplied surface callback and driving the existing humanoid jog restoration pipeline with that reference.

**Architecture:** Stage 1 does not rewrite the restoration solver. Instead, it introduces a reusable parameter-surface geometry helper, a new `surface-slide` task descriptor, and a thin reference-pose adapter that converts `(u,v)` jogging into a single-arm absolute-pose target attached to a surface patch. This keeps the current restoration stack stable while establishing the right abstractions for a later native `surface_slide` commanded/preserved block implementation.

**Tech Stack:** JavaScript, node:test, existing DQ wrapper utilities, humanoid jog manifold/MJ restoration pipeline, browser UI event handlers.

---

## File Map

**Create**
- `src/core/surface-slide-primitive.js`
- `src/core/surface-slide-primitive.test.js`

**Modify**
- `src/core/humanoid-jog-mj-task-adapter.js`
- `src/core/humanoid-jog-mj-task-adapter.test.js`
- `src/core/humanoid-jog-mj-mode-support.js`
- `src/core/humanoid-jog-task-metric-labels.js`
- `src/ui/event-handlers.js`
- `src/ui/mode-switcher.js`
- `src/visualizer.js`

**Optional later-stage files (do not touch in stage 1 unless blocked)**
- `src/core/humanoid-jog-manifold-problem.js`
- `src/core/humanoid-jog-mj-problem.js`

---

## Chunk 1: Geometry Core

### Task 1: Add a reusable parameter-surface geometry helper

**Files:**
- Create: `src/core/surface-slide-primitive.js`
- Test: `src/core/surface-slide-primitive.test.js`

- [ ] **Step 1: Write failing tests for surface evaluation and attached frame construction**

Cover:
- evaluating `position(u,v)`, `du(u,v)`, `dv(u,v)`
- rejecting degenerate tangents
- building a right-handed attached frame
- converting the frame into a reference pose payload the existing jog code can consume

- [ ] **Step 2: Run the new test file to verify it fails**

Run:
```bash
node --test src/core/surface-slide-primitive.test.js
```

Expected:
- FAIL because the module does not exist yet

- [ ] **Step 3: Implement the minimal geometry helper**

Implement:
- `evaluateSurfacePatch(surface, u, v)`
- `buildSurfaceAttachedFrame(surfaceState)`
- `buildSurfaceAttachedPoseTarget({ surface, u, v, normalOffset, tangentAxisMode })`

Keep the first-stage return value simple and solver-facing:
- position in world coordinates
- orientation basis or quaternion
- debug metadata (`u`, `v`, tangents, normal)

- [ ] **Step 4: Run the geometry tests and make them pass**

Run:
```bash
node --test src/core/surface-slide-primitive.test.js
```

Expected:
- PASS

---

## Chunk 2: New Mode Descriptor

### Task 2: Add `surface-slide` mode metadata to the humanoid task adapter

**Files:**
- Modify: `src/core/humanoid-jog-mj-task-adapter.js`
- Modify: `src/core/humanoid-jog-mj-task-adapter.test.js`
- Modify: `src/core/humanoid-jog-mj-mode-support.js`
- Modify: `src/core/humanoid-jog-task-metric-labels.js`

- [ ] **Step 1: Write failing task-adapter tests for the new mode**

Add one canonical descriptor test covering:
- `controlMode: 'surface-slide'`
- `commandKind: 'surface_slide_pose'`
- `activeJointScope: 'left-arm'` for the first-stage single-arm implementation
- invariants describing the non-commanded structure (at minimum preserve the opposite arm and optionally waist fixed)

- [ ] **Step 2: Run the focused adapter tests to verify failure**

Run:
```bash
node --test src/core/humanoid-jog-mj-task-adapter.test.js
```

Expected:
- FAIL on unknown mode

- [ ] **Step 3: Add the mode wiring**

Update:
- `MODE_CONFIGS` in `humanoid-jog-mj-task-adapter.js`
- `supportsHumanoidJogMJRestoreMode`
- human-readable task metric labels if needed for the UI

- [ ] **Step 4: Re-run the mode and adapter tests**

Run:
```bash
node --test src/core/humanoid-jog-mj-task-adapter.test.js
node --test src/core/humanoid-jog-mj-mode-support.test.js
node --test src/core/humanoid-jog-task-metric-labels.test.js
```

Expected:
- PASS

---

## Chunk 3: Reference-Pose Bridge Into the Existing Jog Pipeline

### Task 3: Drive single-arm jog targets from the surface-attached pose helper

**Files:**
- Modify: `src/visualizer.js`
- Test: existing nearby tests if available, otherwise add a focused unit test in `src/core/surface-slide-primitive.test.js`

- [ ] **Step 1: Identify the existing single-arm absolute-pose target path in `visualizer.js`**

Find:
- where robot1/robot2 absolute-pose jog targets are built
- where per-mode command targets are stored or refreshed

- [ ] **Step 2: Add a first-stage surface-slide state object**

Store:
- `surface`
- `u`, `v`
- `normalOffset`
- commanded end-effector side (`left` in stage 1 unless already easy to generalize)

- [ ] **Step 3: Update the target-building path**

When `controlMode === 'surface-slide'`:
- compute the surface-attached pose target from `surface-slide-primitive.js`
- hand that pose target to the existing single-arm absolute-pose jog path

- [ ] **Step 4: Add a regression test or deterministic helper check**

Test that:
- changing `u` or `v` changes the generated pose target consistently
- the reference pose remains finite and correctly oriented on a simple analytic surface

- [ ] **Step 5: Run the relevant focused tests**

Run:
```bash
node --test src/core/surface-slide-primitive.test.js
node --test src/core/humanoid-jog-mj-task-adapter.test.js
```

Expected:
- PASS

---

## Chunk 4: UI and Interaction Wiring

### Task 4: Add a user-facing path to jog the surface parameters

**Files:**
- Modify: `src/ui/mode-switcher.js`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/visualizer.js`

- [ ] **Step 1: Add a selectable `surface-slide` control mode**

Expose the mode in the dual-arm/humanoid mode list using the established UI pattern.

- [ ] **Step 2: Map jogging buttons to `(u,v)` updates**

For stage 1:
- map two translation-axis button pairs to `u+/u-` and `v+/v-`
- keep rotation buttons disabled or no-op for this mode unless a simple tangent-frame roll is already trivial

- [ ] **Step 3: Add a minimal built-in demo surface**

Add one analytic patch for verification, e.g.
\[
z = a u^2 + b v^2
\]
or a sinusoidal patch with closed-form `du`, `dv`.

- [ ] **Step 4: Smoke-test the browser interaction manually**

Run:
```bash
npm run build
```

Then verify in the browser that:
- the mode appears
- button presses update `(u,v)`
- the chosen arm tracks the surface-attached reference

---

## Chunk 5: Verification and Follow-Up

### Task 5: Lock down behavior and document the stage boundary

**Files:**
- Modify: `src/core/surface-slide-primitive.js`
- Modify: `src/core/surface-slide-primitive.test.js`
- Optionally update nearby comments in `src/visualizer.js`

- [ ] **Step 1: Add edge-case tests**

Cover:
- degenerate patch tangents
- invalid callback payloads
- out-of-range `u,v`
- normal-offset handling

- [ ] **Step 2: Run the full focused test set**

Run:
```bash
node --test src/core/surface-slide-primitive.test.js
node --test src/core/humanoid-jog-mj-task-adapter.test.js
node --test src/core/humanoid-jog-mj-mode-support.test.js
node --test src/core/humanoid-jog-task-metric-labels.test.js
```

Expected:
- PASS

- [ ] **Step 3: Note the next-stage upgrade path in code comments**

Document that stage 1:
- bridges surface sliding through a surface-attached pose target

and that stage 2 should:
- add native `surface_slide` commanded/preserved rows in `humanoid-jog-manifold-problem.js`
- split out explicit normal-offset and orientation-preservation residuals

---

Plan complete and saved to `docs/superpowers/plans/2026-04-04-surface-slide-primitive.md`.
