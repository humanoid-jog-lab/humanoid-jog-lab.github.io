# MuJoCo-Style Restore Humanoid Jog Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new humanoid jog solver `mj_restore` that uses MuJoCo-inspired reference restoration for `absolute-target` and reuses existing protected nullspace metric refinement.

**Architecture:** The solver will be introduced as a separate humanoid jog backend. It will build a local equality/inequality problem from cooperative absolute/relative/body-z constraints, run a reference-restoration loop using `r_ref = K c - B y_prev`, then optionally reuse the existing nullspace metric family once restoration converges. UI and diagnostics will be integrated without altering current solver semantics.

**Tech Stack:** JavaScript, existing `RobotVisualizer` humanoid jog pipeline, DQ Robotics cooperative dual-arm WASM APIs, Node test runner, Vite build.

---

## File Map

### New files

- `src/core/humanoid-jog-mj-problem.js`
  - Build the `mj_restore` problem object for `absolute-target`
  - Reuse existing cooperative absolute/relative/body-z values and Jacobians
  - Package bounds, `w_vfi` floor, waist corridor metadata

- `src/core/humanoid-jog-mj-restoration.js`
  - Implement the MuJoCo-style reference restoration loop
  - Own reference residual computation, damped local solve, line search, and acceptance bookkeeping

- `src/core/humanoid-jog-mj-nullspace.js`
  - Thin adapter that reuses the current manifold nullspace metric family after restoration convergence

- `src/core/humanoid-jog-mj-diagnostics.js`
  - Build solver-specific diagnostics and UI labels

- `src/core/humanoid-jog-mj-problem.test.js`
- `src/core/humanoid-jog-mj-restoration.test.js`
- `src/core/humanoid-jog-mj-nullspace.test.js`
- `src/core/humanoid-jog-mj-diagnostics.test.js`

### Existing files to modify

- `src/visualizer.js`
  - Add solver mode config/defaults
  - Add `mj_restore` branch in humanoid `absolute-target`
  - Wire diagnostics and UI-facing status updates

- `src/core/humanoid-jog-solver-presentation.js`
  - Add new solver presentation label

- `src/ui/event-handlers.js`
  - Ensure selection persistence and UI wiring

- `index.html`
  - Add the solver option to the humanoid jog selector

---

## Chunk 1: Problem Builder

### Task 1: Create failing tests for `mj` problem assembly

**Files:**
- Create: `src/core/humanoid-jog-mj-problem.test.js`
- Reference: `src/core/humanoid-jog-manifold-problem.test.js`

- [ ] **Step 1: Write a failing test for stacked absolute/relative/body-z residual assembly**

Cover:
- absolute residual rows present
- relative residual rows present
- optional body-z row present
- bounds and inequality metadata preserved

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
node --test src/core/humanoid-jog-mj-problem.test.js
```

Expected: FAIL because module does not exist.

- [ ] **Step 3: Create `src/core/humanoid-jog-mj-problem.js` with minimal builder implementation**

Implement:
- residual and Jacobian block assembly
- body-z optional block
- bounds packaging
- `w_vfi` floor metadata
- waist corridor metadata

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
node --test src/core/humanoid-jog-mj-problem.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-problem.js src/core/humanoid-jog-mj-problem.test.js
git commit -m "feat: add mj restore problem builder"
```

---

## Chunk 2: Reference Restoration Core

### Task 2: Create failing tests for reference restoration behavior

**Files:**
- Create: `src/core/humanoid-jog-mj-restoration.test.js`
- Create: `src/core/humanoid-jog-mj-restoration.js`

- [ ] **Step 1: Write failing tests for reference restoration**

Cover:
- converges for a simple linear equality problem
- reference damping reduces step magnitude
- reference damping cannot reverse restoration direction
- respects step bounds
- respects `w_vfi` floor rejection/shrink
- respects waist corridor shrink

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
node --test src/core/humanoid-jog-mj-restoration.test.js
```

Expected: FAIL because module does not exist.

- [ ] **Step 3: Implement minimal reference restoration loop**

Implement:
- `r_ref = K c - B y_prev`
- damped local least-squares solve
- clipped damping term so direction cannot invert
- line search
- inequality and bounds screening
- convergence / best_effort / max_iter reporting

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
node --test src/core/humanoid-jog-mj-restoration.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-restoration.js src/core/humanoid-jog-mj-restoration.test.js
git commit -m "feat: add mj restore reference restoration loop"
```

---

## Chunk 3: Nullspace Reuse Adapter

### Task 3: Reuse existing manifold nullspace metric family behind `mj_restore`

**Files:**
- Create: `src/core/humanoid-jog-mj-nullspace.js`
- Create: `src/core/humanoid-jog-mj-nullspace.test.js`
- Reference: `src/core/humanoid-jog-manifold-nullspace.js`

- [ ] **Step 1: Write failing tests for `mj` nullspace adapter**

Cover:
- accepts same metric family names as manifold
- returns skipped when restoration not converged
- passes through accepted/rejected metric refinement results

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
node --test src/core/humanoid-jog-mj-nullspace.test.js
```

Expected: FAIL because module does not exist.

- [ ] **Step 3: Implement minimal adapter**

Reuse:
- metric resolution
- existing nullspace helper hooks already used by manifold

Do not duplicate metric math.

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
node --test src/core/humanoid-jog-mj-nullspace.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-nullspace.js src/core/humanoid-jog-mj-nullspace.test.js
git commit -m "feat: add mj restore nullspace adapter"
```

---

## Chunk 4: Diagnostics

### Task 4: Add solver-specific diagnostics

**Files:**
- Create: `src/core/humanoid-jog-mj-diagnostics.js`
- Create: `src/core/humanoid-jog-mj-diagnostics.test.js`

- [ ] **Step 1: Write failing tests for diagnostics labels**

Cover:
- `restore=...`
- `nullspace=...`
- `mj_ref: abs(...), rel(...), body(...)`
- `rel/body` residual fields
- selected jog metric state

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
node --test src/core/humanoid-jog-mj-diagnostics.test.js
```

Expected: FAIL because module does not exist.

- [ ] **Step 3: Implement diagnostics builder**

Include:
- solver mode metadata
- restoration iterations and stop reason
- reference stiffness/damping fields
- nullspace state
- metric state

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
node --test src/core/humanoid-jog-mj-diagnostics.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-mj-diagnostics.js src/core/humanoid-jog-mj-diagnostics.test.js
git commit -m "feat: add mj restore diagnostics"
```

---

## Chunk 5: Visualizer Integration

### Task 5: Add solver config defaults and selection plumbing

**Files:**
- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Modify: `src/ui/event-handlers.js`
- Modify: `index.html`

- [ ] **Step 1: Write failing tests for solver presentation and config defaults**

Add/extend tests to cover:
- `mj_restore` presentation label
- visualizer config defaults for:
  - absolute stiffness/damping
  - relative stiffness/damping
  - body stiffness/damping

- [ ] **Step 2: Run tests to verify failure**

Run:
```bash
node --test src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-manifold-config.test.js
```

Expected: FAIL because `mj_restore` is not wired yet.

- [ ] **Step 3: Implement UI plumbing**

Add:
- solver option in selector
- persistence / event handler wiring
- presentation labels
- config defaults in `RobotVisualizer`

- [ ] **Step 4: Run tests to verify pass**

Run:
```bash
node --test src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-manifold-config.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-solver-presentation.js src/ui/event-handlers.js index.html
git commit -m "feat: wire mj restore solver mode"
```

---

## Chunk 6: Absolute-Target Solver Path

### Task 6: Add `mj_restore` execution path for humanoid absolute-target

**Files:**
- Modify: `src/visualizer.js`

- [ ] **Step 1: Write failing integration-oriented unit tests**

Extend/create tests that verify:
- `mj_restore` builds problem and runs restoration
- `waist=off` works
- `waist=on` works
- `body_z=on` includes body block
- non-converged restoration skips nullspace
- converged restoration can run nullspace refine

- [ ] **Step 2: Run tests to verify failure**

Run the relevant node tests for the new `mj` modules plus any updated visualizer-facing unit tests.

- [ ] **Step 3: Implement minimal solver branch**

In `absolute-target` path:
- build mj problem
- run restoration
- if converged, optionally run reused nullspace metric refinement
- publish diagnostics
- apply result only on hard acceptance

- [ ] **Step 4: Run tests to verify pass**

Run:
```bash
node --test \
  src/core/humanoid-jog-mj-problem.test.js \
  src/core/humanoid-jog-mj-restoration.test.js \
  src/core/humanoid-jog-mj-nullspace.test.js \
  src/core/humanoid-jog-mj-diagnostics.test.js \
  src/core/humanoid-jog-solver-presentation.test.js
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-mj-*.js src/core/humanoid-jog-mj-*.test.js
git commit -m "feat: add mj restore humanoid absolute solver"
```

---

## Chunk 7: Full Verification

### Task 7: Run complete focused verification

**Files:**
- Modify only if failures require fixes

- [ ] **Step 1: Run focused test suite**

```bash
node --test \
  src/core/humanoid-jog-mj-problem.test.js \
  src/core/humanoid-jog-mj-restoration.test.js \
  src/core/humanoid-jog-mj-nullspace.test.js \
  src/core/humanoid-jog-mj-diagnostics.test.js \
  src/core/humanoid-jog-manifold-restoration.test.js \
  src/core/humanoid-jog-manifold-problem.test.js \
  src/core/humanoid-jog-manifold-nullspace.test.js \
  src/core/humanoid-jog-manifold-diagnostics.test.js \
  src/core/humanoid-jog-directional-manipulability.test.js \
  src/core/humanoid-jog-solver-presentation.test.js
```

Expected: all pass.

- [ ] **Step 2: Run syntax check**

```bash
node --check src/visualizer.js
```

Expected: exit 0.

- [ ] **Step 3: Run production build**

```bash
npm run build
```

Expected: exit 0. Existing `numeric` eval warning and chunk-size warning are acceptable.

- [ ] **Step 4: Commit final verification state**

```bash
git add .
git commit -m "test: verify mj restore humanoid jog integration"
```

