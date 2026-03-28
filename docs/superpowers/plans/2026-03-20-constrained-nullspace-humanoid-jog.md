# Constrained Nullspace Humanoid Jog Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fourth formal humanoid jog solver, `约束零空间`, that improves on the current priority/nullspace behavior while supporting `absolute-target`, `relative-target`, and `distance-target` with waist/body-z options.

**Architecture:** Extract a shared constrained-nullspace solver core into focused JS modules under `src/core`, then wire that solver into the existing humanoid jog selector as a new `cns` mode. Reuse current FK/Jacobian/CCTSM/guard helpers instead of rewriting the humanoid stack from scratch.

**Tech Stack:** Vanilla ES modules, existing humanoid JS solver stack, Node test runner, Vite

---

## File Map

**Create:**

- `docs/superpowers/specs/2026-03-20-constrained-nullspace-humanoid-jog-design.md`
- `docs/superpowers/plans/2026-03-20-constrained-nullspace-humanoid-jog.md`
- `src/core/humanoid-jog-cns-solver.js`
- `src/core/humanoid-jog-cns-solver.test.js`
- `src/core/humanoid-jog-cns-tasks.js`
- `src/core/humanoid-jog-cns-tasks.test.js`
- `src/core/humanoid-jog-cns-feasibility.js`
- `src/core/humanoid-jog-cns-feasibility.test.js`
- `src/core/humanoid-jog-cns-line-search.js`
- `src/core/humanoid-jog-cns-line-search.test.js`

**Modify:**

- `index.html`
- `src/ui/event-handlers.js`
- `src/core/humanoid-jog-solver-presentation.js`
- `src/core/humanoid-jog-solver-presentation.test.js`
- `src/visualizer.js`
- `src/core/humanoid-jog-diagnostics.js`
- `src/core/humanoid-jog-diagnostics.test.js`

**Reference Existing Helpers:**

- `src/core/cctsm.js`
- `src/core/wvfi-guard.js`
- `src/core/humanoid-jog-axis-guard.js`
- `src/core/humanoid-jog-outcome.js`

## Chunk 1: Add The New Formal Solver Mode

### Task 1: Extend the solver selector and presentation layer

**Files:**

- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Test: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Write the failing test**

Add tests that expect:

- `normalizeHumanoidJogSolverMode('cns') === 'cns'`
- `getHumanoidJogSolverDisplayName('cns') === '约束零空间'`

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: FAIL because `cns` is not recognized yet.

- [ ] **Step 3: Implement the minimal presentation update**

Update `src/core/humanoid-jog-solver-presentation.js` so `cns` is normalized and displayed as `约束零空间`.

- [ ] **Step 4: Add the new selector option**

Update:

- `index.html`
- `src/ui/event-handlers.js`

so the humanoid solver selector exposes four options and persists `cns` the same way it already persists `vfi`, `priority`, and `nlopt`.

- [ ] **Step 5: Run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: PASS

## Chunk 2: Build Shared Constrained-Nullspace Core Utilities

### Task 2: Create a shared line-search utility

**Files:**

- Create: `src/core/humanoid-jog-cns-line-search.js`
- Test: `src/core/humanoid-jog-cns-line-search.test.js`

- [ ] **Step 1: Write the failing test**

Add tests for:

- selecting the largest acceptable scale
- returning zero/failed when no scale is acceptable
- retaining the best accepted candidate rather than rolling back to the initial state

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-line-search.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal line-search utility**

Create a helper that accepts:

- candidate step
- evaluation callback
- acceptance callback
- max/min scale and iteration count

and returns the best accepted step plus diagnostics.

- [ ] **Step 4: Run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-cns-line-search.test.js
```

Expected: PASS

### Task 3: Create a shared feasibility utility

**Files:**

- Create: `src/core/humanoid-jog-cns-feasibility.js`
- Test: `src/core/humanoid-jog-cns-feasibility.test.js`

- [ ] **Step 1: Write the failing test**

Add tests for:

- joint-step clipping
- rejecting candidates that violate `w_vfi` floor
- waist corridor acceptance/rejection
- returning a repair-needed status when a candidate can be corrected

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-feasibility.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal feasibility utility**

Create helpers for:

- clipping to effective joint intervals
- evaluating `w_vfi` floor and waist corridor
- computing a small corrective step request shape

- [ ] **Step 4: Run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-cns-feasibility.test.js
```

Expected: PASS

### Task 4: Create a shared task-construction utility

**Files:**

- Create: `src/core/humanoid-jog-cns-tasks.js`
- Test: `src/core/humanoid-jog-cns-tasks.test.js`

- [ ] **Step 1: Write the failing test**

Add tests that confirm task ordering for:

- `absolute-target`
- `relative-target`
- `distance-target`

and that `body_z` is inserted only when enabled and valid.

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal task builder**

Build a pure helper that returns normalized task metadata:

- task names
- errors
- Jacobians
- gains
- protected-task flags

Do not publish diagnostics here.

- [ ] **Step 4: Run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js
```

Expected: PASS

## Chunk 3: Implement The Shared Constrained Nullspace Solver

### Task 5: Create the shared outer solver loop

**Files:**

- Create: `src/core/humanoid-jog-cns-solver.js`
- Test: `src/core/humanoid-jog-cns-solver.test.js`

- [ ] **Step 1: Write the failing test**

Add integration-style tests with stubs for:

- accepting a reduced but valid step
- keeping the last accepted state instead of rolling back to `qInitial`
- stopping with a useful reason when no feasible step exists
- surfacing secondary-objective activity in diagnostics

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-solver.test.js
```

Expected: FAIL because the solver module does not exist yet.

- [ ] **Step 3: Implement the minimal shared solver**

The solver should:

- iterate with re-linearization
- request task metadata from `humanoid-jog-cns-tasks.js`
- generate a task-priority candidate step
- call `humanoid-jog-cns-feasibility.js`
- call `humanoid-jog-cns-line-search.js`
- retain the best accepted configuration
- return a structured result for diagnostics

- [ ] **Step 4: Run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-cns-solver.test.js
```

Expected: PASS

## Chunk 4: Integrate The New Solver Into Humanoid Absolute Mode

### Task 6: Add a `cns` branch for humanoid absolute-target

**Files:**

- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-diagnostics.js`
- Test: `src/core/humanoid-jog-diagnostics.test.js`
- Test: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Write the failing integration test**

Add or extend tests to expect:

- diagnostics can label `solverMode: 'cns'`
- `absolute-target` dispatch selects the new solver when mode is `cns`

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
node --test src/core/humanoid-jog-diagnostics.test.js src/core/humanoid-jog-solver-presentation.test.js
```

Expected: FAIL because `cns` is not wired through dispatch/diagnostics yet.

- [ ] **Step 3: Implement the absolute-path integration**

Update `src/visualizer.js` so:

- mode `cns` dispatches to the new shared solver
- humanoid absolute inputs are assembled once
- result fields are published through the existing diagnostics path

- [ ] **Step 4: Run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-diagnostics.test.js src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-cns-solver.test.js
```

Expected: PASS

## Chunk 5: Integrate Relative And Distance Modes

### Task 7: Wire `relative-target` to the new solver

**Files:**

- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-diagnostics.js`
- Test: `src/core/humanoid-jog-cns-tasks.test.js`
- Test: `src/core/humanoid-jog-cns-solver.test.js`

- [ ] **Step 1: Write the failing test**

Add tests that verify:

- `relative-target` builds the correct task order
- solver mode `cns` dispatches through the shared solver for relative jogs

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js src/core/humanoid-jog-cns-solver.test.js
```

Expected: FAIL because relative mode is not yet integrated.

- [ ] **Step 3: Implement the minimal relative integration**

Update the relative dispatch path in `src/visualizer.js` to call the shared solver with relative-mode task construction.

- [ ] **Step 4: Run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js src/core/humanoid-jog-cns-solver.test.js
```

Expected: PASS

### Task 8: Wire `distance-target` to the new solver

**Files:**

- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-diagnostics.js`
- Test: `src/core/humanoid-jog-cns-tasks.test.js`
- Test: `src/core/humanoid-jog-cns-solver.test.js`

- [ ] **Step 1: Write the failing test**

Add tests that verify:

- `distance-target` preserves absolute pose and relative rotation in its protected task set
- solver mode `cns` dispatches through the shared solver for distance jogs

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js src/core/humanoid-jog-cns-solver.test.js
```

Expected: FAIL because distance mode is not yet integrated.

- [ ] **Step 3: Implement the minimal distance integration**

Update the distance dispatch path in `src/visualizer.js` to use the shared solver and publish diagnostics through the existing path.

- [ ] **Step 4: Run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-cns-tasks.test.js src/core/humanoid-jog-cns-solver.test.js
```

Expected: PASS

## Chunk 6: Final Verification

### Task 9: Run syntax, unit, and build verification

**Files:**

- Verify: `src/core/humanoid-jog-cns-*.js`
- Verify: `src/visualizer.js`
- Verify: `src/core/humanoid-jog-diagnostics.js`

- [ ] **Step 1: Run syntax checks**

Run:

```bash
node --check src/core/humanoid-jog-cns-solver.js
node --check src/core/humanoid-jog-cns-tasks.js
node --check src/core/humanoid-jog-cns-feasibility.js
node --check src/core/humanoid-jog-cns-line-search.js
node --check src/visualizer.js
```

Expected: all exit code 0

- [ ] **Step 2: Run the focused test suite**

Run:

```bash
node --test \
  src/core/humanoid-jog-solver-presentation.test.js \
  src/core/humanoid-jog-diagnostics.test.js \
  src/core/humanoid-jog-cns-line-search.test.js \
  src/core/humanoid-jog-cns-feasibility.test.js \
  src/core/humanoid-jog-cns-tasks.test.js \
  src/core/humanoid-jog-cns-solver.test.js
```

Expected: PASS

- [ ] **Step 3: Run the production build**

Run:

```bash
npm run build
```

Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add index.html src/ui/event-handlers.js src/core/humanoid-jog-solver-presentation.js src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-diagnostics.js src/core/humanoid-jog-diagnostics.test.js src/core/humanoid-jog-cns-*.js src/visualizer.js
git commit -m "feat: add constrained nullspace humanoid jog solver"
```
