# Manifold Humanoid Jog Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new humanoid jog solver `约束流形`/`manifold` that solves `absolute-target` through terminal equality manifold restoration plus nullspace `w_vfi` optimization.

**Architecture:** Implement a browser-side local manifold solver in `src/core` instead of extending `new sqpJ`. Phase A restores the terminal equality manifold for absolute/relative/body_z under hard bounds and `w_vfi` floor. Phase B moves only in the manifold tangent/nullspace to improve `w_vfi` without violating the terminal equalities.

**Tech Stack:** Vanilla ES modules, existing humanoid FK/Jacobian access in `src/visualizer.js`, existing CCTSM / `w_vfi` utilities, Node test runner, Vite

---

## File Map

**Create:**

- `docs/superpowers/specs/2026-03-22-manifold-humanoid-jog-design.md`
- `docs/superpowers/plans/2026-03-22-manifold-humanoid-jog.md`
- `src/core/humanoid-jog-manifold-problem.js`
- `src/core/humanoid-jog-manifold-problem.test.js`
- `src/core/humanoid-jog-manifold-restoration.js`
- `src/core/humanoid-jog-manifold-restoration.test.js`
- `src/core/humanoid-jog-manifold-nullspace.js`
- `src/core/humanoid-jog-manifold-nullspace.test.js`
- `src/core/humanoid-jog-manifold-diagnostics.js`
- `src/core/humanoid-jog-manifold-diagnostics.test.js`

**Modify:**

- `index.html`
- `src/ui/event-handlers.js`
- `src/core/humanoid-jog-solver-presentation.js`
- `src/core/humanoid-jog-solver-presentation.test.js`
- `src/core/humanoid-jog-diagnostics.js`
- `src/core/humanoid-jog-diagnostics.test.js`
- `src/visualizer.js`

**Reference Existing Helpers:**

- `src/core/cctsm.js`
- `src/core/wvfi-guard.js`
- `src/core/humanoid-jog-outcome.js`
- `src/core/humanoid-jog-priority-kernel.js`
- `src/core/body-vertical-utils.js`

## Chunk 1: Add The New Solver Mode

### Task 1: Extend the humanoid solver selector and presentation layer

**Files:**
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Test: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Write the failing test**

Add assertions for:

- `normalizeHumanoidJogSolverMode('manifold') === 'manifold'`
- `getHumanoidJogSolverDisplayName('manifold') === '约束流形'`
- `getHumanoidJogSolverReadyMessage('manifold') === '约束流形求解器已启用，等待点动输入。'`

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: FAIL because `manifold` is not recognized yet.

- [ ] **Step 3: Implement the selector update**

Update:

- `src/core/humanoid-jog-solver-presentation.js`
- `index.html`
- `src/ui/event-handlers.js`

so the new solver mode is selectable and persisted like the existing humanoid solver modes.

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: PASS

## Chunk 2: Build The Equality-Manifold Problem Layer

### Task 2: Create the manifold problem evaluator

**Files:**
- Create: `src/core/humanoid-jog-manifold-problem.js`
- Test: `src/core/humanoid-jog-manifold-problem.test.js`

- [ ] **Step 1: Write the failing test**

Cover:

- absolute equality residual block
- relative equality residual block
- optional `body_z` equality block
- stacked equality Jacobian
- `w_vfi` value, gradient, and floor slack
- step/joint bound metadata passthrough

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-manifold-problem.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal problem builder**

Create a pure helper that accepts callbacks/data for:

- `absolutePose(q)`
- `relativePose(q)`
- `absoluteJacobian(q)`
- `relativeJacobian(q)`
- optional `body_z`
- optional `w_vfi`

and returns:

- `equalities.blocks.absolute`
- `equalities.blocks.relative`
- `equalities.blocks.bodyZ`
- `equalities.residual`
- `equalities.jacobian`
- `inequalities.wvfiFloor`
- bound metadata

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-problem.test.js
```

Expected: PASS

## Chunk 3: Implement Phase A Feasibility Restoration

### Task 3: Create the manifold restoration loop

**Files:**
- Create: `src/core/humanoid-jog-manifold-restoration.js`
- Test: `src/core/humanoid-jog-manifold-restoration.test.js`

- [ ] **Step 1: Write the failing test**

Cover:

- restoration converges a simple equality problem
- accepted trial must reduce equality residual norm
- accepted trial must not violate `w_vfi` floor tolerance
- accepted trial respects step bounds
- restoration reports `converged`, `best_effort`, `line_search_failed`, and `max_iter`

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-manifold-restoration.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the restoration loop**

Implement:

- damped restoration step
  - `dq = -J_c^T (J_c J_c^T + λI)^-1 c`
- bound projection / clipping
- backtracking line search on equality residual norm
- hard rejection when `w_vfi` floor is violated

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-restoration.test.js
```

Expected: PASS

## Chunk 4: Implement Phase B Nullspace `w_vfi` Optimization

### Task 4: Create the manifold nullspace step helper

**Files:**
- Create: `src/core/humanoid-jog-manifold-nullspace.js`
- Test: `src/core/humanoid-jog-manifold-nullspace.test.js`

- [ ] **Step 1: Write the failing test**

Cover:

- projected nullspace step leaves equality Jacobian residual unchanged to first order
- projected step follows `w_vfi` gradient when nullspace exists
- projected step vanishes when there is no nullspace
- `w_vfi` floor violation rejects the nullspace step

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-manifold-nullspace.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the nullspace helper**

Implement:

- nullspace basis / projector from stacked equality Jacobian
- projected `w_vfi` ascent step
- optional step clipping and `w_vfi` floor rejection

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-nullspace.test.js
```

Expected: PASS

## Chunk 5: Add Diagnostics

### Task 5: Create manifold diagnostics payloads

**Files:**
- Create: `src/core/humanoid-jog-manifold-diagnostics.js`
- Test: `src/core/humanoid-jog-manifold-diagnostics.test.js`
- Modify: `src/core/humanoid-jog-diagnostics.js`
- Modify: `src/core/humanoid-jog-diagnostics.test.js`

- [ ] **Step 1: Write the failing tests**

Cover:

- public solver name `约束流形`
- restoration iteration count and residual
- nullspace iteration count
- `w_vfi` current value
- stop reason propagation

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js
```

Expected: FAIL because `manifold` diagnostics are not wired yet.

- [ ] **Step 3: Implement the diagnostics helper**

Expose:

- solver mode
- mode tag
- restoration stats
- nullspace stats
- equality residual norms
- `w_vfi`
- stop reason

- [ ] **Step 4: Re-run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js src/core/humanoid-jog-diagnostics.test.js
```

Expected: PASS

## Chunk 6: Integrate Into `visualizer.js`

### Task 6: Add the new absolute-target manifold solver path

**Files:**
- Modify: `src/visualizer.js`
- Reference: `src/core/humanoid-jog-outcome.js`
- Reference: `src/core/wvfi-guard.js`

- [ ] **Step 1: Write the failing integration-facing tests**

Add/update focused tests so they cover:

- solver mode `manifold` routing
- fallback message for unsupported modes
- strict “do not execute unless terminal residuals <= 1e-12” semantics

- [ ] **Step 2: Run focused tests to verify they fail**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-diagnostics.test.js
```

Expected: FAIL because `visualizer.js` does not yet route `manifold`.

- [ ] **Step 3: Implement the absolute-target manifold path**

In `src/visualizer.js`:

- add `_isHumanoidManifoldSolverEnabled()`
- route `absolute-target` to a new `_updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(...)`
- in that path:
  - build the manifold problem
  - run restoration
  - if restoration converges, optionally run nullspace `w_vfi` improvement
  - compute final residuals
  - refuse to apply unless absolute/relative/body residuals are all `<= 1e-12`
- for `relative-target` / `distance-target`, publish a clear fallback message to the existing solver

- [ ] **Step 4: Re-run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js src/core/humanoid-jog-diagnostics.test.js
```

Expected: PASS

## Chunk 7: Full Verification

### Task 7: Run the manifold-related verification set

**Files:**
- Verify: `src/core/humanoid-jog-manifold-*.js`
- Verify: `src/visualizer.js`

- [ ] **Step 1: Run focused solver tests**

Run:

```bash
node --test \
  src/core/humanoid-jog-manifold-problem.test.js \
  src/core/humanoid-jog-manifold-restoration.test.js \
  src/core/humanoid-jog-manifold-nullspace.test.js \
  src/core/humanoid-jog-manifold-diagnostics.test.js \
  src/core/humanoid-jog-diagnostics.test.js \
  src/core/humanoid-jog-solver-presentation.test.js
```

Expected: PASS

- [ ] **Step 2: Run syntax checks**

Run:

```bash
node --check src/visualizer.js
node --check src/core/humanoid-jog-manifold-problem.js
node --check src/core/humanoid-jog-manifold-restoration.js
node --check src/core/humanoid-jog-manifold-nullspace.js
node --check src/core/humanoid-jog-manifold-diagnostics.js
```

Expected: PASS

- [ ] **Step 3: Run production build**

Run:

```bash
npm run build
```

Expected: PASS with only existing dependency warnings.

- [ ] **Step 4: Commit**

```bash
git add \
  docs/superpowers/specs/2026-03-22-manifold-humanoid-jog-design.md \
  docs/superpowers/plans/2026-03-22-manifold-humanoid-jog.md \
  index.html \
  src/ui/event-handlers.js \
  src/core/humanoid-jog-solver-presentation.js \
  src/core/humanoid-jog-solver-presentation.test.js \
  src/core/humanoid-jog-diagnostics.js \
  src/core/humanoid-jog-diagnostics.test.js \
  src/core/humanoid-jog-manifold-problem.js \
  src/core/humanoid-jog-manifold-problem.test.js \
  src/core/humanoid-jog-manifold-restoration.js \
  src/core/humanoid-jog-manifold-restoration.test.js \
  src/core/humanoid-jog-manifold-nullspace.js \
  src/core/humanoid-jog-manifold-nullspace.test.js \
  src/core/humanoid-jog-manifold-diagnostics.js \
  src/core/humanoid-jog-manifold-diagnostics.test.js \
  src/visualizer.js
git commit -m "feat: add humanoid manifold jog solver"
```
