# SQP Humanoid Jog Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new humanoid jog solver `new sqpJ` that solves `absolute-target` with terminal hard equalities for absolute and relative pose, optional hard `body_z`, and explicit `w_vfi` optimization.

**Architecture:** Implement a browser-side single-state SQP solver under `src/core` that reuses existing humanoid FK, Jacobian, CCTSM, and QP utilities. The first version only supports `absolute-target`; it plugs into the existing humanoid jog solver selector and diagnostics without replacing `new qpJ`.

**Tech Stack:** Vanilla ES modules, existing humanoid JS solver stack, Node test runner, Vite, existing numeric/QP helpers

---

## File Map

**Create:**

- `docs/superpowers/specs/2026-03-21-sqp-humanoid-jog-design.md`
- `docs/superpowers/plans/2026-03-21-sqp-humanoid-jog.md`
- `src/core/humanoid-jog-sqp-problem.js`
- `src/core/humanoid-jog-sqp-problem.test.js`
- `src/core/humanoid-jog-sqp-subproblem.js`
- `src/core/humanoid-jog-sqp-subproblem.test.js`
- `src/core/humanoid-jog-sqp-merit.js`
- `src/core/humanoid-jog-sqp-merit.test.js`
- `src/core/humanoid-jog-sqp-solver.js`
- `src/core/humanoid-jog-sqp-solver.test.js`
- `src/core/humanoid-jog-sqp-diagnostics.js`
- `src/core/humanoid-jog-sqp-diagnostics.test.js`

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
- `src/core/vfi-utils.js`
- `src/core/body-vertical-utils.js`
- `src/core/humanoid-jog-outcome.js`

## Chunk 1: Add The New Formal Solver Mode

### Task 1: Extend the humanoid solver selector and presentation layer

**Files:**

- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Test: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Write the failing test**

Add assertions for:

- `normalizeHumanoidJogSolverMode('sqp') === 'sqp'`
- `getHumanoidJogSolverDisplayName('sqp') === 'new sqpJ'`
- `getHumanoidJogSolverReadyMessage('sqp') === 'new sqpJ 已启用，等待点动输入。'`

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: FAIL because `sqp` is not recognized yet.

- [ ] **Step 3: Implement the presentation update**

Update `src/core/humanoid-jog-solver-presentation.js` so `sqp` is normalized and displayed as `new sqpJ`.

- [ ] **Step 4: Add the new selector option**

Update:

- `index.html`
- `src/ui/event-handlers.js`

so the humanoid solver selector exposes `sqp` and persists it the same way it persists the existing humanoid solver modes.

- [ ] **Step 5: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected: PASS

## Chunk 2: Build The Nonlinear SQP Problem Layer

### Task 2: Create the SQP problem evaluator

**Files:**

- Create: `src/core/humanoid-jog-sqp-problem.js`
- Test: `src/core/humanoid-jog-sqp-problem.test.js`

- [ ] **Step 1: Write the failing test**

Add tests for:

- building absolute/relative terminal equality values at a current `q`
- including `body_z` residual only when enabled
- computing `w_vfi` and floor slack
- exposing the equality Jacobian block shape expected by the subproblem builder

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-sqp-problem.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal problem evaluator**

Create a pure helper that accepts already-available callbacks/data for:

- `absolutePose(q)`
- `relativePose(q)`
- `absoluteJacobian(q)`
- `relativeJacobian(q)`
- optional `body_z`
- optional `w_vfi`

and returns:

- current objective terms
- equality residuals
- inequality residuals/slacks
- Jacobian blocks

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-sqp-problem.test.js
```

Expected: PASS

### Task 3: Create the SQP merit-function helper

**Files:**

- Create: `src/core/humanoid-jog-sqp-merit.js`
- Test: `src/core/humanoid-jog-sqp-merit.test.js`

- [ ] **Step 1: Write the failing test**

Add tests for:

- merit increasing when terminal equality residual grows
- merit penalizing `w_vfi` floor violation
- merit decreasing when joint motion stays small and terminal errors shrink

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-sqp-merit.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal merit helper**

Implement a helper that computes:

- base objective
- absolute-equality penalty
- relative-equality penalty
- optional `body_z` penalty
- `w_vfi` floor violation penalty

and returns both total merit and a structured component breakdown.

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-sqp-merit.test.js
```

Expected: PASS

## Chunk 3: Build The QP Subproblem Layer

### Task 4: Create the SQP QP-subproblem assembler

**Files:**

- Create: `src/core/humanoid-jog-sqp-subproblem.js`
- Test: `src/core/humanoid-jog-sqp-subproblem.test.js`

- [ ] **Step 1: Write the failing test**

Add tests for:

- assembling equality rows for absolute and relative terminal constraints
- adding `body_z` equality rows when enabled
- adding step bounds and joint bounds correctly
- assembling a linearized `w_vfi` lower-bound inequality
- producing a positive-definite regularized Hessian

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-sqp-subproblem.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal subproblem builder**

Build a helper that returns a normalized QP description:

- `H`
- `g`
- `Aeq`
- `beq`
- `Aineq`
- `bineq`
- step/joint lower and upper bounds

Keep this layer pure and solver-agnostic.

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-sqp-subproblem.test.js
```

Expected: PASS

## Chunk 4: Implement The Outer SQP Solver

### Task 5: Create the shared SQP solver loop

**Files:**

- Create: `src/core/humanoid-jog-sqp-solver.js`
- Test: `src/core/humanoid-jog-sqp-solver.test.js`

- [ ] **Step 1: Write the failing test**

Add tests with stubbed problem and subproblem callbacks for:

- accepting a step that reduces merit
- rejecting a step that increases merit
- converging when terminal equality residuals reach tolerance
- stopping with a useful reason when the QP subproblem fails
- preserving the last accepted iterate rather than rolling back blindly

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-sqp-solver.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal outer solver**

Implement the SQP loop:

- evaluate current nonlinear problem
- assemble QP subproblem
- solve one QP using existing helpers from `src/core/vfi-utils.js`
- perform line search using the merit function
- accept or reject the trial iterate
- stop on convergence, line-search failure, or iteration limit

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-sqp-solver.test.js
```

Expected: PASS

### Task 6: Create SQP diagnostics normalization

**Files:**

- Create: `src/core/humanoid-jog-sqp-diagnostics.js`
- Test: `src/core/humanoid-jog-sqp-diagnostics.test.js`

- [ ] **Step 1: Write the failing test**

Add tests that verify diagnostics payloads expose:

- solver name
- SQP iteration count
- terminal absolute residual
- terminal relative residual
- optional `body_z` residual
- `w_vfi`
- line-search scale
- stop reason

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-sqp-diagnostics.test.js
```

Expected: FAIL because the module does not exist yet.

- [ ] **Step 3: Implement the minimal diagnostics adapter**

Create a helper that converts raw SQP solver state into the existing humanoid diagnostics payload shape.

- [ ] **Step 4: Re-run the focused test**

Run:

```bash
node --test src/core/humanoid-jog-sqp-diagnostics.test.js
```

Expected: PASS

## Chunk 5: Wire `new sqpJ` Into `absolute-target`

### Task 7: Integrate the SQP solver into the humanoid absolute-target path

**Files:**

- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-diagnostics.js`
- Test: `src/core/humanoid-jog-diagnostics.test.js`

- [ ] **Step 1: Write the failing integration-style test**

Add tests that verify:

- the diagnostics layer accepts `solverMode: 'sqp'`
- unsupported humanoid jog modes do not route to SQP yet
- `absolute-target` can publish `sqp` diagnostics without throwing

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
node --test src/core/humanoid-jog-diagnostics.test.js
```

Expected: FAIL because `sqp` is not wired yet.

- [ ] **Step 3: Add solver-mode branching in the visualizer**

Update `src/visualizer.js` so:

- `sqp` is recognized as a humanoid solver mode
- `absolute-target` routes to the new SQP solver
- `relative-target` and `distance-target` explicitly fall back to the existing solver for now, with a clear diagnostics message

- [ ] **Step 4: Build the absolute-target SQP problem from existing visualizer callbacks**

Use existing helpers in `src/visualizer.js` to provide:

- absolute pose value/Jacobian
- relative pose value/Jacobian
- optional `body_z` residual/Jacobian
- `w_vfi`
- joint bounds and step bounds

and feed them into the new SQP problem/subproblem/solver modules.

- [ ] **Step 5: Re-run the focused diagnostics test**

Run:

```bash
node --test src/core/humanoid-jog-diagnostics.test.js
```

Expected: PASS

## Chunk 6: Verify On Real Humanoid Regression Cases

### Task 8: Add regression checks for known humanoid absolute-target cases

**Files:**

- Modify: `src/core/humanoid-jog-sqp-solver.test.js`
- Modify: `src/core/humanoid-jog-sqp-diagnostics.test.js`

- [ ] **Step 1: Write regression-oriented tests**

Add tests that encode the expected solver semantics:

- terminal absolute and relative equality residuals must both be driven below tolerance
- `w_vfi` floor must not be violated
- `body_z` must remain within tolerance when enabled
- successful result should keep the accepted iterate instead of reverting to `qInitial`

- [ ] **Step 2: Run the regression test subset**

Run:

```bash
node --test src/core/humanoid-jog-sqp-solver.test.js src/core/humanoid-jog-sqp-diagnostics.test.js
```

Expected: PASS

### Task 9: Run full verification

**Files:**

- Verify: `src/core/humanoid-jog-solver-presentation.test.js`
- Verify: `src/core/humanoid-jog-sqp-problem.test.js`
- Verify: `src/core/humanoid-jog-sqp-merit.test.js`
- Verify: `src/core/humanoid-jog-sqp-subproblem.test.js`
- Verify: `src/core/humanoid-jog-sqp-solver.test.js`
- Verify: `src/core/humanoid-jog-sqp-diagnostics.test.js`
- Verify: `src/core/humanoid-jog-diagnostics.test.js`
- Verify: `src/visualizer.js`

- [ ] **Step 1: Run all focused unit tests**

Run:

```bash
node --test \
  src/core/humanoid-jog-solver-presentation.test.js \
  src/core/humanoid-jog-sqp-problem.test.js \
  src/core/humanoid-jog-sqp-merit.test.js \
  src/core/humanoid-jog-sqp-subproblem.test.js \
  src/core/humanoid-jog-sqp-solver.test.js \
  src/core/humanoid-jog-sqp-diagnostics.test.js \
  src/core/humanoid-jog-diagnostics.test.js
```

Expected: PASS

- [ ] **Step 2: Run syntax verification**

Run:

```bash
node --check src/visualizer.js
```

Expected: PASS

- [ ] **Step 3: Run production build**

Run:

```bash
npm run build
```

Expected: PASS with only existing third-party warnings, if any.

- [ ] **Step 4: Manual browser verification**

In the browser:

- switch humanoid solver to `new sqpJ`
- verify `absolute-target` works in:
  - `waist=off, body_z=off`
  - `waist=on, body_z=off`
  - `waist=on, body_z=on`
- confirm diagnostics show:
  - solver name `new sqpJ`
  - SQP iterations
  - terminal absolute / relative residuals
  - `w_vfi`
  - stop reason

