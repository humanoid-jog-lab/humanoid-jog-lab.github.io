# SQP Humanoid Jog Design

## Goal

Add a new formal humanoid jog solver, `new sqpJ`, alongside the existing:

- `new qpJ`
- `零空间投影`
- `NLopt (实验)`
- `约束零空间`

This solver should target the user’s preferred formulation for humanoid jogging:

- the **terminal absolute pose** must reach the requested target
- the **terminal relative pose** must reach the requested target
- optional `body_z` should remain a hard terminal constraint when enabled
- `w_vfi` should be protected by a lower bound and rewarded in the objective

The first version should support:

- `absolute-target`
- `waist=off/on`
- `body_z=off/on`

`relative-target` and `distance-target` should be added after the absolute path is stable, reusing the same SQP core.

## Motivation

The current `new qpJ` solver in [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js) is a **single-step linearized QP**. Its behavior is governed by local Jacobian constraints such as:

- `J_abs dq = e_abs`
- `J_rel dq = e_rel`

This is useful for reactive motion, but it is not the same as the user’s intended problem:

- the final state should satisfy the desired absolute pose
- the final state should satisfy the desired relative pose
- `w_vfi` and joint motion should be optimized within that feasible set

Those requirements define a **nonlinear constrained optimization problem**, not a single QP.

The more appropriate framing is **SQP**:

- outer iterations re-linearize the nonlinear task and constraint set
- inner iterations solve QP subproblems
- line search or trust-region logic accepts only improving steps

This keeps QP as a subproblem, but makes the full solver match the real task definition.

## Current Context

### Existing Solver Selector

The humanoid jog solver selector is already centralized in:

- [index.html](/home/echoz/dq_vizlab/index.html)
- [src/ui/event-handlers.js](/home/echoz/dq_vizlab/src/ui/event-handlers.js)
- [src/core/humanoid-jog-solver-presentation.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-solver-presentation.js)

So the new solver should be added as another option in the same selector rather than introducing a separate control surface.

### Existing Reusable Building Blocks

The codebase already contains reusable ingredients for an SQP solver:

- pose and relative-pose extraction in [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js)
- absolute and relative Jacobian accessors in [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js)
- `body_z` task construction in [src/core/body-vertical-utils.js](/home/echoz/dq_vizlab/src/core/body-vertical-utils.js)
- CCTSM / `w_vfi` evaluation in [src/core/cctsm.js](/home/echoz/dq_vizlab/src/core/cctsm.js)
- `w_vfi` floor helpers in [src/core/wvfi-guard.js](/home/echoz/dq_vizlab/src/core/wvfi-guard.js)
- QP utilities in [src/core/vfi-utils.js](/home/echoz/dq_vizlab/src/core/vfi-utils.js)

The new SQP solver should reuse those pieces instead of introducing a separate math stack.

## Options Considered

### Option 1: Keep Extending `new qpJ`

Continue tuning the current single-step QP path with more guards, fallback logic, and heuristics.

Pros:

- smallest code change
- reuses the current reactive-QP pathway

Cons:

- still solves the wrong mathematical problem
- terminal hard equalities remain approximated by one local linearization
- likely continues the current pattern of brittle edge-case tuning

### Option 2: Add A New Single-State SQP Solver

Model the jog command as a nonlinear constrained optimization over the next joint state `q_next`, with terminal hard equalities and an outer SQP loop.

Pros:

- matches the user’s intended formulation directly
- keeps QP where it belongs: as an SQP subproblem
- lets `w_vfi` become a first-class optimization term instead of a post-hoc guard

Cons:

- larger implementation than extending `new qpJ`
- requires careful merit function and acceptance design

### Option 3: Build A Short-Horizon MPC/SQP Solver

Optimize a short sequence `q_1 ... q_N` with terminal constraints at the end of the horizon.

Pros:

- best at “绕过去” around local trouble
- most expressive form of terminal control

Cons:

- significantly heavier
- harder to stabilize in the browser for first delivery
- larger diagnostics and tuning surface

## Chosen Direction

Implement **Option 2** as a new solver mode named `new sqpJ`, with internal key `sqp`.

This gives the project a solver that matches the user’s requested problem formulation without immediately taking on full MPC complexity.

## Problem Formulation

### Decision Variable

The first version optimizes the next accepted humanoid joint state:

- `q_next`

This is equivalent to solving over `Δq = q_next - q_current`, but `q_next` is the clearer conceptual object because all hard terminal constraints are imposed on the final state.

### Objective

The first version should minimize joint displacement while rewarding manipulability:

```text
minimize_q_next
    ||q_next - q_current||_W^2
  - λ_w * w_vfi(q_next)
  + λ_margin * joint_margin_reward(q_next)
  + λ_waist * waist_motion_penalty(q_next)
```

Required terms:

- joint displacement regularization
- `w_vfi` reward

Optional later terms:

- joint-limit margin reward
- explicit waist minimization

### Hard Constraints

For `absolute-target`, the first version should enforce:

```text
x_abs(q_next) = x_abs*
x_rel(q_next) = x_rel*
```

When enabled:

```text
body_z(q_next) = 0
```

Always:

```text
q_min <= q_next <= q_max
|q_next - q_current| <= step_max
w_vfi(q_next) >= w_min
```

This means:

- absolute pose is a terminal hard equality
- relative pose is a terminal hard equality
- `body_z` is a terminal hard equality when active
- joint bounds and step bounds remain hard
- the `w_vfi` lower bound remains hard

## Algorithm

### Outer SQP Loop

At each SQP iteration:

1. Evaluate all nonlinear task and constraint functions at `q_k`
2. Build linearized equality and inequality constraints
3. Build a QP subproblem around `q_k`
4. Solve the QP subproblem for a trial step `Δq_k`
5. Run line search or trust-region acceptance
6. Accept `q_{k+1} = q_k + α Δq_k` if the merit function decreases
7. Stop when terminal equality residuals and optimality conditions are small enough

### QP Subproblem

Each SQP subproblem should have the generic form:

```text
minimize_Δq
    1/2 Δq^T H_k Δq + g_k^T Δq

subject to
    Aeq_k Δq + ceq_k = 0
    Aineq_k Δq + cineq_k >= 0
    lb_step <= Δq <= ub_step
```

Where:

- `Aeq_k` contains linearized absolute, relative, and optional `body_z` equalities
- `Aineq_k` contains the linearized `w_vfi` lower bound when treated in subproblem form
- box bounds enforce joint limits and step limits
- `H_k` starts from a regularized positive-definite approximation and can later be upgraded with BFGS updates

### Merit Function

Step acceptance should be governed by a single merit function:

```text
phi(q) =
    objective(q)
  + rho_abs * ||x_abs(q) - x_abs*||_1
  + rho_rel * ||x_rel(q) - x_rel*||_1
  + rho_body * ||body_z(q)||_1
  + rho_vfi * violation_wvfi(q)
```

This avoids the current situation where multiple unrelated guards can reject a step for different local reasons without a coherent notion of progress.

### Step Acceptance

The solver should accept a trial point only if:

- merit decreases sufficiently
- hard bound feasibility is preserved
- equality residuals do not diverge
- `w_vfi` floor is not violated

This acceptance should replace the current “single-step mismatch” style logic for this solver.

## First-Version Scope

### Included

- new solver selector option `new sqpJ`
- `absolute-target`
- `waist=off/on`
- `body_z=off/on`
- terminal hard equality for absolute pose
- terminal hard equality for relative pose
- terminal hard equality for `body_z` when enabled
- hard joint bounds
- hard step bounds
- hard `w_vfi` lower bound
- explicit `w_vfi` reward in the objective
- line search based on a merit function

### Deferred

- `relative-target`
- `distance-target`
- trajectory horizon / MPC
- second-order exact Hessians
- WASM/native acceleration
- replacing existing solvers

## Architecture

### New Core Modules

Add focused modules under [src/core](/home/echoz/dq_vizlab/src/core):

- `humanoid-jog-sqp-problem.js`
  - build nonlinear absolute/relative/body-z/w_vfi quantities
  - expose values and Jacobians at the current iterate

- `humanoid-jog-sqp-subproblem.js`
  - assemble the QP matrices from the current linearization
  - apply regularization and bounds

- `humanoid-jog-sqp-merit.js`
  - evaluate merit values
  - compute constraint violation summaries

- `humanoid-jog-sqp-solver.js`
  - outer SQP iteration loop
  - line search / acceptance
  - stopping conditions

- `humanoid-jog-sqp-diagnostics.js`
  - normalize solver statistics for the existing humanoid diagnostics panel

### Visualizer Integration

Keep [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js) thin:

- detect `sqp` mode
- build the jog command target
- invoke the new SQP solver
- publish diagnostics
- apply the accepted `q_next`

This avoids growing the already large visualizer file with another full solver implementation.

## Diagnostics

The new solver should publish:

- SQP iteration count
- last QP subproblem status
- absolute terminal residual
- relative terminal residual
- optional `body_z` residual
- current `w_vfi`
- `w_vfi` lower-bound slack
- line-search step scale
- final stop reason

The diagnostics panel should continue to show:

- solver name
- mode tag
- `flags: waist/body_z/limit`
- `CCTSM: w / w_vfi`

## Testing Strategy

### Unit Tests

Add unit coverage for:

- problem linearization
- merit-function decrease checks
- QP subproblem assembly
- acceptance / rejection logic
- hard terminal equality handling

### Regression Cases

Use the user’s existing problematic humanoid poses as regression fixtures, especially:

- waist-enabled absolute motions that current `new qpJ` rejects despite being reachable
- cases where current `QP` drifts in non-command axes
- cases where current `QP` fails to preserve relative pose tightly enough

### Browser Verification

For the first `absolute-target` version, verify in the real page that:

- `new sqpJ` reaches the intended absolute and relative pose terminal conditions
- `body_z` remains hard when enabled
- `w_vfi` floor is preserved
- the resulting motion is at least as semantically correct as `new qpJ`, even if it takes more compute

## Success Criteria

The first version is successful if:

- it reaches terminal absolute and relative pose targets as hard equalities for `absolute-target`
- it behaves correctly for both `waist=off/on`
- it supports `body_z=on` as a hard equality
- it preserves the `w_vfi` lower bound
- it offers a clear semantic improvement over the current `new qpJ` on the user’s failure cases
- it plugs into the existing humanoid jog selector and diagnostics cleanly

