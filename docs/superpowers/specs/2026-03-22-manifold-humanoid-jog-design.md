# Manifold Humanoid Jog Design

## Goal

Add a new humanoid jog solver that targets the user’s real requirement more directly than the existing `new qpJ` and `new sqpJ`:

- terminal **absolute pose** must match the requested target
- terminal **relative pose** must match the requested target
- optional `body_z` must remain a hard terminal equality when enabled
- `w_vfi` must remain above a hard lower bound
- once the hard equalities are satisfied, remaining freedom should be used to improve `w_vfi`

The first version should be exposed as a new solver mode in the existing humanoid jog selector and should focus on `absolute-target`.

## Motivation

The current solver family spans two extremes:

- `零空间投影` is strong at continuously reducing task residuals, but its semantics are task-stack reactive control rather than explicit terminal constrained optimization.
- `new sqpJ` matches the desired “terminal constraint” framing better, but its current merit-driven elastic SQP-lite implementation tends to stall in the `1e-4 ~ 1e-5` region instead of reliably driving terminal equalities toward the requested `1e-12` threshold.

The debugging work so far shows a consistent pattern:

- `zero-space projection` behaves well because it moves continuously on or near the protected task manifold
- `new sqpJ` stalls because it is driven by generic merit decrease, not by a dedicated feasibility-restoration process for the terminal equality manifold

This suggests that the most suitable next solver is not another generic QP/SQP variant, and not a sampling-based planner, but a **constraint-manifold solver** specialized for local humanoid jogging.

## Problem Characterization

The user’s task is primarily a **local constrained terminal IK/control problem**, not a global path-planning problem:

- targets are nearby jog commands
- success depends on satisfying a set of terminal equalities and inequalities at the next accepted state
- failures observed so far are local numerical/algorithmic failures, not global narrow-passage planning failures

Because of that:

- pure sampling methods are not a good first fit
- generic elastic SQP is not ideal as the main method
- a local manifold-restoration method is the best match

## Options Considered

### Option 1: Continue Extending `new sqpJ`

Keep improving the current SQP-lite path with stronger Hessians, SOC, longer polish phases, and more fallback behavior.

Pros:

- reuses the current work
- keeps all logic in one solver family

Cons:

- continues fighting the same architectural mismatch
- still treats terminal feasibility as something mediated by a merit function
- has already shown repeated stagnation in near-solution regions

### Option 2: Add a Dedicated Constraint-Manifold Solver

Implement a new solver that explicitly separates:

1. restoration to the terminal equality manifold
2. nullspace optimization on that manifold

Pros:

- directly matches the user’s requested semantics
- explains why the existing `零空间投影` feels better without copying its limitations wholesale
- gives a clean place to enforce “do not execute unless all terminal equalities are below `1e-12`”

Cons:

- introduces a fifth solver mode
- requires a clean new implementation rather than incremental reuse of `new sqpJ`

### Option 3: Use Sampling + Projection

Generate nearby candidate states with random or heuristic sampling, then project candidates onto the terminal equality manifold.

Pros:

- potentially better at escaping local traps

Cons:

- too heavy for the user’s local jog workflow
- weaker fit for responsive, repeated, interactive point jogging
- unnecessary before exhausting more principled local manifold methods

## Chosen Direction

Implement **Option 2** as a new solver mode:

- display name: `约束流形`
- internal mode key: `manifold`

This solver should coexist with:

- `new qpJ`
- `零空间投影`
- `new sqpJ`
- `NLopt (实验)`
- `约束零空间`

so behavior can be compared directly during tuning.

## Solver Formulation

### Hard Equalities

For `absolute-target`, the solver should treat the following as terminal equalities:

```text
x_abs(q) = x_abs*
x_rel(q) = x_rel*
body_z(q) = 0      // only when body_z is enabled
```

### Hard Inequalities

The solver should preserve:

```text
q_min <= q <= q_max
|q - q_current| <= step_max
w_vfi(q) >= w_min
waist corridor constraints
```

### Secondary Objective

There is no primary “move as little as possible” objective.

After the hard equalities are satisfied, remaining freedom should be used to:

- increase `w_vfi`
- optionally improve joint-limit margin
- optionally reduce unnecessary waist motion later

For the first version, only explicit `w_vfi` improvement is required.

## Algorithm

## Phase A: Feasibility Restoration

Define the equality residual stack:

```text
c(q) = [e_abs(q); e_rel(q); e_body(q)]
```

and its Jacobian:

```text
J_c(q)
```

Each restoration iteration should:

1. evaluate `c(q_k)` and `J_c(q_k)`
2. compute a damped least-squares restoration step

```text
dq_restore = - J_c^T (J_c J_c^T + λI)^-1 c(q_k)
```

3. clamp/project the trial step against:
   - joint bounds
   - step bounds
   - `w_vfi` floor
   - waist corridor
4. use backtracking if needed to keep the step feasible and decreasing in equality residual norm

This phase should stop only when:

- terminal equalities are all below the configured hard tolerance, or
- restoration clearly stalls

This is intentionally closer to “projected Newton / damped restoration on a constraint manifold” than to generic merit-driven SQP.

## Phase B: Nullspace Optimization

Once the state is sufficiently close to the equality manifold, compute a nullspace basis for `J_c(q)`:

```text
J_c(q) N(q) = 0
```

Then only move inside the tangent/nullspace:

```text
dq_null = N z
```

where `z` is chosen to improve `w_vfi`, for example through a projected gradient:

```text
dq_null = η * P_null * grad(w_vfi)
```

Every nullspace step must also preserve:

- joint bounds
- step bounds
- `w_vfi` floor
- waist corridor

The nullspace phase must never be allowed to reintroduce equality error larger than the hard threshold.

## Acceptance Semantics

This solver should use the strict semantics requested by the user:

- if terminal absolute error is not below `1e-12`, do not execute
- if terminal relative error is not below `1e-12`, do not execute
- if `body_z` is enabled and not below `1e-12`, do not execute

There should be no “best effort but still apply” mode for this solver.

This differs intentionally from the legacy jog solvers.

## First-Version Scope

The first implementation should support:

- `absolute-target`
- `waist=off/on`
- `body_z=off/on`

The first implementation should not yet support:

- `relative-target`
- `distance-target`

Those modes should explicitly fall back to an existing solver with a clear diagnostics message.

## Integration Plan

The new solver should be integrated into the existing humanoid jog selector path:

- [index.html](/home/echoz/dq_vizlab/index.html)
- [src/ui/event-handlers.js](/home/echoz/dq_vizlab/src/ui/event-handlers.js)
- [src/core/humanoid-jog-solver-presentation.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-solver-presentation.js)

The implementation should live mostly under `src/core`, not inside a large monolithic addition to [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js).

Recommended modules:

- `src/core/humanoid-jog-manifold-problem.js`
- `src/core/humanoid-jog-manifold-restoration.js`
- `src/core/humanoid-jog-manifold-nullspace.js`
- `src/core/humanoid-jog-manifold-diagnostics.js`

## Diagnostics

The solver should publish diagnostics that make the two-phase behavior visible:

- `restore_iters`
- `restore_residual`
- `nullspace_iters`
- `w_vfi_before`
- `w_vfi_after`
- `stop_reason`
- `corridor_violation`
- `wvfi_floor_violation`

Suggested stop reasons:

- `restore_converged`
- `restore_stalled`
- `ineq_projection_failed`
- `nullspace_only`
- `hard_tol_not_reached`

## Risks

### Risk 1: Equality restoration can still stall

Mitigation:

- use damping
- keep restoration and nullspace phases separate
- prefer residual reduction over generic merit decrease

### Risk 2: Nullspace optimization can break the equalities

Mitigation:

- recompute the tangent basis every iteration
- enforce a strict post-step equality check
- reject nullspace steps that leave the hard tolerance band

### Risk 3: Runtime cost may be high

Mitigation:

- keep the first version to `absolute-target`
- reuse existing FK/Jacobian code paths
- defer expensive secondary objectives until after restoration succeeds

## Success Criteria

The first version is successful if:

1. `absolute-target` can be solved with the new solver under `waist=off/on` and `body_z=off/on`
2. execution is refused unless terminal absolute/relative/body residuals are all below `1e-12`
3. near-solution behavior is measurably closer to the existing `零空间投影` than the current `new sqpJ`
4. `w_vfi` remains protected by a hard lower bound and can be improved in the remaining nullspace
