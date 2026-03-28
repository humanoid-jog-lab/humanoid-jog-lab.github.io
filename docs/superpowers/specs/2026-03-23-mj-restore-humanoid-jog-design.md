# MuJoCo-Style Reference Restoration For Humanoid Jog

## Goal

Add a new humanoid jog solver that borrows the most useful part of MuJoCo's
constraint model for this project: smooth reference restoration of constraints,
instead of brittle one-step hard restoration.

This solver is intended to improve the current failure modes of `manifold`:

- near-boundary jerkiness around waist corridor limits
- early `best_effort` / `max_iter` stalls
- over-hard restoration behavior when `absolute / relative / body_z` are all
  forced through the same direct residual-zeroing loop

The new solver will be a separate control backend, not a modification of
`new qpJ`, `new sqpJ`, or `manifold`.

## Scope

### In scope for v1

- New humanoid jog solver mode: `mj_restore`
- `absolute-target` only
- `waist=off/on`
- `body_z=off/on`
- Reuse existing cooperative dual-arm absolute/relative pose and Jacobians from
  DQ Robotics
- Reuse existing nullspace metric family after restoration converges

### Out of scope for v1

- `relative-target`
- `distance-target`
- full dynamics / acceleration-level simulation
- contact dynamics, friction cones, or physics-engine parity with MuJoCo
- full second-order constraint Hessians

## Why This Solver Exists

The current solver family has a gap:

- `new qpJ` is locally robust and often feels smooth, but does not provide the
  structured restoration semantics we want.
- `new sqpJ` is too brittle near exact terminal constraints.
- `manifold` matches the intended hard terminal semantics better, but its
  restoration loop is still basically a residual-zeroing solver with line
  search, which is prone to stalls and jitter.

MuJoCo's constraint model suggests a better restoration pattern:

- define a reference recovery behavior for each constraint
- solve toward that reference behavior
- soften the restoration process itself, while keeping final acceptance strict

## High-Level Architecture

Add a new solver mode in the humanoid jog backend:

- UI label: `MuJoCo 风格恢复`
- internal enum: `mj_restore`

New modules:

- `src/core/humanoid-jog-mj-problem.js`
- `src/core/humanoid-jog-mj-restoration.js`
- `src/core/humanoid-jog-mj-nullspace.js`
- `src/core/humanoid-jog-mj-diagnostics.js`

Thin integration only in:

- `src/visualizer.js`
- `src/ui/event-handlers.js`
- `src/core/humanoid-jog-solver-presentation.js`
- `index.html`

## Problem Definition

For `absolute-target`, define the equality residual stack:

```text
c(q) = [e_abs(q), e_rel(q), e_body(q)]
```

where:

- `e_abs(q)` is the cooperative absolute pose residual
- `e_rel(q)` is the cooperative relative pose residual
- `e_body(q)` is the body vertical residual when enabled

The equality Jacobian stack is:

```text
J_c(q) = [J_abs(q); J_rel(q); J_body(q)]
```

These come from the existing DQ Robotics cooperative dual-arm kinematics and
the current body-z helper chain.

## Reference Restoration Model

The restoration phase does not directly solve:

```text
J_c dq = c(q)
```

Instead, it tracks a reference restoration target:

```text
r_ref = K c(q) - B y_prev
```

where:

- `K` is a per-constraint restoration stiffness
- `B` is a per-constraint restoration damping
- `y_prev = J_c dq_prev`
  is the previously accepted restoration motion in constraint space

This is the MuJoCo-inspired part of the design:

- constraints do not behave as infinitely hard one-shot equalities during the
  inner loop
- restoration has explicit recovery dynamics
- near boundaries and near convergence, damping smooths the updates

## Local Solve

Each restoration iteration solves a damped local least-squares problem:

```text
min_dq  1/2 ||J_c dq - r_ref||_W^2 + 1/2 lambda ||dq||^2
```

subject to:

- joint bounds
- single-step bounds

First version will not solve corridor / `w_vfi floor` inside the same dense
optimization system. Those are enforced via:

- smooth step screening
- line search / shrinkage
- feasibility checks

This keeps v1 simple and local, while still following the reference-restoration
idea.

## Constraint Softness Policy

The solver must distinguish between process softness and final strictness.

### During restoration

Use per-family stiffness / damping:

- absolute: high stiffness, moderate damping
- relative: high stiffness, slightly stronger damping
- body_z: lower stiffness, higher damping

Recommended defaults:

- `mjRestoreAbsoluteStiffness = 1.0`
- `mjRestoreAbsoluteDamping = 0.25`
- `mjRestoreRelativeStiffness = 1.0`
- `mjRestoreRelativeDamping = 0.35`
- `mjRestoreBodyStiffness = 0.6`
- `mjRestoreBodyDamping = 0.45`

### At final acceptance

Keep strict acceptance semantics:

- `absolute <= 1e-12`
- `relative <= 1e-12`
- `body_z <= bodyTolerance`
- inequalities within tolerance

So:

- the recovery process is softened
- the final execution gate remains strict

## Inequality Handling

v1 inequality handling:

- joint bounds: hard box projection
- single-step bounds: hard box projection
- `w_vfi floor`: reject / shrink violating trial steps
- waist corridor: VFI-style shrink / reject of violating trial steps

This explicitly does not try to mimic MuJoCo's full contact solver. The point is
to bring over the reference-restoration idea, not contact mechanics.

## Nullspace Phase

Once restoration converges, reuse the existing metric family from the current
`manifold` solver.

Supported metrics for v1:

- `none`
- `w_vfi`
- `w_cmd`
- `w_abs`
- `w_abs_arm`
- `w_abs_vfi`
- `w_abs_hier`
- `w_abs_hier_vfi`
- `sigma`

Nullspace behavior:

- compute nullspace basis of `J_c`
- take a protected nullspace step for the selected metric
- re-run restoration after the nullspace step
- accept only if strict constraints remain satisfied

This keeps the metric vocabulary stable across `manifold` and `mj_restore`.

## DQ Robotics Dependency Notes

The solver will continue to rely on the existing DQ Robotics cooperative dual-arm
functions:

- cooperative absolute pose
- cooperative relative pose
- cooperative absolute Jacobian
- cooperative relative Jacobian

The repository does expose many Jacobian derivative utilities in DQ Robotics,
but not a ready-to-use cooperative humanoid dual-arm Jacobian derivative API.
Therefore v1 does not depend on `Jdot`.

This is acceptable because the reference-restoration controller can be built
entirely from:

- current residuals
- current Jacobians
- previous accepted constraint-space motion

## Diagnostics

Add solver-specific diagnostics:

- `restore=...`
- `restore_iters=...`
- `restore_scale=...`
- `restore_ref=...`
- `nullspace=...`
- `jog_metric=...`
- `rel=...`
- `body=...`
- `fallback=...`

Additional MJ-specific lines:

- `mj_ref: abs(k=...,b=...) rel(k=...,b=...) body(k=...,b=...)`
- `mj_restore: active_res=... ref_res=... prev_y=...`

The purpose is to make it obvious whether the solver is:

- stalled in restoration
- converged in restoration but skipped metric refinement
- oscillating because of inequality shrinkage

## Acceptance Rules

Execution is allowed only when:

- `absolute error <= 1e-12`
- `relative error <= 1e-12`
- `body_z <= bodyTolerance` when enabled
- inequality violations are within tolerance

Otherwise the solver reports failure and does not apply the step.

## Recommended Implementation Order

1. Add solver mode plumbing and UI selector entry
2. Implement `mj problem` builder for absolute-target
3. Implement `mj restoration` loop with reference recovery
4. Hook `waist=off/on`
5. Hook `body_z=on`
6. Reuse existing nullspace metric logic
7. Add diagnostics
8. Compare behavior against current `manifold` and `new qpJ`

## Success Criteria

The new solver is successful if, compared to `manifold`:

- it reduces early `best_effort` stalls
- it behaves more smoothly near waist corridor boundaries
- it preserves strict final acceptance semantics
- it can reuse the same nullspace metric family without changing button/UI

