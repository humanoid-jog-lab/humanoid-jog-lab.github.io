# Constrained Nullspace Humanoid Jog Design

## Goal

Add a fourth humanoid jog solver, `约束零空间`, alongside the existing:

- `new qpJ`
- `零空间投影`
- `NLopt (实验)`

This new solver must stay inside the current browser-side JS stack and improve on the current `零空间投影` path for humanoid jogging by preserving its multi-step, task-priority behavior while handling constraints more deliberately.

The first version must support all three humanoid jog modes:

- `absolute-target`
- `relative-target`
- `distance-target`

and both full-body settings:

- `waist=off/on`
- `body_z=off/on`

## Motivation

The current `零空间投影` implementation in [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js) is often more willing to move than `new qpJ`, because it repeatedly re-linearizes and walks through task-priority pseudo-inverse steps instead of solving one strict constrained QP. That property is useful for the user’s current humanoid jog workflow.

However, the current priority implementation has structural limitations:

- the three humanoid priority paths are separate large functions
- constraint handling is mostly a mix of hard clipping, nullspace limit-avoidance gradients, and final rollback
- final acceptance can reject useful candidate motions late in the pipeline
- there is no single shared feasibility-repair layer
- `w_vfi` uplift and corridor handling are not expressed as a unified constrained nullspace policy

The requested new solver should keep the “can walk around local trouble through multiple small steps” nature of priority control, but add a more explicit constrained acceptance flow.

## Current Context

### Existing Solver Selector

The humanoid jog solver selector is already centralized in:

- [index.html](/home/echoz/dq_vizlab/index.html)
- [src/ui/event-handlers.js](/home/echoz/dq_vizlab/src/ui/event-handlers.js)
- [src/core/humanoid-jog-solver-presentation.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-solver-presentation.js)

So the new solver should be added as another formal mode in the same selector instead of creating a parallel UI control surface.

### Existing Priority Paths

The current legacy priority implementation lives in three separate humanoid paths inside [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js):

- humanoid absolute priority path around `absolute-target`
- humanoid relative priority path around `relative-target`
- humanoid distance priority path around `distance-target`

These paths already contain useful building blocks:

- weighted damped pseudo-inverse
- nullspace projection
- body-z task construction
- joint-limit gradient generation
- current diagnostics publication

Those pieces should be reused instead of discarded.

## Options Considered

### Option 1: Keep Patching The Existing Priority Branches

Continue tuning the three legacy priority functions in place by changing gains, step limits, guards, and constraint heuristics.

Pros:

- smallest short-term code delta
- no new solver name or structure

Cons:

- keeps duplicating logic across three giant branches
- further entangles solver policy with UI orchestration
- likely produces more symptom-specific fixes instead of one coherent solver

### Option 2: Build A New Constrained Nullspace Solver On Top Of The Existing Priority Idea

Create a new solver mode that keeps hierarchical task-priority motion generation but adds:

- shared task builders
- shared feasibility repair
- shared line search
- shared `w_vfi` / corridor / body-z handling

Pros:

- matches the user’s preference for multi-step nullspace behavior
- reuses the strongest parts of the current priority implementation
- creates cleaner boundaries for future iteration

Cons:

- requires extracting logic out of [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js)
- first version needs careful diagnostics parity

### Option 3: Write A New SQP-Lite Solver

Implement a fully new iterative constrained solver with linearized subproblems each round.

Pros:

- more mathematically uniform than today’s priority code

Cons:

- larger design and implementation jump
- slower to reach a stable first version
- less reuse of the parts the user already likes

## Chosen Direction

Implement **Option 2** as a new formal solver mode named `约束零空间`, with internal mode key `cns`.

This keeps the spirit of the existing `零空间投影` solver but improves it with explicit constrained stepping and acceptance logic.

## Solver Model

### High-Level Structure

For every jog step, the new solver performs repeated nonlinear re-linearization rounds. Each round has four phases:

1. **Task-priority candidate generation**
   Build a priority-consistent candidate `dq_task` from the current mode’s task hierarchy.

2. **Feasibility correction**
   Adjust the candidate by a small correction `δdq` so the step respects the constrained feasible region as closely as possible before acceptance.

3. **Nullspace secondary objective**
   Inject secondary motion in the remaining safe nullspace for:
   - `w_vfi` ascent
   - joint-limit avoidance
   - optional waist minimization preference

4. **Continuous line search acceptance**
   Search for the largest acceptable step scale that:
   - improves the commanded task
   - preserves protected invariants
   - satisfies `w_vfi` floor and corridor checks
   - passes the axis guard

Unlike the current legacy priority path, the new solver should avoid “all-or-nothing” late rollback whenever a smaller acceptable step exists.

### Mode-Specific Priority Order

#### `absolute-target`

Priority order:

1. relative pose preservation
2. optional body-z verticality
3. absolute pose command
4. secondary objectives in residual nullspace

#### `relative-target`

Priority order:

1. absolute pose preservation
2. relative pose command
3. secondary objectives in residual nullspace

#### `distance-target`

Priority order:

1. absolute pose preservation
2. relative rotation preservation
3. distance command
4. secondary objectives in residual nullspace

### Constraints

The new solver must treat these as explicit acceptance requirements:

- joint hard limits
- per-joint step bounds
- waist x corridor
- `w_vfi` lower bound
- `body_z` requirement when enabled

`relative pose` and `absolute pose` preservation tasks remain part of the hierarchical task stack, but the solver’s feasibility and acceptance layers must be aware of them and refuse candidate steps that break protected high-priority tasks beyond configured tolerances.

### `w_vfi` Behavior

The new solver must support both:

- a hard lower bound using the current policy already used elsewhere
- an explicit ascent secondary objective in the remaining admissible nullspace

This keeps the meaning aligned with the user’s earlier requirement: do not relax the primary command or protected constraints merely to gain manipulability, but do actively climb `w_vfi` whenever nullspace is available.

## Architecture

### New Core Modules

Add focused JS modules under [src/core](/home/echoz/dq_vizlab/src/core):

- `humanoid-jog-cns-solver.js`
  - shared outer iteration loop
  - per-round orchestration
  - final diagnostics summary

- `humanoid-jog-cns-tasks.js`
  - mode-specific task/error/Jacobian construction
  - body-z task builder
  - protected-task metadata

- `humanoid-jog-cns-feasibility.js`
  - candidate feasibility correction
  - `w_vfi` floor checks
  - waist corridor handling
  - step clipping against hard bounds

- `humanoid-jog-cns-line-search.js`
  - continuous step scaling
  - acceptance predicates
  - best-step retention

### Reused Existing Modules

The new solver should reuse current logic where possible:

- [src/core/cctsm.js](/home/echoz/dq_vizlab/src/core/cctsm.js)
- [src/core/wvfi-guard.js](/home/echoz/dq_vizlab/src/core/wvfi-guard.js)
- [src/core/humanoid-jog-axis-guard.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-axis-guard.js)
- [src/core/humanoid-jog-outcome.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-outcome.js)
- current Jacobian / FK / body-z helpers already present in [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js)

### Thin Visualizer Integration

[src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js) should only:

- dispatch to the new solver when mode is `cns`
- collect current mode inputs
- publish diagnostics
- apply the accepted `qApplied`

The solver policy itself should live outside the giant visualizer methods.

## UI Integration

### Solver Selector

Extend the existing humanoid jog solver selector to show:

- `new qpJ`
- `零空间投影`
- `NLopt (实验)`
- `约束零空间`

Internal normalization should accept `cns` as a new formal solver key.

### Diagnostics

The existing humanoid diagnostics panel should remain the single place to inspect solver behavior.

New diagnostics should include:

- `solver: 约束零空间`
- `iters`
- `active_joints`
- `err(cmd/inv)`
- `step`
- `CCTSM: w / w_vfi`
- whether feasibility repair was triggered
- final line-search scale
- whether nullspace secondary motion was applied

## Out Of Scope

The first version does not need to:

- replace or remove the legacy `零空间投影`
- rewrite `new qpJ`
- move the solver to WASM
- expose user-facing tuning controls for every internal weight
- unify every existing visualizer helper into new files immediately

## Testing Strategy

### Unit Tests

Add focused tests for:

- solver mode normalization / presentation updates
- mode-specific task building
- feasibility correction decisions
- line search selection
- `w_vfi` ascent only when protected tasks allow it

### Integration Tests

Add integration-style JS tests for:

- `absolute-target` with `waist=on` and `body_z=on`
- `relative-target` preserving absolute pose
- `distance-target` preserving absolute pose and relative rotation
- cases where a smaller acceptable step must be kept instead of rolling back to `qInitial`

### Build Verification

Because this is JS-only, verification must include:

- `node --check` on new core modules
- target unit tests via `node --test`
- `npm run build`

## Rollout

The new solver should be added without removing any existing solver.

Users must be able to compare:

- `new qpJ`
- legacy `零空间投影`
- `NLopt (实验)`
- new `约束零空间`

This allows real-pose comparison before deciding whether the new constrained nullspace solver should eventually replace the old priority path.
