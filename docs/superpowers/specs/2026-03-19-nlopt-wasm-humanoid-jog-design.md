# NLopt WASM Humanoid Jog Design

## Goal

Add a third experimental humanoid jog solver, `NLopt (实验)`, alongside the existing `new qpJ` and `零空间投影` solvers.

The first version only targets the path that currently exhibits the most visible `new qpJ` false negatives:

- Humanoid dual-arm jog
- `absolute-target`
- `waist=off`
- `body_z=off`

The experimental solver must run in the browser through the existing WebAssembly pipeline and must never break the current two production paths.

## Motivation

The current `new qpJ` path solves a single-step linearized subproblem through `numeric.solveQP` in [src/core/vfi-utils.js](/home/echoz/dq_vizlab/src/core/vfi-utils.js). This works well in many poses, but it can reject practically useful steps when the linearized constrained QP is reported as inconsistent even though the task is still reachable through multiple nonlinear corrections.

The repository already has:

- an existing Emscripten + CMake + Embind toolchain in [build.sh](/home/echoz/dq_vizlab/build.sh), [wasm/CMakeLists.txt](/home/echoz/dq_vizlab/wasm/CMakeLists.txt), and [wasm/bindings.cpp](/home/echoz/dq_vizlab/wasm/bindings.cpp)
- an existing pure-JS nonlinear optimizer in [src/core/lbfgs-utils.js](/home/echoz/dq_vizlab/src/core/lbfgs-utils.js)

So the project is already set up to host a browser-side nonlinear solver experiment without introducing a backend dependency.

## Options Considered

### Option 1: NLopt + SLSQP

Use `NLopt` as a C++ nonlinear programming backend compiled to WASM. Start with `SLSQP` as the main algorithm and optionally allow a future fallback to `COBYLA`.

Pros:

- supports general nonlinear optimization with equality and inequality constraints
- much closer to the current humanoid absolute-target problem shape than `numeric.solveQP`
- lighter integration target than a full large-scale NLP stack

Cons:

- requires a new third-party dependency and WASM binding layer
- constraint/objective evaluation must be designed carefully to avoid high JS/WASM crossing cost

### Option 2: Ceres Solver

Use `Ceres Solver` as a nonlinear least-squares backend.

Pros:

- excellent nonlinear least-squares tooling
- strong trust-region and line-search support
- a good candidate for future pose-polishing

Cons:

- not a great fit for the current hard-constrained humanoid jog problem
- official docs position it around least-squares and bounds, not as the best first choice for this general constrained jog path

### Option 3: Ipopt

Use `Ipopt` as a full nonlinear programming backend.

Pros:

- strongest general NLP modeling capability

Cons:

- much heavier dependency and build chain
- less suitable as a first browser/WASM experiment

## Chosen Direction

Implement **Option 1: NLopt + SLSQP** as an experimental third solver.

This is the best balance between:

- stronger nonlinear behavior than the current single-step QP
- explicit constraint support
- realistic browser/WASM integration effort

## Scope

### In Scope

- add `NLopt (实验)` to the humanoid jog solver selector
- support only humanoid dual-arm `absolute-target`
- support only `waist=off`
- support only `body_z=off`
- compile `NLopt` into the existing WASM module
- expose one minimal WASM entry point for this experimental solver
- publish diagnostics in the existing humanoid jog diagnostics panel
- safely fall back when the current request is outside the supported scope

### Out of Scope

- replacing `new qpJ`
- replacing `零空间投影`
- supporting `relative-target`
- supporting `distance-target`
- supporting `waist=on`
- supporting `body_z=on`
- rewriting the entire humanoid jog stack around nonlinear programming

## Solver Model

### Decision Variable

The experimental solver optimizes the active humanoid joints for the current dual-arm absolute-target jog step.

For the first version the optimized variables are the active arm joints only, matching the existing `waist=off` absolute-target path.

### Objective

Use a nonlinear objective that prioritizes absolute pose tracking while strongly discouraging relative pose drift:

```text
min_q  wa * ||e_abs(q)||^2
     + wr * ||e_rel(q)||^2
     + wd * ||q - q0||^2
```

Where:

- `e_abs(q)` is the dual-arm absolute pose error against the requested target
- `e_rel(q)` is the dual-arm relative pose error against the frozen current relative pose
- `q0` is the current joint configuration

Initial weight plan:

- `wa = 1`
- `wr = 50..200`
- `wd = 1e-3..1e-2`

The first version intentionally models relative-pose preservation as a strong penalty, not a hard equality, to keep the first WASM experiment smaller and easier to stabilize.

### Hard Constraints

The first version keeps these as hard constraints:

- joint position bounds
- per-step joint delta bounds
- `w_vfi(q) >= w_min`

This preserves the most important safety and conditioning guardrails from the current jog flow.

### `w_vfi` Floor

For the supported first version:

- use the current normal dual-arm floor: `w_vfi >= 5e-4`

Waist-specific thresholds remain out of scope until `waist=on` support exists.

## Algorithm Choice

### Primary Algorithm

Use `NLopt` `SLSQP` as the primary optimizer.

Reasons:

- it matches the constrained nonlinear problem shape
- it supports equality and inequality constraints if we later upgrade relative-pose preservation from penalty to hard equality
- it is a better first experiment than forcing the current problem into a pure least-squares tool

### Future Fallback

Reserve `COBYLA` as a future fallback for cases where gradient-based constrained solves are numerically fragile.

The first implementation does not need to ship both algorithms on day one if that slows down integration too much.

## WASM Integration

### Build Integration

Add `NLopt` as a vendored third-party dependency under `third_party/`, then compile it into the existing `dqrobotics` WASM target from [wasm/CMakeLists.txt](/home/echoz/dq_vizlab/wasm/CMakeLists.txt).

This preserves the existing browser loading path and avoids introducing a second separate WASM module unless measurements later show that split loading is preferable.

### Binding Shape

Expose a single minimal Embind wrapper from [wasm/bindings.cpp](/home/echoz/dq_vizlab/wasm/bindings.cpp), conceptually similar to:

```text
solveHumanoidAbsoluteTargetNLopt(input) -> result
```

Input includes:

- current `q`
- absolute target pose
- relative reference pose
- active joint indices
- joint lower and upper bounds
- per-joint step bounds
- `w_vfi` lower bound
- solver options and weights

Result includes:

- `qNext`
- `converged`
- `iterations`
- `finalCost`
- `finalAbsoluteResidual`
- `finalRelativeResidual`
- `finalWvfi`
- `reason`

### Performance Rule

Do not design the solver around per-iteration JS callbacks.

The objective and constraint evaluation used during optimization should stay inside C++/WASM as much as possible. JS should only marshal the outer request and consume the final result/diagnostics.

## JS-Side Integration

### UI

Extend the current selector in [index.html](/home/echoz/dq_vizlab/index.html) and [src/ui/event-handlers.js](/home/echoz/dq_vizlab/src/ui/event-handlers.js) with:

- `vfi` -> `new qpJ`
- `priority` -> `零空间投影`
- `nlopt` -> `NLopt (实验)`

### Solver Presentation

Update [src/core/humanoid-jog-solver-presentation.js](/home/echoz/dq_vizlab/src/core/humanoid-jog-solver-presentation.js) so the new display name and ready message remain centralized.

### Dispatch

Update [src/visualizer.js](/home/echoz/dq_vizlab/src/visualizer.js) so `updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(...)` dispatches to a new NLopt path when:

- solver mode is `nlopt`
- mode is `absolute-target`
- `includeWaist === false`
- `keepBodyZVertical === false`

For any unsupported configuration, report that the experimental solver is out of scope and fall back to the current `new qpJ` path.

## Diagnostics

Reuse the existing humanoid jog diagnostics panel.

The experimental solver should report at least:

- solver name
- mode tag
- convergence status
- iteration count
- final absolute residual
- final relative residual
- final `w_vfi`
- fallback reason if the request was unsupported

The diagnostics should make it obvious whether the result came from:

- real `NLopt` execution
- unsupported-mode fallback to `new qpJ`

## Error Handling

### Unsupported Scope

If the user selects `NLopt (实验)` outside the supported first-version scope, do not attempt a partial solve. Fall back immediately to `new qpJ` and report why.

### WASM Availability

If the NLopt-enabled WASM module is unavailable, fail closed:

- do not silently no-op
- either fall back to `new qpJ` with an explicit diagnostic or report a solver-unavailable message

### Numerical Failure

If the nonlinear optimizer returns failure or a numerically invalid state:

- do not partially apply the result
- preserve the current humanoid joint state
- surface the solver reason in diagnostics

## Testing

### Unit Tests

Add small JS-side tests for:

- solver mode presentation
- unsupported-mode fallback decision logic
- result payload normalization and diagnostic formatting

### Integration Tests

Add focused tests for the JS adapter that calls the WASM wrapper and handles:

- success
- unsupported mode fallback
- unavailable solver
- invalid numeric result

### Browser Validation

Run a manual browser check for the target scenario:

- humanoid dual-arm `absolute-target`
- `waist=off`
- `body_z=off`
- compare `new qpJ`, `零空间投影`, and `NLopt (实验)` on at least one previously problematic pose

### Benchmarking

Use the existing humanoid jog benchmark flow as a follow-up, not a blocker for first integration. The first milestone is correctness and safe coexistence, not immediate production superiority.

## Rollout Plan

Phase 1:

- land the experimental solver behind the third selector option
- keep scope narrow
- gather qualitative comparisons against `new qpJ`

Phase 2:

- tune weights and stop criteria
- decide whether relative-pose preservation should become a hard equality
- decide whether `COBYLA` fallback is needed

Phase 3:

- consider extending support to `waist=on` and `body_z=on`
- decide whether the solver should remain experimental or replace part of `new qpJ`

## Success Criteria

The first experimental release is successful if:

- it builds into the existing WASM pipeline
- it can be selected from the current humanoid jog UI
- it safely coexists with the two current solvers
- it runs on the target `absolute-target / no-waist / no-body-z` path
- it provides readable diagnostics
- it demonstrates at least some poses where the current `new qpJ` fails but the nonlinear solver still finds a usable move
