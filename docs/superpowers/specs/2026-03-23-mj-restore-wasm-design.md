# MuJoCo-Style Restore WASM Offload Design

## Goal

Reduce browser-thread latency for humanoid `absolute-target` jogging by moving the expensive `mj_restore` restoration inner loop from JavaScript into the existing DQ Robotics WASM module.

## Why This Slice

Current hot path cost is dominated by repeated `evaluateProblem(...)` work inside restoration and line search:

- cooperative absolute / relative pose evaluation
- cooperative absolute / relative Jacobian evaluation
- optional `body_z` residual / Jacobian
- repeated trial-state inequality checks

These happen multiple times per jog step and all run on the browser main thread.

The nullspace metric family is also expensive, but much of that cost comes from numeric gradients and can remain in JS for a first pass. Offloading restoration first gives a large latency win without requiring a full rewrite of all secondary metrics.

## Scope

### In scope

- New WASM entry point for `mj_restore` restoration on humanoid `absolute-target`
- Support:
  - `waist=off/on`
  - `body_z=off/on`
  - joint bounds
  - step bounds
  - `w_vfi` floor
  - waist `x` corridor step inequality
- JS `mj_restore` solver path will call WASM restoration when available
- Existing nullspace metric family remains in JS

### Out of scope

- Rewriting all manifold / metric-only buttons in WASM
- Moving numeric manipulability gradients to WASM in this first pass
- `relative-target` / `distance-target` support
- Web Worker integration

## Architecture

### Current

JS builds a problem object and runs:

1. restoration loop in JS
2. optional nullspace metric refine in JS
3. final acceptance in JS

### Proposed

JS still builds the problem inputs, but restoration becomes:

1. JS assembles one restoration request
2. WASM runs the `mj_restore` reference-restoration loop
3. JS receives:
   - `qApplied`
   - convergence / reason
   - iteration counts
   - last accepted scale
   - residual summaries
4. If converged, JS may run existing nullspace metric refine

This preserves current metric semantics while moving the most repetitive linear algebra and cooperative Jacobian evaluation into C++.

## WASM API

Add a new `HumanoidURDF_Wrapper` method:

`solveAbsoluteTargetMJRestore(request)`

### Request fields

- `qCurrent`
- `activeStart`
- `activeCount`
- `absoluteTarget`
- `relativeTarget`
- `keepBodyZVertical`
- `bodyEqualityTolerance`
- `lowerBounds`
- `upperBounds`
- `stepBounds`
- `wvfiLowerBound`
- `referenceModel`
  - `absolute.stiffness`
  - `absolute.damping`
  - `relative.stiffness`
  - `relative.damping`
  - `bodyZ.stiffness`
  - `bodyZ.damping`
- `waistXCorridor`
  - `enabled`
  - `rows`
  - `rhs`
- `maxIterations`
- `damping`
- `initialScale`
- `minScale`
- `reductionFactor`
- `equalityTolerance`
- `inequalityTolerance`

### Result fields

- `qApplied`
- `converged`
- `reason`
- `iterations`
- `acceptedIterations`
- `lastAcceptedScale`
- `commandResidualNorm`
- `relativeResidualNorm`
- `bodyResidualNorm`
- `inequalityViolation`
- `referenceResidual`
- `constraintMotion`

## C++ Solver Structure

Create a dedicated WASM-side solver file pair:

- `wasm/mj_restore_humanoid_solver.hpp`
- `wasm/mj_restore_humanoid_solver.cpp`

It will:

1. evaluate cooperative absolute / relative pose and Jacobians from `DQ_HumanoidURDF`
2. optionally evaluate `body_z` task
3. build weighted equality rows
4. compute `r_ref = Kc - By_prev`
5. solve the local damped least-squares step
6. apply bounds projection and line search
7. enforce:
   - `w_vfi` floor
   - waist corridor step inequality

## Constraint Handling Model

This remains MuJoCo-inspired, not a direct MuJoCo port:

- equality restoration is soft during the iterative solve
- final acceptance remains hard in JS
- joint/step bounds are box projections
- `w_vfi` floor and waist corridor are screened during line search

## JS Integration

`src/visualizer.js` will:

- detect `this.humanoidModel.solveAbsoluteTargetMJRestore`
- prefer WASM restoration when solver mode is `mj_restore`
- fall back to JS `restoreHumanoidJogMJ(...)` if WASM is unavailable
- keep existing diagnostics shape

No UI changes are required beyond the already-added `mj_restore` solver mode.

## Testing

### JS tests

- request normalization / bridging
- fallback to JS when WASM method unavailable
- diagnostics compatibility

### C++ / WASM validation

- build succeeds via `npm run build:wasm`
- browser runtime returns valid result object

### Regression checks

- `mj_restore` still supports `waist=off/on`
- `body_z=on` still uses body tolerance
- existing manifold solver behavior unchanged

## Success Criteria

- `mj_restore` absolute jog path still behaves the same from the user perspective
- browser-thread stutter is reduced because restoration no longer runs its iterative inner loop in JS
- diagnostics still show the same high-level solver states
- unsupported cases still fail fast or fall back cleanly
