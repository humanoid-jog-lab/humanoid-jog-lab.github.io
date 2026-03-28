# Manifold Jog Secondary Metric Selector Design

## Goal

Add a **secondary-metric selector** for the `约束流形` humanoid jog solver so that normal jog buttons (`x/y/z/rx/ry/rz`) can optionally run a protected nullspace refinement after manifold restoration.

The selector should let the user choose which metric is optimized during jog execution, instead of forcing a single built-in secondary objective.

The first version should support these options:

- `none`
- `w_vfi`
- `w_cmd`
- `w_abs`
- `w_abs_arm`
- `w_abs_vfi`
- `w_abs_hier`
- `w_abs_hier_vfi`
- `sigma`

## Motivation

The current `约束流形` jog flow already has two distinct phases:

1. restoration back to the protected equality manifold
2. optional nullspace refinement

However, the metric used in the refinement step is still effectively hard-wired in the jog path. The user already has several dedicated optimization buttons (`w_vfi`, `w_cmd`, `w_abs`, `w_abs_arm`, `w_abs_hier`, `w_abs_hier_vfi`, `sigma`), but those are separate from normal jog commands.

The missing capability is:

- perform a normal jog command
- while explicitly selecting which secondary metric should be improved if nullspace freedom is available

This is especially useful for comparing how different conditioning criteria affect the same point-jog trajectory.

## Scope

### In Scope

- Add a UI selector for `约束流形` jog secondary metric
- Apply the selected metric only to manifold jog commands
- Preserve the current restoration-first semantics
- Reuse the existing metric definitions already exposed by the dedicated optimization buttons
- Extend diagnostics so the selected jog metric and its application state are visible

### Out of Scope

- Changing the semantics of existing dedicated optimization buttons
- Changing `new qpJ`, `new sqpJ`, `零空间投影`, `约束零空间`, or `NLopt (实验)`
- Adding brand new manipulability metrics beyond the listed set
- Reworking manifold restoration itself

## User Experience

### UI

In the humanoid control panel, when the active solver is `约束流形`, add a selector near the existing speed / manipulability controls:

- label: `点动伴随优化`
- values:
  - `none`
  - `w_vfi`
  - `w_cmd`
  - `w_abs`
  - `w_abs_arm`
  - `w_abs_vfi`
  - `w_abs_hier`
  - `w_abs_hier_vfi`
  - `sigma`

The selector should persist through the same UI state path used for other humanoid jog controls.

### Behavior

For a normal manifold jog step:

1. execute the existing restoration phase toward the jog target
2. if restoration converges, inspect the selected secondary metric
3. if the selection is not `none`, run the corresponding nullspace refinement
4. restore again if needed so `absolute / relative / body_z` stay protected
5. accept the refined state only if:
   - restoration remains valid
   - metric direction matches the selected optimization intent
   - all existing hard constraints stay satisfied

If the selector is `none`, the jog path behaves like pure restoration with no extra nullspace metric step.

## Metric Semantics

The selector options should reuse the same metric meanings already present in the codebase:

- `w_vfi`: VFI-weighted overall cooperative manipulability
- `w_cmd`: projected commanded-task manipulability
- `w_abs`: full-body absolute manipulability
- `w_abs_arm`: arm-only absolute manipulability, even when waist is active
- `w_abs_vfi`: absolute manipulability with VFI attenuation
- `w_abs_hier`: hierarchical metric whose primary stage is `w_abs_arm`, then full-body `w_abs`
- `w_abs_hier_vfi`: hierarchical metric whose primary stage is `w_abs_arm`, then `w_abs_vfi`
- `sigma`: directional manipulability along the last commanded jog axis

## Chosen Direction

Implement the selector as a **manifold-jog-only secondary-metric chooser**.

This is preferred over merging all semantics into the dedicated optimization buttons because:

- it keeps jog semantics and standalone optimization semantics separate
- it reuses the existing metric implementations
- it gives a single comparable entry point for testing jog behavior under different secondary objectives

## Architecture

### UI Layer

Add a selector in:

- `index.html`
- `src/ui/event-handlers.js`

The chosen value should be stored in the same UI state path used for other humanoid jog settings and should only influence manifold jog execution.

### Solver Dispatch

Extend the manifold jog path in `src/visualizer.js` so that the absolute jog flow reads a `manifoldJogSecondaryMetric` setting.

The jog path already computes:

- restoration result
- nullspace result
- restore-after-nullspace result

The new selector should only choose **which nullspace objective** is used in that refine stage.

### Metric Resolution

Add a small resolver layer that maps selector values to the corresponding objective-gradient / evaluation pair. This should live in a focused helper rather than inlining more conditional branches inside `visualizer.js`.

Expected helper responsibilities:

- normalize selector values
- expose metric evaluation for diagnostics
- expose the objective-gradient choice for nullspace refinement
- route hierarchical metrics to their existing specialized paths

## Acceptance Rules

The selector must not weaken manifold hard-equality semantics.

A refined jog state is acceptable only if:

- restoration still converges
- `absolute` hard tolerance is preserved
- `relative` hard tolerance is preserved
- `body_z` tolerance is preserved when enabled
- `w_vfi` floor stays satisfied
- waist corridor stays satisfied

For directional metrics (`sigma`), the direction must remain tied to the most recent jog axis.

For hierarchical metrics, the existing primary/secondary guard semantics must remain intact.

## Diagnostics

Augment manifold jog diagnostics with:

- `jog_metric=<selected>`
- `jog_metric_opt=<state>`
- `jog_metric_before=<value>`
- `jog_metric_after=<value>`
- `jog_metric_applied=yes/no`

Examples of states:

- `skipped(none)`
- `skipped(waiting_for_restore)`
- `active`
- `rejected(direction_mismatch)`
- `rejected(hard_constraints)`

The current dedicated optimization diagnostics should remain unchanged.

## Compatibility Notes

- Existing dedicated optimization buttons continue to work unchanged
- Existing manifold jog behavior with no explicit secondary objective becomes `none`
- Existing metrics are reused rather than redefined

## Tests

Add or update tests for:

1. selector normalization and persistence
2. manifold jog dispatch using `none`
3. manifold jog dispatch using each supported metric family:
   - direct metric
   - hierarchical metric
   - directional metric
4. diagnostics labeling for jog secondary metric state
5. solver mismatch behavior stays unchanged for non-manifold solvers

## Implementation Notes

Recommended implementation order:

1. add selector UI and persistence
2. add metric-resolution helper
3. thread selected metric into manifold jog path
4. update diagnostics
5. add targeted tests

## Success Criteria

The feature is successful when:

- the user can choose a manifold jog secondary metric from the UI
- normal jog buttons honor that selection during `约束流形` execution
- existing standalone `w_*` / `sigma` buttons still work
- diagnostics clearly show which metric, if any, was used during the jog step
