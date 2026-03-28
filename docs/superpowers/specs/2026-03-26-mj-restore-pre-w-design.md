# MuJoCo-Style Restoration With Pre-w Optimization Design

## Goal

Add a new humanoid jog solver mode that performs a protected `w` optimization
stage before running the existing `MuJoCo 风格恢复` controller.

This solver is meant to preserve the semantic clarity of plain `mj_restore`
while giving users an explicit way to bias the configuration toward better
manipulability before each jog command.

The key requirement is separation of concerns:

- plain `mj_restore` must remain a clean baseline controller
- `pre-w` behavior must only happen when the new solver is selected
- failure in the `pre-w` stage must never block the actual jog command

## Scope

### In scope for v1

- New humanoid jog solver mode:
  - UI label: `MuJoCo 风格恢复 + w预优化`
  - internal enum: `mj_restore_pre_w`
- Supported control modes:
  - `absolute`
  - `decoupled`
  - `decoupled-upright`
- Reuse the existing `w` selector, limited to:
  - `w_abs_hier`
  - `w_abs_hier_vfi`
- Two-stage execution:
  1. protected metric-only `pre-w`
  2. normal `mj_restore` jog from the pre-optimized joint state
- Diagnostic output that makes `pre-w` state explicit

### Out of scope for v1

- `relative`
- `relative-fullbody`
- `distance`
- `distance-fullbody`
- any change to the semantics of plain `mj_restore`
- any automatic `w` optimization inside plain `mj_restore`

## Why This Solver Exists

The current UI already distinguishes between:

- plain jog control
- explicit `w+ / w-` metric actions

But there is still a useful control pattern that is not captured by either one:

- before executing a jog command, first move the robot to a nearby
  higher-quality configuration
- then execute the actual jog command from that improved configuration

This behavior should not be folded into plain `mj_restore`, because that would
pollute the baseline controller and make comparisons ambiguous.

The new solver therefore exists to make this behavior explicit, optional, and
easy to compare against the plain controller.

## High-Level Architecture

Add a fourth humanoid jog solver mode:

- `vfi`
- `priority`
- `mj_restore`
- `mj_restore_pre_w`

Integration remains intentionally thin:

- `index.html`
- `src/ui/event-handlers.js`
- `src/core/humanoid-jog-solver-presentation.js`
- `src/visualizer.js`

No new global UI toggle is needed. The existing `w` metric selector is reused.

## Control Flow

For supported modes, each jog command follows this sequence:

1. Capture the current joint vector `q0`
2. Capture the current protected pose targets from `q0`
3. Run a metric-only protected `pre-w` stage to obtain `q_pre`
4. Run the normal `mj_restore` jog using `q_pre` as the starting state

If the `pre-w` stage:

- succeeds with a useful update, use `q_pre`
- fails, is rejected, or makes no progress, fall back to `q0`

So the runtime behavior is:

```text
best case:  q0 -> pre-w(q_pre) -> mj_restore(target)
fallback:   q0 ----------------> mj_restore(target)
```

This guarantees that selecting the new solver cannot make the controller lose
the ability to execute an otherwise valid jog command merely because the metric
stage failed.

## Per-Mode Protection Rules

### `absolute`

During `pre-w`:

- keep current absolute pose fixed
- keep current relative pose fixed

Then run the actual absolute jog command with plain `mj_restore`.

### `decoupled`

During `pre-w`:

- keep current absolute pose fixed
- keep current relative pose fixed
- allow waist joints in the active set

Then run the full-body absolute jog command with plain `mj_restore`.

### `decoupled-upright`

During `pre-w`:

- keep current absolute pose fixed
- keep current relative pose fixed
- keep the body-z vertical constraint active
- allow waist joints in the active set

Then run the full-body upright absolute jog command with plain `mj_restore`.

## Unsupported Modes

For:

- `relative`
- `relative-fullbody`
- `distance`
- `distance-fullbody`

the new solver does not attempt `pre-w`.

Instead it uses the rollback baseline for that mode family and reports:

- `pre_w: disabled_for_mode`

Fallback policy:

- `relative` and `relative-fullbody`
  - use the same backend path the current rollback baseline already uses for
    those modes
- `distance` and `distance-fullbody`
  - use the existing plain `mj_restore` distance path

Diagnostic rule:

- diagnostics still identify the selected solver as `mj_restore_pre_w`
- `pre_w: disabled_for_mode` explains why the stage was skipped

This keeps v1 small and avoids introducing half-implemented semantics for mode
families that are not part of the current comparison goal.

## Metric Selection

The new solver uses the existing `w` selector.

Allowed metrics:

- `w_abs_hier`
- `w_abs_hier_vfi`

The metric selector affects only:

- `w+ / w-`
- `mj_restore_pre_w`

It must not affect plain `mj_restore`.

Implementation constraint:

- use a dedicated `pre-w` metric resolver, or an explicit
  `solverMode === 'mj_restore_pre_w'` gate, when reading the selected `w`
  metric for jog control
- plain `mj_restore` must continue to ignore the UI `w` selector unless an
  explicit metric override is passed for a metric-only action

## Pre-w Stage Semantics

The `pre-w` stage is a metric-only protected optimization step. It does not
attempt to execute the user jog command itself.

It should reuse the same protected metric mechanisms already available for the
`w+ / w-` flow under `mj_restore`, but as a non-committing candidate stage,
including:

- selected jog metric override
- mode-aware active joint set
- body-z protection when upright mode is active

For v1, one `pre-w` pass per jog command is sufficient. The goal is explicit
preconditioning, not iterative co-optimization.

Direction policy:

- always run `pre-w` in the `increase` direction
- never inherit or reuse any previous `w+ / w-` button intent

Candidate-only rule:

- the `pre-w` stage may compute and return a candidate `q_pre`
- it may emit mapped status and diagnostics
- it must not commit live humanoid joint state by itself
- only stage 2 may commit the actual joint update

## Failure Handling

The `pre-w` stage may end in one of these states:

- `applied`
- `skipped(no_progress)`
- `skipped(no_gradient)`
- `rejected(...)`
- `failed(...)`
- `disabled_for_mode`

Behavioral rule:

- none of these non-success states may abort the actual jog command

Only the second-stage `mj_restore` result determines whether the jog itself
ultimately succeeded.

Normative mapping from optimizer outcomes:

- usable candidate outcomes
  - map to `pre_w: applied`
  - may produce `q_pre`
- hierarchical partial-success outcomes such as:
  - `active(primary_only; ...)`
  - `active(primary_only; secondary=...)`
  - count as `pre_w: applied` if they return a concrete accepted candidate
  - otherwise they must downgrade to `pre_w: skipped(...)`
- outcomes such as:
  - `skipped(no_progress)`
  - `skipped(no_gradient)`
  - `skipped(projection_only_no_progress)`
  - map to `pre_w: skipped(...)`
  - must not produce `q_pre`
- outcomes such as:
  - `rejected(hard_constraints)`
  - `rejected(...)`
  - map to `pre_w: rejected(...)`
  - must not produce `q_pre`
- thrown errors, async failures, FK/Jacobian failures, or invalid metric setup
  - map to `pre_w: failed(...)`
  - must not produce `q_pre`

Stage-2 start-state rule:

- only `pre_w: applied` may cause stage 2 to start from `q_pre`
- every other `pre_w` state must cause stage 2 to start from the original `q0`

## Diagnostics

Extend the existing humanoid jog diagnostics with explicit `pre-w` fields:

- `pre_w: applied | skipped(...) | rejected(...) | failed(...) | disabled_for_mode`
- `pre_w_metric: w_abs_hier | w_abs_hier_vfi`
- `pre_w_start: q0 | q_pre`

These diagnostics are required so users can distinguish:

- plain `mj_restore`
- `mj_restore_pre_w`
- metric-only `w+ / w-`

without inferring behavior indirectly from CCTSM values.

## UI Behavior

The solver dropdown adds one new option:

- `MuJoCo 风格恢复 + w预优化`

No other new controls are added.

The current `w` selector remains visible and continues to expose only:

- `w_abs_hier`
- `w_abs_hier_vfi`

This keeps the UI simple and makes the new controller easy to compare against
plain `mj_restore`.

## Testing Strategy

Use test-first development.

### Required coverage

1. Solver presentation
- the new solver mode is recognized and normalized correctly
- the display name is correct
- persistence round-trips correctly

2. Dispatch behavior
- `absolute / decoupled / decoupled-upright` run `pre-w` before `mj_restore`
- `relative / relative-fullbody` bypass `pre-w` and use the rollback-baseline
  backend path for those modes
- `distance / distance-fullbody` bypass `pre-w` and use the existing plain
  `mj_restore` distance path
- diagnostics still report selected solver `mj_restore_pre_w` with
  `pre_w: disabled_for_mode`

3. Failure behavior
- failed or skipped `pre-w` does not block the actual jog command
- plain `mj_restore` still does not read the `w` selector

4. Diagnostics
- `pre_w` state is included for the new solver
- `pre_w_metric` matches the selected `w` metric

## Non-Goals

This design intentionally does not:

- merge `w` optimization into plain `mj_restore`
- change the current `w+ / w-` semantics
- add support for pre-w in relative or distance control modes
- introduce a new independent metric family

## Success Criteria

The change is successful when:

- users can select `MuJoCo 风格恢复 + w预优化` in the solver dropdown
- only `absolute / decoupled / decoupled-upright` perform `pre-w`
- the selected `w` metric is reused cleanly
- plain `mj_restore` remains unaffected
- failed `pre-w` never prevents the jog command from running
- diagnostics make the pre-w stage explicit
