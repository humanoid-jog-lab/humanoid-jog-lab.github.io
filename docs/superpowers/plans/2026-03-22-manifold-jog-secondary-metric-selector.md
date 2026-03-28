# Manifold Jog Secondary Metric Selector Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `点动伴随优化` selector for the `约束流形` humanoid jog solver so normal jog buttons can optionally run a protected nullspace refinement using a selected secondary metric.

**Architecture:** Keep manifold jog semantics restoration-first. After a jog step converges back to the protected equality manifold, resolve the selected metric through a focused helper and optionally run the existing nullspace refinement path with that metric. Leave dedicated `w_*` and `sigma` buttons unchanged.

**Tech Stack:** Vanilla ES modules, existing humanoid manifold solver path, Node test runner, Vite, existing manipulability helpers and diagnostics pipeline

---

## File Map

**Create:**

- `docs/superpowers/plans/2026-03-22-manifold-jog-secondary-metric-selector.md`
- `src/core/humanoid-jog-manifold-jog-metric.js`
- `src/core/humanoid-jog-manifold-jog-metric.test.js`

**Modify:**

- `index.html`
- `src/ui/event-handlers.js`
- `src/ui/humanoid-manip-controls.test.js`
- `src/core/humanoid-jog-manifold-config.test.js`
- `src/core/humanoid-jog-manifold-diagnostics.js`
- `src/core/humanoid-jog-manifold-diagnostics.test.js`
- `src/visualizer.js`

**Reference Existing Helpers:**

- `src/core/humanoid-jog-manifold-nullspace.js`
- `src/core/humanoid-jog-directional-manipulability.js`
- `src/core/humanoid-jog-manifold-objective-direction.js`
- `src/core/humanoid-jog-solver-presentation.js`

## Chunk 1: Add The Jog Secondary Metric UI

### Task 1: Add selector UI and persistence wiring

**Files:**

- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Test: `src/ui/humanoid-manip-controls.test.js`
- Test: `src/core/humanoid-jog-manifold-config.test.js`

- [ ] **Step 1: Write the failing UI test**

Add assertions that the humanoid controls render a `点动伴随优化` selector containing:

- `none`
- `w_vfi`
- `w_cmd`
- `w_abs`
- `w_abs_arm`
- `w_abs_vfi`
- `w_abs_hier`
- `w_abs_hier_vfi`
- `sigma`

- [ ] **Step 2: Run the focused UI test and verify failure**

Run:

```bash
node --test src/ui/humanoid-manip-controls.test.js
```

Expected: FAIL because the selector is not rendered yet.

- [ ] **Step 3: Write the failing config persistence test**

Add assertions that the default manifold jog metric normalizes to `none` and survives the same persistence path used for humanoid jog controls.

- [ ] **Step 4: Run the config test and verify failure**

Run:

```bash
node --test src/core/humanoid-jog-manifold-config.test.js
```

Expected: FAIL because the config field does not exist yet.

- [ ] **Step 5: Add the selector markup**

Update `index.html` to render the `点动伴随优化` selector in the manifold-related jog controls area, following the existing control naming and layout patterns.

- [ ] **Step 6: Add state hydration and persistence**

Update `src/ui/event-handlers.js` so the selector:

- reads from persisted UI state
- writes back on change
- keeps existing non-manifold controls untouched

- [ ] **Step 7: Add the default config value**

Update `src/core/humanoid-jog-manifold-config.test.js` coverage target and the corresponding config source so the new default is `none`.

- [ ] **Step 8: Re-run the focused tests**

Run:

```bash
node --test src/ui/humanoid-manip-controls.test.js src/core/humanoid-jog-manifold-config.test.js
```

Expected: PASS

## Chunk 2: Build The Metric Resolver

### Task 2: Add a dedicated manifold jog metric resolver helper

**Files:**

- Create: `src/core/humanoid-jog-manifold-jog-metric.js`
- Test: `src/core/humanoid-jog-manifold-jog-metric.test.js`

- [ ] **Step 1: Write the failing resolver test**

Add tests covering:

- normalization of every supported selector value
- direct metrics (`w_vfi`, `w_cmd`, `w_abs`, `w_abs_arm`, `w_abs_vfi`)
- hierarchical metrics (`w_abs_hier`, `w_abs_hier_vfi`)
- directional metric (`sigma`)
- `none` returning a skipped refinement descriptor

- [ ] **Step 2: Run the resolver test and verify failure**

Run:

```bash
node --test src/core/humanoid-jog-manifold-jog-metric.test.js
```

Expected: FAIL because the helper does not exist yet.

- [ ] **Step 3: Implement normalization and resolution**

Create `src/core/humanoid-jog-manifold-jog-metric.js` with helpers that:

- normalize the selector value
- expose whether the selected metric should skip refinement
- return the metric family (`direct`, `hierarchical`, `directional`, `none`)
- provide the metric key used for before/after diagnostics

- [ ] **Step 4: Reuse existing semantics without duplicating metric math**

Wire the helper so it points at the same metric names already used by the dedicated optimization button paths instead of creating new metric definitions.

- [ ] **Step 5: Re-run the resolver test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-jog-metric.test.js
```

Expected: PASS

## Chunk 3: Thread The Selector Through Manifold Jog

### Task 3: Apply the selected metric inside the manifold jog path

**Files:**

- Modify: `src/visualizer.js`
- Test: `src/core/humanoid-jog-manifold-diagnostics.test.js`

- [ ] **Step 1: Write the failing behavior test**

Add a focused diagnostics-level test that exercises manifold jog with:

- `none`
- one direct metric (`w_vfi`)
- one hierarchical metric (`w_abs_hier`)
- one directional metric (`sigma`)

and asserts the emitted diagnostics label the selected jog metric and whether refinement was skipped or active.

- [ ] **Step 2: Run the diagnostics test and verify failure**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js
```

Expected: FAIL because manifold jog diagnostics do not include jog-metric state yet.

- [ ] **Step 3: Read the selected selector value inside manifold jog**

Update the manifold jog path in `src/visualizer.js` so the restoration flow reads `manifoldJogSecondaryMetric` from config/UI state.

- [ ] **Step 4: Preserve pure-restoration behavior for `none`**

Keep the existing restoration-only path when the selector resolves to `none`, and explicitly mark the refinement state as skipped.

- [ ] **Step 5: Route direct metrics into nullspace refinement**

When the selector resolves to a direct metric, reuse the existing nullspace refinement machinery with the corresponding metric evaluator and direction guard.

- [ ] **Step 6: Route hierarchical metrics through their existing guarded paths**

For `w_abs_hier` and `w_abs_hier_vfi`, reuse the same primary/secondary plus arm-guard semantics already implemented for the dedicated buttons.

- [ ] **Step 7: Route `sigma` through the existing last-axis semantics**

Require the manifold jog path to pass the last commanded jog axis so `sigma` keeps the same directional meaning as the standalone button.

- [ ] **Step 8: Re-run the diagnostics test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js
```

Expected: PASS

## Chunk 4: Expand Diagnostics And Verify Integration

### Task 4: Add explicit jog-metric diagnostics and finish verification

**Files:**

- Modify: `src/core/humanoid-jog-manifold-diagnostics.js`
- Modify: `src/core/humanoid-jog-manifold-diagnostics.test.js`
- Modify: `src/visualizer.js`

- [ ] **Step 1: Write the failing diagnostics-format test**

Add assertions that manifold jog diagnostics include:

- `jog_metric=<selected>`
- `jog_metric_opt=<state>`
- `jog_metric_before=<value>`
- `jog_metric_after=<value>`
- `jog_metric_applied=yes/no`

with `none` producing `skipped(none)` and active metrics producing `active` plus before/after values.

- [ ] **Step 2: Run the diagnostics-format test and verify failure**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js
```

Expected: FAIL because the formatter does not render these fields yet.

- [ ] **Step 3: Implement the formatter update**

Update `src/core/humanoid-jog-manifold-diagnostics.js` so manifold jog diagnostics render the new jog-metric fields without changing the dedicated optimization diagnostics.

- [ ] **Step 4: Thread final diagnostic payloads from the solver path**

Update `src/visualizer.js` so the manifold jog result object always carries:

- selected jog metric
- optimization state
- before/after metric values
- applied flag

- [ ] **Step 5: Re-run the focused diagnostics test**

Run:

```bash
node --test src/core/humanoid-jog-manifold-diagnostics.test.js
```

Expected: PASS

- [ ] **Step 6: Run the integration test set**

Run:

```bash
node --test src/core/humanoid-jog-manifold-jog-metric.test.js src/core/humanoid-jog-manifold-config.test.js src/core/humanoid-jog-manifold-diagnostics.test.js src/ui/humanoid-manip-controls.test.js
```

Expected: PASS

- [ ] **Step 7: Run syntax and build verification**

Run:

```bash
node --check src/visualizer.js
node --check src/ui/event-handlers.js
npm run build
```

Expected: PASS (existing `numeric` warning only if unchanged)

- [ ] **Step 8: Commit**

```bash
git add docs/superpowers/plans/2026-03-22-manifold-jog-secondary-metric-selector.md \
  index.html \
  src/ui/event-handlers.js \
  src/ui/humanoid-manip-controls.test.js \
  src/core/humanoid-jog-manifold-config.test.js \
  src/core/humanoid-jog-manifold-jog-metric.js \
  src/core/humanoid-jog-manifold-jog-metric.test.js \
  src/core/humanoid-jog-manifold-diagnostics.js \
  src/core/humanoid-jog-manifold-diagnostics.test.js \
  src/visualizer.js
git commit -m "feat: add manifold jog secondary metric selector"
```
