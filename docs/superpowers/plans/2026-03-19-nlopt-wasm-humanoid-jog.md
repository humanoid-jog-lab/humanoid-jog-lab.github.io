# NLopt WASM Humanoid Jog Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a third experimental humanoid jog solver, `NLopt (实验)`, that runs in-browser through the existing WASM pipeline and supports the humanoid dual-arm `absolute-target` path with `waist=off` and `body_z=off`.

**Architecture:** Vendor `NLopt` into the existing C++/WASM build, expose one focused Embind wrapper for the supported humanoid absolute-target problem, then add a small JS adapter and UI wiring so the new solver coexists with `new qpJ` and `零空间投影`. Keep the first version narrow: unsupported modes must fall back cleanly to the current `new qpJ` path with explicit diagnostics.

**Tech Stack:** C++, NLopt, Emscripten, Embind, CMake, existing DQ Robotics WASM module, vanilla ES modules, Node test runner, Vite

---

## File Map

**Create:**

- `docs/superpowers/plans/2026-03-19-nlopt-wasm-humanoid-jog.md`
- `wasm/nlopt_humanoid_solver.hpp`
- `wasm/nlopt_humanoid_solver.cpp`
- `src/core/humanoid-jog-nlopt.js`
- `src/core/humanoid-jog-nlopt.test.js`

**Modify:**

- `package.json`
- `package-lock.json`
- `build.sh`
- `wasm/CMakeLists.txt`
- `wasm/bindings.cpp`
- `index.html`
- `src/ui/event-handlers.js`
- `src/core/humanoid-jog-solver-presentation.js`
- `src/core/humanoid-jog-solver-presentation.test.js`
- `src/visualizer.js`

**Verify With Existing Tests:**

- `src/core/vfi-utils.test.js`
- `src/core/humanoid-jog-diagnostics.test.js`
- any existing humanoid jog tests touched by the adapter wiring

## Chunk 1: Vendor NLopt And Extend The WASM Build

### Task 1: Add a dependency entry and build notes for NLopt

**Files:**

- Modify: `package.json`
- Modify: `build.sh`

- [ ] **Step 1: Write the failing smoke check command into the plan notes**

Run:

```bash
test -d third_party/nlopt
```

Expected:

```text
exit code 1
```

- [ ] **Step 2: Add the vendor/build expectations**

Update `package.json` comments/scripts only if needed for developer discoverability, and update `build.sh` so the script fails clearly when `third_party/nlopt` is missing instead of producing a vague downstream CMake error.

Use a small shell check pattern such as:

```bash
if [ ! -d "./third_party/nlopt" ]; then
    echo -e "${RED}错误: NLopt 源码不存在: ./third_party/nlopt${NC}"
    exit 1
fi
```

- [ ] **Step 3: Verify the shell/build script is still syntactically valid**

Run:

```bash
bash -n build.sh
```

Expected:

```text
exit code 0
```

- [ ] **Step 4: Commit**

```bash
git add package.json build.sh
git commit -m "build: prepare WASM pipeline for vendored NLopt"
```

### Task 2: Link NLopt into the existing Emscripten target

**Files:**

- Modify: `wasm/CMakeLists.txt`

- [ ] **Step 1: Write a failing configure/build reproduction note**

Run:

```bash
npm run build:wasm
```

Expected before implementation:

```text
configure/build does not know about NLopt yet
```

- [ ] **Step 2: Add NLopt to the CMake graph**

Follow the existing third-party style already used for TOPPRA in `wasm/CMakeLists.txt`.

Add a focused section that:

- defines `NLOPT_DIR`
- checks for the vendored source
- adds the library with `add_subdirectory(...)` or a small static target
- links it to `dqrobotics`
- exposes a compile definition such as:

```cmake
target_compile_definitions(dqrobotics PRIVATE NLOPT_AVAILABLE=1)
```

and an `OFF` path when the source is unavailable:

```cmake
target_compile_definitions(dqrobotics PRIVATE NLOPT_AVAILABLE=0)
```

- [ ] **Step 3: Re-run the configure/build command**

Run:

```bash
npm run build:wasm
```

Expected:

```text
configure step recognizes the NLopt section and either builds successfully or fails in the new code path for a real NLopt-specific reason
```

- [ ] **Step 4: Commit**

```bash
git add wasm/CMakeLists.txt
git commit -m "build: link NLopt into dqrobotics wasm target"
```

## Chunk 2: Add A Focused C++ NLopt Humanoid Solver Wrapper

### Task 3: Create a small C++ solver unit outside `bindings.cpp`

**Files:**

- Create: `wasm/nlopt_humanoid_solver.hpp`
- Create: `wasm/nlopt_humanoid_solver.cpp`
- Modify: `wasm/CMakeLists.txt`

- [ ] **Step 1: Write the failing interface-first unit shell**

Add a header that declares a compact request/result API rather than placing the whole implementation into `bindings.cpp`.

Use a shape like:

```cpp
struct HumanoidAbsoluteNloptRequest {
    std::vector<double> q0;
    std::vector<double> absolute_target;
    std::vector<double> relative_reference;
    std::vector<int> active_indices;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;
    std::vector<double> step_bounds;
    double wvfi_min = 5e-4;
    double weight_absolute = 1.0;
    double weight_relative = 100.0;
    double weight_delta = 1e-3;
    int max_eval = 80;
    double xtol_rel = 1e-6;
    double ftol_rel = 1e-8;
};

struct HumanoidAbsoluteNloptResult {
    bool converged = false;
    std::string reason;
    int iterations = 0;
    double final_cost = std::numeric_limits<double>::infinity();
    double final_absolute_residual = std::numeric_limits<double>::infinity();
    double final_relative_residual = std::numeric_limits<double>::infinity();
    double final_wvfi = 0.0;
    std::vector<double> q_next;
};
```

- [ ] **Step 2: Implement the minimal solver shell**

In `wasm/nlopt_humanoid_solver.cpp`, implement a first-pass solver wrapper that:

- validates request sizes
- creates an `nlopt::opt` using `LD_SLSQP`
- sets lower and upper bounds derived from both joint limits and `q0 +/- step_bounds`
- installs an objective callback
- returns a structured failure if unsupported data is missing

Do not optimize prematurely. Keep all helper functions file-local.

- [ ] **Step 3: Register the new compilation unit**

Add `wasm/nlopt_humanoid_solver.cpp` to the `dqrobotics` executable sources in `wasm/CMakeLists.txt`.

- [ ] **Step 4: Rebuild the WASM target**

Run:

```bash
npm run build:wasm
```

Expected:

```text
the new C++ files compile and link into the module
```

- [ ] **Step 5: Commit**

```bash
git add wasm/CMakeLists.txt wasm/nlopt_humanoid_solver.hpp wasm/nlopt_humanoid_solver.cpp
git commit -m "feat: add NLopt humanoid solver core for wasm"
```

### Task 4: Implement the first supported nonlinear objective and constraints

**Files:**

- Modify: `wasm/nlopt_humanoid_solver.cpp`
- Modify: `wasm/nlopt_humanoid_solver.hpp`

- [ ] **Step 1: Add a failing C++ smoke path through the wrapper**

Implement temporary result messages such as:

```cpp
result.reason = "not_implemented";
return result;
```

and verify the JS adapter can observe this before finishing the real implementation.

- [ ] **Step 2: Implement the objective**

Encode:

```text
wa * ||e_abs(q)||^2 + wr * ||e_rel(q)||^2 + wd * ||q - q0||^2
```

Use the existing humanoid kinematics available in the DQ Robotics module already linked into the same WASM target. Keep all residual computations inside C++.

- [ ] **Step 3: Implement hard bounds**

Intersect:

- joint lower/upper limits
- `q0 +/- step_bounds`

before passing them to NLopt.

- [ ] **Step 4: Implement the `w_vfi` inequality**

Add a nonlinear inequality callback enforcing:

```text
w_vfi_min - w_vfi(q) <= 0
```

If the exact current JS-side `w_vfi` helper cannot be mirrored immediately, stop and port the minimal math needed into the C++ unit instead of calling back into JS every iteration.

- [ ] **Step 5: Implement result extraction**

Populate:

- `converged`
- `reason`
- `iterations`
- `final_cost`
- `final_absolute_residual`
- `final_relative_residual`
- `final_wvfi`
- `q_next`

- [ ] **Step 6: Rebuild**

Run:

```bash
npm run build:wasm
```

Expected:

```text
successful WASM build with the NLopt solver logic linked
```

- [ ] **Step 7: Commit**

```bash
git add wasm/nlopt_humanoid_solver.hpp wasm/nlopt_humanoid_solver.cpp
git commit -m "feat: implement experimental NLopt humanoid objective"
```

### Task 5: Bind the solver through Embind

**Files:**

- Modify: `wasm/bindings.cpp`

- [ ] **Step 1: Write the failing interface expectation**

Document the JS-facing method name and payload shape first:

```cpp
class_<NloptHumanoidWrapper>("NloptHumanoidWrapper")
    .class_function("solveHumanoidAbsoluteTarget", &NloptHumanoidWrapper::solveHumanoidAbsoluteTarget);
```

- [ ] **Step 2: Implement a minimal Embind wrapper**

Translate between JS arrays/objects and the C++ request/result structs from `nlopt_humanoid_solver.*`.

Keep `bindings.cpp` thin:

- parse request
- call the solver
- serialize result

- [ ] **Step 3: Rebuild and smoke-check the generated module**

Run:

```bash
npm run build:wasm
```

Expected:

```text
public/wasm output is regenerated without binding errors
```

- [ ] **Step 4: Commit**

```bash
git add wasm/bindings.cpp
git commit -m "feat: expose experimental NLopt humanoid solver to JS"
```

## Chunk 3: Add A Small JS Adapter And Wire It Into The UI

### Task 6: Add the JS adapter layer

**Files:**

- Create: `src/core/humanoid-jog-nlopt.js`
- Test: `src/core/humanoid-jog-nlopt.test.js`

- [ ] **Step 1: Write the failing tests**

Cover:

- supported-scope detection
- unsupported-scope fallback decision
- successful result normalization
- unavailable-WASM error handling

Use Node tests similar to the existing `src/core/*.test.js` style.

Start with concrete cases like:

```js
test('isHumanoidNloptSupported only accepts absolute-target without waist/body_z', () => {
    assert.equal(isHumanoidNloptSupported({
        mode: 'absolute-target',
        includeWaist: false,
        keepBodyZVertical: false
    }), true);
    assert.equal(isHumanoidNloptSupported({
        mode: 'relative-target',
        includeWaist: false,
        keepBodyZVertical: false
    }), false);
});
```

- [ ] **Step 2: Run the focused tests to confirm failure**

Run:

```bash
node --test src/core/humanoid-jog-nlopt.test.js
```

Expected:

```text
FAIL because the adapter file/functions do not exist yet
```

- [ ] **Step 3: Implement the minimal adapter**

Create a focused module that exports functions like:

```js
export function isHumanoidNloptSupported(options) { ... }
export function buildHumanoidNloptRequest(input) { ... }
export function normalizeHumanoidNloptResult(result, fallback = {}) { ... }
```

Keep this file free of DOM work.

- [ ] **Step 4: Re-run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-nlopt.test.js
```

Expected:

```text
PASS
```

- [ ] **Step 5: Commit**

```bash
git add src/core/humanoid-jog-nlopt.js src/core/humanoid-jog-nlopt.test.js
git commit -m "feat: add humanoid NLopt JS adapter"
```

### Task 7: Add the third solver to presentation and UI state

**Files:**

- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/core/humanoid-jog-solver-presentation.js`
- Modify: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Write the failing presentation tests**

Extend `src/core/humanoid-jog-solver-presentation.test.js` with:

```js
assert.equal(getHumanoidJogSolverDisplayName('nlopt'), 'NLopt (实验)');
assert.equal(getHumanoidJogSolverReadyMessage('nlopt'), 'NLopt (实验) 已启用，等待点动输入。');
```

- [ ] **Step 2: Run the focused presentation tests**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected:

```text
FAIL for the missing nlopt presentation branch
```

- [ ] **Step 3: Implement the UI wiring**

Update:

- `index.html` to add `<option value="nlopt">NLopt (实验)</option>`
- `src/ui/event-handlers.js` so the saved mode recognizes `nlopt`
- `src/core/humanoid-jog-solver-presentation.js` so display text is centralized

- [ ] **Step 4: Re-run the focused tests**

Run:

```bash
node --test src/core/humanoid-jog-solver-presentation.test.js
```

Expected:

```text
PASS
```

- [ ] **Step 5: Commit**

```bash
git add index.html src/ui/event-handlers.js src/core/humanoid-jog-solver-presentation.js src/core/humanoid-jog-solver-presentation.test.js
git commit -m "feat: expose NLopt experimental humanoid jog solver"
```

### Task 8: Wire the experimental solver into the humanoid absolute-target path

**Files:**

- Modify: `src/visualizer.js`
- Modify: `src/core/humanoid-jog-diagnostics.js` if needed
- Test: `src/core/humanoid-jog-nlopt.test.js`

- [ ] **Step 1: Write one failing adapter/distribution test**

Add a JS-side test that exercises the unsupported-mode branch and expected fallback metadata.

If a focused unit test is impractical, add a pure helper test around a new dispatcher function instead of trying to unit-test the entire `visualizer.js`.

- [ ] **Step 2: Add a narrow dispatcher helper**

In `src/visualizer.js`, route only this combination to the experimental solver:

```text
solver mode = nlopt
mode = absolute-target
includeWaist = false
keepBodyZVertical = false
```

Everything else must:

- fall back to `new qpJ`
- set diagnostics that the experimental solver is out of scope

- [ ] **Step 3: Call the WASM wrapper**

Marshal:

- current humanoid q
- absolute target
- relative reference
- active indices
- limits
- step bounds
- current `w_vfi` lower bound

Apply the result only when:

- result is finite
- result indicates convergence or acceptable practical success

Otherwise preserve the current state and set a clear failure payload.

- [ ] **Step 4: Surface diagnostics**

Make the diagnostics clearly show:

- solver: `NLopt (实验)`
- mode: `absolute-target`
- whether the result is real NLopt output or fallback-to-`new qpJ`

- [ ] **Step 5: Run the focused JS tests**

Run:

```bash
node --test src/core/humanoid-jog-nlopt.test.js src/core/humanoid-jog-solver-presentation.test.js
```

Expected:

```text
PASS
```

- [ ] **Step 6: Commit**

```bash
git add src/visualizer.js src/core/humanoid-jog-nlopt.js src/core/humanoid-jog-nlopt.test.js src/core/humanoid-jog-diagnostics.js
git commit -m "feat: wire experimental NLopt solver into humanoid jog"
```

## Chunk 4: Verify The End-To-End Path And Baseline Safety

### Task 9: Run focused verification

**Files:**

- Verify: `src/core/humanoid-jog-nlopt.test.js`
- Verify: `src/core/humanoid-jog-solver-presentation.test.js`
- Verify: any touched humanoid jog diagnostics tests

- [ ] **Step 1: Run the focused Node tests**

Run:

```bash
node --test \
  src/core/humanoid-jog-nlopt.test.js \
  src/core/humanoid-jog-solver-presentation.test.js \
  src/core/humanoid-jog-diagnostics.test.js
```

Expected:

```text
PASS
```

- [ ] **Step 2: Run syntax checks on touched JS files**

Run:

```bash
node --check src/core/humanoid-jog-nlopt.js
node --check src/ui/event-handlers.js
node --check src/visualizer.js
```

Expected:

```text
exit code 0 for each file
```

- [ ] **Step 3: Rebuild the app**

Run:

```bash
npm run build
```

Expected:

```text
successful Vite build
```

- [ ] **Step 4: Rebuild the WASM module**

Run:

```bash
npm run build:wasm
```

Expected:

```text
successful WASM build
```

- [ ] **Step 5: Commit**

```bash
git add .
git commit -m "test: verify experimental NLopt humanoid jog integration"
```

### Task 10: Manual browser validation against a known problematic pose

**Files:**

- Verify runtime behavior through the existing app

- [ ] **Step 1: Launch the dev app**

Run:

```bash
npm run dev
```

Expected:

```text
Vite dev server starts successfully
```

- [ ] **Step 2: Validate solver selection**

In the UI:

- switch between `new qpJ`, `零空间投影`, and `NLopt (实验)`
- confirm the diagnostics panel updates solver identity correctly

- [ ] **Step 3: Validate supported scope**

In humanoid dual-arm `absolute-target`, with `waist=off` and `body_z=off`:

- run one previously problematic pose for `new qpJ`
- compare `new qpJ` vs `NLopt (实验)`
- record whether the experimental solver finds a usable move

- [ ] **Step 4: Validate unsupported fallback**

Toggle into an unsupported case such as:

- `waist=on`
- or `body_z=on`
- or a non-absolute humanoid jog mode

Expected:

- the app falls back to `new qpJ`
- diagnostics explicitly say the experimental solver is out of scope

- [ ] **Step 5: Commit any diagnostic-only adjustments**

```bash
git add index.html src/ui/event-handlers.js src/visualizer.js src/core/humanoid-jog-diagnostics.js
git commit -m "fix: clarify NLopt experimental solver diagnostics"
```

## Notes For Execution

- Use `@superpowers:test-driven-development` before implementing each new helper or wrapper.
- Use `@superpowers:systematic-debugging` before changing solver behavior if the first NLopt pass produces unstable or misleading results.
- Use `@superpowers:verification-before-completion` before declaring the integration done.
- Do not expand the supported mode matrix during this plan. If `waist=on` or `body_z=on` becomes tempting, write a separate spec and plan.

## Hand-off

Plan complete and saved to `docs/superpowers/plans/2026-03-19-nlopt-wasm-humanoid-jog.md`. Ready to execute?
