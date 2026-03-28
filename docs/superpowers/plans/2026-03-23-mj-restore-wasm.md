# MuJoCo-Style Restore WASM Offload Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offload the `mj_restore` humanoid `absolute-target` restoration inner loop from JavaScript to the existing DQ Robotics WASM module while preserving current nullspace metric behavior in JS.

**Architecture:** Add a dedicated C++ WASM solver for reference restoration, expose it through `HumanoidURDF_Wrapper`, then switch the JS `mj_restore` path to prefer the WASM method and fall back to the current JS restoration loop when unavailable.

**Tech Stack:** JavaScript, C++17, Emscripten/Embind, DQ Robotics humanoid cooperative APIs, Node test runner, Vite, existing WASM build pipeline.

---

## Chunk 1: WASM Solver Core

### Task 1: Add dedicated C++ `mj_restore` solver files

**Files:**
- Create: `wasm/mj_restore_humanoid_solver.hpp`
- Create: `wasm/mj_restore_humanoid_solver.cpp`
- Modify: `wasm/CMakeLists.txt`

- [ ] **Step 1: Write the failing JS-side smoke test for missing WASM method expectation**
- [ ] **Step 2: Add the new C++ solver request/result structs**
- [ ] **Step 3: Implement cooperative absolute/relative/body-z evaluation and reference restoration loop**
- [ ] **Step 4: Add the new C++ source to `wasm/CMakeLists.txt`**
- [ ] **Step 5: Run `npm run build:wasm`**

## Chunk 2: Embind Surface

### Task 2: Expose the new solver through `HumanoidURDF_Wrapper`

**Files:**
- Modify: `wasm/bindings.cpp`

- [ ] **Step 1: Add request parsing helpers**
- [ ] **Step 2: Add `solveAbsoluteTargetMJRestore(...)` wrapper method**
- [ ] **Step 3: Return a JS object compatible with current diagnostics needs**
- [ ] **Step 4: Run `npm run build:wasm`**

## Chunk 3: JS Bridge

### Task 3: Prefer WASM restoration in `mj_restore`

**Files:**
- Modify: `src/visualizer.js`
- Create: `src/core/humanoid-jog-mj-wasm.test.js`

- [ ] **Step 1: Write failing tests for “prefer WASM, fallback to JS” behavior**
- [ ] **Step 2: Add request assembly for `solveAbsoluteTargetMJRestore(...)`**
- [ ] **Step 3: Normalize the WASM result into the existing `mj_restore` flow**
- [ ] **Step 4: Preserve current nullspace metric refine after restoration convergence**
- [ ] **Step 5: Run the focused JS tests**

## Chunk 4: Verification

### Task 4: Verify runtime integration

**Files:**
- Modify if needed: `src/core/humanoid-jog-mj-diagnostics.js`

- [ ] **Step 1: Run targeted JS tests**
- [ ] **Step 2: Run `node --check` on touched JS files**
- [ ] **Step 3: Run `npm run build:wasm`**
- [ ] **Step 4: Run `npm run build`**
- [ ] **Step 5: If needed, smoke-test one `mj_restore` absolute case in the browser-facing runtime**
