# Remove NLopt Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove all remaining NLopt integration from `dq_vizlab_academic` without breaking the default non-NLopt humanoid WASM build.

**Architecture:** The work deletes the vendored NLopt tree and the dedicated humanoid solver implementation, then simplifies the build graph and bindings so only supported solvers remain. Source-level regression tests guard against NLopt references returning through future edits.

**Tech Stack:** Node.js test runner, shell build tooling, CMake, Emscripten, apply_patch

---

### Task 1: Lock in the expected post-removal state

**Files:**
- Create: `src/core/nlopt-removal.test.js`
- Test: `src/core/nlopt-removal.test.js`

- [ ] **Step 1: Write the failing regression test**

The test should assert that:
- `third_party/nlopt/` is absent
- `wasm/nlopt_humanoid_solver.cpp` and `.hpp` are absent
- `build.sh`, `package.json`, `wasm/CMakeLists.txt`, and `wasm/bindings.cpp` contain no `nlopt` references

- [ ] **Step 2: Run the test and confirm RED**

Run: `node --test src/core/nlopt-removal.test.js`
Expected: FAIL because the vendor directory and build entrypoints still exist.

### Task 2: Remove source and build integration

**Files:**
- Delete: `third_party/nlopt/**`
- Delete: `wasm/nlopt_humanoid_solver.cpp`
- Delete: `wasm/nlopt_humanoid_solver.hpp`
- Modify: `build.sh`
- Modify: `package.json`
- Modify: `wasm/CMakeLists.txt`
- Modify: `wasm/bindings.cpp`
- Modify: `src/core/humanoid-jog-solver-presentation.test.js`

- [ ] **Step 1: Delete NLopt-owned source trees**

Remove the vendored library and dedicated humanoid solver files.

- [ ] **Step 2: Remove build toggles**

Strip NLopt-specific environment checks and npm scripts from `build.sh` and `package.json`.

- [ ] **Step 3: Remove CMake and Embind entrypoints**

Delete NLopt discovery, linkage, compile definitions, and JS binding exposure.

- [ ] **Step 4: Drop NLopt-specific compatibility assertions**

Update tests so they only describe the still-supported solver ids.

### Task 3: Verify green without touching generated install artifacts

**Files:**
- Verify: `src/core/nlopt-removal.test.js`
- Verify: `src/core/humanoid-jog-solver-presentation.test.js`
- Verify: `src/ui/humanoid-manip-controls.test.js`

- [ ] **Step 1: Re-run regression tests**

Run: `node --test src/core/nlopt-removal.test.js src/core/humanoid-jog-solver-presentation.test.js src/ui/humanoid-manip-controls.test.js`
Expected: PASS.

- [ ] **Step 2: Compile WASM without install**

Run a fresh `emcmake cmake` plus `emmake make` in `build/`, but do not run `make install`.
Expected: build succeeds and does not overwrite `public/wasm/dqrobotics.js`.

- [ ] **Step 3: Inspect worktree**

Run: `git status --short`
Expected: NLopt deletions and targeted source edits, plus the pre-existing dirty generated artifact if still present.
