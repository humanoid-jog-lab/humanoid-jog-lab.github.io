# Remove NLopt Design

## Goal

Remove NLopt completely from `dq_vizlab_academic`, including vendored source, WASM solver entrypoints, build switches, and NLopt-specific regression assertions, while keeping the remaining humanoid solvers and the default WASM build path intact.

## Scope

- Delete `third_party/nlopt/`.
- Delete `wasm/nlopt_humanoid_solver.cpp` and `wasm/nlopt_humanoid_solver.hpp`.
- Remove NLopt-specific CMake and shell build logic.
- Remove the Embind entrypoint for `solveAbsoluteTargetNlopt`.
- Remove test assertions that explicitly preserve `nlopt` as a historical solver label.

## Design

NLopt is isolated behind a dedicated WASM solver implementation plus optional build wiring in `build.sh`, `package.json`, and `wasm/CMakeLists.txt`. No active UI option depends on NLopt anymore, and the current UI tests already assert that NLopt is not exposed to users. The cleanest design is therefore full deletion of the remaining NLopt integration points rather than leaving dead optional paths in place.

To keep the removal safe, verification should happen at two levels:

- source-level regression tests that assert the vendor directory and NLopt entrypoints no longer exist
- a non-installing WASM compile that proves the remaining CMake graph still builds without overwriting the already-dirty generated artifact in `public/wasm/dqrobotics.js`

## Verification

- `node --test src/core/nlopt-removal.test.js`
- `node --test src/core/humanoid-jog-solver-presentation.test.js src/ui/humanoid-manip-controls.test.js`
- non-installing Emscripten build from `build/` only, skipping `make install`
