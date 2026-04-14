# Remove `python_sdk` Design

## Goal

Remove the standalone `python_sdk/` subtree from `dq_vizlab_academic` while leaving the main web visualizer, WASM build flow, and existing backend scripts unchanged.

## Scope

- Delete `python_sdk/` and everything below it.
- Keep non-SDK Python usage such as `server/ros2-backend.py` and shell scripts unchanged.
- Clean only direct repository references if any are found.

## Design

`python_sdk/` is currently isolated from the rest of the repository. A repository-wide search found no direct references from `package.json`, top-level build scripts, or other source files outside the subtree itself. That means the safest and simplest design is full removal of the subtree with no wider refactor.

## Verification

- Search the repository for remaining `python_sdk` references outside the removed directory.
- Inspect `git status` to confirm the intended deletions and avoid touching unrelated existing worktree changes.
