# Remove `python_sdk` Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the deprecated `python_sdk/` subtree from `dq_vizlab_academic` without changing the main app or unrelated Python tooling.

**Architecture:** This is a deletion-only change. The work removes the isolated SDK subtree, keeps the rest of the repository untouched, and verifies that no direct references remain outside the deleted path.

**Tech Stack:** Git, ripgrep, apply_patch

---

### Task 1: Confirm deletion boundary

**Files:**
- Modify: `docs/superpowers/specs/2026-04-14-remove-python-sdk-design.md`
- Verify: repository-wide search results

- [ ] **Step 1: Search for direct references**

Run: `rg -n "python_sdk" .`
Expected: Only results inside `python_sdk/` or documentation written for this removal.

- [ ] **Step 2: Confirm the rest of the app does not depend on it**

Run: `sed -n '1,220p' package.json`
Expected: No scripts or dependencies require `python_sdk/`.

### Task 2: Remove the subtree

**Files:**
- Delete: `python_sdk/**`

- [ ] **Step 1: Delete tracked SDK files**

Use `apply_patch` delete hunks for every tracked file under `python_sdk/`.

- [ ] **Step 2: Remove empty directories**

Run: `find python_sdk -depth -type d -empty -delete`
Expected: `python_sdk/` disappears entirely if nothing remains.

### Task 3: Verify the result

**Files:**
- Verify: `git status`, repository-wide search results

- [ ] **Step 1: Search again for references**

Run: `rg -n "python_sdk" .`
Expected: Only the new design/plan docs mention it.

- [ ] **Step 2: Inspect worktree changes**

Run: `git status --short`
Expected: Deleted `python_sdk/` files plus any unrelated pre-existing user changes.

- [ ] **Step 3: Do not commit automatically**

The repository already contains unrelated user worktree changes, so leave commit control with the user.
