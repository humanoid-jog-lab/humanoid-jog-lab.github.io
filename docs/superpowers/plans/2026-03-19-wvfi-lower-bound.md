# Wvfi Lower Bound Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enforce a real post-step `w_vfi` lower bound across humanoid VFI step backends, with stricter floor when waist joints participate.

**Architecture:** Add a shared `w_vfi` guard that evaluates the exact post-step CCTSM result, backtracks the candidate step until it satisfies the configured lower bound, and reports whether the guard accepted, scaled, or rejected the step. Wire it into the shared VFI stepping paths so `qp`, `projection`, and predictive-cert branch-local steps all honor the same rule.

**Tech Stack:** Vanilla ES modules, existing CCTSM helpers, Node test runner

---

## Chunk 1: Shared Guard Helper

### Task 1: Add red tests for the guard behavior

- [ ] Write failing tests for accept / scale / reject / threshold selection behavior

- [ ] Run `node --test ...` to verify the new tests fail for the right reason

- [ ] Implement the smallest helper API to satisfy the tests

- [ ] Re-run the focused tests until they pass


### Task 2: Keep the helper file focused

- [ ] Put the backtracking logic in a small `src/core` helper instead of growing `visualizer.js` further

- [ ] Expose only the values the UI/solver needs: accepted scale, adjusted dq, resulting `w_vfi`, and reason label
