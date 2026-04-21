# Default World Frame Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make world-frame teach jogging the default for all tasks while preserving the user's last chosen toggle state across reloads.

**Architecture:** Keep the existing teach order toggle and visualizer API, but move initialization to a single persisted source of truth in the UI layer. When no saved value exists, fall back to world-frame mode.

**Tech Stack:** Vanilla JS, DOM initialization in `index.html`, event wiring in `src/ui/event-handlers.js`, visualizer bootstrap in `src/core/robot-manager.js`, node-based UI tests.

---

### Task 1: Add a persisted default for the teach order toggle

**Files:**
- Modify: `index.html`
- Modify: `src/ui/event-handlers.js`
- Modify: `src/core/robot-manager.js`

- [ ] **Step 1: Write or extend a failing test**
  Cover first-load default and persisted restore behavior for `teach-multiply-order-toggle`.

- [ ] **Step 2: Run the targeted test to confirm the current default is still local frame**

- [ ] **Step 3: Implement the minimal initialization change**
  Make the toggle default to world frame when no saved value exists, and save changes into `localStorage`.

- [ ] **Step 4: Keep visualizer bootstrap in sync**
  Ensure `initVisualizer()` consumes the persisted/default toggle state so the visualizer starts in `current-first`.

- [ ] **Step 5: Run targeted tests and a production build**
  Verify the UI text, saved state restoration, and build output.
