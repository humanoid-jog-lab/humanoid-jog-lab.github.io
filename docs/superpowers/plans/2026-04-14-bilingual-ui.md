# Bilingual UI Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an English-by-default bilingual UI with persistent language selection for static interface text and key runtime messages.

**Architecture:** Introduce a lightweight in-repo i18n layer that owns language state, DOM translation, and change subscriptions. Keep English as the base HTML language, then translate static DOM and runtime UI messages through shared translation keys so modules can re-render correctly after a language switch.

**Tech Stack:** Vite, vanilla ES modules, Node test runner, localStorage

---

### Task 1: Add translation regression tests

**Files:**
- Create: `src/i18n.test.js`
- Test: `src/i18n.test.js`

- [ ] **Step 1: Write the failing test**

Cover default English, persisted language selection, and DOM translation for text plus placeholders.

- [ ] **Step 2: Run test to verify it fails**

Run: `node --test src/i18n.test.js`
Expected: FAIL because the i18n module and wiring do not exist yet.

### Task 2: Add the i18n core and language switcher

**Files:**
- Create: `src/i18n.js`
- Modify: `index.html`
- Modify: `src/main.js`

- [ ] **Step 1: Implement language state and translation helpers**

Add supported languages, default English, localStorage persistence, DOM translation helpers, and language change subscriptions.

- [ ] **Step 2: Add the switcher UI and English-first static shell**

Add a compact language switcher to the control panel header and update the page shell to English defaults.

- [ ] **Step 3: Initialize i18n before app setup**

Ensure saved language choice is applied during app startup.

- [ ] **Step 4: Run the new tests**

Run: `node --test src/i18n.test.js`
Expected: PASS

### Task 3: Translate key runtime UI modules

**Files:**
- Modify: `src/ui/event-handlers.js`
- Modify: `src/ui/mode-switcher.js`
- Modify: `src/ui/environment-panel.js`
- Modify: `src/ui/drag-drop.js`
- Modify: `src/ui/joint-controls.js`
- Modify: `src/ros2/ros2-panel.js`
- Modify: `src/core/humanoid-jog-solver-presentation.js`

- [ ] **Step 1: Replace hard-coded UI strings with translation keys**

Translate runtime button labels, status text, prompts, alerts, and mode descriptions through `t(...)`.

- [ ] **Step 2: Subscribe modules to language changes where current UI state must be re-rendered**

Refresh mode labels, status text, and generated environment content after the language changes.

- [ ] **Step 3: Run targeted tests and build**

Run: `node --test src/i18n.test.js src/ui/planned-motion-removal.test.js`
Expected: PASS

Run: `npm run build`
Expected: PASS

### Task 4: Final verification

**Files:**
- Verify only

- [ ] **Step 1: Run source search**

Run: `rg -n "language-select|initializeI18n|setLanguage|onLanguageChange|t\\(" index.html src`
Expected: matches in the new i18n integration points.

- [ ] **Step 2: Review git diff**

Run: `git diff --stat`
Expected: i18n files and relevant UI modules changed, without unrelated reverts.
