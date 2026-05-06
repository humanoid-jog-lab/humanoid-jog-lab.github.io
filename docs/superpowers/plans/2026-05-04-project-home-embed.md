# Project Home Embed Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Nerfies-style project homepage that embeds the existing Humanoid Jog Lab app further down the page.

**Architecture:** Keep the current full-screen robot controller isolated as `app.html` so its fixed canvas and floating panels keep working. Replace `index.html` with a scrollable project page containing hero, video, abstract, results, and an iframe pointing to `app.html`.

**Tech Stack:** Vite multi-page HTML, plain HTML/CSS, existing Node test runner.

---

### Task 1: Tests For Split Entry Points

**Files:**
- Modify: `src/ui/humanoid-manip-controls.test.js`
- Modify: `src/ui/surface-slide-removal.test.js`
- Modify: `src/ui/planned-motion-removal.test.js`
- Modify: `src/ui/info-panel-removal.test.js`
- Modify: `src/features/dq-calculator.test.js`
- Create: `src/ui/project-home.test.js`

- [ ] Write failing tests that expect app controls in `app.html`, not `index.html`.
- [ ] Add project-home assertions for title, video section, iframe `/app.html`, full-demo link, and Nerfies-inspired section structure.
- [ ] Run the targeted tests and confirm they fail before implementation.

### Task 2: Preserve The Existing App

**Files:**
- Create: `app.html`

- [ ] Copy the current controller HTML to `app.html`.
- [ ] Keep the existing `/src/main.js` script and `/src/styles.css` link unchanged.

### Task 3: Build The Project Home Page

**Files:**
- Replace: `index.html`
- Create: `src/project-home.css`

- [ ] Add the project hero, action links, video panel, abstract, method/results sections, and iframe demo section.
- [ ] Keep the visual style publication-like, readable, and responsive.
- [ ] Use a local video placeholder card that can later point at a real video asset.

### Task 4: Build Configuration

**Files:**
- Create: `vite.config.js`

- [ ] Configure Vite/Rollup to build both `index.html` and `app.html`.

### Task 5: Verification

**Files:**
- No production files.

- [ ] Run targeted tests.
- [ ] Run `npm run build`.
- [ ] Run `git diff --check`.
- [ ] Start or reuse the dev server and inspect the homepage/app embed.
