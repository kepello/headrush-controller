# Phase 1 — Grammar engine + Board Menu skeleton

Goal: make every screen obey the universal grammar from `UX_MODEL.md`, introduce
the Home ↔ Board Menu structure, and retire the click-to-cycle "ring" — **without**
adding any per-rig logic yet. After Phase 1 the device behaves consistently; it
just isn't rig-aware. For review before any code is written.

## Universal grammar (Phase 1 wiring)

| Screen | Turn | Single click | Double click | Hold (1 s) |
|---|---|---|---|---|
| **Home** — control or group | adjust focused value² | next member in group¹ | toggle focused block bypass³ | open Board Menu |
| **Board Menu** (list) | move cursor | enter item | — | back → Home |
| **Assign this knob** (list) | move cursor | toggle member in/out of group⁴ | — | confirm group → back |
| **Settings** + sub-screens | move/edit | confirm | — | back one level |

Each gesture has one meaning: turn = move, single-click = advance focus
(enter item / next group member), double-click = bypass (Home only), hold = back.

¹ A Home holds one control or an **ordered group**; single-click cycles the
focused member (no-op for a group of one).

² Tuner Home = read-only readout (turn no-op); Tempo Home = adjustable BPM;
value controls adjust live.

³ **Double-click toggles the focused member's effect-block enable/bypass**
(e.g. a board on Reverb: turn sets mix, double-click kills/enables it). No-op for
globals (Output, Input, Tempo, Width, Global EQ) and the tuner, which have no
block. Per-block bypass property path confirmed against the Prime in Phase 2.

⁴ Assign is **multi-select**: each click toggles a control in/out of this knob's
group; hold confirms the (ordered) selection. Picking one = a single-control Home.

## Screen model (replaces the scattered flags)

Today the UI state is spread across `uiMode`, `cfg` (`ConfigState`), `libActive`,
`tunerActive`, plus the implicit default dial. Phase 1 introduces an explicit
screen + a small back-stack:

```
enum Screen { HOME, BOARD_MENU, ASSIGN, SETTINGS, /* existing: */ SET_ID, SET_WIFI… };
Screen navStack[6]; int navDepth;   // push on descend, pop on hold
```

- `hold` at depth 0 (Home) → push `BOARD_MENU`.
- `hold` at depth > 0 → pop to parent.
- Net-driven overlays (OTA progress, boot/splash, update-check) stay **top
  priority and unchanged** — they short-circuit before the screen model, exactly
  as now (`loop()` lines ~876–896).

The existing multi-state **WiFi flow** (`CFG_WIFI` → `CFG_WIFI_PW` →
`CFG_WIFI_CONNECTING`) is **kept intact internally** and entered from
Settings; only its entry/exit are rerouted to the new back-stack. We do not
rewrite the WiFi or password-entry screens in Phase 1.

## Board Menu items (Phase 1 behavior)

| Item | Phase 1 behavior |
|---|---|
| **Assign this knob →** | Multi-select list of the curated catalog (current `PARAM_CATALOG` + Tuner). Click toggles each control in/out of the knob's group; hold confirms the ordered group as Home and persists to NVS (global for now; per-rig in Phase 3). |
| **Rigs / Setlists →** | Routes into the **existing** Library navigator as-is. Full re-grammar deferred to Phase 4. |
| **Tempo** | Simple value screen on `/Evil/Engine/Tempo` (reuses the continuous-value renderer). |
| **Tuner** | Sets Home to the tuner view (a board can be a dedicated tuner). |
| **Settings →** | The existing config menu (Device ID, WiFi, Update firmware) re-parented here, minus the Views editor. |

## What gets retired in Phase 1

- **Click-to-cycle ring across view *types*** (dials + tuner + library): the
  `clicked && ringSize() > 1` block (`loop()` ~1227) is removed. What a board
  shows is now set via **Assign this knob**. (Note: single-click still cycles —
  but only among the *members of one knob's group*, not across unrelated views.)
- **`ringSlot` / `viewMask` / `ringSize()` / `ringViewAt()`** and the
  `CFG_EDIT_VIEWS` multi-select + `drawViewSelect` usage — all gone, replaced by a
  per-board Home **group** (ordered list of controls).
- The old "**hold → config**" becomes "hold → Board Menu → Settings".

## Concrete change list

- **`src/main.cpp`**
  - Add `Screen`/`navStack` model + push/pop helpers; replace the `cfg`/`libActive`/
    `tunerActive` dispatch in `loop()` with a screen switch.
  - **Add single/double-click discrimination** to the button read (lines ~899–921).
    On Home, single-click (next member) must wait out a ~**250 ms** double-click
    window before firing, so a second press can register as a double-click
    (bypass). In **lists/menus there is no double-click action**, so click fires
    immediately (no added latency where it would hurt). Set `LONG_PRESS_MS` to
    **1000 ms** (was 3000) — hold is now a frequent per-level "back" gesture.
  - Remove ring state + `CFG_EDIT_VIEWS`; route `setup()` to load the Home **group**
    (ordered list of control indices) from NVS (drop `viewMask`/`ringSlot` load at
    ~833–847). Track the focused member index in RAM (resets to 0 on boot).
- **`src/Render.h`**
  - Add `drawBoardMenu()` and `drawAssignList()` (reuse `drawListNav` styling;
    Assign shows checkboxes for multi-select).
  - Extend the Home/continuous renderer to draw **N group dots** (one per member,
    active one highlighted) — omit dots for a group of one.
  - `drawConfigMenu` loses the "Views: N" row; keep ID / WiFi / Update / Exit.
  - `drawViewSelect` becomes unused (remove or leave dormant).
- **NVS**: store the Home **group** as an ordered list of control indices (replaces
  the single `param`); `views`/`slot` keys retired. Per-rig keying added in Phase 3.

## Non-goals (explicitly deferred)

- Per-rig assignment + persistence → Phase 3.
- Rig-change detection, Chain introspection, rig-filtered catalog → Phase 2.
- Library re-grammar under the universal rules → Phase 4.
- Polished Tempo screen, mic strip, on-screen text entry → Phase 5.
## Resolved decisions

1. **Long-press = 1000 ms** for "back".
2. **Single-click = next group member; double-click = toggle focused block bypass**
   (bypass is no-op for global/tuner controls; per-block bypass path confirmed in
   Phase 2). ~250 ms double-click window on Home; menus click immediately.
   A board's Home can hold a **group** (ordered multi-select); the screen shows
   **one dot per member**, active highlighted (no dots for a group of one).
3. **No idle return-to-Home** — a board stays on the screen it was left on
   (screen-dim still applies).
4. **Tuner and Tempo are assignable Home controls** (a board can be a dedicated
   tuner or tempo knob), not just transient menu views.

## Verify

- `pio run -e crowpanel_128` clean.
- On-device matrix: turn adjusts Home value; hold opens Board Menu; turn moves
  cursor; click enters; hold backs out level-by-level to Home; Assign changes
  Home and survives reboot; Settings → WiFi flow still completes; OTA overlay and
  boot splash still preempt correctly.
