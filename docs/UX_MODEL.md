# HeadRush Controller — UX & Structure Spec

Status: agreed 2026-06-01. This is the canonical interaction model. New UI work
conforms to this; existing code is refactored toward it (see "Migration").

## 1. Purpose

A hand-height control surface for the things on the HeadRush Prime that are
**awkward to do with your feet** — i.e. everything you'd normally crouch down
and poke the touchscreen for. It is **not** a footswitch replacement: rig/scene
switching, stomp on/off, looper transport, and tap tempo already work by foot.

The fleet's advantage over the floor unit: **4 hand-height knobs = 4
always-live controls**, no menu-diving for your most-wanted parameters (the
Prime's own 3 macro encoders require crouching).

## 2. Hardware & input model

- 4× Elecrow CrowPanel 1.28" (ESP32-S3, round GC9A01 screen + rotary encoder),
  one Prime. One firmware image; per-board/per-rig config in NVS.
- **Input is knob-only.** The capacitive touchscreen is deliberately unused
  (glove/dark-stage proof, one mental model). Do not add touch navigation
  without revisiting this spec.

## 3. The universal gesture grammar (no exceptions, every screen)

| Gesture | Meaning |
|---|---|
| **Turn** | *Move*: change the focused value, or move the list cursor. Value edits are live (written to the Prime immediately). |
| **Click** (short) | *Advance focus*: in a list, enter/select the highlighted item; on Home, cycle to the next control in the knob's group. |
| **Double-click** | *Bypass* (Home only): toggle the focused effect control's block. No-op for globals/EQ/tuner/Setlist/Rig (nothing to bypass). |
| **Hold** (~1 s) | *Back*: up one level. At a board's Home screen, "up" opens the Board Menu. |

Each gesture has exactly one meaning — no precedence rules. Turn never changes
screens; single-click advances focus; double-click bypasses; hold leaves. This
eliminates the prior inconsistencies (turn meaning "adjust" on dials but
"navigate" in the Library; click meaning "cycle view" vs "select"). A board
**stays on whatever screen it was left on** — there is no inactivity auto-return
to Home (the screen still dims when idle).

**Knob groups.** A board's Home can hold **one control or an ordered group** of
related controls (e.g. `[Bass, Treble]`, a 3-band EQ, or several effect mixes
when there aren't enough knobs for one each). Turn adjusts the focused member;
single-click steps to the next member; double-click bypasses the focused
member's block. The Home screen shows **one dot per group member** with the
active member's dot highlighted (two members → two dots, etc.) — no "1/2" text.
A group of one is just a single control (single-click is then a no-op).

## 4. Screen hierarchy (identical on all 4 boards)

```
HOME  ─ this board's assigned control/group for the loaded rig
│       a value dial, OR Tuner, OR a Rigs/Setlists readout
│       turn = adjust · single-click = next member · double-click = member action · hold = Board Menu
│
└─ BOARD MENU            (hold from Home)
   ├─ Assign this knob → …   multi-select from controls AVAILABLE IN THE LOADED RIG
   │                          (dials, Tuner, Setlist, Rig — pick one or form an ordered group)
   └─ Settings →             device/global: ID, WiFi, firmware, mic strip, rename/save…
```

**Control types are assignable, not menu destinations.** Tuner, **Setlist**, and
**Rig** aren't fixed menu items — they're control *types* in the Assign list, so a
knob can BE the setlist selector or rig selector (and can sit in a group with
dials). **Setlist and Rig are scroll-to-select lists**: the Home screen shows the
current item; **turn scrolls** the names and, after a brief pause (~0.6 s) on one,
that item is **loaded** — no click-to-select, no drill-down, no back row. Single-
click still cycles group members. (Rig scrolls the current setlist's rigs; Setlist
scrolls all setlists and loading one switches the active setlist.)

## 5. Per-rig dynamic layout (the core idea)

A board's HOME assignment is **per rig**, not global.

- **Rig key:** `/Evil/API/Rigs.loadedID` (+ `loadedSrID` for setlist context) — a
  stable id, not the display name. Rig changes arrive over the WebSocket
  (`PresetName` / loadedID), so layout swaps need no polling.
- **On rig load:** each board adopts *that rig's* saved assignment. Boards are
  independent — board 1's layout for a rig is unrelated to board 2's.
- **Available controls are rig-dependent:** the "Assign this knob" list only
  offers controls the loaded rig actually contains (no phaser block ⇒ no phaser
  control). Existence is read from `/Evil/Engine/Patch/Chain` module/effect
  slots, plus the always-present globals.
- **Unconfigured rig ⇒ sensible defaults** derived from its blocks, e.g.
  board1=Output, board2=first drive's gain, board3=reverb mix, board4=delay mix.
- **Reassigning a knob saves that layout for the current rig.** A new rig starts
  from the sensible defaults.

### Storage — NVS per board

Each board stores its own `{rigId → control}` map in NVS (namespace `hrctrl`).
Chosen over central-on-Prime because:
- The Prime has **no writable general store** — `FileAccess` is `ls`-only; the
  only writable persistence is structured rig/settings objects. Stashing data in
  a rig's `labels` field was rejected (risks corrupting rigs).
- Each board only needs its own data, so cross-board sharing was never required.
- Local read fits the network-independence rule (a rig change must not block on a
  fetch).

Tradeoff: a full USB reflash loses layouts (rare — OTA is the normal path).
Optional future mitigation: export the layout blob to the OTA host as a backup.

## 6. Control catalog — curated now, discovery later

Ship a **curated** set of meaningful controls, filtered to those present in the
loaded rig. Design the data structures so full per-block parameter discovery can
be layered on later without rework.

- **Always available (globals):** Output (RigVolume), Input (InputGain), Tempo,
  Width (RigWidth), Global EQ bands (`GlobalEQMain` Gain1/Gain4, …).
- **Rig-dependent (shown when the block exists):** Drive gain, Mod mix/rate,
  Delay mix/time/feedback, Reverb mix/decay, Comp sustain — mapped to the rig's
  actual blocks under `/Evil/Engine/Patch/{Block}`.

## 7. Functional scope (all four groups in scope; phase as needed)

1. **Live param knobs** — continuous tweaking of amp/drive/mod/delay/reverb/comp
   + levels + Global EQ. The core use; HOME value screens.
2. **Rig/setlist browse & load** — jump to any rig/setlist by name across the
   library (`/Evil/API/Rigs`, `/Evil/API/Setlists`).
3. **Tempo & tuner** — precise BPM (`/Evil/Engine/Tempo`); tuner readout
   (`/Evil/Engine/FFTCtrl`).
4. **Setup/admin** — rename/save rigs, mic channel strip (`/Evil/Engine/Mic`),
   global settings, scene labels. Needs the on-screen text-entry primitive
   already flagged as load-bearing.

## 8. Migration from current code

- `ringSlot` / `viewMask` (global per-board view ring, click-to-cycle) →
  replaced by the per-rig HOME assignment + Board Menu. The "ring" concept and
  click-to-cycle go away.
- Library view (turn-to-open, click-to-select) → regrammar'd as the "Rigs /
  Setlists" menu branch under the universal grammar.
- Config (long-press menu) → folded into "Settings →" under Board Menu; same
  grammar.
- Static `PARAM_CATALOG` → becomes the curated catalog, filtered per rig from the
  Chain, with per-rig assignment persisted in NVS.
- Keep the UI/net task split: rig-change detection + Chain introspection live in
  the net task; the UI task reads the resolved available-controls list + current
  assignment via the shared-state mailbox.
```
