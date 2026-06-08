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

## 3. The gesture grammar

Revised 2026-06-07. The grammar is **per control type**: *turn* and *click* each
do the natural thing for the focused control, and *double-click* / *hold* are
universal. The unifying rule:

> **Turn interacts cheaply** — live-adjust a value, or move a highlight. **A
> press is the only thing that causes anything destructive** — loading a rig,
> flipping a block on/off.

| Gesture | Continuous value | Toggle (on/off) | Rig / Setlist list |
|---|---|---|---|
| **Turn** | adjust **live** (written immediately), speed-sensitive | — *ignored* (a brushed knob can't flip it) | **browse** names — nothing loads, speed-sensitive |
| **Click** | — | **flip On/Off** | **load** the highlighted item |
| **Double-click** | next control in the dial's group | next control | next control |
| **Hold** (~1 s, repeats) | up one level; keep holding climbs to the top | up one level | cancel browse → current item, then up a level |
| **~10 s idle** (browsing a list) | — | — | revert the highlight to the loaded item (no load) |

This replaces the earlier "turn always previews / pause-to-load" idea. Value
knobs stay **instantly live** (the headline feature); only *lists* defer to a
press, which removes the old failure where browsing rigs auto-loaded each one and
spun the whole fleet. Double-click cycling has a built-in ~250 ms detect window,
so a click acts after that — fine for load/flip, not used for live values.

**Acceleration.** Turning *and* list-scrolling are velocity-sensitive: a slow
creep moves 1 step / 1 item (precise); faster spins move in larger increments.
Tunable by feel on hardware.

**Toggles are a first-class control type.** A block's enable (`On`) and any other
boolean param are assignable like a value — this is where "bypass" lives now,
not a hidden gesture. On Home a toggle shows big **ON / OFF** (green / dim) and
**click flips it**.

**Knob groups.** A board's Home holds **one control or an ordered group**
(e.g. `[Bass, Treble]`, or a Delay's `Mix` + its `Enable`, or several params of
one device). Turn/click act on the focused member; **double-click** steps to the
next; hold leaves. One dot per member, the active one highlighted.

## 4. Screen hierarchy (identical on all 4 boards)

```
HOME  ─ this board's assigned control/group for the loaded rig
│       a value dial, a toggle, OR a Rigs/Setlists browser
│       turn/click = per control type (§3) · double-click = next member · hold = Board Menu
│
└─ BOARD MENU            (hold from Home)
   ├─ Assign this knob → …   build the dial's group (§5.1): This-dial / globals / rig devices → params
   └─ Settings →             device/global: ID, WiFi, firmware, mic strip, rename/save…
```

**Control types are assignable, not menu destinations.** Tuner, **Setlist**, and
**Rig** aren't fixed menu items — they're control *types* you can assign to a
knob (and group with others). **Setlist and Rig are browse-then-load lists**: the
Home screen shows the loaded item; **turn browses** the names with *nothing
loading*, and a **single click loads** the highlighted one (10 s idle reverts the
highlight — no load). This is deliberate: browsing no longer cascades a rig load +
fleet-wide re-resolve on every name. (Rig browses the current setlist's rigs;
Setlist browses all setlists and loading one switches the active setlist.)

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
- **Unconfigured rig ⇒ sensible defaults** derived from its blocks (the default
  layout, below).
- **Reassigning a knob saves that layout for the current rig.** A new rig starts
  from the sensible defaults.

### Default layout (applied when a rig has no saved assignment)

Agreed 2026-06-03. Each of the 4 boards (by device id 1–4) gets a role:

| Board | Role | Default assignment | Top member |
|---|---|---|---|
| 1 | Navigation | group `[Rig, Setlist]` | **Rig** |
| 2 | Gain / character | top present of the **Gain bucket** | (single param) |
| 3 | Ambience / space | top present of the **Ambience bucket** | (single param) |
| 4 | Levels | group `[Output, Input]` (+ per-block `Level`, later) | **Output** |

Dials 2 & 3 use **two complementary buckets** rather than a flat priority list,
so any rig yields one gain knob *and* one space knob:

- **Gain bucket** (dial 2), first present: **Drive → Amp → Compressor**
  - Drive → `Drive` · Amp → `GainA` · Compressor → `Sustain`
- **Ambience bucket** (dial 3), first present: **Reverb → Delay → Mod → Filter → Pitch**
  - Reverb/Delay/Mod/Pitch → `Mix` · Filter → `Freq`

**Cross-fill:** if a bucket is empty, that dial borrows the next-priority unused
type from the other bucket; if the rig is sparse (amp+cab only), it falls back to
a global (Tempo/Width). Duplicate blocks → first in chain order; parallel `_2`
path blocks are ignored for defaults.

Each board carries the *category* (generic "Drive", "Reverb", …), not a specific
block — on rig load the net task resolves the category to whatever block fills
that slot in the loaded rig (`/Evil/Engine/Patch/{Block}`), so "compression" maps
to whichever compressor the rig actually uses.

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

### 5.1 Assigning a dial (drill-in, toggle add/remove)

Revised 2026-06-07. Building a dial's group is a two-level drill-in; adding and
removing are the **same click-toggle**, so it's symmetric.

**Level 1 — pick a source** (spinner + arc dots; turn browses, dots show what's in
the group), in order:
1. **This dial** — the group's current members, in order. Click one to **remove**
   it directly (no drilling). (Later: reorder here.)
2. **Globals / System** — Output, Input, Tempo, Width, Bass, Treble, Tuner,
   Setlist, Rig. Single-value leaves: **click toggles** in/out of the group.
3. **Rig devices** — grouped by **category, then name**. Click a device → Level 2.

**Level 2 — pick the device's params:** the **primary** is pre-selected on top;
turn for the others (its value params, its **Enable** toggle, and any boolean
params). **Click toggles** that device+param in/out of the group — so one device
can contribute **several** members (e.g. a Delay's `Mix` + `Feedback` + `Enable`).

- **Selection feedback:** chosen items show in accent + the bright arc dot; a
  Level-1 device dot is bright if **any** of its params are in the group, and its
  row shows a count when more than one (`Multi Chorus ·2`).
- **Back / commit:** hold or 10 s idle = up a level. **Hold at Level 1 commits**
  the group (saves per-rig → re-resolve) and exits to the Board Menu.
- The group's **order** is add order — what double-click cycles in operate.

## 6. Control catalog — curated now, discovery later

Ship a **curated** set of meaningful controls, filtered to those present in the
loaded rig. Design the data structures so full per-block parameter discovery can
be layered on later without rework.

- **Always available (globals):** Output (RigVolume), Input (InputGain), Tempo,
  Width (RigWidth), Global EQ bands (`GlobalEQMain` Gain1/Gain4, …).
- **Rig-dependent (shown when the block exists):** Drive gain, Mod mix/rate,
  Delay mix/time/feedback, Reverb mix/decay, Comp sustain — mapped to the rig's
  actual blocks under `/Evil/Engine/Patch/{Block}`.

**Generic categorization is authoritative and fully runtime, not baked.** Every
block's category comes from the Prime itself (`/Evil/API/Blocks.categoryOfBlock`),
collapsed to the generic set the generator reasons about. The chain slot's
`ModuleTypeN` is an index into `Blocks.ModuleTypes`, which the firmware fetches
live and caches (`src/BlockCatalog.h` holds **no** baked snapshot — only the
generic taxonomy + param conventions). A stale local copy is deliberately
avoided: it would let resolution succeed with the wrong blocks after a firmware
reorder, silently. If the live block list can't load, the dial shows the
resolving spinner and retries rather than guessing. The primary param per
category is resolved against the actual block's properties (first candidate that
exists wins), since blocks in a category aren't identical.

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
