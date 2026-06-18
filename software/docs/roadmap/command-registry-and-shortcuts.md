# Command registry, editable shortcuts, command palette & recent-views switcher

Status: ✅ shipped — `core/CommandRegistry`, `core/ShortcutsManager`, `core/ViewHistory`
and the `ui/widgets/{ShortcutsEditorDialog,CommandPalette,RecentViewsOverlay,AboutDialog}`
are live. Generalises the Phase-0 `0l` shortcuts. Wired into
[`../requirements.md`](../requirements.md) as **FR-UX-19** (command registry + editable
shortcuts), **FR-UX-20** (command palette), **FR-UX-21** (recent-views switcher),
**FR-UX-22** (About), **FR-UX-23** (Documentation); Phase-1 item `1g`. The body below
remains as design rationale.

> As-built note: the recent-views (MRU) switcher landed on **Ctrl+Tab** by default,
> not the Firefox-style `Ctrl+`` ` the mockup originally sketched.

## Context

Phase-0 `0l` shipped hard-coded `QShortcut`s in `MainWindow` (`Ctrl+1..8`,
`Ctrl+K`, `Ctrl+L`, `F11`). The mockup pushes this into a full **command layer**:
a searchable, **editable** keybinding table (VS Code style), a fuzzy **command
palette**, and a Firefox-style **`Ctrl+`` ` most-recently-used view switcher** —
plus Help ▸ Documentation / About. All of these want a single source of truth
for "what commands exist," which `MainWindow`'s scattered shortcuts don't give.

## Why a registry (the small structural piece)

Scattered `QShortcut`s can't be listed, searched, rebound, or conflict-checked.
A central registry makes every command a first-class, addressable object — the
editor, palette, menus, and toolbar all read from it.

```
core/CommandRegistry  ──┬──▶ menus / toolbar (QAction)
   Command{id,title,    ├──▶ ShortcutsEditorDialog  (rebind, search, conflicts)
   category,default,    ├──▶ CommandPalette         (fuzzy run)
   when, QAction}       └──▶ ShortcutsManager        (apply overrides → QKeySequence)
```

## Architecture

- **`core/Command`** — `{ id, title, category, defaultSeq (QKeySequence),
  when (context predicate), QAction* }`. A command *is* a `QAction`; menus and
  the toolbar use the same `QAction`, so a rebind updates everywhere for free.

- **`core/CommandRegistry`** — owns all `Command`s; `register()`, `byId()`,
  `all()`. Populated at startup (nav-to-view, connect, ARM, replay transport,
  calibration, autotune-apply, help, etc.). Replaces the `0l` `QShortcut`s.

- **`core/ShortcutsManager`** — loads per-command overrides from
  `SettingsManager` (`shortcuts/<id>` → key string), applies them as
  `QKeySequence` on the `QAction`s, and resolves conflicts (last-bound wins;
  warn via `Notify`). Falls back to `defaultSeq`.

- **Context (`when`)** — a small enum/flags (`Always`, `Connected`, `Replay`,
  `Flying`, `SimRunning`…) evaluated against app state so e.g. replay transport
  keys only fire in replay. `QAction::setEnabled` reflects it, so the menus also
  grey out correctly. (This pairs with the replay `SessionMode`.)

- **`ui/widgets/ShortcutsEditorDialog`** — the VS Code-style table: search across
  command/key/category/when, click-to-record a new chord (capture
  `QKeyEvent` → `QKeySequence`), conflict detection, per-row + global reset,
  "modified" markers. Reads/writes through `ShortcutsManager`. Mirrors the
  mockup's `#kbModal`.

- **`ui/widgets/CommandPalette`** (`Ctrl+Shift+P`) — a `QLineEdit` + filtered
  `QListView` over `CommandRegistry::all()`; Enter triggers the `QAction`.
  Fuzzy match on title/category.

- **Recent-views switcher** (`Ctrl+`` `) —
  - `core/ViewHistory` — an MRU stack of page ids, pushed on every
    `QStackedWidget::setCurrentIndex` (hook the nav commands). Depth is a
    setting (`ui/recentViewsCount`, default 5, range 2–9).
  - `ui/widgets/RecentViewsOverlay` — a centred frameless popup listing the top-N
    MRU views with icons; hold `Ctrl`, tap `` ` `` to advance (Shift reverses),
    release to commit, `Esc` cancels. A quick tap toggles to the previous view.
    Native `Ctrl+Tab` is interceptable in a Qt desktop app (unlike a browser), so
    in the *real app* the default may be `Ctrl+Tab`; the mockup uses `Ctrl+`` `
    only because browsers reserve `Ctrl+Tab`. Both are just registry defaults.

- **Help** — `ui/widgets/AboutDialog` (static: name, version, build, protocol,
  flight-stack components) and a Documentation command that opens the bundled
  `docs/` via `QDesktopServices::openUrl`. Trivial (FR-UX-22 / FR-UX-23).

## Work items

1. **`CommandRegistry` + `Command`** — define types; migrate the `0l`
   `QShortcut`s into registered commands; menus/toolbar build their `QAction`s
   from the registry. Pure refactor — verify all existing shortcuts still fire.
2. **`ShortcutsManager`** — override persistence in `SettingsManager`, apply +
   conflict resolution, default fallback.
3. **`ShortcutsEditorDialog`** — search, record, conflicts, reset; Help ▸
   Keyboard Shortcuts opens it.
4. **`CommandPalette`** — fuzzy list over the registry.
5. **`ViewHistory` + `RecentViewsOverlay`** — MRU tracking + the hold-to-cycle
   overlay; depth setting in `SettingsManager`.
6. **`AboutDialog` + Documentation command** (FR-UX-22 / FR-UX-23).
7. **Docs/tests** — FR-UX-19–23 + Phase-1 `1g` are in `requirements.md`;
   unit-test conflict resolution and MRU ordering (toggle = previous; cycle =
   N-deep).

## Decisions

- **One `QAction` per command, shared by menu + toolbar + shortcut** — rebinding
  is automatically reflected everywhere; no duplicate state.
- **`when`-contexts gate via `QAction::setEnabled`** — the menu greys out and the
  shortcut no-ops with the same predicate; no parallel logic.
- **MRU depth is a setting**, matching the mockup's Settings ▸ Units & Display
  control.

## Out of scope (v1)

Chorded keybindings (`Ctrl+K Ctrl+S`), per-profile keymaps, importing VS Code
keymaps, palette actions with arguments (e.g. "go to packet #").

## Note

No GCS ships an MRU view switcher — it's an IDE/browser idiom. It's cheap and
self-contained, but it's a *developer-convenience* feature, not a GCS
convention; keep it behind the registry so it's trivially rebindable or droppable.
