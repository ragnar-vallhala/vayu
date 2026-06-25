#pragma once

#include <QAction>
#include <QKeySequence>
#include <QString>

// A single addressable command — the unit the menus, toolbar, shortcut
// editor, palette, and recent-views switcher all read from. A command *is*
// a QAction: menu items, toolbar buttons, and the keyboard shortcut share
// the one QAction, so rebinding or context-gating it updates everywhere for
// free (FR-UX-19; see docs/roadmap/command-registry-and-shortcuts.md).
//
// Phase 1A migrates the previously scattered QShortcuts/menu actions into
// registered Commands with no behaviour change. The `when` contexts below
// are defined now but every migrated command is `Always`; the gating becomes
// load-bearing in Phase 1D (SessionMode) and later phases.
enum class CmdContext {
  Always,     // enabled unconditionally
  Connected,  // a live link (serial/UDP) is up
  Live,       // session is in Live mode (not replay)
  Replay,     // session is in Replay mode
};

struct Command {
  QString id;                 // stable, dotted: "view.home", "link.toggle"
  QString title;              // human label, may carry an "&" menu mnemonic
  QString category;           // grouping: "View", "Link", "Window"
  QKeySequence defaultSeq;    // factory binding; user overrides layer on top
  CmdContext when = CmdContext::Always;
  QAction *action = nullptr;  // the one shared QAction (owned by the registry)
};

// Strip Qt menu-mnemonic markers from a title for display outside a menu (the
// shortcuts editor, the command palette). A single '&' marks the mnemonic and
// is removed; "&&" is a literal ampersand. Menus keep the raw title so their
// underline/Alt-access still works.
inline QString commandDisplayTitle(const QString &title) {
  QString out;
  out.reserve(title.size());
  for (int i = 0; i < title.size(); ++i) {
    if (title[i] == QLatin1Char('&')) {
      if (i + 1 < title.size() && title[i + 1] == QLatin1Char('&')) {
        out += QLatin1Char('&');
        ++i;
      }
      // otherwise: drop the lone mnemonic marker
    } else {
      out += title[i];
    }
  }
  return out;
}
