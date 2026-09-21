#include "SitlModule.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include <QProcessEnvironment>

// QLibrary rather than dlopen/LoadLibrary behind an #ifdef: Qt is already a
// dependency, and it resolves the platform's own naming (libvayu_sitl.so,
// vayu_sitl.dll) from one base name.

namespace {

// Where to look, in order. An explicit override wins so a developer can point
// the GCS at a firmware build without installing it; otherwise we look beside
// the executable (how it ships), then in the build tree (how it is developed).
QStringList candidatePaths() {
  QStringList out;

  const QString override =
      QProcessEnvironment::systemEnvironment().value("VAYU_SITL_MODULE");
  if (!override.isEmpty())
    out << override;

  const QDir appDir(QCoreApplication::applicationDirPath());
  out << appDir.absoluteFilePath("lib" VAYU_SITL_MODULE_NAME);
  out << appDir.absoluteFilePath("../lib/lib" VAYU_SITL_MODULE_NAME);

  // Development layout: a sibling SITL build dir next to the source tree.
  for (const char *build : {"build_sitl_rtos", "build_sitl"})
    out << appDir.absoluteFilePath(
        QStringLiteral("../../%1/lib%2")
            .arg(QLatin1String(build), QLatin1String(VAYU_SITL_MODULE_NAME)));

  return out;
}

} // namespace

QStringList SitlModule::searchPaths() { return candidatePaths(); }

SitlModule::SitlModule() {
  QStringList tried;

  for (const QString &candidate : candidatePaths()) {
    QLibrary lib(candidate);
    // RESOLVE_ALL_SYMBOLS_HINT: fail here on a module that is not
    // self-contained, rather than on whichever vtable call first needs the
    // missing symbol.
    lib.setLoadHints(QLibrary::ResolveAllSymbolsHint);
    if (!lib.load()) {
      // QLibrary tries several names around the base (libX.so, X.so, ...) and
      // its errorString reports only the last, so quoting it verbatim prints a
      // path nobody passed. The candidate is what the user can act on.
      tried << (QFileInfo(QFileInfo(candidate).path()).isDir()
                    ? QStringLiteral("%1 (not found)").arg(candidate)
                    : QStringLiteral("%1 (no such directory)").arg(candidate));
      continue;
    }

    auto get_api = reinterpret_cast<vayu_sitl_get_api_fn>(
        lib.resolve(VAYU_SITL_ENTRY_SYMBOL));
    if (!get_api) {
      tried << QStringLiteral("%1: no %2 symbol")
                   .arg(candidate, QLatin1String(VAYU_SITL_ENTRY_SYMBOL));
      lib.unload();
      continue;
    }

    const vayu_sitl_api_t *api = get_api(VAYU_SITL_ABI_VERSION);
    if (!api) {
      // The module is there but speaks a different ABI. Say so plainly: this
      // is a firmware/GCS version mismatch, not a missing file, and the two
      // have completely different fixes.
      tried << QStringLiteral("%1: ABI mismatch (this build needs v%2)")
                   .arg(candidate)
                   .arg(VAYU_SITL_ABI_VERSION);
      lib.unload();
      continue;
    }

    // Deliberately left loaded for the process lifetime. boot() starts vaios
    // tasks that never stop, so unloading the code underneath them would
    // crash; there is nothing to gain by trying.
    api_ = api;
    path_ = QFileInfo(lib.fileName()).absoluteFilePath();
    buildId_ = api->build_id ? QString::fromUtf8(api->build_id)
                             : QStringLiteral("unknown");
    return;
  }

  error_ =
      QStringLiteral("no SITL module found (lib%1). Build the firmware, or set "
                     "VAYU_SITL_MODULE. Looked in:\n  %2")
          .arg(QLatin1String(VAYU_SITL_MODULE_NAME), tried.join("\n  "));
}

bool SitlModule::send(const QByteArray &frame) const {
  if (!api_ || frame.isEmpty())
    return false;
  api_->uart2_rx(reinterpret_cast<const uint8_t *>(frame.constData()),
                 static_cast<size_t>(frame.size()));
  return true;
}

SitlModule &SitlModule::instance() {
  static SitlModule inst;
  return inst;
}
