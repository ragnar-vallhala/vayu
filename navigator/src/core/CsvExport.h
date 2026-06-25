#pragma once

#include <QList>
#include <QString>
#include <QStringList>

class QTextStream;
class QWidget;
class RealTimeGraph;

// Shared CSV-export helpers for telemetry widgets (FR-LOG-04 /
// Phase-1 1d).
//
// Single-graph callers should prefer `RealTimeGraph::writeCsv` directly.
// `writeCombined` is for panels (ImuPanel, ControlLoopPlot) whose data
// is spread across several `RealTimeGraph` instances but logically
// belongs in one file with a shared timestamp axis.
//
// Conventions:
//   * Output is comma-separated, UTF-8, LF line endings.
//   * First column is always `timestamp_ms` (Unix ms, UTC).
//   * One column per series, in the order graphs are listed and series
//     within each graph are appended.
//   * Cells where a series has no sample at the row's timestamp are
//     left empty (not zero — zero is a valid value).
//
// `promptAndWriteCombined` is the one-stop wrapper for an Export
// button: opens a save dialog, writes, returns the path on success
// (empty QString on cancel / failure).
namespace CsvExport {

struct GraphSource {
  const RealTimeGraph* graph;
  QStringList headers;  // one per series in this graph; missing names → series_N
};

int writeCombined(QTextStream& out, const QList<GraphSource>& sources);

// UI-side convenience. `defaultName` is the suggested filename (no
// directory — the dialog picks one). Returns the absolute path
// written, or empty if the user cancelled or the write failed.
QString promptAndWriteCombined(QWidget* parent, const QString& defaultName,
                               const QList<GraphSource>& sources);

}  // namespace CsvExport
