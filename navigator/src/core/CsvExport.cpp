#include "CsvExport.h"

#include "../ui/widgets/RealTimeGraph.h"

#include <QDateTime>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTextStream>

#include <algorithm>
#include <limits>
#include <vector>

namespace CsvExport {

namespace {

QString quoteIfNeeded(const QString &s) {
  if (!s.contains(',') && !s.contains('"') && !s.contains('\n'))
    return s;
  QString q = s;
  q.replace('"', "\"\"");
  return '"' + q + '"';
}

} // namespace

int writeCombined(QTextStream &out, const QList<GraphSource> &sources) {
  // 1) Header row.
  out << "timestamp_ms";
  int seriesOrdinal = 0;
  for (const auto &src : sources) {
    if (!src.graph)
      continue;
    const int n = src.graph->numSeries();
    for (int i = 0; i < n; ++i) {
      out << ',';
      if (i < src.headers.size() && !src.headers[i].isEmpty()) {
        out << quoteIfNeeded(src.headers[i]);
      } else {
        out << "series_" << seriesOrdinal;
      }
      ++seriesOrdinal;
    }
  }
  out << '\n';

  // 2) Build cursor positions for each (source, series).
  struct Cursor {
    const std::deque<RealTimeGraph::DataPoint> *data;
    size_t idx;
  };
  std::vector<Cursor> cursors;
  for (const auto &src : sources) {
    if (!src.graph)
      continue;
    const auto &s = src.graph->series();
    for (const auto &deq : s) {
      cursors.push_back({&deq, 0});
    }
  }
  if (cursors.empty())
    return 0;

  // 3) Walk a unified timestamp axis across all cursors. At each step,
  //    pick the smallest unconsumed ts, emit a row, advance any cursors
  //    sitting on that ts.
  int rows = 0;
  while (true) {
    qint64 next = std::numeric_limits<qint64>::max();
    bool any = false;
    for (const auto &c : cursors) {
      if (c.idx < c.data->size()) {
        any = true;
        next = std::min(next, (*c.data)[c.idx].timestamp);
      }
    }
    if (!any)
      break;

    out << next;
    for (auto &c : cursors) {
      out << ',';
      if (c.idx < c.data->size() && (*c.data)[c.idx].timestamp == next) {
        out << QString::number((*c.data)[c.idx].value, 'g', 6);
        ++c.idx;
      }
    }
    out << '\n';
    ++rows;
  }
  return rows;
}

QString promptAndWriteCombined(QWidget *parent, const QString &defaultName,
                               const QList<GraphSource> &sources) {
  // Suggest the user's Documents/Downloads dir. The save dialog
  // remembers its own last-used directory across launches.
  QString suggestion =
      QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
  if (!suggestion.isEmpty())
    suggestion += '/';
  suggestion += defaultName.isEmpty() ? "telemetry.csv" : defaultName;

  const QString path = QFileDialog::getSaveFileName(
      parent, QObject::tr("Export CSV"), suggestion,
      QObject::tr("CSV (*.csv);;All files (*)"));
  if (path.isEmpty())
    return {};

  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(parent, QObject::tr("Export failed"),
                         QObject::tr("Could not open %1 for writing: %2")
                             .arg(path, f.errorString()));
    return {};
  }
  QTextStream out(&f);
  const int rows = writeCombined(out, sources);
  f.close();

  // Toast / status feedback could go here via Notify::ok in callers;
  // we just return the path so the widget can format its own message.
  Q_UNUSED(rows);
  return path;
}

} // namespace CsvExport
