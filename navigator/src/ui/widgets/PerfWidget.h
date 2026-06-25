#pragma once

#include "Types.h"
#include <QColor>
#include <QHash>
#include <QLabel>
#include <QSet>
#include <QStyledItemDelegate>
#include <QTableWidget>
#include <QVector>
#include <QWidget>

// ---------------------------------------------------------------------------
// Sparkline: a tiny inline trend chart (area + line), no axes. Used in the
// detail panel to show the selected task's CPU history or a FIFO's fill/drops.
// ---------------------------------------------------------------------------
class Sparkline : public QWidget {
  Q_OBJECT
public:
  explicit Sparkline(QWidget *parent = nullptr);
  void setData(const QVector<double> &data, double maxHint = 0.0);
  void setColor(const QColor &c) { m_color = c; update(); }

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QVector<double> m_data;
  double m_maxHint = 0.0;
  QColor m_color{0x4f, 0xc3, 0xf7};
};

// ---------------------------------------------------------------------------
// StatCard: a compact metric tile (title, big value, subtitle, optional bar).
// ---------------------------------------------------------------------------
class StatCard : public QWidget {
  Q_OBJECT
public:
  explicit StatCard(const QString &title, QWidget *parent = nullptr);
  void setValue(const QString &v) { m_value->setText(v); }
  void setSubtitle(const QString &s) { m_subtitle->setText(s); }
  // pct < 0 hides the bar.
  void setBar(double pct, const QColor &c);

private:
  QLabel *m_value;
  QLabel *m_subtitle;
  double m_pct = -1.0;
  QColor m_barColor;
  QWidget *m_bar;
};

// ---------------------------------------------------------------------------
// BarDelegate: paints a horizontal fill bar behind right-aligned text for
// percentage columns. Percent in Qt::UserRole, bar colour in Qt::UserRole+1.
// ---------------------------------------------------------------------------
class BarDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  using QStyledItemDelegate::QStyledItemDelegate;
  void paint(QPainter *p, const QStyleOptionViewItem &opt,
             const QModelIndex &idx) const override;
};

// ---------------------------------------------------------------------------
// PerfWidget: graphical kernel-observability view. Overview cards + compact
// task/FIFO lists (state, CPU %, stack %, fill %, drops); click a row to open
// a detail panel with every field plus a live trend sparkline.
// ---------------------------------------------------------------------------
class PerfWidget : public QWidget {
  Q_OBJECT

public:
  explicit PerfWidget(QWidget *parent = nullptr);
  ~PerfWidget() override = default;

public slots:
  void updateReport(const PerfReport &report);
  // Cache a task name resolved on demand (reply to requestTaskName).
  void setTaskName(int taskId, const QString &name);

signals:
  void backToHomeRequested();
  // Emitted (at most once per id) when a task's name isn't cached yet, so the
  // host can issue a PERF_TASKNAME request to the FC.
  void requestTaskName(int taskId);

private:
  static QString stateName(uint8_t state);
  static QColor stateColor(uint8_t state);
  static QString fifoName(uint8_t id);
  static QColor stackColor(int pct);
  static QColor fillColor(int pct);

  void buildTopCards(class QHBoxLayout *row);
  void rebuildTasks(const PerfReport &r, const QHash<int, double> &cpu);
  void rebuildFifos(const PerfReport &r);
  void renderDetail();

  // Top overview cards.
  StatCard *m_cardLoad = nullptr;
  StatCard *m_cardHeap = nullptr;
  StatCard *m_cardSwitch = nullptr;
  StatCard *m_cardSysTick = nullptr;
  StatCard *m_cardIpc = nullptr;
  StatCard *m_cardUptime = nullptr;
  QLabel *m_seq = nullptr;
  QLabel *m_enabledBadge = nullptr;

  QTableWidget *m_taskTable = nullptr;
  QTableWidget *m_fifoTable = nullptr;

  // Detail panel.
  QLabel *m_detailTitle = nullptr;
  QLabel *m_detailBody = nullptr;
  QLabel *m_detailHint = nullptr;
  Sparkline *m_detailSpark = nullptr;
  QLabel *m_sparkCaption = nullptr;

  QString taskLabel(int id) const; // cached name, or "#id" fallback

  // On-demand task-name cache + ids the FC has answered (so we stop asking).
  // An id is "answered" only when a reply lands, not when a request is sent —
  // so dropped requests get retried on the next report.
  QHash<int, QString> m_nameCache;
  QSet<int> m_answered;

  // Most recent report + selection (preserved across refreshes by id).
  PerfReport m_current;
  bool m_haveCurrent = false;
  QHash<int, double> m_taskCpu; // id -> CPU% this report
  int m_selTaskId = -1;
  int m_selFifoId = -1;

  // Rolling history for sparklines, keyed by id.
  QHash<int, QVector<double>> m_taskCpuHist;
  QHash<int, QVector<double>> m_fifoFillHist;
  QHash<int, QVector<double>> m_fifoDropHist;

  // Previous-report state for CPU% deltas.
  QHash<int, uint32_t> m_taskPrevCyc;
  uint32_t m_prevCpuTotal = 0;
  bool m_havePrev = false;

  static constexpr int kHistLen = 60;
};
