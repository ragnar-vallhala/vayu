#pragma once

#include "../../core/Types.h"
#include <QLabel>
#include <QProgressBar>
#include <QVector>
#include <QWidget>

class RcChannelsWidget : public QWidget {
  Q_OBJECT

public:
  explicit RcChannelsWidget(QWidget *parent = nullptr);

signals:
  void backToHomeRequested();

public slots:
  void updateChannels(const RcData &data);

private:
  QVector<QProgressBar *> m_bars;
  QVector<QLabel *> m_labels;
};
