#pragma once

#include <QStringList>
#include <QWidget>

// AUX segmented switch indicator (mockup .sw2/.sw3): a labelled row of segments
// with the active one highlighted, driven by an RC aux channel value. 2-way
// splits at 1500µs; 3-way splits at 1300/1700µs.
class AuxSwitch : public QWidget {
  Q_OBJECT

public:
  AuxSwitch(const QString &label, const QStringList &segments,
            QWidget *parent = nullptr);

  // Set the active segment directly, or from an RC channel value (µs).
  void setActive(int index);
  void setFromChannel(int us);

protected:
  void paintEvent(QPaintEvent *event) override;
  QSize sizeHint() const override { return {220, 22}; }

private:
  QString m_label;
  QStringList m_segments;
  int m_active = -1; // -1 = N/A
};
