#pragma once

#include <QString>
#include <QWidget>

// Vertical bar gauge matching the mockup's .vgauge (Dashboard temp-gauge
// column): a caption, a rounded track with quarter ticks + tick labels, a
// gradient fill scaled to the value, and a numeric readout below.
//
// Two palettes are provided: Temp (cold-blue → hot-red, bottom→top) and
// Battery (low-red → high-green). The readout string is formatted from the
// value + unit by default, or set explicitly via setReadoutText().
class VGauge : public QWidget {
  Q_OBJECT

public:
  enum Palette { Temp, Battery };

  VGauge(const QString &caption, double minVal, double maxVal,
         const QString &unit, Palette pal, QWidget *parent = nullptr);

  // Set the current value (clamped to [min,max]); marks the gauge as having
  // live data and refreshes the readout.
  void setValue(double v);
  // Mark the gauge as having no vehicle data: empty track + "-" readout, so an
  // absent telemetry source is distinct from a real zero.
  void setUnavailable();
  // Override the readout label (e.g. while a telemetry source is absent).
  void setReadoutText(const QString &text);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  QString m_caption;
  QString m_unit;
  double m_min;
  double m_max;
  double m_value;
  Palette m_palette;
  QString m_readout;
  bool m_readoutPinned = false;
  bool m_hasData = false;  // false → empty track + "-" readout (no vehicle data)
};
