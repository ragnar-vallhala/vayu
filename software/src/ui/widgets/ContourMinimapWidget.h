// ContourMinimapWidget.h — top-down contour minimap of the procedural terrain.
//
// A toggleable PiP overlay (like HORIZON / DOWN CAM) that helps judge terrain
// height, which is hard to read in the perspective 3D view. North-up,
// hypsometric-tinted + hillshaded, with contour lines and a heading marker at
// the centre. It draws from a height-sampler callback so it's decoupled from
// how the terrain is generated (finite arena heightfield or endless field).
#pragma once

#include <QWidget>

#include <functional>

class ContourMinimapWidget : public QWidget {
  Q_OBJECT
 public:
  explicit ContourMinimapWidget(QWidget* parent = nullptr);

  // World-(north,east) -> elevation [m] above the ground plane. Null clears the
  // map (shows a "no terrain" placeholder).
  void setSampler(std::function<float(float, float)> sampler);
  bool hasSampler() const { return static_cast<bool>(sampler_); }

  // Centre the map on world (north, east) metres, oriented by NED heading
  // (radians; 0 = north). Triggers a repaint.
  void setView(float north, float east, float headingRad);

  // Half-extent shown each side of centre [m] (so the map spans 2*range).
  void setRangeM(float r);
  // Elevation that maps to the top of the hypsometric ramp [m].
  void setReferenceHeight(float m) { refHeightM_ = m > 1.0f ? m : 1.0f; }
  // Spacing between contour lines [m].
  void setContourInterval(float m) { contourM_ = m > 0.1f ? m : 0.1f; }

 protected:
  void paintEvent(QPaintEvent*) override;

 private:
  std::function<float(float, float)> sampler_;
  float n_ = 0.0f, e_ = 0.0f, headingRad_ = 0.0f;
  float rangeM_ = 200.0f;
  float refHeightM_ = 80.0f;
  float contourM_ = 10.0f;
};
