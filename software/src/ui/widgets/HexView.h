#pragma once

#include <QByteArray>
#include <QWidget>

/**
 * Canonical hex dump (offset · 16 hex bytes · ASCII) with a highlightable byte
 * range — the "bytes" pane of the packet analyzer. setHighlight(off,len) lights
 * the bytes of the field selected in the dissection tree (Wireshark-style).
 */
class HexView : public QWidget {
  Q_OBJECT
public:
  explicit HexView(QWidget *parent = nullptr);

  void setData(const QByteArray &data);
  void setHighlight(int off, int len); // off < 0 clears
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  int rows() const;
  int charW() const;
  int lineH() const;

  QByteArray m_data;
  int m_hlOff = -1;
  int m_hlLen = 0;
};
