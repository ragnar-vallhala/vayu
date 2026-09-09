#pragma once

#include <QAbstractTableModel>
#include <QByteArray>
#include <QString>
#include <QTimer>
#include <cstdint>
#include <deque>
#include <vector>

/**
 * One row in the packet log. Derived fields are computed ONCE at enqueue so the
 * model's data() is O(1) and never re-decodes during painting.
 */
struct PacketEntry {
  quint64 no = 0;     // stable packet number (Wireshark "No.")
  qint64 tMs = 0;     // epoch ms (for expression filters / sort)
  QString time;       // cached "HH:mm:ss.zzz"
  bool tx = false;    // direction: false = RX, true = TX
  quint8 type = 0xFF; // type nibble (0xFF = RAW / non-frame)
  quint8 dev = 0;
  int len = 0;
  QString typeName;
  QString info; // one-line summary (kept for the display-filter expression)
  QString crc;  // trailing CRC32 as "0x........" (frames only)
  QString payloadHex; // payload bytes as spaced hex (truncated for display)
  QByteArray raw;
};

/**
 * Table model over a fixed-capacity ring of packets. Incoming packets are
 * staged via enqueue() and flushed to the view in batches on a timer, so the
 * UI is decoupled from the packet rate (no per-packet layout, no hang).
 */
class PacketLogModel : public QAbstractTableModel {
  Q_OBJECT
public:
  // Columns match the mockup: Time | Dir | Type | Dev | Len | CRC | Payload (hex).
  enum Column {
    ColTime,
    ColDir,
    ColType,
    ColDev,
    ColLen,
    ColCrc,
    ColPayload,
    ColCount
  };

  explicit PacketLogModel(QObject *parent = nullptr);

  // Stage a packet; it appears after the next batch flush (~30 Hz).
  void enqueue(bool tx, const QByteArray &bytes);
  void clearAll();
  void setCapacity(int cap) { m_cap = qMax(1, cap); }

  const PacketEntry *entry(int row) const;

  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;

private slots:
  void flush();

private:
  std::deque<PacketEntry> m_rows;
  std::vector<PacketEntry> m_pending;
  int m_cap = 5000;
  quint64 m_counter = 0;
  QTimer m_flushTimer;
};
