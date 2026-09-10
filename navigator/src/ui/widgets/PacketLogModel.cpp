#include "PacketLogModel.h"

#include "protocol/PacketDissector.h"

#include <QBrush>
#include <QColor>
#include <QDateTime>

PacketLogModel::PacketLogModel(QObject *parent) : QAbstractTableModel(parent) {
  m_flushTimer.setInterval(33); // ~30 Hz batch flush
  connect(&m_flushTimer, &QTimer::timeout, this, &PacketLogModel::flush);
  m_flushTimer.start();
}

void PacketLogModel::enqueue(bool tx, const QByteArray &bytes) {
  if (bytes.isEmpty())
    return;
  PacketEntry e;
  e.no = ++m_counter;
  e.tMs = QDateTime::currentMSecsSinceEpoch();
  e.time = QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  e.tx = tx;
  e.raw = bytes;
  e.len = bytes.size();
  const auto *r = reinterpret_cast<const uint8_t *>(bytes.constData());
  // Spaced-hex helper, truncated so a giant payload can't bloat a row.
  auto hexSpaced = [](const QByteArray &b) {
    constexpr int kMax = 24; // bytes shown before eliding
    QByteArray shown = b.left(kMax);
    QString s = QString::fromLatin1(shown.toHex(' '));
    if (b.size() > kMax)
      s += " …";
    return s;
  };
  if (bytes.size() >= 8 && r[0] == 0x56) {
    e.type = (r[1] >> 4) & 0x0F;
    e.dev = r[3];
    e.typeName = PacketDissector::typeName(e.type);
    // NavLink frame: 8-byte header + payload + 4-byte trailing CRC32 (LE).
    if (bytes.size() >= 12) {
      const int n = bytes.size();
      const quint32 crc = quint32(r[n - 4]) | (quint32(r[n - 3]) << 8) |
                          (quint32(r[n - 2]) << 16) | (quint32(r[n - 1]) << 24);
      e.crc = QString::asprintf("0x%08X", crc);
      e.payloadHex = hexSpaced(bytes.mid(8, n - 12));
    }
  } else {
    e.type = 0xFF;
    e.typeName = "RAW";
    e.crc = "—";
    e.payloadHex = hexSpaced(bytes);
  }
  e.info = PacketDissector::summary(bytes);
  m_pending.push_back(std::move(e));
}

void PacketLogModel::flush() {
  if (m_pending.empty())
    return;

  // Evict from the front if the batch would exceed capacity.
  const int incoming = static_cast<int>(m_pending.size());
  const int cur = static_cast<int>(m_rows.size());
  int evict = cur + incoming - m_cap;
  if (evict > 0) {
    if (evict >= cur) { // whole buffer rolls over
      beginRemoveRows({}, 0, cur - 1);
      m_rows.clear();
      endRemoveRows();
    } else {
      beginRemoveRows({}, 0, evict - 1);
      m_rows.erase(m_rows.begin(), m_rows.begin() + evict);
      endRemoveRows();
    }
  }

  // Append the (possibly trimmed) batch.
  int keepFrom = 0;
  if (incoming > m_cap)
    keepFrom = incoming - m_cap; // batch alone exceeds cap: keep the newest
  const int toAdd = incoming - keepFrom;
  const int first = static_cast<int>(m_rows.size());
  beginInsertRows({}, first, first + toAdd - 1);
  for (int i = keepFrom; i < incoming; ++i)
    m_rows.push_back(std::move(m_pending[i]));
  endInsertRows();
  m_pending.clear();
}

void PacketLogModel::clearAll() {
  beginResetModel();
  m_rows.clear();
  m_pending.clear();
  m_counter = 0;
  endResetModel();
}

const PacketEntry *PacketLogModel::entry(int row) const {
  if (row < 0 || row >= static_cast<int>(m_rows.size()))
    return nullptr;
  return &m_rows[row];
}

int PacketLogModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(m_rows.size());
}

int PacketLogModel::columnCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : ColCount;
}

QVariant PacketLogModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() >= static_cast<int>(m_rows.size()))
    return {};
  const PacketEntry &e = m_rows[index.row()];

  if (role == Qt::DisplayRole) {
    switch (index.column()) {
    case ColTime:
      return e.time;
    case ColDir:
      return e.tx ? "TX" : "RX";
    case ColType:
      return e.typeName;
    case ColDev:
      return e.type == 0xFF ? QString("—") : QString::number(e.dev);
    case ColLen:
      return e.len;
    case ColCrc:
      return e.crc;
    case ColPayload:
      return e.payloadHex;
    default:
      break; /* unknown column -> empty QVariant */
    }
  } else if (role == Qt::ForegroundRole) {
    if (e.type == 0xFF)
      return QBrush(QColor("#E06C75")); // RAW/garbage rows in red
    if (index.column() == ColDir)
      return QBrush(QColor(e.tx ? "#61AFEF" : "#98C379"));
  } else if (role == Qt::TextAlignmentRole) {
    if (index.column() == ColLen || index.column() == ColDev)
      return int(Qt::AlignRight | Qt::AlignVCenter);
  }
  return {};
}

QVariant PacketLogModel::headerData(int section, Qt::Orientation o,
                                    int role) const {
  if (o != Qt::Horizontal || role != Qt::DisplayRole)
    return {};
  switch (section) {
  case ColTime:
    return "Time";
  case ColDir:
    return "Dir";
  case ColType:
    return "Type";
  case ColDev:
    return "Dev";
  case ColLen:
    return "Len";
  case ColCrc:
    return "CRC";
  case ColPayload:
    return "Payload (hex)";
  default:
    break; /* unknown section -> empty QVariant */
  }
  return {};
}
