#include "RecordSink.h"

RecordSink::~RecordSink() { close(); }

bool RecordSink::open(const QString &path, quint32 protocolVersion,
                      quint64 startWallClockMs) {
  close();
  m_file.setFileName(path);
  if (!m_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    return false;

  QDataStream s(&m_file);
  s.setByteOrder(RecordFormat::kByteOrder);
  s << RecordFormat::kMagic << RecordFormat::kVersion << protocolVersion
    << startWallClockMs;
  if (s.status() != QDataStream::Ok) {
    close();
    return false;
  }
  return true;
}

void RecordSink::writeFrame(quint64 tUs, const QByteArray &bytes) {
  if (!m_file.isOpen())
    return;
  QDataStream s(&m_file);
  s.setByteOrder(RecordFormat::kByteOrder);
  // Explicit [t_us][len][raw bytes] — not QDataStream's QByteArray operator,
  // which prepends its own length and would couple us to the stream version.
  s << quint64(tUs) << quint32(bytes.size());
  if (!bytes.isEmpty())
    s.writeRawData(bytes.constData(), bytes.size());
}

void RecordSink::close() {
  if (m_file.isOpen()) {
    m_file.flush();
    m_file.close();
  }
}

// ---------------------------------------------------------------------------

bool RecordReader::open(const QString &path) {
  close();
  m_file.setFileName(path);
  if (!m_file.open(QIODevice::ReadOnly))
    return false;

  QDataStream s(&m_file);
  s.setByteOrder(RecordFormat::kByteOrder);
  s >> m_header.magic >> m_header.formatVersion >> m_header.protocolVersion >>
      m_header.startWallClockMs;
  if (s.status() != QDataStream::Ok || m_header.magic != RecordFormat::kMagic) {
    close();
    return false;
  }
  return true;
}

bool RecordReader::next(RecordFormat::Frame &out) {
  if (!m_file.isOpen() || m_file.atEnd())
    return false;

  QDataStream s(&m_file);
  s.setByteOrder(RecordFormat::kByteOrder);
  quint32 len = 0;
  s >> out.tUs >> len;
  if (s.status() != QDataStream::Ok)
    return false;

  out.bytes.resize(int(len));
  if (len > 0) {
    const int got = s.readRawData(out.bytes.data(), int(len));
    if (got != int(len))
      return false; // truncated record
  }
  return true;
}

void RecordReader::close() {
  if (m_file.isOpen())
    m_file.close();
}
