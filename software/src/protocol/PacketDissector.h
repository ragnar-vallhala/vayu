#pragma once

#include <QByteArray>
#include <QString>
#include <QVector>
#include <cstdint>

/**
 * One node in a packet's field tree (Wireshark-style dissection).
 *
 * `off`/`len` are the byte range this field occupies in the *raw packet* (off
 * measured from byte 0 = sync), used to highlight the bytes in the hex pane.
 * Group nodes that don't map to a contiguous range use off = -1.
 */
struct DissectField {
  QString name;
  QString value;
  int off = -1;
  int len = 0;
  QVector<DissectField> children;

  DissectField() = default;
  DissectField(QString n, QString v, int o = -1, int l = 0)
      : name(std::move(n)), value(std::move(v)), off(o), len(l) {}

  DissectField &addChild(const QString &n, const QString &v, int o = -1,
                         int l = 0) {
    children.append(DissectField(n, v, o, l));
    return children.last();
  }
};

/**
 * Stateless, byte-level dissector for the vayu telemetry wire format. Unlike
 * PacketDecoder it works on a *single* raw packet (no perf reassembly), so even
 * one PERF_STATS fragment is fully broken out. Used by the packet-analyzer
 * detail pane; `summary()` is the cheap one-liner cached in the packet list.
 */
class PacketDissector {
public:
  // Full field tree for the detail pane.
  static DissectField dissect(const QByteArray &data);
  // Cheap one-line description for the "Info" column (no tree allocation).
  static QString summary(const QByteArray &data);
  // Short name for a packet type nibble.
  static QString typeName(uint8_t type);
  // Origin name for SYSTEM_STATUS payload[0].
  static QString originName(uint8_t origin);
};
