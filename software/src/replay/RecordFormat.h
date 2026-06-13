#pragma once

#include <QByteArray>
#include <QDataStream>
#include <QtGlobal>

// On-disk format for a recorded telemetry session (.bin), written live by
// RecordSink (FR-LOG-05) and consumed by replay (ReplaySource, Phase 2D).
//
// Layout (little-endian throughout):
//   header:  [magic:u32][formatVersion:u32][protocolVersion:u32]
//            [startWallClockMs:u64]
//   records: N x  [t_us:u64][len:u32][bytes:len]
//
// t_us is a monotonic microsecond timestamp supplied by the recorder; only
// inter-frame deltas matter for replay, so the time base need not be zero on
// the first frame (ReplaySource normalises). The record stores raw inbound
// byte chunks, not decoded packets, so the replay decode path is identical to
// live and new packet types replay for free.
namespace RecordFormat {

inline constexpr quint32 kMagic = 0x56524543u;  // "VREC"
inline constexpr quint32 kVersion = 1u;
inline constexpr QDataStream::ByteOrder kByteOrder = QDataStream::LittleEndian;

struct Header {
  quint32 magic = kMagic;
  quint32 formatVersion = kVersion;
  quint32 protocolVersion = 0;
  quint64 startWallClockMs = 0;
};

struct Frame {
  quint64 tUs = 0;
  QByteArray bytes;
};

}  // namespace RecordFormat
