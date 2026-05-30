/**
 * @file comm.h
 * @brief Public umbrella header for the communications module (COMM).
 *
 * @implements R2.1
 *
 * Single public entry point for the comms subsystem: the NavLink packet
 * protocol/types, the serial channel layer, packet (de)serialisation, the
 * iBUS RC link, and the RC sample buffer. External modules include only
 * this header.
 */
#ifndef VAYU_COMM_H
#define VAYU_COMM_H

#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/deserializer.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "comm/serializer.h"

#endif // VAYU_COMM_H
