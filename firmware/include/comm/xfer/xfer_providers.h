/**
 * @file include/comm/xfer/xfer_providers.h
 * @brief Concrete xfer providers (file / log / stream) + their registration.
 *
 * Each provider is a codec-blind xfer_provider_t backed by the fs_owner SD
 * gateway (file/log) or a telemetry source (stream). xfer_service_task calls
 * xfer_providers_register_all() at boot. Service ids are the wire `service_id`
 * in XFER_OPEN (navlink/dialect.json). Design: docs/plans/navlink-xfer-substrate.md.
 */
#ifndef XFER_PROVIDERS_H
#define XFER_PROVIDERS_H

#include <stdint.h>

#include "comm/xfer/navlink_xfer.h"

/* Wire service ids (XFER_OPEN.service_id). */
#define XFER_SVC_FILE 0u   /* generic read/write any SD path (up + down)       */
#define XFER_SVC_LOG 1u    /* download-only: the circular blackbox files       */
#define XFER_SVC_STREAM 2u /* download stream: a named live telemetry source   */

/* Individual registrations (each adds one provider to the SM registry). */
int file_provider_register(void);
int log_provider_register(void);
int stream_provider_register(void);

/* Register all three. Returns 0 if all succeeded, <0 if any failed. */
int xfer_providers_register_all(void);

/* ---- stream sources -----------------------------------------------------
 * The stream provider is generic: a live source is a named poll function that
 * fills `buf` with the next due sample (<= max bytes) and returns the byte
 * count, or 0 if nothing is due. Firmware/tests register sources by name; the
 * XFER_OPEN `arg` selects one. */
typedef int (*xfer_stream_source_fn)(uint8_t *buf, uint16_t max);
int xfer_stream_register_source(const char *name, xfer_stream_source_fn poll);

#endif /* XFER_PROVIDERS_H */
