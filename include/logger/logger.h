#ifndef LOGGER_H
#define LOGGER_H
#include "vfs.h"
#include <stdint.h>
typedef enum {
  NAVLINK_LOGGER,
  SYSTEM_LOGGER,
  GENERAL_LOGGER,
} logger_type_t;
void logger_init(void);
void logger_write(logger_type_t logger_type, void *buffer, uint32_t len);
vfs_fd_t get_navlink_logger_fd(void);
vfs_fd_t get_system_logger_fd(void);
vfs_fd_t get_general_logger_fd(void);

/**
 * @brief Number of times a log's circular write position has wrapped
 *        (oldest records overwritten). Per-log and aggregate.
 * @implements LOG-SD-002
 */
uint32_t logger_wrap_count(logger_type_t type);
uint32_t logger_wrap_count_total(void);
#endif