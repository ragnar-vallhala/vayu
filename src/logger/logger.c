#include "logger/logger.h"
#include "ipc.h"
#include "vaios_config_default.h"
#include "variables.h"
#include "vfs.h"

/* =========================
 * File descriptors
 * ========================= */
static volatile vfs_fd_t navlink_logger_fd = -1;
static volatile vfs_fd_t system_logger_fd = -1;
static volatile vfs_fd_t general_logger_fd = -1;

/* =========================
 * Write positions (CRITICAL FIX)
 * ========================= */
static uint32_t navlink_write_pos = 0;
static uint32_t system_write_pos = 0;
static uint32_t general_write_pos = 0;

/* =========================
 * Mutexes
 * ========================= */
static MutexHandle_t navlink_logger_mutex;
static MutexHandle_t system_logger_mutex;
static MutexHandle_t general_logger_mutex;

/* =========================
 * Internal helper
 * ========================= */
static inline void logger_write_internal(vfs_fd_t fd, MutexHandle_t mutex,
                                         uint32_t *write_pos,
                                         uint32_t file_size,
                                         const uint8_t *data, uint32_t len) {
  if (fd < 0 || data == NULL || len == 0)
    return;

  v_mutex_lock(mutex, 0);

  /* Ensure we never overflow */
  if (*write_pos > (file_size - len)) {
    *write_pos = 0;
  }

  /* Seek to deterministic position */
  vfs_lseek(fd, (long)(*write_pos), VFS_SEEK_SET);

  int res = vfs_write(fd, data, len);
  int sync_res = vfs_sync(fd);

  if (res < 0 || sync_res < 0) {

  } else {
    /* Advance circularly */
    *write_pos = (*write_pos + len) % (file_size - len);
  }

  v_mutex_unlock(mutex);
}

static void ensure_file_size(vfs_fd_t fd, uint32_t file_size) {
  int current_size = vfs_lseek(fd, 0, VFS_SEEK_END);

  if (current_size < (int)file_size) {

    vfs_lseek(fd, file_size - 1, VFS_SEEK_SET);

    uint8_t dummy = 0;
    vfs_write(fd, &dummy, 1);
    vfs_sync(fd);
  }

  /* Reset pointer */
  vfs_lseek(fd, 0, VFS_SEEK_SET);
}

/* =========================
 * Initialization
 * ========================= */
void logger_init(void) {

  navlink_logger_mutex = v_mutex_create();
  system_logger_mutex = v_mutex_create();
  general_logger_mutex = v_mutex_create();

  /* Preallocate files */
  if (vfs_preallocate(NAVLINK_LOGGING_FILENAME, NAVLINK_LOGGING_FILE_SIZE) !=
      0) {
    PANIC("Navlink prealloc failed");
  }

  if (vfs_preallocate(SYS_LOGGING_FILENAME, SYS_LOGGING_FILE_SIZE) != 0) {
    PANIC("System prealloc failed");
  }

  if (vfs_preallocate(GENERAL_LOGGING_FILENAME, GENERAL_LOGGING_FILE_SIZE) !=
      0) {
    PANIC("General prealloc failed");
  }

  /* Open files */
  navlink_logger_fd =
      vfs_open(NAVLINK_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);

  system_logger_fd = vfs_open(SYS_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);

  general_logger_fd =
      vfs_open(GENERAL_LOGGING_FILENAME, VFS_O_RDWR | VFS_O_CREAT);

  ensure_file_size(navlink_logger_fd, NAVLINK_LOGGING_FILE_SIZE);
  ensure_file_size(system_logger_fd, SYS_LOGGING_FILE_SIZE);
  ensure_file_size(general_logger_fd, GENERAL_LOGGING_FILE_SIZE);

  if (navlink_logger_fd < 0 || system_logger_fd < 0 || general_logger_fd < 0) {
    PANIC("Logger open failed");
  }

  /* Reset write pointers */
  navlink_write_pos = 0;
  system_write_pos = 0;
  general_write_pos = 0;
}

/* =========================
 * Public API
 * ========================= */
void logger_write(logger_type_t type, void *buffer, uint32_t len) {
  if (!buffer || len == 0)
    return;

  switch (type) {
  case NAVLINK_LOGGER:
    logger_write_internal(navlink_logger_fd, navlink_logger_mutex,
                          &navlink_write_pos, NAVLINK_LOGGING_FILE_SIZE, buffer,
                          len);
    break;

  case SYSTEM_LOGGER:
    logger_write_internal(system_logger_fd, system_logger_mutex,
                          &system_write_pos, SYS_LOGGING_FILE_SIZE, buffer,
                          len);
    break;

  case GENERAL_LOGGER:
  default:
    logger_write_internal(general_logger_fd, general_logger_mutex,
                          &general_write_pos, GENERAL_LOGGING_FILE_SIZE, buffer,
                          len);
    break;
  }
}

/* =========================
 * Getters
 * ========================= */
vfs_fd_t get_navlink_logger_fd(void) { return navlink_logger_fd; }
vfs_fd_t get_system_logger_fd(void) { return system_logger_fd; }
vfs_fd_t get_general_logger_fd(void) { return general_logger_fd; }