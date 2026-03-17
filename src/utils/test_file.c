#include "navhal.h"
#include "vaios.h"
#include "logger/logger.h"
#include "vfs.h"
int write_pos = 0;
void test_task(void *args) {
  while (1) {
    logger_write(GENERAL_LOGGER, "Hello Single Task!\n", 19);
    // vfs_fd_t fd = vfs_open("0:test.txt", VFS_O_RDWR | VFS_O_CREAT);
    // vfs_lseek(fd, write_pos, VFS_SEEK_SET);
    // uart2_write("fd: ");
    // uart2_write(fd);
    // uart2_write("\n\r");

    // if (fd >= 0) {
    //   int n = vfs_write(fd, "Hello Single Task!\n", 19);
    //   vfs_sync(fd);
    //   vfs_close(fd);
    //   write_pos += 19;
    //   uart2_write("n: ");
    //   uart2_write(n);
    //   uart2_write("\n\r");
    // };

    v_delay(500);
  }
}