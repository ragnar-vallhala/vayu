/*
 * host_main.c -- entry point for the standalone vayu_sitl binary.
 * Just sets up signal handling and forwards to vayu_sitl_start()
 * with no iface (legacy /tmp FIFO mode). The lifecycle code itself
 * lives in host_lifecycle.c so the Navigator host (which has its
 * own main()) shares the same boot path.
 */
#define _GNU_SOURCE
#include "host_clock.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"     /* v_get_ticks */
#include "vaios.h"
#include "vsim_iface.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

extern volatile int g_vayu_sitl_running;   /* defined in host_lifecycle.c */

static void on_sigint(int sig) { (void)sig; vayu_sitl_stop(); }

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    signal(SIGINT, on_sigint);

    const char *pt_env = getenv("VAYU_SITL_PASSTHROUGH");
    if (pt_env && pt_env[0] && pt_env[0] != '0') vayu_sitl_set_passthrough(1);

    if (vayu_sitl_start(NULL) != 0) {
        fprintf(stderr, "vayu_sitl: start failed\n");
        return EXIT_FAILURE;
    }

    while (g_vayu_sitl_running) {
        host_wall_delay_ms(1000);  /* process-alive heartbeat: wall time, not
                                    * sim time (must tick even if sim is idle) */
        fprintf(stderr, "host_main: alive @ t=%u ms state=0x%x\n",
                v_get_ticks(), (unsigned)system_state_get());
    }

    fprintf(stderr, "vayu_sitl: shutting down\n");
    return EXIT_SUCCESS;
}
