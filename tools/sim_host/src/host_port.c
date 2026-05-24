/*
 * host_port.c -- definitions for the vaios host port globals declared in
 * tools/sim_host/include/port.h.
 */
#include "port.h"

pthread_mutex_t host_critical_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile uint32_t critical_nesting = 0;
