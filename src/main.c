#include "memory.h"
#include "task.h"
#include "utils/physical_heartbeat.h"
#include "vaios.h"

int main() {
  v_init();
  v_heap_memory_init();

  scheduler_init();
  // Task Create
  task_create(physical_heartbeat, NULL,
              128,0); // equal priority to idle task will indicate no starvation
  scheduler_start();
  while (1)
    ;
}
