#include "vaios.h"
#include "utils.h"
#include "task.h"
#include "memory.h"
#include "navhal.h"
#include "sensor/bmx160.h"

int count = 0;

void task1(void *arg)
{
    // while (1)
    // {
    //     count++;
    //     v_log(LOG_INFO, "Entered task %d, count %d", GET_CURRENT_TASK_ID(), count);
    // }
}
void run_bmx();

int main()
{
    v_init();
    v_heap_memory_init();
    v_log(LOG_INFO, "VAIOS Started");
    scheduler_init();
    task_create(task1, NULL, 512, 0);
    task_create(run_bmx, NULL, 2048, 0);
    scheduler_start();
    while (1)
        ;
}
