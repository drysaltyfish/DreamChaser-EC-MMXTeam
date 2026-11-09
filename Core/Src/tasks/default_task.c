#include "default_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app/app.h"

void StartDefaultTask(void* argument) {
    (void)argument;

    App_Init();
    vTaskDelete(NULL);
}
