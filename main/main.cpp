#include "UI.hpp"
#include "display.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

constexpr auto UI_TASK_PRIORITY = 5;

void renderUITask(void* args) {
  initDisplay(renderUI);
}

extern "C" void app_main() {
  xTaskCreatePinnedToCore(renderUITask, "render_ui_task", 4096, NULL,
                          UI_TASK_PRIORITY, NULL, 1);
}
