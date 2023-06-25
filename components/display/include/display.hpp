#pragma once

#include <functional>
#include "lvgl.h"

void initDisplay(std::function<void(lv_disp_t*)> render);
