#include "display.hpp"
#include <lvgl.h>
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_timer.h"
#include "esp_timer_cxx.hpp"
#include "gpio_cxx.hpp"
#include "i2c_cxx.hpp"

namespace {

// should be at least ~10% of screen height
constexpr auto BUFFER_HEIGHT = 170;
constexpr auto LCD_WIDTH = 320;
constexpr auto LCD_HEIGHT = 170;
constexpr auto PSRAM_DATA_ALIGNMENT = 64;
constexpr auto LVGL_TICK_PERIOD_MS = 2;

TaskHandle_t currentTask;
lv_disp_draw_buf_t lvDispBuf;
lv_disp_drv_t lvDispDrv;
lv_indev_drv_t lvTouchDrv;

void IRAM_ATTR onLVGLTickTimer() {
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void IRAM_ATTR onLCDTouch(esp_lcd_touch_handle_t lcdTouchHandle) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveIndexedFromISR(currentTask, 0, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool onLCDDataTransfer(esp_lcd_panel_io_handle_t lcdPanelIO,
                       esp_lcd_panel_io_event_data_t* data,
                       void* user_ctx) {
  lv_disp_flush_ready((lv_disp_drv_t*)user_ctx);
  return false;
}

void onLVGLFlush(lv_disp_drv_t* drv,
                 const lv_area_t* area,
                 lv_color_t* colorData) {
  auto lcdPanel = (esp_lcd_panel_handle_t)drv->user_data;
  esp_lcd_panel_draw_bitmap(lcdPanel, area->x1, area->y1, area->x2 + 1,
                            area->y2 + 1, colorData);
}

void onLVGLTouchRead(lv_indev_drv_t* drv, lv_indev_data_t* inputData) {
  // I still have 0 idea why x and y are arrays and have 0 as their 2 element
  // but this is how it is in the examples
  uint16_t touchpadX[1] = {0};
  uint16_t touchpadY[1] = {0};
  uint8_t touchpadCount = 0;

  esp_lcd_touch_handle_t lcdTouchHandle =
      (esp_lcd_touch_handle_t)drv->user_data;

  if (ulTaskNotifyTakeIndexed(0, pdTRUE, 0)) {
    esp_lcd_touch_read_data(lcdTouchHandle);
  }

  bool isPressed = esp_lcd_touch_get_coordinates(
      lcdTouchHandle, touchpadX, touchpadY, NULL, &touchpadCount, 1);

  if (isPressed && touchpadCount > 0) {
    inputData->point.x = touchpadX[0];
    // mirror in y - can't be done with the config flag as it's applied before
    // the swap, not after which is what I need
    inputData->point.y = LCD_HEIGHT - touchpadY[0];
    inputData->state = LV_INDEV_STATE_PRESSED;
  } else {
    inputData->state = LV_INDEV_STATE_RELEASED;
  }
}

lv_disp_t* initLCD() {
  // LCD_PWR, LCD_BL, LCD_RD
  idf::GPIO_Output(idf::GPIONum(15)).set_high();
  idf::GPIO_Output(idf::GPIONum(38)).set_high();
  idf::GPIO_Output(idf::GPIONum(9)).set_high();

  esp_lcd_i80_bus_handle_t lcdI80BusHandle;
  esp_lcd_i80_bus_config_t lcdI80BusConfig{
      .dc_gpio_num = GPIO_NUM_7,
      .wr_gpio_num = GPIO_NUM_8,
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .data_gpio_nums{39, 40, 41, 42, 45, 46, 47, 48},
      .bus_width = 8,
      .max_transfer_bytes = LCD_WIDTH * BUFFER_HEIGHT * sizeof(uint16_t),
      .psram_trans_align = PSRAM_DATA_ALIGNMENT,
      .sram_trans_align = 4,
  };

  ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&lcdI80BusConfig, &lcdI80BusHandle));

  esp_lcd_panel_io_handle_t lcdPanelIOHandle;
  esp_lcd_panel_io_i80_config_t lcdPanelIOConfig{
      .cs_gpio_num = GPIO_NUM_6,
      // should be no less than ~LCD_WIDTH * LCD_HEIGHT * refresh_rate
      // supposedly even 10mhz can be used here if drive capability is increased
      .pclk_hz = 4 * 1000 * 1000,
      .trans_queue_depth = 10,
      .on_color_trans_done = onLCDDataTransfer,
      .user_ctx = &lvDispDrv,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .dc_levels =
          {
              .dc_idle_level = 0,
              .dc_cmd_level = 0,
              .dc_dummy_level = 0,
              .dc_data_level = 1,
          },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(lcdI80BusHandle, &lcdPanelIOConfig,
                                           &lcdPanelIOHandle));

  esp_lcd_panel_handle_t lcdPanelHandle;
  esp_lcd_panel_dev_config_t lcdPanelDevConfig{.reset_gpio_num = GPIO_NUM_5,
                                               .rgb_endian = LCD_RGB_ENDIAN_RGB,
                                               .bits_per_pixel = 16};
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcdPanelIOHandle, &lcdPanelDevConfig,
                                           &lcdPanelHandle));

  esp_lcd_panel_reset(lcdPanelHandle);
  esp_lcd_panel_init(lcdPanelHandle);

  esp_lcd_panel_invert_color(lcdPanelHandle, true);
  esp_lcd_panel_swap_xy(lcdPanelHandle, true);
  esp_lcd_panel_mirror(lcdPanelHandle, true, false);
  esp_lcd_panel_set_gap(lcdPanelHandle, 0, 35);

  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcdPanelHandle, true));

  lv_init();

  // 2 buffers are recommended for better performance
  lv_color_t *lvBuffer1, *lvBuffer2;

  lvBuffer1 = (lv_color_t*)heap_caps_aligned_alloc(
      PSRAM_DATA_ALIGNMENT, LCD_WIDTH * BUFFER_HEIGHT * sizeof(lv_color_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  lvBuffer2 = (lv_color_t*)heap_caps_aligned_alloc(
      PSRAM_DATA_ALIGNMENT, LCD_WIDTH * BUFFER_HEIGHT * sizeof(lv_color_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  assert(lvBuffer1 && lvBuffer2);

  lv_disp_draw_buf_init(&lvDispBuf, lvBuffer1, lvBuffer2,
                        LCD_WIDTH * BUFFER_HEIGHT);
  lv_disp_drv_init(&lvDispDrv);

  lvDispDrv.hor_res = LCD_WIDTH;
  lvDispDrv.ver_res = LCD_HEIGHT;
  lvDispDrv.draw_buf = &lvDispBuf;
  lvDispDrv.user_data = lcdPanelHandle;
  lvDispDrv.flush_cb = onLVGLFlush;

  auto lvDisp = lv_disp_drv_register(&lvDispDrv);

  auto lvglTickTimer =
      new idf::esp_timer::ESPTimer(onLVGLTickTimer, "lvglTick");
  lvglTickTimer->start_periodic(
      std::chrono::microseconds(LVGL_TICK_PERIOD_MS * 1000));

  return lvDisp;
}

void initTouch(lv_disp_t* lvDisp) {
  new idf::I2CMaster(idf::I2CNumber::I2C0(), idf::SCL_GPIO(17),
                     idf::SDA_GPIO(18), idf::Frequency::MHz(1));

  esp_lcd_panel_io_handle_t lcdTouchIOHandle;

  esp_lcd_panel_io_i2c_config_t lcdTouchIOConfig =
      ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0,
                                           &lcdTouchIOConfig,
                                           &lcdTouchIOHandle));

  esp_lcd_touch_config_t lcdTouchConfig{
      .x_max = LCD_WIDTH,
      .y_max = LCD_HEIGHT,
      .rst_gpio_num = GPIO_NUM_21,
      .int_gpio_num = GPIO_NUM_16,
      .levels{.reset = 0, .interrupt = 0},
      .flags{.swap_xy = 1, .mirror_x = 0, .mirror_y = 0},
      .interrupt_callback = onLCDTouch,
  };

  esp_lcd_touch_handle_t lcdTouchHandle;

  esp_lcd_touch_new_i2c_cst816s(lcdTouchIOHandle, &lcdTouchConfig,
                                &lcdTouchHandle);

  lv_indev_drv_init(&lvTouchDrv);

  lvTouchDrv.type = LV_INDEV_TYPE_POINTER;
  lvTouchDrv.read_cb = onLVGLTouchRead;
  lvTouchDrv.disp = lvDisp;
  lvTouchDrv.user_data = lcdTouchHandle;

  lv_indev_drv_register(&lvTouchDrv);
}
}  // namespace

void initDisplay(std::function<void(lv_disp_t*)> render) {
  currentTask = xTaskGetCurrentTaskHandle();

  auto lvDisp = initLCD();

  initTouch(lvDisp);

  render(lvDisp);

  while (true) {
    uint32_t nextDelay = lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(nextDelay));
  }
}
