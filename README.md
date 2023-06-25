### ESP-IDF setup for LilyGO T-Display-S3 with touch support

Uses C++ with exceptions enabled, esp_lcd as display driver with LVGL for UI rendering

esp-idf-cxx in IDF registry is currently bugged and can't be built, if e.g. their I2C component is used, until a release
with [this PR](https://github.com/espressif/esp-idf-cxx/pull/21/), so for now I moved it to components, patched and
added to .gitignore.

There's also a component for SCD30 sensor, I plan to implement HALs for all sensor
used [here](https://github.com/real-marshal/esp32-aq-monitor) at some point, but I'm not sure yet if
I really need this.