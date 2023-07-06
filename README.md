# ESP32-2432S028

Basic example using esp-idf components on a [Sunton 2432S028 board](https://nl.aliexpress.com/item/1005004502250619.html).

- Display: ILI9341 using the [esp_lcd_ili9341](https://components.espressif.com/components/espressif/esp_lcd_ili9341) component by Espressif.
- Touch: XPT2046 using the [esp_lcd_touch_xpt2046](https://components.espressif.com/components/atanisoft/esp_lcd_touch_xpt2046) component by Atanisoft.

A very basic lvgl example just to show both components working with the hardware.

For information about the pcb and available GPIO connections, check the excellent work by [Macsbug](https://macsbug.wordpress.com/2022/08/17/esp32-2432s028/).

### Building
    git clone https://github.com/limpens/esp32-2432S028
    cd esp32-2432S028
    source your idf exports if necessary
    idf.py build flash monitor
