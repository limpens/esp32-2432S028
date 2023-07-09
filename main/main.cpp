
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_xpt2046.h"
#include "lvgl.h"

#define CONFIG_LCD_HRES     320
#define CONFIG_LCD_VRES     240
#define CONFIG_LCD_BUF_SIZE (CONFIG_LCD_HRES * 30)

#include "hardware.h"

#include "sdkconfig.h"

#define TAG _PROJECT_NAME_

static SemaphoreHandle_t xGuiSemaphore = NULL;
static TaskHandle_t g_lvgl_task_handle;

void lvgl_acquire(void);
void lvgl_release(void);

//
// just for demo, should use ledc_channel to pwm the led
//
void InitBacklight(void)
{
    gpio_set_direction(CONFIG_LCD_BACKLIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_LCD_BACKLIGHT, 1);
}

static bool lvgl_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);

    return false;
}

//
// Setup lcd panel
//
esp_lcd_panel_handle_t InitDisplay(lv_disp_drv_t *disp_drv)
{
    esp_lcd_panel_io_handle_t lcd_io_handle = nullptr;
    esp_lcd_panel_handle_t lcd_panel = nullptr;

    esp_lcd_panel_io_spi_config_t lcd_io_config = { .cs_gpio_num = (gpio_num_t)CONFIG_LCD_CS,
        .dc_gpio_num = CONFIG_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = CONFIG_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 3,
        .on_color_trans_done = lvgl_notify_flush_ready,
        .user_ctx = disp_drv,
        .lcd_cmd_bits = CONFIG_LCD_CMD_BITS,
        .lcd_param_bits = CONFIG_LCD_PARAM_BITS,
        .flags = { .dc_low_on_data = 0, .octal_mode = 0, .sio_mode = 0, .lsb_first = 0, .cs_high_active = 0 } };

    const spi_bus_config_t lcd_spi_buscfg = { .mosi_io_num = CONFIG_LCD_SPI_MOSI,
        .miso_io_num = CONFIG_LCD_SPI_MISO,
        .sclk_io_num = CONFIG_LCD_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = CONFIG_LCD_BUF_SIZE * sizeof(uint16_t),
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
        .isr_cpu_id = INTR_CPU_ID_AUTO,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM };

    const esp_lcd_panel_dev_config_t lcd_panel_devcfg
        = { .reset_gpio_num = GPIO_NUM_NC, .rgb_endian = LCD_RGB_ENDIAN_RGB, .bits_per_pixel = 16, .flags = { .reset_active_high = 0 }, .vendor_config = nullptr };

    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_LCD_SPI, &lcd_spi_buscfg, SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_LCD_SPI, &lcd_io_config, &lcd_io_handle));
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(lcd_io_handle, &lcd_panel_devcfg, &lcd_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));

    return lcd_panel;
}

//
// Fix coordinates
//
uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    uint16_t value = (n - in_min) * (out_max - out_min) / (in_max - in_min);
    return (value < out_min) ? out_min : ((value > out_max) ? out_max : value);
}

//
// TOUCH_*_MAX and TOUCH_*_MIN values are determined by printing them here and clicking on the outer edges of the display
//
void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    // ESP_LOGI(TAG, "PRE process_coordinates: X: %" PRIu16 " Y: %" PRIu16, *x, *y);
    *x = map(*x, TOUCH_X_RES_MIN, TOUCH_X_RES_MAX, 0, CONFIG_LCD_HRES);
    *y = map(*y, TOUCH_Y_RES_MIN, TOUCH_Y_RES_MAX, 0, CONFIG_LCD_VRES);
    // ESP_LOGI(TAG, "POST process_coordinates: X: %" PRIu16 " Y: %" PRIu16, *x, *y);
}

//
// Configure/setup touch driver
//
esp_lcd_touch_handle_t InitTouchPanel(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = nullptr;
    static esp_lcd_touch_handle_t tp = NULL;

    // See ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG macro
    const esp_lcd_panel_io_spi_config_t tp_io_config = { .cs_gpio_num = CONFIG_TOUCH_CS,
        .dc_gpio_num = CONFIG_TOUCH_DC,
        .spi_mode = 0,
        .pclk_hz = CONFIG_TOUCH_CLOCK_HZ,
        .trans_queue_depth = 3,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = { .dc_low_on_data = 0, .octal_mode = 0, .sio_mode = 0, .lsb_first = 0, .cs_high_active = 0 } };

    static const int SPI_MAX_TRANSFER_SIZE = 32768;
    const spi_bus_config_t buscfg_touch = { .mosi_io_num = CONFIG_TOUCH_SPI_MOSI,
        .miso_io_num = CONFIG_TOUCH_SPI_MISO,
        .sclk_io_num = CONFIG_TOUCH_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
        .isr_cpu_id = INTR_CPU_ID_AUTO,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM };

    esp_lcd_touch_config_t tp_cfg = {.x_max = CONFIG_LCD_HRES,
                                   .y_max = CONFIG_LCD_VRES,
                                   .rst_gpio_num = CONFIG_TOUCH_RST,
                                   .int_gpio_num = CONFIG_TOUCH_IRQ,
                                   .levels = {.reset = 0, .interrupt = 0},
                                   .flags =
                                       {
                                           .swap_xy = 0,
                                           .mirror_x = 0,
                                           .mirror_y = 0,
                                       },
                                   .process_coordinates = process_coordinates,
                                   .interrupt_callback = nullptr};

    // Initialize spi bus for touch driver:
    ESP_ERROR_CHECK(spi_bus_initialize(CONFIG_TOUCH_SPI, &buscfg_touch, SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CONFIG_TOUCH_SPI, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp));

    return tp;
}

//
// callback for lvgl
//
static void lcd_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    auto panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    lvgl_acquire();
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
    lvgl_release();
}

//
// callback for touch
//
static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = { 0 };
    uint16_t touchpad_y[1] = { 0 };
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data((esp_lcd_touch_handle_t)drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates((esp_lcd_touch_handle_t)drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

//
// event handler for display touch
//
void ui_event_Screen(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    // GuiData_t *gd = (GuiData_t *)lv_event_get_user_data(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        ESP_LOGI(TAG, "clicked");
        // lv_obj_t *object = lv_event_get_target(e);
        // lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
    }
}
//
//
// Setup LVGL
//
void InitLVGL(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle, lv_disp_drv_t *disp_drv)
{
    static lv_disp_draw_buf_t disp_buf;

    lv_init();

    static lv_color_t buf1[CONFIG_LCD_BUF_SIZE];
    static lv_color_t buf2[CONFIG_LCD_BUF_SIZE];

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, CONFIG_LCD_BUF_SIZE);

    lv_disp_drv_init(disp_drv);
    disp_drv->hor_res = CONFIG_LCD_HRES;
    disp_drv->ver_res = CONFIG_LCD_VRES;
    disp_drv->flush_cb = lcd_lvgl_flush_cb;
    disp_drv->draw_buf = &disp_buf;
    disp_drv->user_data = panel_handle;

    lv_disp_t *disp = lv_disp_drv_register(disp_drv);

    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = touch_handle;

    lv_indev_drv_register(&indev_drv);
}

void lvgl_acquire(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    }
}

void lvgl_release(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreGive(xGuiSemaphore);
    }
}

void lvUpdateTask(void *ptr)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }
}

uint8_t lvSetupScreen(lv_obj_t **obj_array)
{
    uint8_t n = 0;

    lvgl_acquire();

    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen, lv_color_black(), LV_STATE_DEFAULT);

    /*
        lv_obj_t *label = lv_label_create(screen);
        lv_obj_set_style_text_color(label, LV_COLOR_MAKE16(0xd2, 0xe3, 0x36), LV_STATE_DEFAULT);
        lv_obj_set_align(label, LV_ALIGN_CENTER);
        lv_label_set_text(label, "Hello LVGL World!");
        lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(label, ui_event_Screen, LV_EVENT_ALL, nullptr);
    */

    lv_obj_t *lblText = lv_label_create(screen);
    lv_obj_set_style_text_color(lblText, LV_COLOR_MAKE16(20, 238, 231), LV_STATE_DEFAULT);
    lv_obj_set_align(lblText, LV_ALIGN_CENTER);
    lv_label_set_text(lblText, "");
    obj_array[n++] = lblText;

    lv_obj_t *lblRed = lv_label_create(screen);
    lv_obj_set_style_text_color(lblRed, LV_COLOR_MAKE16(0xff, 0x00, 0x00), LV_STATE_DEFAULT);
    lv_obj_set_align(lblRed, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(lblRed, "RED");
    obj_array[n++] = lblRed;

    lv_obj_t *lblGreen = lv_label_create(screen);
    lv_obj_set_style_text_color(lblGreen, LV_COLOR_MAKE16(0x00, 0xff, 0x00), LV_STATE_DEFAULT);
    lv_obj_set_align(lblGreen, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(lblGreen, "GREEN");
    obj_array[n++] = lblGreen;

    lv_obj_t *lblBlue = lv_label_create(screen);
    lv_obj_set_style_text_color(lblBlue, LV_COLOR_MAKE16(0x00, 0x00, 0xff), LV_STATE_DEFAULT);
    lv_obj_set_align(lblBlue, LV_ALIGN_BOTTOM_LEFT);
    lv_label_set_text(lblBlue, "BLUE");
    obj_array[n++] = lblBlue;

    lv_obj_t *lblWhite = lv_label_create(screen);
    lv_obj_set_style_text_color(lblWhite, LV_COLOR_MAKE16(0xff, 0xff, 0xff), LV_STATE_DEFAULT);
    lv_obj_set_align(lblWhite, LV_ALIGN_BOTTOM_RIGHT);
    lv_label_set_text(lblWhite, "WHITE");
    obj_array[n++] = lblWhite;

    lv_disp_load_scr(screen);

    lvgl_release();

    return n;
}

void TaskDisplay(void *)
{
    char buf[32];
    lv_obj_t *Labels[8];
    uint8_t NrLabels;

    static lv_disp_drv_t disp_drv;
    InitBacklight();

    esp_lcd_panel_handle_t display = InitDisplay(&disp_drv);
    esp_lcd_touch_handle_t touchpanel = InitTouchPanel();

    xGuiSemaphore = xSemaphoreCreateMutex();
    InitLVGL(display, touchpanel, &disp_drv);
    NrLabels = lvSetupScreen(Labels);

    xTaskCreatePinnedToCore(&lvUpdateTask, "lv_update", 8192, nullptr, 6, &g_lvgl_task_handle, 1);
    uint32_t n = 0;
    while (42)
    {
        n++;
        sprintf(buf, "%" PRIu32, n);

        lvgl_acquire();

        for (uint8_t i = 0; i < NrLabels; i++)
            lv_label_set_text(Labels[i], buf);

        lvgl_release();

        vTaskDelay(125 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(TaskDisplay, "Display", 6 * configMINIMAL_STACK_SIZE, NULL, 5, NULL, 1);

    vTaskDelay(portMAX_DELAY);
}
