#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

esp_lcd_panel_handle_t panel1 = nullptr;
esp_lcd_panel_handle_t panel2 = nullptr;

class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_left_;
    LcdDisplay* display_right_;
    esp_lcd_panel_handle_t panel_left;
    esp_lcd_panel_handle_t panel_right;

    void InitializeSpiLeft() {
        spi_bus_config_t buscfg_left = {};
        buscfg_left.mosi_io_num = DISPLAY_LEFT_MOSI_PIN;
        buscfg_left.miso_io_num = GPIO_NUM_NC;
        buscfg_left.sclk_io_num = DISPLAY_LEFT_CLK_PIN;
        buscfg_left.quadwp_io_num = GPIO_NUM_NC;
        buscfg_left.quadhd_io_num = GPIO_NUM_NC;
        buscfg_left.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg_left, SPI_DMA_CH_AUTO));
    }
    
    void InitializeSpiRight() {        
        spi_bus_config_t buscfg_right = {};
        buscfg_right.mosi_io_num = DISPLAY_RIGHT_MOSI_PIN;
        buscfg_right.miso_io_num = GPIO_NUM_NC;
        buscfg_right.sclk_io_num = DISPLAY_RIGHT_CLK_PIN;
        buscfg_right.quadwp_io_num = GPIO_NUM_NC;
        buscfg_right.quadhd_io_num = GPIO_NUM_NC;
        buscfg_right.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg_right, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplayLeft() {
        // // 硬件复位
        // gpio_set_level(DISPLAY_LEFT_RST_PIN, 0);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        // gpio_set_level(DISPLAY_LEFT_RST_PIN, 1);
        // vTaskDelay(120 / portTICK_PERIOD_MS);  // 必须等待120ms以上
        // 1. 确保右屏 CS 取消选中（避免干扰左屏）
        // gpio_set_level(DISPLAY_RIGHT_CS_PIN, 1);  // 右屏 CS = 高电平（取消选中）
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        // // 2. 选中左屏 CS
        // gpio_set_level(DISPLAY_LEFT_CS_PIN, 0);  // 左屏 CS = 低电平（选中）
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_LEFT_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_LEFT_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        // io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.pclk_hz = 10 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_LEFT_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel_left));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel_left));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel_left));
#endif
        
        esp_lcd_panel_reset(panel_left);
 

        esp_lcd_panel_init(panel_left);
        esp_lcd_panel_invert_color(panel_left, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel_left, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel_left, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_left_ = new SpiLcdDisplay(panel_io, panel_left,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_16_4,
                                        .icon_font = &font_awesome_16_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
#endif
                                    });
        panel1 = panel_left;
    }

    
    void InitializeLcdDisplayRight() {
        
        // // 硬件复位
        // gpio_set_level(DISPLAY_RIGHT_RST_PIN, 0);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
        // gpio_set_level(DISPLAY_RIGHT_RST_PIN, 1);
        // vTaskDelay(120 / portTICK_PERIOD_MS);  // 必须等待120ms以上
        
        // // 1. 确保右屏 CS 取消选中（避免干扰左屏）
        // gpio_set_level(DISPLAY_LEFT_CS_PIN, 1);  // 右屏 CS = 高电平（取消选中）
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        // // 2. 选中左屏 CS
        // gpio_set_level(DISPLAY_RIGHT_CS_PIN, 0);  // 左屏 CS = 低电平（选中）
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel 2 IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_RIGHT_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_RIGHT_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 10 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD 2 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RIGHT_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel_right));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel_right));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel_right));
#endif
        
        esp_lcd_panel_reset(panel_right);
 
        // return;

        esp_lcd_panel_init(panel_right);
        esp_lcd_panel_invert_color(panel_right, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel_right, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel_right, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif

        display_right_ = new SpiLcdDisplay(panel_io, panel_right,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_16_4,
                                        .icon_font = &font_awesome_16_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
#endif
                                    });
        panel2 = panel_right;
    }


 
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
            InitializeSpiLeft();
            InitializeSpiRight();
            InitializeLcdDisplayLeft();
            // vTaskDelay(500 / portTICK_PERIOD_MS);
            InitializeLcdDisplayRight();
            gpio_set_level(DISPLAY_LEFT_CS_PIN, 0);  // 选中左屏
            gpio_set_level(DISPLAY_RIGHT_CS_PIN, 1);  // 取消选中右屏
            display_left_->SetChatMessage("system", "0001左屏幕内容");
            // 发送数据到右屏前
            gpio_set_level(DISPLAY_LEFT_CS_PIN, 1);  // 取消选中左屏
            gpio_set_level(DISPLAY_RIGHT_CS_PIN, 0);  // 选中右屏
            display_right_->SetChatMessage("system", "0000右屏幕内容");
            InitializeButtons();
            InitializeIot();
            if (DISPLAY_LEFT_BACKLIGHT_PIN != GPIO_NUM_NC) {
                GetBacklight()->RestoreBrightness();
            }
            // 1. 单独测试左屏（蓝色）
            
            gpio_set_level(DISPLAY_LEFT_CS_PIN, 0);  // 选中左屏
            gpio_set_level(DISPLAY_RIGHT_CS_PIN, 1);  // 取消选中右屏
            // esp_lcd_panel_draw_bitmap(panel1, 0, 0, 240, 240, (uint16_t[]) {0xF800});
            // std::vector<uint16_t> buffer1(DISPLAY_WIDTH, 0xF080);
            std::vector<uint16_t> buffer1(DISPLAY_WIDTH, 0x0000);
            for (int y = 0; y < DISPLAY_HEIGHT; y++) {
                esp_lcd_panel_draw_bitmap(panel1, 0, y, DISPLAY_WIDTH, y + 1, buffer1.data());
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        
            // // 2. 单独测试右屏（红色）
            // 发送数据到右屏前
            gpio_set_level(DISPLAY_LEFT_CS_PIN, 1);  // 取消选中左屏
            gpio_set_level(DISPLAY_RIGHT_CS_PIN, 0);  // 选中右屏
            // std::vector<uint16_t> buffer(DISPLAY_WIDTH, 0x07E0);
            std::vector<uint16_t> buffer(DISPLAY_WIDTH, 0x0000);
            for (int y = 0; y < DISPLAY_HEIGHT; y++) {
                esp_lcd_panel_draw_bitmap(panel2, 0, y, DISPLAY_WIDTH, y + 1, buffer.data());
            }
            // esp_lcd_panel_draw_bitmap(panel2, 0, 0, 240, 240, (uint16_t[]) {0x07E0});
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        
            // // 3. 同时测试双屏
            // esp_lcd_panel_draw_bitmap(panel1, 0, 0, 240, 240, (uint16_t[]) {0xF800});
            // esp_lcd_panel_draw_bitmap(panel2, 0, 0, 240, 240, (uint16_t[]) {0x07E0});
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_left_;
    }
    
    virtual Display* GetDisplayLeft() override {
        return display_left_;
    }
    
    virtual Display* GetDisplayRight() override {
        return display_left_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_LEFT_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_LEFT_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
