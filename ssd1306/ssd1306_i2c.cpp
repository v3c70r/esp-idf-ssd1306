#include "ssd1306_i2c.h"

#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_NUM I2C_NUM_0
namespace SSD1306
{
void SSD1306I2C::Init()
{
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = m_GPIO.SDA;
    i2c_config.scl_io_num = m_GPIO.SCL;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    if (m_GPIO.RESET >= 0)
    {
        // gpio_pad_select_gpio(m_GPIO.RESET);
        gpio_reset_pin(m_GPIO.RESET);
        gpio_set_direction(m_GPIO.RESET, GPIO_MODE_OUTPUT);
        gpio_set_level(m_GPIO.RESET, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(m_GPIO.RESET, 1);
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2CAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);    // AE
    i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);  // A8
    i2c_master_write_byte(cmd, 0x3F, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);  // D3
    i2c_master_write_byte(cmd, 0x00, true);
    // i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);	// 40
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE, true);  // 40
                                                                        // i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true);		// A1
    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true);     // A1
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);       // C8
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);     // D5
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);  // DA
    i2c_master_write_byte(cmd, 0x12, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);  // 81
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);        // A4
    i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true);  // DB
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);  // 20
    // i2c_master_write_byte(cmd, OLED_CMD_SET_HORI_ADDR_MODE, true);	// 00
    i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true);  // 02
    // Set Lower Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x00, true);
    // Set Higher Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);  // 8D
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true);  // 2E
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);   // A6
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);       // AF

    i2c_master_stop(cmd);

    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        ESP_LOGI(TAG, "OLED configured successfully");
    }
    else
    {
        ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
}

void SSD1306I2C::DisplayPage(uint8_t page, const uint8_t* pPageData)
{
    if (page >= BackBuffer::PAGE_COUNT) return;

    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2CAddress << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    // Set Lower Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x00, true);
    // Set Higher Column Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0x10, true);
    // Set Page Start Address for Page Addressing Mode
    i2c_master_write_byte(cmd, 0xB0 | page, true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2CAddress << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, pPageData, 128, true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
void SSD1306I2C::Flip()
{
    for (uint8_t page = 0; page < BackBuffer::PAGE_COUNT; page++)
    {
        DisplayPage(page, m_backBuffer.PageBegin(page));
    }
}

}  // namespace SSD1306
