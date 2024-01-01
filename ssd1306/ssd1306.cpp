#include "ssd1306.h"

#include <string.h>  // memset

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HOST_ID SPI2_HOST

namespace SSD1306
{

void SSD1306SPI::Init()
{
    esp_err_t ret;

    gpio_reset_pin(m_GPIO.CS);
    gpio_set_direction(m_GPIO.CS, GPIO_MODE_OUTPUT);
    gpio_set_level(m_GPIO.CS, 0);

    // gpio_pad_select_gpio( m_GPIO.DC );
    gpio_reset_pin(m_GPIO.DC);
    gpio_set_direction(m_GPIO.DC, GPIO_MODE_OUTPUT);
    gpio_set_level(m_GPIO.DC, 0);

    if (m_GPIO.RES >= 0)
    {
        // gpio_pad_select_gpio( m_GPIO.RES );
        gpio_reset_pin(m_GPIO.RES);
        gpio_set_direction(m_GPIO.RES, GPIO_MODE_OUTPUT);
        gpio_set_level(m_GPIO.RES, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(m_GPIO.RES, 1);
    }

    spi_bus_config_t spi_bus_config = {};
    spi_bus_config.mosi_io_num = m_GPIO.MOSI,
    spi_bus_config.miso_io_num = -1,
    spi_bus_config.sclk_io_num = m_GPIO.SCLK,
    spi_bus_config.quadwp_io_num = -1,
    spi_bus_config.quadhd_io_num = -1,
    spi_bus_config.max_transfer_sz = 0,
    spi_bus_config.flags = 0;

    ESP_LOGI(TAG, "SPI HOST_ID=%d", HOST_ID);
    ret = spi_bus_initialize(HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_bus_initialize=%d", ret);
    assert(ret == ESP_OK);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.clock_speed_hz = SPI_Frequency;
    devcfg.spics_io_num = m_GPIO.CS;
    devcfg.queue_size = 1;

    spi_device_handle_t handle;
    ret = spi_bus_add_device(HOST_ID, &devcfg, &handle);
    ESP_LOGI(TAG, "spi_bus_add_device=%d", ret);
    assert(ret == ESP_OK);

    m_deviceHandle = handle;


	MasterWriteCommand(OLED_CMD_DISPLAY_OFF);			// AE
	MasterWriteCommand(OLED_CMD_SET_MUX_RATIO);			// A8
	MasterWriteCommand(0x3F);
	MasterWriteCommand(OLED_CMD_SET_DISPLAY_OFFSET);		// D3
	MasterWriteCommand(0x00);
	MasterWriteCommand(OLED_CONTROL_BYTE_DATA_STREAM);	// 40
    MasterWriteCommand(OLED_CMD_SET_SEGMENT_REMAP_1);	// A1
	MasterWriteCommand(OLED_CMD_SET_COM_SCAN_MODE);		// C8
	MasterWriteCommand(OLED_CMD_SET_DISPLAY_CLK_DIV);	// D5
	MasterWriteCommand(0x80);
	MasterWriteCommand(OLED_CMD_SET_COM_PIN_MAP);		// DA
	MasterWriteCommand(0x12);
	MasterWriteCommand(OLED_CMD_SET_CONTRAST);			// 81
	MasterWriteCommand(0xFF);
	MasterWriteCommand(OLED_CMD_DISPLAY_RAM);			// A4
	MasterWriteCommand(OLED_CMD_SET_VCOMH_DESELCT);		// DB
	MasterWriteCommand(0x40);
	MasterWriteCommand(OLED_CMD_SET_MEMORY_ADDR_MODE);	// 20
	MasterWriteCommand(OLED_CMD_SET_PAGE_ADDR_MODE);		// 02
	MasterWriteCommand(0x00);
	MasterWriteCommand(0x10);
	MasterWriteCommand(OLED_CMD_SET_CHARGE_PUMP);		// 8D
	MasterWriteCommand(0x14);
	MasterWriteCommand(OLED_CMD_DEACTIVE_SCROLL);		// 2E
	MasterWriteCommand(OLED_CMD_DISPLAY_NORMAL);			// A6
	MasterWriteCommand(OLED_CMD_DISPLAY_ON);				// AF
}

void SSD1306SPI::MasterWriteByte(const uint8_t* pData, size_t nSize)
{

	if ( nSize > 0 ) {
        spi_transaction_t SPITransaction = {};
        SPITransaction.length = nSize * 8;
        SPITransaction.tx_buffer = pData;
        spi_device_transmit(m_deviceHandle, &SPITransaction);
	}
}

void SSD1306SPI::MasterWriteCommand(uint8_t command)
{
    gpio_set_level(m_GPIO.DC, SPI_Command_Mode);
    MasterWriteByte(&command, 1);
}

void SSD1306SPI::MasterWriteData(const uint8_t* pData, size_t nSize)
{
    if (nSize > 0)
    {
        gpio_set_level(m_GPIO.DC, SPI_Data_Mode);
        MasterWriteByte(pData, nSize);
    }
}

void SSD1306SPI::Flip()
{

    // Display backbuffer
    for (uint8_t page = 0; page < BackBuffer::PAGE_COUNT; page++)
    {
        uint8_t firstElem = m_backBuffer.PageBegin(page)[0];
        ESP_LOGI(TAG, "First Elem %d", firstElem);
        MasterWriteCommand(0x00);
        MasterWriteCommand(0x10);
        uint8_t command = 0xB0 | page;
        MasterWriteCommand(command);
        MasterWriteData(m_backBuffer.PageBegin(page), m_backBuffer.BYTES_PER_PAGE);
    }
}
}  // namespace SSD1306
