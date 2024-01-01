#pragma once
#include <driver/gpio.h>
#include <driver/spi_master.h>

#include <cstdint>

#define TAG "SSD1306"
namespace SSD1306
{

// Following definitions are bollowed from
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

/* Control byte for i2c
Co : bit 8 : Continuation Bit
 * 1 = no-continuation (only one byte to follow)
 * 0 = the controller should expect a stream of bytes.
D/C# : bit 7 : Data/Command Select bit
 * 1 = the next byte or byte stream will be Data.
 * 0 = a Command byte or byte stream will be coming up next.
 Bits 6-0 will be all zeros.
Usage:
0x80 : Single Command byte
0x00 : Command Stream
0xC0 : Single Data byte
0x40 : Data Stream
*/
static const uint8_t OLED_CONTROL_BYTE_CMD_SINGLE = 0x80;
static const uint8_t OLED_CONTROL_BYTE_CMD_STREAM = 0x00;
static const uint8_t OLED_CONTROL_BYTE_DATA_SINGLE = 0xC0;
static const uint8_t OLED_CONTROL_BYTE_DATA_STREAM = 0x40;

// Fundamental commands (pg.28)
static const uint8_t OLED_CMD_SET_CONTRAST = 0x81;  // follow with 0x7F
static const uint8_t OLED_CMD_DISPLAY_RAM = 0xA4;
static const uint8_t OLED_CMD_DISPLAY_ALLON = 0xA5;
static const uint8_t OLED_CMD_DISPLAY_NORMAL = 0xA6;
static const uint8_t OLED_CMD_DISPLAY_INVERTED = 0xA7;
static const uint8_t OLED_CMD_DISPLAY_OFF = 0xAE;
static const uint8_t OLED_CMD_DISPLAY_ON = 0xAF;

// Addressing Command Table (pg.30)
static const uint8_t OLED_CMD_SET_MEMORY_ADDR_MODE = 0x20;
static const uint8_t OLED_CMD_SET_HORI_ADDR_MODE = 0x00;  // Horizontal Addressing Mode
static const uint8_t OLED_CMD_SET_VERT_ADDR_MODE = 0x01;  // Vertical Addressing Mode
static const uint8_t OLED_CMD_SET_PAGE_ADDR_MODE = 0x02;  // Page Addressing Mode
static const uint8_t OLED_CMD_SET_COLUMN_RANGE = 0x21;    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
static const uint8_t OLED_CMD_SET_PAGE_RANGE = 0x22;      // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP_0 0xA0
#define OLED_CMD_SET_SEGMENT_REMAP_1 0xA1
#define OLED_CMD_SET_MUX_RATIO 0xA8  // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE 0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3  // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP 0xDA     // follow with 0x12
#define OLED_CMD_NOP 0xE3                 // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV 0xD5  // follow with 0x80
#define OLED_CMD_SET_PRECHARGE 0xD9        // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT 0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP 0x8D  // follow with 0x14

// Scrolling Command
#define OLED_CMD_HORIZONTAL_RIGHT 0x26
#define OLED_CMD_HORIZONTAL_LEFT 0x27
#define OLED_CMD_CONTINUOUS_SCROLL 0x29
#define OLED_CMD_DEACTIVE_SCROLL 0x2E
#define OLED_CMD_ACTIVE_SCROLL 0x2F
#define OLED_CMD_VERTICAL 0xA3

#define I2CAddress 0x3C
#define SPIAddress 0xFF

typedef enum
{
    SCROLL_RIGHT = 1,
    SCROLL_LEFT = 2,
    SCROLL_DOWN = 3,
    SCROLL_UP = 4,
    SCROLL_STOP = 5
} ssd1306_scroll_type_t;

struct BackBuffer
{
    static const size_t BYTES_PER_ROW = 16;  // 128 bits per row
    static const size_t ROW_COUNT = 64;
    static const size_t PAGE_COUNT = 8;
    static const size_t BYTES_PER_PAGE = 128;
    uint8_t buffer[BYTES_PER_ROW * ROW_COUNT];
    const uint8_t* PageBegin(uint8_t nPage) const { return buffer + nPage * BYTES_PER_PAGE; }
};

class SSD1306
{
public:
    virtual void Flip() = 0;  // present
    BackBuffer& GetBackBuffer() {return m_backBuffer;}

protected:
    BackBuffer m_backBuffer;

};

class SSD1306SPI : public SSD1306
{
    struct SPIPins
    {
        gpio_num_t MOSI;
        gpio_num_t SCLK;
        gpio_num_t CS;
        gpio_num_t DC;
        gpio_num_t RES;
    };

public:
    void Flip() override;

    SSD1306SPI(gpio_num_t MOSI, gpio_num_t SCLK, gpio_num_t CS, gpio_num_t DC, gpio_num_t RES)
    {
        m_GPIO.MOSI = MOSI;
        m_GPIO.SCLK = SCLK;
        m_GPIO.CS = CS;
        m_GPIO.DC = DC;
        m_GPIO.RES = RES;
        Init();
    }

private:
    void Init();
    void MasterWriteByte(const uint8_t* pData, size_t nSize);
    void MasterWriteCommand(uint8_t command);
    void MasterWriteData(const uint8_t* pData, size_t nSize);

    SPIPins m_GPIO;
    spi_device_handle_t m_deviceHandle = {};

    static const int SPI_Command_Mode = 0;
    static const int SPI_Data_Mode = 1;
    static const int SPI_Frequency = 1000000;  // 1MHz
};
}  // namespace SSD1306
