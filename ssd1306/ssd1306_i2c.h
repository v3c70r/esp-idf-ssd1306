#pragma once

#include "ssd1306.h"

namespace SSD1306
{
class SSD1306I2C : public SSD1306
{
    struct I2CPins
    {
        gpio_num_t SCL;
        gpio_num_t SDA;
        gpio_num_t RESET;
    };
public:
    SSD1306I2C(gpio_num_t SCL, gpio_num_t SDA, gpio_num_t RESET)
    {
        m_GPIO.SCL = SCL;
        m_GPIO.SDA = SDA;
        m_GPIO.RESET = RESET;
        Init();
    }

    void Flip() override;

private:
    void Init();
    void DisplayPage(uint8_t page, const uint8_t* pPageData);
    I2CPins m_GPIO;
    const uint32_t I2C_MASTER_FREQ_HZ = 400000;
};
}  // namespace SSD1306
