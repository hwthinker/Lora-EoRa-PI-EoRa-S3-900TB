#pragma once

#define UNUSE_PIN                   (0)

#if defined(EoRa_PI_V1)

#define I2C_SDA                     18
#define I2C_SCL                     17
#define OLED_RST                    UNUSE_PIN

#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              3
#define RADIO_MOSI_PIN              6
#define RADIO_CS_PIN                7
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              34
#define RADIO_RST_PIN               8

#define SDCARD_MOSI                 11
#define SDCARD_MISO                 2
#define SDCARD_SCLK                 14
#define SDCARD_CS                   13

#define BOARD_LED                   37
#define LED_ON                      HIGH

#define BAT_ADC_PIN                 1
#define BUTTON_PIN                  0

#define HAS_SDCARD
#define HAS_DISPLAY

#else
#error "For the first use, please define the board version and model in <utilities. h>"
#endif






