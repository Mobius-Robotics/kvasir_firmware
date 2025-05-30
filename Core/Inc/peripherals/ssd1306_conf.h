/**
 * Private configuration file for the SSD1306 library.
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Choose a microcontroller family
#define STM32H5

// Choose a bus
#define SSD1306_USE_I2C

// I2C Configuration
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Mirror the screen if needed
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ

// Set inverse color if needed
// # define SSD1306_INVERSE_COLOR

// Include only needed fonts
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#define SSD1306_INCLUDE_FONT_16x24

#define SSD1306_INCLUDE_FONT_16x15

// If your screen horizontal axis does not start
// in column 0 you can use this define to
// adjust the horizontal offset
// #define SSD1306_X_OFFSET
#endif /* __SSD1306_CONF_H__ */
