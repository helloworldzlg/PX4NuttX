#ifndef OV7740_H_
#define OV7740_H_

#include <stdint.h>
#include "settings.h"

/* Constants */
#define TIMEOUT_MAX      				  (20000)

/* Camera I2C registers */
#define OV7740_DEVICE_WRITE_ADDRESS       (0x42)
#define OV7740_DEVICE_READ_ADDRESS        (0x43)

/*chip id*/
#define OV7740_CHIPIDH_R                  (0x0a)
#define OV7740_CHIPIDL_R                  (0x0b)
#define OV7740_CHIPIDH                    (0x7742)


#define RESET_PIN			              GPIO_Pin_0
#define RESET_GPIO_PORT		              GPIOD
#define RESET_GPIO_CLK		              RCC_AHB1Periph_GPIOD

#define PWD_PIN			                  GPIO_Pin_7
#define PWD_GPIO_PORT	                  GPIOD
#define PWD_GPIO_CLK		              RCC_AHB1Periph_GPIOD


uint8_t Ov7740_WriteReg(uint16_t Addr, uint8_t Data);
uint8_t Ov7740_ReadReg(uint16_t Addr);
uint8_t ov7740_init(void);
void set_ov7740_brightness(int8_t brightness_level);
void set_ov7740_contrast(int8_t contrast_level);

#endif
