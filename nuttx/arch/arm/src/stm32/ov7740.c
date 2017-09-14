/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
//#include "main.h"
#include "ov7740.h"


/*
reset and pwd operation
*/
#define RESET_ON()  GPIO_SetBits(RESET_GPIO_PORT,RESET_PIN)
#define RESET_OFF() GPIO_ResetBits(RESET_GPIO_PORT,RESET_PIN)
#define PWD_ON()    GPIO_SetBits(PWD_GPIO_PORT,PWD_PIN)
#define PWD_OFF()   GPIO_ResetBits(PWD_GPIO_PORT,PWD_PIN)


/* -------- OV7740_REG0C : (Address: 0x0b) -------- */
#define OV7740_REG0C                   (0x0c)
#define OV7740_REG0C_MAX_EXPOSURE_Pos    (1)
#define OV7740_REG0C_MAX_EXPOSURE_Msk    (0x3u << OV7740_REG0C_MAX_EXPOSURE_Pos) /**< \brief (OV7740_REG0C) Max exposure = frame length - limit x 2 */
#define OV7740_REG0C_MAX_EXPOSURE(value) ((OV7740_REG0C_MAX_EXPOSURE_Msk & ((value) << OV7740_REG0C_MAX_EXPOSURE_Pos)))
#define OV7740_REG0C_BYTE_SWAP_Msk       (0x1u << 3) /**< \brief (OV7740_REG0C) High 8-bit MSB and LSB swap */
#define OV7740_REG0C_BYTE_SWAP_DISABLE   (0x0u << 3) /**< \brief (OV7740_REG0C) output Y9,Y8...Y3,Y2,Y1,Y0 */
#define OV7740_REG0C_BYTE_SWAP_ENABLE    (0x1u << 3) /**< \brief (OV7740_REG0C) output Y3,Y2...Y8,Y9,Y1,Y0 */
#define OV7740_REG0C_YUV_SWAP_Msk        (0x1u << 4) /**< \brief (OV7740_REG0C) YUV output, Y <-> UV swap */
#define OV7740_REG0C_YUV_SWAP_DISABLE    (0x0u << 4) /**< \brief (OV7740_REG0C) output YUYVYUYV */
#define OV7740_REG0C_YUV_SWAP_ENABLE     (0x1u << 4) /**< \brief (OV7740_REG0C) output UYVYUYVY */
#define OV7740_REG0C_MIRROR_ENABLE       (0x1u << 6) /**< \brief (OV7740_REG0C) Mirror enable */
#define OV7740_REG0C_FLIP_ENABLE         (0x1u << 7) /**< \brief (OV7740_REG0C) Flip enable */
/**
 * \brief Addresses and values of the OV7740 registers for the
 * OV7740_VGA_YUV422_20FPS configuration:
 *  - 640*480 pixel by picture (VGA)
 *  - pixel data in YUV422 format (Y1, U, Y2, V)
 *  - 20 frames per second
 */
uint8_t OV7740_VGA_YUV422_20FPS[][2] = {
    {0x0e, 0x00},

    {0x12, 0x80},
    /* flag for soft reset delay */
    {0xFE, 0x05},
    {0x13, 0x00},

    /**************************************************************/
    /*  30fps  11 01 ;clock_divider ;sysclk=24MHz at XCLK=24MHz   */
    /*  20fps  11 02 ;clock_divider ;sysclk=16MHz at XCLK=24MHz   */
    /*  15fps  11 03 ;clock_divider ;sysclk=12MHz at XCLK=24MHz   */
    /*  10fps  11 05 ;sysclk=8MHz at XCLK=24MHz                   */
    /*  7p5fps 11 07 ;sysclk=6MHz at XCLK=24MHz                   */
    /**************************************************************/
    {0x11, 0x00},
    /**************************************************************/

    {0x12, 0x00},
    {0xd5, 0x10},
    {0x0c, (OV7740_REG0C_MIRROR_ENABLE | OV7740_REG0C_FLIP_ENABLE | OV7740_REG0C_MAX_EXPOSURE(2) | OV7740_REG0C_YUV_SWAP_ENABLE)},
    {0x0d, 0x34},
    {0x16, 0x01},
    {0x17, 0x25},
    {0x18, 0xa0},
    {0x19, 0x03},
    {0x1a, 0xf0},
    {0x1b, 0x89},
    {0x22, 0x03},
    {0x29, 0x18},
    {0x2b, 0xf8},
    {0x2c, 0x01},
    {0x31, 0xa0},
    {0x32, 0xf0},
    {0x33, 0xc4},
    {0x3a, 0xb4},
    {0x36, 0x3f},

    {0x04, 0x60},
    {0x27, 0x80},
    {0x3d, 0x0f},
    {0x3e, 0x80},
    {0x3f, 0x40},
    {0x40, 0x7f},
    {0x41, 0x6a},
    {0x42, 0x29},
    {0x44, 0xe5},
    {0x45, 0x41},
    {0x47, 0x02},
    {0x49, 0x64},
    {0x4a, 0xa1},
    {0x4b, 0x70},
    {0x4c, 0x1a},
    {0x4d, 0x50},
    {0x4e, 0x13},
    {0x64, 0x00},
    {0x67, 0x88},
    {0x68, 0x1a},

    {0x14, 0x38},
    {0x24, 0x3c},
    {0x25, 0x30},
    {0x26, 0x72},
    {0x50, 0x97},
    {0x51, 0x7e},
    {0x52, 0x00},
    {0x53, 0x00},
    {0x20, 0x00},
    {0x21, 0x23},
    /*********************************/
    /* To enable Static Test Pattern */
    /*********************************/
    /*	{0x38, 0x07}, */
    /*	{0x84, 0x02}, */

    /*********************************/
    /* Normal Mode / No test pattern */
    {0x38, 0x14},
    /*********************************/
    {0xe9, 0x00},
    {0x56, 0x55},
    {0x57, 0xff},
    {0x58, 0xff},
    {0x59, 0xff},
    {0x5f, 0x04},
    {0xec, 0x00},
    {0x13, 0xff},

    {0x80, 0x7f},
    {0x81, 0x3f},
    {0x82, 0x32},
    {0x83, 0x01},
    {0x38, 0x11},
    {0x85, 0x00},
    {0x86, 0x03},
    {0x87, 0x01},
    {0x88, 0x05},
    {0x89, 0x30},
    {0x8d, 0x30},
    {0x8f, 0x85},
    {0x93, 0x30},
    {0x95, 0x85},
    {0x99, 0x30},
    {0x9b, 0x85},

    {0x9c, 0x08},
    {0x9d, 0x12},
    {0x9e, 0x23},
    {0x9f, 0x45},
    {0xa0, 0x55},
    {0xa1, 0x64},
    {0xa2, 0x72},
    {0xa3, 0x7f},
    {0xa4, 0x8b},
    {0xa5, 0x95},
    {0xa6, 0xa7},
    {0xa7, 0xb5},
    {0xa8, 0xcb},
    {0xa9, 0xdd},
    {0xaa, 0xec},
    {0xab, 0x1a},

    {0xce, 0x78},
    {0xcf, 0x6e},
    {0xd0, 0x0a},
    {0xd1, 0x0c},
    {0xd2, 0x84},
    {0xd3, 0x90},
    {0xd4, 0x1e},

    {0x5a, 0x24},
    {0x5b, 0x1f},
    {0x5c, 0x88},
    {0x5d, 0x60},

    {0xac, 0x6e},
    {0xbe, 0xff},
    {0xbf, 0x00},

    /* 64*64 */
//    {0x31, 0x10},
//    {0x32, 0x20},
//    {0x82, 0x3f},

    /* 80*80 */
//    {0x31, 0x14},
//    {0x32, 0x28},
//    {0x82, 0x3f},

    /* 120*120 */
    {0x31, 0x1e},  //width
    {0x32, 0x3c},  //hight
    {0x82, 0x3f},    

    /* 160*120 */
//    {0x31, 0x28},  //width
//    {0x32, 0x3c},  //hight
//    {0x82, 0x3f},

    /* 640x480 */
//	{0x31, 0xA0},
//	{0x32, 0xF0},
//	{0x82, 0x3f},

    /* YUV */
    {0x12, 0x00},
    {0x36, 0x3f},
    {0x53, 0x00},

    {0x33, 0xc4},
    {0x1b, 0x89},
    {0x22, 0x03},

    /* VSYNC, inverse */
    {0x28, 0x2},

//	{0xFF, 0xFF}
};

void delay(uint32_t msec)
{
    int32_t delay_time = msec*30000;
    while (delay_time-- > 0){};
}

/*
  * @brief  initialize the ov7740 camera module
  * @param  none
  * @retval 0x00 init is OK.
  *         0x01 init failed
*/

uint8_t ov7740_init(void)
{

    /*test reset and pwd*/
#if 0
//	RESET_OFF();
//	PWD_OFF();
    RESET_ON();
    PWD_ON();
    while(1);
    while(1)
    {
        RESET_ON();
        PWD_ON();
        delay(5);
        RESET_OFF();
        PWD_OFF();
        delay(5);
    }
#endif

    RESET_OFF();
    PWD_OFF();
    delay(20);
    RESET_ON();
    PWD_OFF();
    delay(20);
    RESET_OFF();
    PWD_OFF();
    delay(20);
    RESET_OFF();
    PWD_ON();
    delay(4);

    IIC_Init();
    delay(50);

    uint16_t reg;
    reg = Ov7740_ReadReg(OV7740_CHIPIDH_R);
    reg = Ov7740_ReadReg(OV7740_CHIPIDH_R);   //读取ID 高八位
    reg <<= 8;
    reg |= Ov7740_ReadReg(OV7740_CHIPIDL_R);  //读取ID 低八位
    if (reg != OV7740_CHIPIDH)
    {
        printf("chip id error!!!\n\n");
        return 1;
    }

    for (uint16_t i = 0; i < sizeof(OV7740_VGA_YUV422_20FPS) / 2; i++)
    {
        if (OV7740_VGA_YUV422_20FPS[i][0] == 0xfe)
        {
            usleep(5000);
        }
        else 
        {
            Ov7740_WriteReg(OV7740_VGA_YUV422_20FPS[i][0], OV7740_VGA_YUV422_20FPS[i][1]);
        }
    }

    Ov7740_ReadReg(0X11);
    Ov7740_ReadReg(0X31);
    Ov7740_ReadReg(0X32);

    delay(3000);
    printf("%s\n", __FUNCTION__);
    return 0;
}

#if 0

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: ov7740 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t Ov7740_WriteReg(uint16_t Addr, uint8_t Data)
{
    uint32_t timeout = TIMEOUT_MAX;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on I2C2 EV5 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C1, OV7740_DEVICE_WRITE_ADDRESS, I2C_Direction_Transmitter);

    /* Test on I2C2 EV6 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send I2C2 location address LSB */
    I2C_SendData(I2C1, (uint8_t)(Addr));

    /* Test on I2C2 EV8 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send Data */
    I2C_SendData(I2C1, Data);

    /* Test on I2C2 EV8 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }

    /* Send I2C2 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* If operation is OK, return 0 */
    return 0;
}


/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: ov7740 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t Ov7740_ReadReg(uint16_t Addr)
{
    uint32_t timeout = TIMEOUT_MAX;
    uint8_t Data = 0;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on I2C2 EV5 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }
    printf("1\n");
    usleep(100);

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C1, OV7740_DEVICE_READ_ADDRESS, I2C_Direction_Transmitter);

    /* Test on I2C2 EV6 and clear it */
//    timeout = TIMEOUT_MAX; /* Initialize timeout value */
//    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//    {
//        /* If the timeout delay is exeeded, exit with error code */
//        if ((timeout--) == 0) return 0xFF;
//    }
    printf("2\n");
    usleep(100);

    /* Send I2C2 location address LSB */
    I2C_SendData(I2C1, (uint8_t)(Addr));

    /* Test on I2C2 EV8 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }
    printf("3\n");

    /* Clear AF flag if arised */
    I2C1->SR1 |= (uint16_t)0x0400;

    /* Generate the Start Condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on I2C2 EV6 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }
    printf("4\n");

    /* Send DCMI selcted device slave Address for write */
    I2C_Send7bitAddress(I2C1, OV7740_DEVICE_READ_ADDRESS, I2C_Direction_Receiver);

    /* Test on I2C2 EV6 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }
    printf("5\n");

    /* Prepare an NACK for the next data received */
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    /* Test on I2C2 EV7 and clear it */
    timeout = TIMEOUT_MAX; /* Initialize timeout value */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
        /* If the timeout delay is exeeded, exit with error code */
        if ((timeout--) == 0) return 0xFF;
    }
    printf("6\n");

    /* Prepare Stop after receiving data */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* Receive the Data */
    Data = I2C_ReceiveData(I2C1);

    /* return the read data */
    return Data;
}
#endif

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Addr: ov7740 register address.
  * @param  Data: Data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *       0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t Ov7740_WriteReg(uint16_t Addr, uint8_t Data)
{
    IICwriteBytes(OV7740_DEVICE_WRITE_ADDRESS, Addr, 1, &Data);

    return 0;
}


/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Addr: ov7740 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t Ov7740_ReadReg(uint16_t Addr)
{
    uint8_t Data = 0;
    
    Data = I2C_ReadOneByte(OV7740_DEVICE_WRITE_ADDRESS, Addr);

    /* return the read data */
    return Data;
}

#define write_SCCB Ov7740_WriteReg
#define read_SCCB  Ov7740_ReadReg
/**
  * @brief  set the brightness of ov7740
  * @param  brightness_level  the level of brightness
  * @retval none
  */
void set_ov7740_brightness(int8_t brightness_level)
{
    uint8_t temp;
    uint8_t SCCB_salve_Address;
    switch(brightness_level)
    {
    case +4:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x0E);
        write_SCCB(0xE3, 0x40);
        break;


    case +3:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x0E);
        write_SCCB(0xE3, 0x30);
        break;

    case +2:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x0E);
        write_SCCB(0xE3, 0x20);
        break;

    case +1:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x0E);
        write_SCCB(0xE3, 0x10);
        break;

    case 0:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x0E);
        write_SCCB(0xE3, 0x00);
        break;

    case -1:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x06);
        write_SCCB(0xE3, 0x10);
        break;

    case -2:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x06);
        write_SCCB(0xE3, 0x20);
        break;

    case -3:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x06);
        write_SCCB(0xE3, 0x30);
        break;

    case -4:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE4, 0x06);
        write_SCCB(0xE3, 0x40);
        break;

    }

    SCCB_salve_Address = SCCB_salve_Address;
}


/**
  * @brief  set the contrast of ov7740
  * @param  brightness_level  the level of brightness
  * @retval none
  */
void set_ov7740_contrast(int8_t contrast_level)
{
    uint8_t temp;
    uint8_t SCCB_salve_Address;
    switch(contrast_level)
    {
    case +4:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x30);
        write_SCCB(0xE3, 0x00);
        temp = read_SCCB(0xE4);
        temp &= 0xfb;
        write_SCCB(0xE4, temp);
        break;

    case +3:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x2c);
        write_SCCB(0xE3, 0x00);
        temp = read_SCCB(0xE4);
        temp &= 0xfb;
        write_SCCB(0xE4, temp);
        break;

    case +2:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x28);
        write_SCCB(0xE3, 0x00);
        temp = read_SCCB(0xE4);
        temp &= 0xfb;
        write_SCCB(0xE4, temp);
        break;

    case +1:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x24);
        write_SCCB(0xE3, 0x00);
        temp = read_SCCB(0xE4);
        temp &= 0xfb;
        write_SCCB(0xE4, temp);
        break;

    case 0:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x20);
        write_SCCB(0xE3, 0x00);
        temp = read_SCCB(0xE4);
        temp &= 0xfb;
        write_SCCB(0xE4, temp);
        break;

    case -1:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x1c);
        write_SCCB(0xE3, 0x20);
        temp = read_SCCB(0xE4);
        temp |= 0x04;
        write_SCCB(0xE4, temp);
        break;

    case -2:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x18);
        write_SCCB(0xE3, 0x48);
        temp = read_SCCB(0xE4);
        temp |= 0x04;
        write_SCCB(0xE4, temp);
        break;

    case -3:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x14);
        write_SCCB(0xE3, 0x80);
        temp = read_SCCB(0xE4);
        temp |= 0x04;
        write_SCCB(0xE4, temp);
        break;

    case -4:
        SCCB_salve_Address = 0x42;
        temp = read_SCCB(0x81);
        temp |= 0x20;
        write_SCCB(0x81, temp);
        temp = read_SCCB(0xDA);
        temp |= 0x04;
        write_SCCB(0xDA, temp);
        write_SCCB(0xE1, 0x20);
        write_SCCB(0xE2, 0x10);
        write_SCCB(0xE3, 0xD0);
        temp = read_SCCB(0xE4);
        temp |= 0x04;
        write_SCCB(0xE4, temp);
        break;

    }

    SCCB_salve_Address = SCCB_salve_Address;
}
