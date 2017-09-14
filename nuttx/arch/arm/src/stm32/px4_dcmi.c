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
#if 0
#include <px4_config.h>
#include <px4_macros.h>
#include "no_warnings.h"
#include "mavlink_bridge_header.h"
#include <mavlink.h>
#include "utils.h"
#include "dcmi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx.h"

#include <uavcan_if.h>

//#define CONFIG_USE_PROBES
#include <bsp/probes.h>
#endif
#include "misc.h"
#include <string.h>
#include "px4_dcmi.h"
#include "settings.h"
#include "no_warnings.h"

#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "chip/stm32f40xxx_dma.h"

#if defined(CONFIG_STM32_STM32F40XX)
#include "stm32_dma.h"


#define FULL_IMAGE_SIZE                  (120*120)

typedef enum {DMA_Memory_0 = 0, DMA_Memory_1 = 1}PX4_DMA_MEM_E;

/* counters */
volatile uint8_t image_counter = 0;
volatile uint32_t frame_counter;
volatile uint32_t time_last_frame = 0;
volatile uint32_t cycle_time = 0;
volatile uint32_t time_between_next_images;
volatile uint8_t dcmi_calibration_counter = 0;

/* state variables */
volatile uint8_t dcmi_image_buffer_memory0 = 1;
volatile uint8_t dcmi_image_buffer_memory1 = 2;
volatile uint8_t dcmi_image_buffer_unused  = 3;
volatile uint8_t calibration_used;
volatile uint8_t calibration_unused;
volatile uint8_t calibration_mem0;
volatile uint8_t calibration_mem1;

/* image buffers */
uint8_t dcmi_image_buffer_8bit_1[FULL_IMAGE_SIZE*2];
uint8_t dcmi_image_buffer_8bit_2[FULL_IMAGE_SIZE*2];

uint8_t image_buffer_8bit_1[FULL_IMAGE_SIZE];
uint8_t image_buffer_8bit_2[FULL_IMAGE_SIZE];
uint8_t image_buffer_1_flag;
uint8_t image_buffer_2_flag;


uint32_t g_dma_data_flag = 0;

uint32_t time_between_images;

DMA_HANDLE g_dma_handle;

/* extern functions */
extern uint32_t get_boot_time_us(void);
extern void delay(unsigned msec);

typedef enum
{
    BUFFER_DMA    = 0,
    BUFFER_CPU    = 1,
    BUFFER_NOBODY = 2,
    BUFFER_USED   = 3,
}BUFFER_USER_E;

uint32_t g_dcmi_mem1_flag = BUFFER_NOBODY;
uint32_t g_dcmi_mem2_flag = BUFFER_NOBODY;

/**
 * @brief Initialize DCMI DMA and enable image capturing
 */
void enable_image_capture(void)
{
	global_data_reset_param_defaults();
		
	dcmi_clock_init();
        
	dcmi_hw_init();

    //ov7740_init();    
	
	dcmi_dma_init();

	
	//mt9v034_context_configuration();
	//dcmi_dma_enable();
	printf("%s\n", __FUNCTION__);
}

#if 0
/**
 * @brief DMA reconfiguration after changing image window
 */
void dma_reconfigure(void)
{
	dcmi_dma_disable();

	if (FLOAT_AS_BOOL(global_data.param[PARAM_VIDEO_ONLY]))
		dcmi_dma_init(FULL_IMAGE_SIZE);
	else
		dcmi_dma_init(global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT]);

	dcmi_dma_enable();
}

/**
 * @brief Calibration image collection routine restart
 */
void dcmi_restart_calibration_routine(void)
{
	/* wait until we have all 4 parts of image */
	while(frame_counter < 4){}
	frame_counter = 0;
	dcmi_dma_enable();
}

/**
 * @brief Interrupt handler of DCMI
 */
void DCMI_IRQHandler(void)
{
    printf("dcmi capture data complete\n");
	if (DCMI_GetITStatus(DCMI_IT_FRAME) != RESET)
	{
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
	}

	return;
}
#endif


/**
 * @brief Interrupt handler of DCMI DMA stream
 */
void DMA2_Stream1_IRQHandler(DMA_HANDLE handle, uint8_t status, void *arg)
{
    uint16_t pixel = 0;
    static uint64_t current_mem = 0;
    
	/* transfer completed */
	if (status & DMA_STATUS_TCIF)
	{
        if (current_mem%2 == 0)
        {
            if ((image_buffer_1_flag == BUFFER_NOBODY) || (image_buffer_1_flag == BUFFER_USED))
            {
                image_buffer_1_flag = BUFFER_DMA;

                for (pixel = 0; pixel < FULL_IMAGE_SIZE; pixel++)
                {
                    image_buffer_8bit_1[pixel] = (uint8_t)(dcmi_image_buffer_8bit_1[pixel*2]);
                }
                            
                image_buffer_1_flag = BUFFER_NOBODY;
            }
        }
        else
        {
            if ((image_buffer_2_flag == BUFFER_NOBODY) || (image_buffer_2_flag == BUFFER_USED))
            {
                image_buffer_2_flag = BUFFER_DMA;

                for (pixel = 0; pixel < FULL_IMAGE_SIZE; pixel++)
                {
                    image_buffer_8bit_2[pixel] = (uint8_t)(dcmi_image_buffer_8bit_2[pixel*2]);
                }
                            
                image_buffer_2_flag = BUFFER_NOBODY;
            }
        }
        current_mem++;
	}

	/* transfer half completed */
	if (status & DMA_STATUS_HTIF)
	{}
}

/**
 * @brief Swap DMA image buffer addresses
 */
void dma_swap_buffers(void)
{
    DMA_HANDLE dma_handle = g_dma_handle;
    
	/* check which buffer is in use */
	if (stm32_dmagetcurrentmemtarget(dma_handle))
	{
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
		    stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_1, DMA_Memory_0);
		else if (dcmi_image_buffer_unused == 2)
		    stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_2, DMA_Memory_0);
		else
		    {};//stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_3, DMA_Memory_0);

		int tmp_buffer = dcmi_image_buffer_memory0;
		dcmi_image_buffer_memory0 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}
	else
	{
		/* swap dcmi image buffer */
		if (dcmi_image_buffer_unused == 1)
		    stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_1, DMA_Memory_1);
		else if (dcmi_image_buffer_unused == 2)
		    stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_2, DMA_Memory_1);
		else
		    {};//stm32_dmamemtargetconfig(dma_handle, (uint32_t) dcmi_image_buffer_8bit_3, DMA_Memory_1);

		int tmp_buffer = dcmi_image_buffer_memory1;
		dcmi_image_buffer_memory1 = dcmi_image_buffer_unused;
		dcmi_image_buffer_unused = tmp_buffer;
	}
#if 0
	/* set next time_between_images */
	cycle_time = get_boot_time_us() - time_last_frame;
	time_last_frame = get_boot_time_us();

	if (image_counter) // image was not fetched jet
	{
		time_between_next_images = time_between_next_images + cycle_time;
	}
	else
	{
		time_between_next_images = cycle_time;
	}
#endif
	/* set new image true and increment frame counter*/
	image_counter += 1;

	return;
}

#if 0
uint32_t get_time_between_images(void){
	return time_between_images;
}
#endif

uint32_t get_frame_counter(void)
{
	return frame_counter;
}


/**
 * @brief Copy image to fast RAM address
 *
 * @param current_image Current image buffer
 * @param previous_image Previous image buffer
 * @param image_size Image size of the image to copy
 * @param image_step Image to wait for (if 1 no waiting)
 */
void dma_copy_image_buffers(uint8_t ** current_image, uint8_t ** previous_image, uint16_t image_size, uint8_t image_step)
{
	/* swap image buffers */
	uint8_t * tmp_image = *current_image;
	*current_image = *previous_image;
	*previous_image = tmp_image;

//TODO(NB dma_copy_image_buffers is calling uavcan_run());

	/* wait for new image if needed */
	while (image_counter < image_step) {
            //PROBE_1(false);
            //uavcan_run();
            //PROBE_1(true);
	}

	image_counter = 0;

	/* time between images */
	time_between_images = time_between_next_images;

	/* copy image */
	if (dcmi_image_buffer_unused == 1)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_1[pixel]);
	}
	else if (dcmi_image_buffer_unused == 2)
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_2[pixel]);
	}
	else
	{
		for (uint16_t pixel = 0; pixel < image_size; pixel++)
			{};//(*current_image)[pixel] = (uint8_t)(dcmi_image_buffer_8bit_3[pixel]);
	}	
}

/**
 * @brief Send calibration image with MAVLINK over USB
 *
 * @param image_buffer_fast_1 Image buffer in fast RAM
 * @param image_buffer_fast_2 Image buffer in fast RAM
 */
void send_calibration_image(uint8_t ** image_buffer_fast_1, uint8_t ** image_buffer_fast_2) {
#if 0
	/*  transmit raw 8-bit image */
	/* TODO image is too large for this transmission protocol (too much packets), but it works */
	mavlink_msg_data_transmission_handshake_send(
			MAVLINK_COMM_2,
			MAVLINK_DATA_STREAM_IMG_RAW8U,
			FULL_IMAGE_SIZE * 4,
			FULL_IMAGE_ROW_SIZE * 2,
			FULL_IMAGE_COLUMN_SIZE * 2,
			FULL_IMAGE_SIZE * 4 / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
			MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
			100);

	uint16_t frame = 0;
	uint8_t image = 0;
	uint8_t frame_buffer[MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN];

	for (int i = 0; i < FULL_IMAGE_SIZE * 4; i++)
	{

		if (i % FULL_IMAGE_SIZE == 0 && i != 0)
		{
			image++;
		}

		if (i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN == 0 && i != 0)
		{
			mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);
			frame++;
			delay(2);
		}

		if (image == 0 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_1)[i % FULL_IMAGE_SIZE];
		}
		else if (image == 1 )
		{
			frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = (uint8_t)(*image_buffer_fast_2)[i % FULL_IMAGE_SIZE];
		}
		else if (image == 2)
		{
			if (calibration_unused == 1)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
			else if (calibration_unused == 2)
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
			else
				frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
		}
		else
		{
			if (calibration_used)
			{
				if (calibration_mem0 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
				else if (calibration_mem0 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
			}
			else
			{
				if (calibration_mem1 == 1)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_1[i % FULL_IMAGE_SIZE];
				else if (calibration_mem1 == 2)
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_2[i % FULL_IMAGE_SIZE];
				else
					frame_buffer[i % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = dcmi_image_buffer_8bit_3[i % FULL_IMAGE_SIZE];
			}
		}
	}

	mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2, frame, frame_buffer);
#endif
}

/**
 * @brief Initialize/Enable DCMI Interrupt
 */
void dcmi_it_init(void)
{
    DCMI_ClearITPendingBit(DCMI_IT_FRAME);
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
}

/**
 * @brief Initialize/Enable DMA Interrupt
 */
void dma_it_init(void)
{
#if 0
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_HT, ENABLE); // half transfer interrupt
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE); // transfer complete interrupt
#endif	
}




/**
 * @brief Enable DCMI DMA stream
 */
void dcmi_dma_enable(void)
{
#if 0
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	DMA_Cmd(DMA2_Stream1, ENABLE);
	int32_t arg;
	stm32_dmastart(g_dma_handle, (dma_callback_t)dma_transfer_complete, (void*)&arg, true);
	dcmi_it_init();
	
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
	dma_it_init();

    for (int i = 0; i < 20; i++)
    {
        stm32_dmastart(g_dma_handle, (dma_callback_t)dma_transfer_complete, (void*)&arg, false);
        printf("abc\n");
        sleep(1);
    };	
#endif
}

/**
 * @brief Disable DCMI DMA stream
 */
void dcmi_dma_disable()
{
	/* Disable DMA2 stream 1 and DCMI interface then stop image capture */
	//DMA_Cmd(DMA2_Stream1, DISABLE);
	stm32_dmastop(g_dma_handle);
	DCMI_Cmd(DISABLE);
	DCMI_CaptureCmd(DISABLE);
}

void reset_frame_counter()
{
	frame_counter = 0;
}

/**
 * @brief HW initialization of DCMI clock
 */
void dcmi_clock_init()
{
#if 0
	GPIO_InitTypeDef GPIO_InitStructure;	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
	TIM_OCInitTypeDef TIM_OCInitStructure;	

	/* TIM3 clock enable */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	

	/* GPIOC clock enable */	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	

	/* GPIOC Configuration:  TIM3 CH3 (PC8)  */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	/* Connect TIM3 pins to AF2 */;	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);	

	/* Time base configuration */	
	TIM_TimeBaseStructure.TIM_Period = 3;	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	

	/* PWM1 Mode configuration: Channel3 */	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_Pulse = 2;// TIM_TimeBaseStructure.TIM_Period/2;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);	
	TIM_ARRPreloadConfig(TIM3, ENABLE);	

	/* TIM3 enable counter */	
	TIM_Cmd(TIM3, ENABLE);
#endif
	/* GPIOC clock enable */	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* clock generate */
    RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	printf("%s\n", __FUNCTION__);
}

/**
 * @brief HW initialization DCMI
 */
void dcmi_hw_init(void)
{
    //uint16_t image_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT] *2;

    /* Reset image buffers */
    for (int i = 0; i < sizeof(dcmi_image_buffer_8bit_1); i++)
    {
        dcmi_image_buffer_8bit_1 [i] = 0;
        dcmi_image_buffer_8bit_2 [i] = 0;
        //dcmi_image_buffer_8bit_3 [i] = 0;
    }

    GPIO_InitTypeDef GPIO_InitStructure;
    //I2C_InitTypeDef I2C_InitStruct;

    /*** Configures the DCMI GPIOs to interface with the OV7740 camera module ***/
    /* Enable DCMI GPIOs clocks */
    RCC_AHB1PeriphClockCmd(
            RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
                    | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

    /* Connect DCMI pins to AF13 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);   //DCMI_HSYNC
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);   //DCMI_PIXCL

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);   //DCMI_D5
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);   //DCMI_VSYNC

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);   //DCMI_D0 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);   //DCMI_D1

    //GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_DCMI); //DCMI_D8
    //GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_DCMI); //DCMI_D9

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);   //DCMI_D2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);   //DCMI_D3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);  //DCMI_D4  
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);   //DCMI_D6
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);   //DCMI_D7

    /* DCMI GPIO configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | 
                                  GPIO_Pin_12| GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;         
    GPIO_Init(GPIOE, &GPIO_InitStructure);
#if 0
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    
    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Connect I2C1 pins to AF4 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    /* Configure I2C1 GPIOs */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C DeInit */
    I2C_DeInit(I2C1);
    
    /* Enable the I2C peripheral */
    I2C_Cmd(I2C1, ENABLE);

    /* Set the I2C structure parameters */
    I2C_InitStruct.I2C_Mode        = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle   = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
    I2C_InitStruct.I2C_Ack         = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed  = 100000;

    /* Initialize the I2C peripheral w/ selected parameters */
    I2C_Init(I2C1, &I2C_InitStruct);

    /* Initialize GPIOs for EXPOSURE and STANDBY lines of the camera */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
#endif    
    /* reset and pwd init */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
	printf("%s\n", __FUNCTION__);
}

/**
  * @brief  Configures DCMI/DMA to capture image from the mt9v034 camera.
  *
  * @param  buffer_size Buffer size in bytes
  */
void dcmi_dma_init(void)
{
	reset_frame_counter();

    uint16_t buffer_size = global_data.param[PARAM_IMAGE_WIDTH] * global_data.param[PARAM_IMAGE_HEIGHT] * 2;

	DCMI_InitTypeDef DCMI_InitStructure;

	/*** Configures the DCMI to interface with the mt9v034 camera module ***/
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode      = DCMI_CaptureMode_Continuous;
	DCMI_InitStructure.DCMI_SynchroMode      = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity      = DCMI_PCKPolarity_Falling;
	DCMI_InitStructure.DCMI_VSPolarity       = DCMI_VSPolarity_Low;
	DCMI_InitStructure.DCMI_HSPolarity       = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_CaptureRate      = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* DCMI configuration */
	DCMI_Init(&DCMI_InitStructure);

	//mt9v034_context_configuration();
	ov7740_init();

    set_ov7740_brightness(-4);
	set_ov7740_contrast(0);

	//dcmi_it_init();

    DCMI_Cmd(ENABLE);
    g_dma_handle = stm32_dmachannel(DMAMAP_DCMI_1);
    if (g_dma_handle == NULL)
    {
        printf("alloc dma channel fail!!!\n");
        return;
    }

    uint32_t scr_reg = 0;
    scr_reg = DMA_SCR_TCIE | DMA_SCR_HTIE | 
              DMA_SCR_DIR_P2M |
              DMA_SCR_CIRC | 
              DMA_SCR_MINC | 
              DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS |
              DMA_SCR_PRIHI |
              DMA_SCR_DBM |
              DMA_SCR_PBURST_SINGLE |
              DMA_SCR_MBURST_SINGLE |
              DMA_SCR_CHSEL(1);

    stm32_dmasetup(g_dma_handle, (uint32_t)DCMI_DR_ADDRESS, (uint32_t)dcmi_image_buffer_8bit_1,
                    buffer_size/4, scr_reg);

    stm32_dmasetbuffer_1(g_dma_handle, (uint32_t)dcmi_image_buffer_8bit_2);
   
    DCMI_CaptureCmd(ENABLE);
    uint32_t arg;
    stm32_dmastart(g_dma_handle, (dma_callback_t)DMA2_Stream1_IRQHandler, (void*)&arg, true);

    printf("%s\n", __FUNCTION__);
    return;
}


#endif
