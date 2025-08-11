/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "bmp5.h" 
#include "bmp5_defs.h"
#include "stm32f4xx_hal.h"  // Kartına göre değişebilir (örn. stm32f1xx_hal.h)

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BMP5 shuttle id */
#define BMP5_SHUTTLE_ID_PRIM  UINT16_C(0x1B3)
#define BMP5_SHUTTLE_ID_SEC   UINT16_C(0x1D3)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;
extern I2C_HandleTypeDef hi2c1;
/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr,
                                  uint8_t *reg_data,
                                  uint32_t length,
                                  void *intf_ptr)
{
	uint8_t device_addr = BMP581_I2C_ADDR<<1;

	    (void)intf_ptr;
    HAL_StatusTypeDef status;
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;

    // Hattın meşgul olup olmadığını kontrol et

        HAL_I2C_DeInit(hi2c);
        HAL_Delay(10);
        HAL_I2C_Init(hi2c);



    // Cihaz hazır mı?
    if (HAL_I2C_IsDeviceReady(hi2c, device_addr, 3, 10) != HAL_OK)
    {
        printf("I2C Cihaz hazır değil! Adres: 0x%02X\n", device_addr);
        return BMP5_E_COM_FAIL;
    }
    status = HAL_I2C_IsDeviceReady(hi2c, 0x8C, 3, 10);
    printf("IsDeviceReady: %s\n", (status == HAL_OK) ? "OK" : "FAIL");


    // Okuma işlemi
    status = HAL_I2C_Mem_Read(hi2c,
    		BMP581_I2C_ADDR << 1,
                               reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               reg_data,
                               length,
                               10);
    printf("READ: addr=0x%02X reg=0x%02X len=%lu\n", dev_addr, reg_addr, length);

    if (status != HAL_OK)
    {
        printf("HAL_I2C_Mem_Read FAILED! Status=%d\n", status);
        return BMP5_E_COM_FAIL;
    }




    if (status != HAL_OK)
    {
        printf("I2C READ ERROR: Reg=0x%02X Len=%lu Status=%d\n", reg_addr, length, status);
        return BMP5_E_COM_FAIL;
    }

    return BMP5_INTF_RET_SUCCESS;
}





/*!
 * I2C write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr,
                                   const uint8_t *reg_data,
                                   uint32_t length,
                                   void *intf_ptr)
{
    HAL_StatusTypeDef status;

    // I2C handle'ı interface pointer'dan al
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;

    // I2C 8-bit adres (0x46 << 1 = 0x8C, veya 0x47 << 1 = 0x8E)
    uint8_t dev_addr = BMP581_I2C_ADDR;  // Örn: #define BMP581_I2C_ADDR (0x46 << 1)

    // Yazma işlemini gerçekleştir
    status = HAL_I2C_Mem_Write(hi2c,
    		0x8C,
                               reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)reg_data,
                               length,
                               10);  // timeout: 100 ms

    // Başarı kontrolü
    if (status != HAL_OK)
    {
        printf("I2C WRITE FAIL: Reg=0x%02X Len=%lu Status=%d\n", reg_addr, length, status);
        return BMP5_E_COM_FAIL;
    }

    return BMP5_INTF_RET_SUCCESS;
}


/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP5_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            printf("Error [%d] : Invalid chip id\r\n", rslt);
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            printf("Error [%d] : Power up error\r\n", rslt);
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
void bmp5_delay(uint32_t period, void *intf_ptr)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000) * period;
    while ((DWT->CYCCNT - start) < cycles);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t init_bmp5_interface(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
    int8_t rslt = BMP5_OK;

    if (bmp5_dev == NULL)
    {
        return BMP5_E_NULL_PTR;
    }

    // I2C arayüzü kullanılacaksa
    if (intf == BMP5_I2C_INTF)
    {
        printf("I2C Interface\n");

        bmp5_dev->intf_ptr = &hi2c1;
        bmp5_dev->read = bmp5_i2c_read;
        bmp5_dev->write = bmp5_i2c_write;
        bmp5_dev->intf = BMP5_I2C_INTF;
        //bmp5_dev->delay_us = bmp5_delay;

        // INT pini ayarla (PB6)
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
//        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        //HAL_Delay(10);  // Stabilite için bekleme
    }
    else
    {
        return BMP5_E_COM_FAIL;  // Geçersiz arayüz tipi
    }

    return rslt;  // Başarıyla tamamlandıysa BMP5_OK döner
}





