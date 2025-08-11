/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>

#include "bmp5.h"
#include "common.h"
#include "bmp5_defs.h"


/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations of the sensor.
 *
 *  @param[in,out] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev                   : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor data.
 *
 *  @param[in] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev               : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

static int8_t set_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    //BMP5 sensörünü ölçüm yapmaya hazır hale getirecek şekilde konfigüre etmek
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    //bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);
        //şu anki sıcaklık/basınç oversampling ve data rate bilgileri okunur.
        //bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;
            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
            //Basınç ölçümünü etkinleştir ve yeni ayarı yaz
            //bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            //IIR (Filtre) ayarlarını yap:
            set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1; //COEFF_1 = en hafif filtreleme (hızlı tepki, daha az yumuşatma).
            set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
            set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
            //bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            //bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.drdy_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                //Veri hazır (Data Ready - DRDY) olduğunda kesme oluşturulsun
                //bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /* Set powermode as continous */
        //Sensörü sürekli ölçüm moduna al (continuous):
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_CONTINOUS, dev);
        //bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);
    }

    return rslt;
}

static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
	int8_t rslt = 0;
	    uint8_t idx = 0;
	    extern struct bmp5_sensor_data sensor_data;

	    while (idx < 3)
	    {
            //BMP5 sensöründen 3 defa ölçüm alarak sıcaklık ve basınç verilerini okumak
	        rslt = bmp5_get_sensor_data(&sensor_data, osr_odr_press_cfg, dev);
	        bmp5_error_codes_print_result("bmp5_get_sensor_data", rslt);

	        if (rslt == BMP5_OK)
	        {
	#ifdef BMP5_USE_FIXED_POINT
	            //printf("%d, %lu, %ld\n", idx, pressure, temperature);
	#else
	            printf("%d, %f, %f\n", idx, sensor_data.pressure, sensor_data.temperature);
	#endif
	            idx++;
	        }

	        HAL_Delay(10);  // 20ms bekle, sensör ODR'sine göre ayarla
	    }

	    return rslt;
	}

void BMP581(void){
int8_t rslt;
	      struct bmp5_dev dev;
	      struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

	      /* Interface reference is given as a parameter
	       * For I2C : BMP5_I2C_INTF
	       * For SPI : BMP5_SPI_INTF
	       */
	      rslt = init_bmp5_interface(&dev, BMP5_I2C_INTF);
	      bmp5_error_codes_print_result("bmp5_interface_init", rslt);

	      if (rslt == BMP5_OK)
	      {
	          rslt = bmp5_init(&dev);
	          //bmp5_error_codes_print_result("bmp5_init", rslt);
	          rslt = set_config(&osr_odr_press_cfg, &dev);
	          //bmp5_error_codes_print_result("set_config", rslt);
	          rslt = get_sensor_data(&osr_odr_press_cfg, &dev);
	          //bmp5_error_codes_print_result("get_sensor_data", rslt);
	          
	      }
	      return rslt;
        }

