/*
 * control.c
 *
 *  Created on: Aug 11, 2025
 *      Author: aliya
 */

#include "control.h"


float previous_gyr_rps[3] = {0.0f, 0.0f, 0.0f};
float angular_accel_rps2[3] = {0.0f, 0.0f, 0.0f};
uint32_t last_gyro_tick = 0;




void control_init(BMI088 *imu)              
{

BMI088_Init_I2C(&imu, &hi2c3, acc_adress, gyro_adress);

HAL_StatusTypeDef kontrol = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

}


void control_motor(float tvcx, float tvcy){

    




}


void control_tvc(float angularacc_x, float angularacc_y ){

    float tvc_X = (referans - angularacc_x) * Kp_x;

    float tvc_Y = (referans - angularacc_y) * KP_y;

    void control_motor(tvc_X,tvc_Y);


}


void control_data(BMI088 *imu)
{

uint32_t now_tick = HAL_GetTick();
float dt = (now_tick - last_gyro_tick) / 1000.0f; 

BMI088_ReadAccelerometer(&imu);
BMI088_ReadGyroscope(&imu);
rollpitchyaw(&imu);



if (dt > 0.0001f)
    {
        angular_accel_rps2[0] = (imu->gyr_rps[0] - previous_gyr_rps[0]) / dt;   // x ekseni TVC hesaplamalarında bunlar kullanılacak
        angular_accel_rps2[1] = (imu->gyr_rps[1] - previous_gyr_rps[1]) / dt;   // y ekseni TVC hesaplamalarında bunlar kullanılacak
        angular_accel_rps2[2] = (imu->gyr_rps[2] - previous_gyr_rps[2]) / dt;   // z ekseni
        
        // Bir sonraki döngü için mevcut verileri sakla
        previous_gyr_rps[0] = imu->gyr_rps[0];
        previous_gyr_rps[1] = imu->gyr_rps[1];
        previous_gyr_rps[2] = imu->gyr_rps[2];
        
        last_gyro_tick = now_tick;
    }           


    control_motor(angular_accel_rps2[0],angular_accel_rps2[1])






HAL_DELAY(DELAY);

}

