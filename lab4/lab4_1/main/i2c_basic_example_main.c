/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "icm42670.h"
#include <math.h>

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           8       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           10       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000

static icm42670_handle_t icm42670 = NULL;
static i2c_master_bus_handle_t i2c_handle = NULL;

static void i2c_bus_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_new_master_bus(&bus_config, &i2c_handle);
}

static void i2c_sensor_icm42670_init(void)
{

    i2c_bus_init();
    vTaskDelay(20/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(icm42670_create(i2c_handle, ICM42670_I2C_ADDRESS, &icm42670));

    /* Configuration of the accelerometer and gyroscope */
    const icm42670_cfg_t imu_cfg = {
        .acce_fs = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_1_5625HZ,
        .gyro_fs = GYRO_FS_250DPS,
        .gyro_odr = GYRO_ODR_12_5HZ,
    };
    vTaskDelay(20/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(icm42670_config(icm42670, &imu_cfg));
    vTaskDelay(20/portTICK_PERIOD_MS);
}

void app_main(void)
    {
    icm42670_value_t acc, gyro;
    complimentary_angle_t val;

    i2c_sensor_icm42670_init();
    vTaskDelay(20/portTICK_PERIOD_MS);

    /* Set accelerometer and gyroscope to ON */
    ESP_ERROR_CHECK(icm42670_acce_set_pwr(icm42670, ACCE_PWR_ON));
    vTaskDelay(20/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(icm42670_gyro_set_pwr(icm42670, 2));
    vTaskDelay(20/portTICK_PERIOD_MS);
    bool set = false;

    while(1) {
        vTaskDelay(20/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(icm42670_get_gyro_value(icm42670, &gyro));
        vTaskDelay(20/portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(icm42670_get_acce_value(icm42670, &acc));
        vTaskDelay(20/portTICK_PERIOD_MS);;
        ESP_ERROR_CHECK(icm42670_complimentory_filter(icm42670, &acc, &gyro, &val));
        float initR;
        float initP;

        if(!set){
            set = true;
            initR = val.roll;
            initP = val.pitch;
        }

        float roll = val.roll - initR;
        float pitch = val.pitch - initP;
        if (pitch > 180){
            pitch -= 360;
        }
        if (roll > 180){
            roll -= 360;
        }
        bool v = (abs((int)pitch) > 25) ? true : false;
        bool h = (abs((int)roll) > 25) ? true : false;
        if(v && roll<0){
            ESP_LOGI(TAG, "RIGHT");
        } else if(v && roll>0){
            ESP_LOGI(TAG, "LEFT");
        } else{
            ESP_LOGI(TAG, "ROLL IS FLAT");
        }
        if(h && pitch<0){
            ESP_LOGI(TAG, "DOWN");
        } else if(h && pitch>0){
            ESP_LOGI(TAG, "UP");
        } else{
            ESP_LOGI(TAG, "PITCH IS FLAT");
        }
        ESP_LOGI(TAG, "roll:%.2f, pitch:%.2f", roll, pitch);
        ESP_LOGI(TAG, "===================================");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

    icm42670_delete(icm42670);
    i2c_del_master_bus(i2c_handle);

    vTaskDelay(10); // Give FreeRTOS some time to free its resources
}
