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

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           8       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           10       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SHTC3_SENSOR_ADDR           0x70        /*!< Address of the MPU9250 sensor */
#define WAKEUP_CMD                  0x3517 
#define SLEEP                       0xB098

uint8_t crc8(const uint8_t *data, uint32_t len) {
    const uint8_t CRC8_POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

//data must be big enough for all data
esp_err_t shtc3_read_data(i2c_master_dev_handle_t dev_handle, uint8_t* data_out, uint8_t data_length)
{
    esp_err_t err = i2c_master_receive(dev_handle, data_out, data_length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    bool success = (crc8(data_out, 2) == data_out[2]);
    success &= (crc8(data_out+3, 2) == data_out[5]);
    if(!success){
        ESP_LOGE(TAG, "CHECKSUM INVALID");
    }
    return err;
}

esp_err_t shtc3_send_command(i2c_master_dev_handle_t dev_handle, uint16_t cmd)
{
    uint8_t data[2] = {(uint8_t)((cmd & 0xff00) >> 8), (uint8_t)(cmd & 0xff)};
    return i2c_master_transmit(dev_handle, data, sizeof(data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

uint16_t getTemp(uint8_t* data){
    uint16_t temp = (data[0] << 8 | data[1]);
    return -45 + 175 * (((float)temp)/(1<<16));
}

uint16_t getHumidity(uint8_t* data){
    uint16_t h = (data[0] << 8 | data[1]);
    return 100 * (((float)h)/(1<<16));
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC3_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{
    uint8_t data[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    while(1){
        shtc3_send_command(dev_handle, WAKEUP_CMD);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        shtc3_send_command(dev_handle, 0x7CA2);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        shtc3_read_data(dev_handle, data, sizeof(data));

        shtc3_send_command(dev_handle, SLEEP);
        uint16_t celsius = getTemp(data);
        uint16_t humidity = getHumidity(data+3);
        uint16_t fahrenheit = celsius * (9/5) + 32;

        ESP_LOGI(TAG, "Temperature is %dC (or %dF) with a %d%% humidity", celsius, fahrenheit, (int) humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
