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
#include "driver/gptimer.h"
#include "driver/gpio.h"

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

#define TRIG_GPIO GPIO_NUM_5
#define ECHO_GPIO GPIO_NUM_4

static gptimer_handle_t gptimer = NULL;
static uint64_t pulse_time = 0;
static bool measurement_ready = false;

static void IRAM_ATTR echo_edge_isr_handler(void *arg) {

    int level = gpio_get_level(ECHO_GPIO);

    if (level == 1) {
        // Rising edge - start timing
        gptimer_set_raw_count(gptimer, 0);
        gptimer_start(gptimer);
    } else {
        // Falling edge - stop timing
        gptimer_get_raw_count(gptimer, &pulse_time);
        gptimer_stop(gptimer);
        measurement_ready = true;
    }
}

void initGPIO(void){
    gpio_config_t trig_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TRIG_GPIO
    };
    gpio_config(&trig_conf);

    // Configure echo pin
    gpio_config_t echo_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << ECHO_GPIO,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&echo_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_GPIO, echo_edge_isr_handler, NULL);

    // Setup GPTimer (counts in microseconds)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000 // 1 MHz = 1 tick per microsecond
    };
    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_enable(gptimer);
}

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

    initGPIO();

    while(1){
        shtc3_send_command(dev_handle, WAKEUP_CMD);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        shtc3_send_command(dev_handle, 0x7CA2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        shtc3_read_data(dev_handle, data, sizeof(data));

        shtc3_send_command(dev_handle, SLEEP);
        uint16_t celsius = getTemp(data);

        gpio_set_level(TRIG_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_GPIO, 0);

        vTaskDelay(pdMS_TO_TICKS(60));

        ESP_LOGI(TAG, "Temperature is %dC", celsius);

        if (measurement_ready) {
            measurement_ready = false;
            float distance_cm = pulse_time / 58.0f;
            ESP_LOGI(TAG, "Distance: %.2f cm\n\n", distance_cm);
        } else {
            ESP_LOGW(TAG, "Sensor too far");
        }
        vTaskDelay(600 / portTICK_PERIOD_MS);
    }

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
