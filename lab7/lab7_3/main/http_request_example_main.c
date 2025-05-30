/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

#include "driver/i2c_master.h"

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

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.0.186"
#define WEB_PORT "2234"
#define WEB_PATH "/"

static const char *TAG = "example";

static const char *WTTRGETREQUEST = "GET /%s?format=%%t HTTP/1.0\r\n"
    "Host: wttr.in:80\r\n"
    "User-Agent: esp-idf/1.0 esp32 curl\r\n"
    "\r\n";

static const char *GETREQUEST = "GET " WEB_PATH " HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "User-Agent: esp-idf/1.0 esp32 curl\r\n"
    "\r\n";

static const char *REQUEST = "POST " WEB_PATH " HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "Content-Type: text/plain\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Content-Length: %d\r\n"
    "\r\n"
    "%s";

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

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    uint8_t data[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    char postRequest[456];

    while(1) {

        shtc3_send_command(dev_handle, WAKEUP_CMD);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        shtc3_send_command(dev_handle, 0x7CA2);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        shtc3_read_data(dev_handle, data, sizeof(data));

        shtc3_send_command(dev_handle, SLEEP);
        uint16_t celsius = getTemp(data);
        uint16_t humidity = getHumidity(data+3);
        uint16_t fahrenheit = celsius * (9/5) + 32;

        ESP_LOGW(TAG, "Temperature is %dC (or %dF) with a %d%% humidity", celsius, fahrenheit, (int) humidity);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        

//==============================================================================================
//GET REQUEST LOCATION
        if (write(s, GETREQUEST, strlen(GETREQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        char buffer[400];
        memset(buffer, 0, 400);
        do {
            char* b = buffer;
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }if(r){
                memcpy(b, recv_buf, r);
                b += r;
            }

        } while(r > 0);
        close(s);
        //char * loc = "Santa_Cruz";
        
        char * loc;
        loc = strstr(buffer, "\r\n\r\n");
        loc += 4;

        ESP_LOGW(TAG, "loc = %s", loc);
        ESP_LOGW(TAG, "buffer = %s", buffer);
        for (int i = 0; loc[i] != '\0'; i++){
            if (loc[i] == ' '){
                loc[i] = '_';
            }
        }

        //GET REQUEST WTTR.IN
        snprintf(postRequest, sizeof(postRequest), WTTRGETREQUEST, loc);


        struct addrinfo *res2;
        struct in_addr *addr2;

        int err2 = getaddrinfo("wttr.in", "80", &hints, &res2);

        if(err2 != 0 || res2 == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res2);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        addr2 = &((struct sockaddr_in *)res2->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr2));

        s = socket(res2->ai_family, res2->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res2);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res2->ai_addr, res2->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res2);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        if (write(s, postRequest, strlen(postRequest)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        memset(buffer, 0, 400);
        do {
            char* b = buffer;
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }if(r){
                memcpy(b, recv_buf, r);
                b += r;
            }

        } while(r > 0);

        char * temp = strstr(buffer, "+");
        temp +=1;
        temp[3] = '\0';
        int wttrTemp = atoi(temp);
        close(s);

        //POST REQUEST SERVER

        char content[200];
        snprintf(content, sizeof(content), "Temperature from Wttr.in is %dC\nTemperature from sensor is %dC (or %dF) with a %d%% humidity", wttrTemp, celsius, fahrenheit, humidity);
        snprintf(postRequest, sizeof(postRequest), REQUEST, strlen(content), content);

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        if (write(s, postRequest, strlen(postRequest)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

//============================================================================================================
        freeaddrinfo(res);
        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
}
