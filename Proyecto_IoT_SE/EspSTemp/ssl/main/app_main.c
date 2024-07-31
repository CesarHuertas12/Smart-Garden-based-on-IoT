//Esp32 Sensor Temperatura
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdlib.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
//#include "my_data.h"
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include <driver/adc.h>

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU6050_SENSOR_ADDR 0x68       /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR 0x75 /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT 7

#define POLY 0xEDB88320L
#define MAX_DATOS_SIZE 1000

typedef struct {
    char cabecera;
    uint32_t longitud;
    char datos[MAX_DATOS_SIZE];
    char fin;
    uint32_t crc32;
} PaqueteDatos;

uint32_t tabla_crc[256];

void generar_tabla_crc() {
    uint32_t crc;
    int i, j;

    for (i = 0; i < 256; i++) {
        crc = i;
        for (j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ POLY : (crc >> 1);
        }
        tabla_crc[i] = crc;
    }
}

uint32_t calcular_crc32(const char *datos, int longitud) {
    uint32_t crc = 0xFFFFFFFF;
    int i;

    for (i = 0; i < longitud; i++) {
        crc = (crc >> 8) ^ tabla_crc[(crc ^ datos[i]) & 0xFF];
    }

    return crc ^ 0xFFFFFFFF;
}

#define MAX_PAQUETE_SIZE (sizeof(char) + sizeof(int) + MAX_DATOS_SIZE + sizeof(char) + sizeof(uint32_t))

void empaquetar_paquete(const PaqueteDatos *paquete, char *paquete_serializado) {
    int offset = 0;

    // Copiar cada campo de la estructura al paquete serializado
    memcpy(&paquete_serializado[offset], &(paquete->cabecera), sizeof(char));
    offset += sizeof(char);

    memcpy(&paquete_serializado[offset], &(paquete->longitud), sizeof(int));
    offset += sizeof(int);

    memcpy(&paquete_serializado[offset], paquete->datos, paquete->longitud); // Considerando longitud como cantidad de datos válidos
    offset += paquete->longitud;

    memcpy(&paquete_serializado[offset], &(paquete->fin), sizeof(char));
    offset += sizeof(char);

    memcpy(&paquete_serializado[offset], &(paquete->crc32), sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Asegurarse de agregar el carácter nulo al final si se utilizará como cadena
    paquete_serializado[offset] = '\0';
}

extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

void delayMs(uint16_t ms){
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

//Configura SPI
/**
 * @brief Read a sequence of bytes from a sensor registers
 */
static esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a sensor register
 */
static esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

float SensorTemp(){
    uint8_t data[2];
    uint16_t valor_combi = 0;
    float temperatura = 0;
    uint8_t registros[3] = {0x75, 0x41, 0x42}; //Registros de temperatura

    for(int i = 0; i<3; i++){
            ESP_ERROR_CHECK(device_register_read(registros[i], data, 1));
            if(i == 0) {
               // ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
                valor_combi = 0;
            }
            if(i == 1) {
               // ESP_LOGI(TAG, "TEMP_OUT_H = %X", data[0]);
                valor_combi = (data[0] << 8);
            }
            if(i == 2) {
               // ESP_LOGI(TAG, "TEMP_OUT_L = %X", data[0]);
                valor_combi |= data[0];
            }
    
        }
        //ESP_LOGI(TAG, "Combined Value = %X", valor_combi);
        temperatura = (float)valor_combi / 340.0 - 160;
        ESP_LOGI(TAG, "Temperatura = %.2f °C", temperatura);

        printf("\n------------------------------------------\n");
        //delayMs(1000);
        return temperatura;
}

//Se configura el wifi
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    // 1 - Wi-Fi/LwIP Init Phase
    esp_netif_init();                    // TCP/IP initiation 					s1.1
    esp_event_loop_create_default();     // event loop 			                s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station 	                    s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); // 					                    s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "EspSirc",
            .password = "Sirc8500"}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_start();
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
}

//Se configura el mqtt
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "sirc", 0);
        esp_mqtt_client_publish(client, "EspSensorTmpSirc", "Conexion realizada...", 0, 1, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("\nTOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}


static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtts://test.mosquitto.org:8883",
        .cert_pem = (const char *)mqtt_eclipseprojects_io_pem_start,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    while(1){
        PaqueteDatos paquete;
        char paquete_serializado[MAX_PAQUETE_SIZE]; 
        char messageContent[50];  // Suponemos un tamaño máximo de 20 caracteres
        snprintf(messageContent, sizeof(messageContent), "%.2f", SensorTemp());

        // Crear el paquete según la estructura definida
        paquete.cabecera = 'S'; // Ejemplo de cabecera
        strcpy(paquete.datos, messageContent); // Ejemplo de datos
        paquete.longitud = strlen(paquete.datos);
        paquete.fin = 'E';
        paquete.crc32 = calcular_crc32((char *)&paquete, sizeof(paquete) - sizeof(paquete.crc32));

        empaquetar_paquete(&paquete, paquete_serializado);

        printf("Cabecera: %c\n", paquete.cabecera);
        printf("Longitud: %d\n", paquete.longitud);
        printf("Datos: %s\n", paquete.datos);
        printf("Fin: %c\n", paquete.fin);
        printf("Crc32: %u\n", paquete.crc32);

        esp_mqtt_client_publish(client, "EspSensorTmpSirc",  paquete_serializado, MAX_PAQUETE_SIZE, 0, 0);
        delayMs(5000);
    }
    
}



void app_main(void)
{
    uint8_t data[2];
    nvs_flash_init();
    wifi_connection();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_ERROR_CHECK(device_register_read(0X6B, data, 1));
    ESP_LOGI(TAG, "BITS = %X", data[0]);
    ESP_ERROR_CHECK(device_register_write_byte(0x6B, (0<<6)));
    ESP_ERROR_CHECK(device_register_read(0x6B, data, 1));
    ESP_LOGI(TAG, "BITS = %X", data[0]);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n");

    mqtt_app_start();

}
