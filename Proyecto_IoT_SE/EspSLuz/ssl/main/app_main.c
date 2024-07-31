//Esp Sensor Luz
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
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


#define SensorTemp GPIO_NUM_39

static const char *TAG = "MQTT_TCP";

extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");


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

void delayMs(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

// Función para configurar el ADC
void configurar_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);                        // Configura la precisión a 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Configura la atenuación para el sensor de luz
}

// Función para leer el valor del sensor de luz
float leerSensorLuz()
{
    int valor_adc = adc1_get_raw(ADC1_CHANNEL_0); // Valor leído del ADC (de 0 a 4095)
    
    float porcentaje = (valor_adc/ 4095.0) * 100.0 * 6;

    printf("\n------------------------------------------\n");
    ESP_LOGI(TAG, "Luz = %.1f %%", porcentaje);

    return porcentaje;
}

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

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, "EspSensorLuzSirc", "Conexion realizada...", 0, 1, 0);
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
        char messageContent[50]; 
        snprintf(messageContent, sizeof(messageContent), "%.1f", leerSensorLuz());

        // Crear el paquete según la estructura definida
        paquete.cabecera = 'I'; // Ejemplo de cabecera
        strcpy(paquete.datos, messageContent); // Ejemplo de datos
        paquete.longitud = strlen(paquete.datos);
        paquete.fin = 'F';
        paquete.crc32 = calcular_crc32((char *)&paquete, sizeof(paquete) - sizeof(paquete.crc32));

        empaquetar_paquete(&paquete, paquete_serializado);

        printf("Cabecera: %c\n", paquete.cabecera);
        printf("Longitud: %d\n", paquete.longitud);
        printf("Datos: %s\n", paquete.datos);
        printf("Fin: %c\n", paquete.fin);
        printf("Crc32: %u\n", paquete.crc32);

        esp_mqtt_client_publish(client, "EspSensorLuzSirc", paquete_serializado, MAX_PAQUETE_SIZE, 0, 0);
        delayMs(5000);

    }
    
}

void app_main(void)
{
    configurar_adc();
    nvs_flash_init();
    wifi_connection();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n");

    mqtt_app_start();

}