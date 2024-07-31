//Esp32 GateWay
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

#include "cJSON.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"

static const char *TAG = "MQTT_TCP";

#define MAX_PAQUETE_SIZE (sizeof(char) + sizeof(int) + MAX_DATOS_SIZE + sizeof(char) + sizeof(uint32_t))

extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

float MenTemp;  
int MenLuz;
char global_data[100];  

static esp_mqtt_client_handle_t mqtt_client_1 = NULL;
static esp_mqtt_client_handle_t mqtt_client_2 = NULL;
static esp_mqtt_client_handle_t mqtt_client_3 = NULL;


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

void guardar_datos_recibidos(const PaqueteDatos *paquete, int topic) {
    // Verifica que los datos no excedan el tamaño del array global
    if (paquete->longitud < sizeof(global_data)) {
        // Copia los datos recibidos a la variable global
        strncpy(global_data, paquete->datos, paquete->longitud);
        // Asegura que la cadena esté terminada adecuadamente
        global_data[paquete->longitud] = '\0';
    }

    if(topic == 1){
        MenTemp = atof(global_data);
    }
    if(topic == 2){
        MenLuz = atoi(global_data);
    }
}

void desempaquetar_paquete(const char *paquete_serializado, int longitud_paquete, int topic) {
    if (longitud_paquete > MAX_PAQUETE_SIZE) {
        printf("Error: Longitud de datos incorrecta.\n");
        return;
    }

    int offset = 0;
    PaqueteDatos paquete;

    // Extraer cada campo de la cadena de datos y asignarlo a la estructura 'paquete'
    memcpy(&(paquete.cabecera), &paquete_serializado[offset], sizeof(char));
    offset += sizeof(char);

    memcpy(&(paquete.longitud), &paquete_serializado[offset], sizeof(int));
    offset += sizeof(int);

    memcpy(paquete.datos, &paquete_serializado[offset], paquete.longitud);
    offset += paquete.longitud;

    memcpy(&(paquete.fin), &paquete_serializado[offset], sizeof(char));
    offset += sizeof(char);

    memcpy(&(paquete.crc32), &paquete_serializado[offset], sizeof(uint32_t));
    offset += sizeof(uint32_t);

    uint32_t crc_calculado = calcular_crc32((char *)&paquete, sizeof(paquete) - sizeof(paquete.crc32));
    if (crc_calculado == paquete.crc32) {
        /*printf("Cabecera: %c\n", paquete.cabecera);
        printf("Longitud: %d\n", paquete.longitud);
        printf("Datos: %s\n", paquete.datos);
        printf("Fin: %c\n", paquete.fin);
        printf("Crc32: %u\n", paquete.crc32);*/
        guardar_datos_recibidos(&paquete, topic);
    } else {
        printf("Paquete inválido\n");
    }
}

void delayMs(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
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
            .ssid = "CesarPhone",
            .password = "12345678"}};
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
        esp_mqtt_client_subscribe(mqtt_client_3, "EspSensorTmpSirc", 0);
        esp_mqtt_client_subscribe(mqtt_client_3, "EspSensorLuzSirc", 0);
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
        if (event->topic_len >= 16 && strncmp(event->topic, "EspSensorTmpSirc", 16) == 0) {
            desempaquetar_paquete(event->data, event->data_len, 1);
            printf("Temp: %.1f\n", MenTemp);
        
            cJSON *json_root = cJSON_CreateObject();
            cJSON_AddNumberToObject(json_root, "temperatura", MenTemp); // Cambia el valor de temperatura según tus necesidades
            char *json_payload = cJSON_Print(json_root);

            esp_mqtt_client_publish(mqtt_client_1, "v1/devices/me/telemetry", json_payload, 0, 1, 0);

            cJSON_Delete(json_root);
            free(json_payload);
        }
        if (event->topic_len >= 16 && strncmp(event->topic, "EspSensorLuzSirc", 16) == 0) {
            desempaquetar_paquete(event->data, event->data_len, 2);
            printf("Luz: %d\n", MenLuz);
        
            cJSON *json_root = cJSON_CreateObject();
            cJSON_AddNumberToObject(json_root, "luz", MenLuz); // Cambia el valor de temperatura según tus necesidades
            char *json_payload = cJSON_Print(json_root);

            esp_mqtt_client_publish(mqtt_client_2, "v1/devices/me/telemetry", json_payload, 0, 1, 0);

            cJSON_Delete(json_root);
            free(json_payload);
            
        }
        
        delayMs(1000);
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

esp_mqtt_client_handle_t  mqtt_app_start(const char *server_uri, const char *username, const char *password, const char *cert)

{
    esp_mqtt_client_config_t mqtt_cfg = {
                .uri = server_uri,
                .username = username,
                .password = password,
                .cert_pem = cert,                    
            };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    return client;
}

void app_main(void)
{
    nvs_flash_init();
    wifi_connection();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated ...........\n");

    mqtt_client_1 = mqtt_app_start("mqtt://mqtt.thingsboard.cloud", "z9VuNOiw1R8d5aeB74uj", NULL, NULL);
    mqtt_client_2 = mqtt_app_start("mqtt://mqtt.thingsboard.cloud", "jvUsBjJpwCpX4Hl0zNGv", NULL, NULL);
    mqtt_client_3 = mqtt_app_start("mqtts://test.mosquitto.org:8883", NULL, NULL, (const char *)mqtt_eclipseprojects_io_pem_start);
    
    

}