#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h" 
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "motor_logic.h"
#include "wifi_server.h"

static const char *TAG = "MAIN_GRUA";

#define GPIO_POWER_EN  23

#define BIT_CALIB_X (1 << 0)
#define BIT_CALIB_Y (1 << 1)

typedef struct {
    char eje; int pin_dir; int pin_endstop; ledc_channel_t channel;
    EventGroupHandle_t ev_group; EventBits_t done_bit;
} calib_args_t;

void tarea_homing(void *pvParameters) {
    calib_args_t *args = (calib_args_t *)pvParameters;
    homing_eje(args->eje, args->pin_dir, args->pin_endstop, args->channel);
    xEventGroupSetBits(args->ev_group, args->done_bit);
    vTaskDelete(NULL); 
}

void app_main(void) {
    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, "  INICIANDO SISTEMA MECATRONICO    ");
    ESP_LOGI(TAG, "===================================");

    // 1. CLAVAR PINES A 0V ANTES DE DAR ENERGÍA
    // Inicializa la lógica, PWM y deja todos los motores frenados por hardware
    init_motor_logic();

    
    // 2. Energizar etapa de potencia de forma segura
    gpio_reset_pin(GPIO_POWER_EN);
    gpio_set_direction(GPIO_POWER_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_POWER_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera corta solo para estabilizar capacitores

    // 3. Levantar comunicaciones
    init_wifi();
    
    // 4. Pasar a estado seguro
    estado_actual = ESTADO_ESPERA_HOMING;
    
    ESP_LOGW(TAG, "=====================================================");
    ESP_LOGW(TAG, " ATENCION: SISTEMA EN ESPERA_HOMING.                 ");
    ESP_LOGW(TAG, " LOS MOTORES ESTAN BLOQUEADOS POR SEGURIDAD.         ");
    ESP_LOGW(TAG, "=====================================================");
}