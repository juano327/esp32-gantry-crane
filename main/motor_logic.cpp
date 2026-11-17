#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h" 
#include "driver/gpio.h"
#include "driver/ledc.h" 
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_logic.h"
#include "VL53L0X.h"


static const char *TAG = "MOTOR_LOGIC";

// Pines PWM (Los de Dirección ya están en el .h)
#define PIN_IN3 GPIO_NUM_32 // PWM Y
#define PIN_IN1 GPIO_NUM_18 // PWM X
#define PIN_IN5 GPIO_NUM_26 // PWM Z

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT 
#define LEDC_FREQUENCY          (5000)           

QueueHandle_t xMotorQueue = NULL;
// Instanciamos la máquina de estados
estado_grua_t estado_actual = ESTADO_INIT;

int pos_actual_x = 0;
int pos_actual_y = 0;
int tiempo_total_x_ms = 6000; 
int tiempo_total_y_ms = 4000; 

//Instanciar el objeto sensor
VL53L0X tof_sensor;

extern "C" void resetear_posicion(char eje) {
    if (eje == 'X') pos_actual_x = 0;
    if (eje == 'Y') pos_actual_y = 0;
}

// ==========================================
//  INICIALIZAR EL SENSOR LÁSER
// ==========================================
static void i2c_tof_init() {
    i2c_config_t conf ;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 400000;
        conf.clk_flags = 0;
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    //Iniciallizar el sensor VL53L0X con metodos nativos
    if (tof_sensor.init()){
        ESP_LOGI(TAG, "Sensor ToF inicializado correctamente.");
        
    } else {
        ESP_LOGE(TAG, "Error al inicializar el sensor ToF. Verifique conexiones.");
        
    }
}

static void configure_pwm() {
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.timer_num = LEDC_TIMER;
    ledc_timer.freq_hz = LEDC_FREQUENCY;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);
        
    ledc_channel_config_t ch_y = {};
    ch_y.gpio_num = PIN_IN3;
    ch_y.speed_mode = LEDC_MODE;
    ch_y.channel = LEDC_CHANNEL_Y;
    ch_y.intr_type = LEDC_INTR_DISABLE;
    ch_y.timer_sel = LEDC_TIMER;
    ch_y.duty = 0;
    ch_y.hpoint = 0;

    ledc_channel_config_t ch_x = {};
    ch_x.gpio_num = PIN_IN1;
    ch_x.speed_mode = LEDC_MODE;
    ch_x.channel = LEDC_CHANNEL_X;
    ch_x.intr_type = LEDC_INTR_DISABLE;
    ch_x.timer_sel = LEDC_TIMER;
    ch_x.duty = 0;
    ch_x.hpoint = 0;

    ledc_channel_config_t ch_z = {};
    ch_z.gpio_num = PIN_IN5;
    ch_z.speed_mode = LEDC_MODE;
    ch_z.channel = LEDC_CHANNEL_Z;
    ch_z.intr_type = LEDC_INTR_DISABLE;
    ch_z.timer_sel = LEDC_TIMER;
    ch_z.duty = 0;
    ch_z.hpoint = 0;
    
    ledc_channel_config(&ch_y);
    ledc_channel_config(&ch_x);
    ledc_channel_config(&ch_z);
}

extern "C" void homing_eje(char eje, int pin_dir, int pin_endstop, ledc_channel_t channel) {
    ESP_LOGW(TAG, "[%c] Buscando origen (Homing)...", eje);

    if (eje == 'Y'){
        // 1. BAJAR (DIR 0) buscando el láser o el fin de carrera
        gpio_set_level((gpio_num_t)pin_dir, 0); 
        ledc_set_duty(LEDC_MODE, channel, PWM_HOMING); 
        ledc_update_duty(LEDC_MODE, channel);

        int fallos_consecutivos_i2c = 0; //inicializamos el watchdog 

        while (1) {
            uint16_t actual_mm = 0;
            if (tof_sensor.read(&actual_mm)) {
                fallos_consecutivos_i2c = 0;
                if ((actual_mm <= TOF_DISTANCIA_MIN_MM && actual_mm > 0) || gpio_get_level((gpio_num_t)pin_endstop) == 0){
                    ESP_LOGW(TAG, "[Y] Tope inferior alcanzado.");
                    break;
                }
                else{
                    fallos_consecutivos_i2c++;
                    if (fallos_consecutivos_i2c >= 10) {
                        ESP_LOGE(TAG, "[Y] Error al leer el sensor ToF. Verifique conexiones.");
                        disparar_emergencia();
                        estado_actual = ESTADO_EMERGENCIA;
                        break;
                        
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }

        // 2. RUTINA DE DESATASCO (Back-off)
        if (gpio_get_level((gpio_num_t)pin_endstop) == 0) {
            ESP_LOGW(TAG, "[Y] Fin de carrera pisado. Liberando sensor...");
            
            gpio_set_level((gpio_num_t)pin_dir, 1); // Dirección: SUBIR
            ledc_set_duty(LEDC_MODE, channel, PWM_HOMING_INV); // Usamos la macro invertida
            ledc_update_duty(LEDC_MODE, channel);
            
            while (gpio_get_level((gpio_num_t)pin_endstop) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }

    } else {   
        // ==========================================
        // HOMING DEL EJE X
        // ==========================================
        gpio_set_level((gpio_num_t)pin_dir, 0); // Ir hacia el origen
        ledc_set_duty(LEDC_MODE, channel, PWM_HOMING); 
        ledc_update_duty(LEDC_MODE, channel);

        while (gpio_get_level((gpio_num_t)pin_endstop) != 0) { 
            vTaskDelay(pdMS_TO_TICKS(10)); 
        } 
        
        // Rutina de Desatasco para X
        ESP_LOGW(TAG, "[X] Fin de carrera pisado. Liberando sensor...");
        gpio_set_level((gpio_num_t)pin_dir, 1); // Alejar del origen
        ledc_set_duty(LEDC_MODE, channel, PWM_HOMING_INV); // Usamos la macro invertida
        ledc_update_duty(LEDC_MODE, channel);
        
        while (gpio_get_level((gpio_num_t)pin_endstop) == 0) { 
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    
    // 3. Freno total y reseteo
    ledc_set_duty(LEDC_MODE, channel, 0); 
    ledc_update_duty(LEDC_MODE, channel);
    gpio_set_level((gpio_num_t)pin_dir, 0);
    
    resetear_posicion(eje); 
    ESP_LOGI(TAG, "[%c] Homing completado. Sensor liberado y posicion reseteada a 0.", eje);
}

extern "C" void medir_rango_eje(char eje, int pin_dir, int pin_endstop, ledc_channel_t channel) {
    if (eje == 'Y') return;

    int pwm_velocidad = PWM_X ;    
    ESP_LOGI(TAG, "[%c] ARRANCANDO. Esperando que suelte el sensor...", eje);
    gpio_set_level((gpio_num_t)pin_dir, 1); 
    ledc_set_duty(LEDC_MODE, channel, 255 - pwm_velocidad); 
    ledc_update_duty(LEDC_MODE, channel);

    while (gpio_get_level((gpio_num_t)pin_endstop) == 0) { vTaskDelay(pdMS_TO_TICKS(10)); }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "[%c] ¡Sensor liberado! Ahora si, TOCA EL TOPE FINAL.", eje);
    int64_t tiempo_arranque = esp_timer_get_time(); 

    while (gpio_get_level((gpio_num_t)pin_endstop) != 0) { vTaskDelay(pdMS_TO_TICKS(10)); }

    ledc_set_duty(LEDC_MODE, channel, 0);
    ledc_update_duty(LEDC_MODE, channel);
    gpio_set_level((gpio_num_t)pin_dir, 0);

    int64_t tiempo_final = esp_timer_get_time();
    int tiempo_recorrido_ms = (int)((tiempo_final - tiempo_arranque) / 1000); 

    tiempo_total_x_ms = tiempo_recorrido_ms; pos_actual_x = 100;
    

    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, " [%c] CALIBRADO: %d milisegundos", eje, tiempo_recorrido_ms);
    ESP_LOGI(TAG, "===================================");
    vTaskDelay(pdMS_TO_TICKS(1500)); 
}
extern "C" void disparar_emergencia(void) {
    estado_actual = ESTADO_EMERGENCIA;
    
    // 1. Clavar todos los PWM a 0 instantáneamente
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, 0); ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, 0); ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Z, 0); ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Z);
    
    // 2. Apagar pines de dirección
    gpio_set_level(PIN_IN4, 0);
    gpio_set_level(PIN_IN2, 0);
    gpio_set_level(PIN_IN6, 0);
    
    // 3. Vaciar la memoria de la cola de comandos
    xQueueReset(xMotorQueue);
    
    ESP_LOGE(TAG, "  PARADA DE EMERGENCIA ACTIVADA ");
}

// =========================================================
// INTERRUPCIÓN DE HARDWARE (ISR) PARA FINES DE CARRERA
// =========================================================
void IRAM_ATTR endstop_isr_handler(void* arg) {
    int pin = (int)arg;
    
    // 1. Freno instantáneo a nivel eléctrico (IRAM Safe)
    // Usamos ledc_stop que corta el hardware del PWM en nanosegundos sin usar FreeRTOS
    if (pin == GPIO_ENDSTOP_Y) {
        gpio_set_level((gpio_num_t)PIN_IN4, 0); 
        ledc_stop(LEDC_MODE, LEDC_CHANNEL_Y, 0); 
    } else if (pin == GPIO_ENDSTOP_X) {
        gpio_set_level((gpio_num_t)PIN_IN2, 0);
        ledc_stop(LEDC_MODE, LEDC_CHANNEL_X, 0);
    }

    // 2. Inteligencia de Estado
    // Si estábamos operando normalmente y se pisó un límite, es un choque.
    // (Si estábamos en ESTADO_MOVIENDO por el Homing, simplemente frenó el motor y la tarea de homing seguirá su curso).
    if (estado_actual == ESTADO_LISTO || estado_actual == ESTADO_ENSAYO) {
        estado_actual = ESTADO_EMERGENCIA;
        // NOTA: Adentro de una ISR no podemos usar ESP_LOG ni xQueueReset porque bloquean el procesador.
        // Solo cambiamos el estado, y la motor_task principal se encargará de imprimir el error y vaciar la cola.
    }
}

extern "C" void motor_task(void *pvParameters) {
    motor_command_t received_cmd;
    while (1) {
        if (xQueueReceive(xMotorQueue, &received_cmd, portMAX_DELAY) == pdPASS) {
            
            if (received_cmd.eje == 'H') {
                estado_actual = ESTADO_MOVIENDO;
                ESP_LOGW(TAG, "--- INICIANDO SECUENCIA DE HOMING MANUAL ---");
                
                homing_eje('Y', PIN_IN4, GPIO_ENDSTOP_Y, LEDC_CHANNEL_Y);
                homing_eje('X', PIN_IN2, GPIO_ENDSTOP_X, LEDC_CHANNEL_X);
                medir_rango_eje('X', PIN_IN2, GPIO_ENDSTOP_X, LEDC_CHANNEL_X);
                
                estado_actual = ESTADO_LISTO; // ¡Ahora sí destrabamos la máquina!
                ESP_LOGI(TAG, "--- GRUA CALIBRADA Y LISTA PARA OPERAR ---");
                continue; 
            }

            // =====================================
            // EJE Z: MANUAL DIRECTO (Joystick)
            // =====================================
            if (received_cmd.eje == 'Z') {
                if (received_cmd.direccion == 1) { // Horario
                    gpio_set_level(PIN_IN6, 1); ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Z, 255 - PWM_Z);
                } else if (received_cmd.direccion == -1) { // Antihorario
                    gpio_set_level(PIN_IN6, 0); ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Z, PWM_Z);
                } else { // Soltó el botón -> Freno
                    gpio_set_level(PIN_IN6, 0); ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Z, 0);
                }
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Z);
                continue; // Vuelve a esperar comandos de inmediato
            }
            //======================================
            //EJE Y (LAZO CERRADO CON TOF)
            //======================================
            if (received_cmd.eje == 'Y') {
                //Traduce el porcentaje a mm reales
                int target_mm = TOF_DISTANCIA_MIN_MM + (received_cmd.posicion_destino * (TOF_DISTANCIA_MAX_MM - TOF_DISTANCIA_MIN_MM)) / 100;
                ESP_LOGI (TAG, "Y -> Destino: %d%% -> %d mm", received_cmd.posicion_destino, target_mm);

                int fallos_consecutivos_i2c = 0; //inicializamos el watchdog
                int impresiones = 0; //contador para no saturar la consola

                while(1) {
                    uint16_t actual_mm = 0;
                    bool lectura_ok = tof_sensor.read(&actual_mm);
                    // --- REVISIÓN DEL WATCHDOG ---
                    if (!lectura_ok) {
                        fallos_consecutivos_i2c++;
                        if (fallos_consecutivos_i2c >= 10) {
                            ESP_LOGE(TAG, "WATCHDOG: Fallo critico del I2C en movimiento.");
                            disparar_emergencia();
                            estado_actual = ESTADO_ERROR_SENS;
                            break;
                        }
                    } else {
                        fallos_consecutivos_i2c = 0; // Reseteo por lectura exitosa

                        int error_mm = target_mm - (int)actual_mm;

                        if (impresiones++ %15 == 0){
                         ESP_LOGW(TAG, "Y -> Pos Actual: %d mm ", actual_mm);
                        }

                    //Si piso el fin de carrera mecanico frenamos por seguridad
                    
                    if (estado_actual == ESTADO_EMERGENCIA) {
                        ESP_LOGE(TAG, "¡EMERGENCIA! Fin de carrera pisado en eje Y. Deteniendo motor.");
                        disparar_emergencia(); 
                        break;
                    }

                    //LLego a destino dentro de la tolerancia, frenamos
                    if (abs(error_mm) <= TOF_TOLERANCIA_MM) {
                        ESP_LOGI(TAG, "Y -> Destino alcanzado. Frenando motor.");
                        break;
                    }

                    //Logica bang-bang (adelante o atras segun el error)
                    if (error_mm > 0) { //subir pluma
                        gpio_set_level(PIN_IN4, 1); 
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, 255 - PWM_Y);
                    } else { //bajar pluma
                        gpio_set_level(PIN_IN4, 0); 
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, PWM_Y );
                    }
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
                    }
                    vTaskDelay(pdMS_TO_TICKS(15)); //Pequeña espera para leer rapido pero sin saturar el bus I2C
                }
            
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, 0); ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
                gpio_set_level(PIN_IN4, 0);
                pos_actual_y = received_cmd.posicion_destino;
                continue; // Vuelve a esperar comandos de inmediato
            }


            // =====================================
            // EJES X PSEUDOLAZO CERRADO (POR TIEMPO)
            // =====================================
            
            if (received_cmd.eje == 'X') {
                    int delta_porcentaje = received_cmd.posicion_destino - pos_actual_x;
                    if (delta_porcentaje == 0) continue; 

                int tiempo_a_mover = (abs(delta_porcentaje) * tiempo_total_x_ms) / 100;
                
                if (delta_porcentaje > 0) {
                    gpio_set_level(PIN_IN2, 1); ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, 255 - PWM_X);
                } else {
                    gpio_set_level(PIN_IN2, 0); ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, PWM_X);
                }
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);

                int tiempo_transcurrido = 0;
                while (tiempo_transcurrido < tiempo_a_mover) {
                    
                    // Si la interrupción de hardware saltó, el estado cambió mágicamente.
                    if (estado_actual == ESTADO_EMERGENCIA) {
                        ESP_LOGE(TAG, "¡EMERGENCIA FÍSICA DETECTADA! Cola vaciada.");
                        xQueueReset(xMotorQueue); // Vaciamos la cola aquí, de forma segura fuera de la ISR
                        break;
                    }

                    vTaskDelay(pdMS_TO_TICKS(10));
                    tiempo_transcurrido += 10;
                }
                
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, 0); ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
                gpio_set_level(PIN_IN2, 0);
                if (tiempo_transcurrido >= tiempo_a_mover){
                    pos_actual_x = received_cmd.posicion_destino;
                    // Regla de 3 simple: (Posición actual * Extensión máxima) / 100
                    float extension_cm = (pos_actual_x * 36.0f) / 100.0f;
                    
                    ESP_LOGI(TAG, "=========================================");
                    ESP_LOGI(TAG, " X -> Destino alcanzado con exito");
                    ESP_LOGI(TAG, " X -> Posicion logica: %d%%", pos_actual_x);
                    ESP_LOGI(TAG, " X -> Extension fisica: %.1f cm", extension_cm);
                    ESP_LOGI(TAG, "=========================================");
                
                }
            }
        }
    }
}
// =========================================================
//  RUTINA DE ENSAYO AUTOMATIZADA
// =========================================================
typedef struct {
    char eje; int pos1; int pos2; int ciclos; int espera_ms;
} rutina_args_t;

rutina_args_t args_rutina; 
TaskHandle_t rutina_task_handle = NULL;

void rutina_automatica_task(void *pvParameters) {
    rutina_args_t *args = (rutina_args_t *)pvParameters;
    int tiempo_total_eje = (args->eje == 'X') ? tiempo_total_x_ms : tiempo_total_y_ms;

    ESP_LOGW("ENSAYO", "=============================================");
    ESP_LOGW("ENSAYO", " INICIANDO ENSAYO DE REPETITIVIDAD - EJE %c", args->eje);
    ESP_LOGW("ENSAYO", "=============================================");

    for (int i = 0; i < args->ciclos; i++) {
        ESP_LOGI("ENSAYO", "--- CICLO %d de %d ---", i + 1, args->ciclos);

        // --- VIAJE A POSICION 1 ---
        ESP_LOGI("ENSAYO", ">>> Yendo a Posicion 1: %d%%", args->pos1);
        motor_command_t cmd1 = { .eje = args->eje, .posicion_destino = args->pos1, .direccion = 0 };
        xQueueSend(xMotorQueue, &cmd1, portMAX_DELAY);
        
        // Esperamos que termine de viajar + el tiempo de pausa solicitado
        vTaskDelay(pdMS_TO_TICKS(tiempo_total_eje + args->espera_ms));
        
        ESP_LOGW("ENSAYO", "🛑 POSICION 1 ALCANZADA. Mida con el calibre ahora.");
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 segundos extra para que el humano mida tranquilo

        // --- VIAJE A POSICION 2 ---
        ESP_LOGI("ENSAYO", ">>> Yendo a Posicion 2: %d%%", args->pos2);
        motor_command_t cmd2 = { .eje = args->eje, .posicion_destino = args->pos2, .direccion = 0 };
        xQueueSend(xMotorQueue, &cmd2, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(tiempo_total_eje + args->espera_ms));
        
        ESP_LOGW("ENSAYO", "🛑 POSICION 2 ALCANZADA. Mida con el calibre ahora.");
        vTaskDelay(pdMS_TO_TICKS(3000)); 
    }
    
    ESP_LOGW("ENSAYO", "=============================================");
    ESP_LOGW("ENSAYO", " ENSAYO FINALIZADO. ESPERANDO COMANDOS.");
    ESP_LOGW("ENSAYO", "=============================================");
    
    rutina_task_handle = NULL;
    vTaskDelete(NULL); // La tarea se suicida al terminar para liberar memoria
}

void iniciar_rutina_automatica(char eje, int pos1, int pos2, int ciclos, int espera_ms) {
    if (rutina_task_handle != NULL) {
        ESP_LOGE("ENSAYO", "Ya hay una rutina en ejecucion. Espere a que termine.");
        return;
    }
    // Si el eje es Z, cancelamos porque no tiene posiciones porcentuales
    if (eje == 'Z') {
        ESP_LOGE("ENSAYO", "El Eje Z no soporta rutinas automaticas (Lazo Abierto Puro).");
        return;
    }
    args_rutina.eje = eje; args_rutina.pos1 = pos1; args_rutina.pos2 = pos2;
    args_rutina.ciclos = ciclos; args_rutina.espera_ms = espera_ms;
    
    // Creamos la tarea en el Core 1 para no bloquear el WiFi
    xTaskCreatePinnedToCore(rutina_automatica_task, "rutina_task", 4096, (void*)&args_rutina, 5, &rutina_task_handle, 1);
}
// =========================================================
void init_motor_logic(void) {
    
    gpio_reset_pin(PIN_IN4); gpio_set_direction(PIN_IN4, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN4, 0);
    gpio_reset_pin(PIN_IN2); gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN2, 0);
    gpio_reset_pin(PIN_IN6); gpio_set_direction(PIN_IN6, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN6, 0);
    
    gpio_reset_pin(PIN_IN3); gpio_set_direction(PIN_IN3, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN3, 0); // PWM Y
    gpio_reset_pin(PIN_IN1); gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN1, 0); // PWM X
    gpio_reset_pin(PIN_IN5); gpio_set_direction(PIN_IN5, GPIO_MODE_OUTPUT); gpio_set_level(PIN_IN5, 0); // PWM Z
    
    i2c_tof_init();//encender laser
    
    configure_pwm();

// ---  CONFIGURACIÓN DE INTERRUPCIONES ---
    
    // 1. Configuramos los pines de seguridad como entradas con resistencia Pull-Up
    gpio_reset_pin((gpio_num_t)GPIO_ENDSTOP_Y); 
    gpio_set_direction((gpio_num_t)GPIO_ENDSTOP_Y, GPIO_MODE_INPUT); 
    gpio_set_pull_mode((gpio_num_t)GPIO_ENDSTOP_Y, GPIO_PULLUP_ONLY);
    
    gpio_reset_pin((gpio_num_t)GPIO_ENDSTOP_X); 
    gpio_set_direction((gpio_num_t)GPIO_ENDSTOP_X, GPIO_MODE_INPUT); 
    gpio_set_pull_mode((gpio_num_t)GPIO_ENDSTOP_X, GPIO_PULLUP_ONLY);

    // 2. Configurar para que la ISR salte en el flanco de bajada (cuando el botón conecta a GND)
    gpio_set_intr_type((gpio_num_t)GPIO_ENDSTOP_Y, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)GPIO_ENDSTOP_X, GPIO_INTR_NEGEDGE);
    
    // 3. Instalar el servicio global de interrupciones del ESP32 (el 0 es una bandera por defecto)
    gpio_install_isr_service(0);
    
    // 4. Enganchar nuestros dos pines a la función de emergencia que creamos
    gpio_isr_handler_add((gpio_num_t)GPIO_ENDSTOP_Y, endstop_isr_handler, (void*)GPIO_ENDSTOP_Y);
    gpio_isr_handler_add((gpio_num_t)GPIO_ENDSTOP_X, endstop_isr_handler, (void*)GPIO_ENDSTOP_X);

    // 5. Creación de colas y tareas 
    xMotorQueue = xQueueCreate(10, sizeof(motor_command_t));
    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
}
