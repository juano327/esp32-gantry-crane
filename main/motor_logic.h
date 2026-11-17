#ifndef MOTOR_LOGIC_H
#define MOTOR_LOGIC_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/ledc.h"

// Fines de Carrera
#define GPIO_ENDSTOP_Y 25
#define GPIO_ENDSTOP_X 14

// Pines de Dirección (L298N)
#define PIN_IN4 GPIO_NUM_33 // DIR Y
#define PIN_IN2 GPIO_NUM_19 // DIR X
#define PIN_IN6 GPIO_NUM_27 // DIR Z

// Canales PWM
#define LEDC_CHANNEL_Y LEDC_CHANNEL_0
#define LEDC_CHANNEL_X LEDC_CHANNEL_1
#define LEDC_CHANNEL_Z LEDC_CHANNEL_2 

// Velocidades fijas por eje 
#define PWM_Y  255/2
#define PWM_X  255/2
#define PWM_Z  255/2

#define PWM_HOMING 255/2 
#define PWM_HOMING_INV   (255 - PWM_HOMING) 

// ==========================================
// CONFIGURACIÓN I2C Y LÁSER TOF
// ==========================================
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_SDA_IO  21
#define I2C_MASTER_NUM     I2C_NUM_0

// Calibración física de la grúa (Distancia desde el láser hasta el piso/carga)
#define TOF_DISTANCIA_MIN_MM  82    // 0% (Gancho arriba del todo)
#define TOF_DISTANCIA_MAX_MM  260  // 100% (Gancho tocando el piso)
#define TOF_TOLERANCIA_MM     15    // Margen de error (frena si está a +/- 5mm)

// ==========================================
// MÁQUINA DE ESTADOS FINITOS (FSM)
// ==========================================
typedef enum {
    ESTADO_INIT,           // Arrancando hardware
    ESTADO_ESPERA_HOMING,  // Bloqueado hasta que el usuario referencie la máquina
    ESTADO_LISTO,          // Homing OK. Esperando comandos manuales
    ESTADO_MOVIENDO,       // Motor en movimiento
    ESTADO_ENSAYO,         // Rutina automática en ejecución
    ESTADO_EMERGENCIA,     // Parada de emergencia
    ESTADO_ERROR_SENS      // Fallo crítico en I2C o Láser
} estado_grua_t;

extern estado_grua_t estado_actual; // Variable global visible para todo el sistema

//Puente de C con C++
//Para poder usar esta lógica desde el main.c sin problemas de linkage
#ifdef __cplusplus
extern "C" {   
#endif

typedef struct {
    char eje;
    int posicion_destino; // Para Sliders (X, Y)
    int direccion;        // Para Botones Z (1 = Horario, -1 = Anti, 0 = Freno)
} motor_command_t;

extern QueueHandle_t xMotorQueue;

void motor_task(void *pvParameters);
void init_motor_logic(void);
void homing_eje(char eje, int pin_dir, int pin_endstop, ledc_channel_t channel);
void medir_rango_eje(char eje, int pin_dir, int pin_endstop, ledc_channel_t channel);
void iniciar_rutina_automatica(char eje, int pos1, int pos2, int ciclos, int espera_ms);
void disparar_emergencia(void);

#ifdef __cplusplus
}
#endif

#endif
