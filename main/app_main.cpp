#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/ledc.h" 
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "SERVO_SLAVE";

// =========================================================
//                   CONFIGURACIÓN SERVOS
// =========================================================
#define LEDC_TIMER            LEDC_TIMER_0
#define LEDC_MODE             LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES         LEDC_TIMER_10_BIT 
#define LEDC_FREQUENCY        50                

#define DUTY_MIN    26    
#define DUTY_MAX    128   

#define SERVO_PIN_1           14       // Servo 1: GPIO 14 (Metal/Papel)
#define LEDC_CHANNEL_1        LEDC_CHANNEL_0 

#define SERVO_PIN_2           13       // Servo 2: GPIO 13 (Plástico/Cartón)
#define LEDC_CHANNEL_2        LEDC_CHANNEL_1 

// =========================================================
//                   CONFIGURACIÓN ESP-NOW
// =========================================================
// Estructura de datos (DEBE SER IDÉNTICA a la del Maestro)
typedef struct {
    int class_id;
} class_data_t;

// Cola para manejar los mensajes ESP-NOW de forma segura
static QueueHandle_t class_queue;


// =========================================================
//                   FUNCIONES DE SERVO
// =========================================================

int angle_to_duty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return (angle * (DUTY_MAX - DUTY_MIN) / 180) + DUTY_MIN;
}

void set_servo_1_angle(int angle) {
    int duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}

void set_servo_2_angle(int angle) {
    int duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}

void servos_to_initial_position() {
    set_servo_1_angle(0); 
    set_servo_2_angle(0); 
    ESP_LOGI(TAG, "Servos en posición inicial (0 grados)");
}

void ledc_init_servos() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_1 = {
        .gpio_num       = SERVO_PIN_1,       
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,    
        .intr_type      = LEDC_INTR_DISABLE, 
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = {0}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));

    ledc_channel_config_t ledc_channel_2 = {
        .gpio_num       = SERVO_PIN_2,       
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_2,    
        .intr_type      = LEDC_INTR_DISABLE, 
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags          = {0}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_2));

    ESP_LOGI(TAG, "Servos inicializados.");
}

// Lógica de movimiento basada en la clase (MODIFICADA PARA MOVIMIENTO COMPLETO 0-180-0)
static void move_servos_based_on_class(int predicted_class) {
    // 0: carton, 1: metal, 2: papel, 3: plastico
    
    // Tiempo de espera para observar el movimiento de ida (1.0 segundo)
    const TickType_t MOVE_DELAY = pdMS_TO_TICKS(1000); 

    // Aseguramos que ambos servos estén en reposo antes de mover
    servos_to_initial_position(); 

    if (predicted_class == 1) { // METAL (Servo 1)
        ESP_LOGI(TAG, "🔩 Servo 1: METAL. Movimiento 0° -> 180° -> 0°");

// IDA: Mover de 0 a 180
        set_servo_1_angle(180);
 
// VUELTA: Mover de 180 a 0
        set_servo_1_angle(0);
        vTaskDelay(MOVE_DELAY); // Espera en 0°
    } else if (predicted_class == 2) { // PAPEL (Servo 1)
        ESP_LOGI(TAG, "📃 Servo 1: PAPEL. Movimiento 0° -> 90° -> 0°"); // Movimiento más pequeño para diferenciar

// IDA: Mover de 0 a 90
        set_servo_1_angle(90);
        vTaskDelay(MOVE_DELAY); 

// VUELTA: Mover de 90 a 0
        set_servo_1_angle(0);
        vTaskDelay(MOVE_DELAY);
    } 

// Lógica para SERVO 2 (Plástico/Cartón)
    else if (predicted_class == 3) { // PLÁSTICO (Servo 2)
        ESP_LOGI(TAG, "🥤 Servo 2: PLÁSTICO. Movimiento 0° -> 180° -> 0°");
 
// IDA: Mover de 0 a 180
        set_servo_2_angle(180);
        vTaskDelay(MOVE_DELAY); // Espera 1.0s en 180° 
// VUELTA: Mover de 180 a 0
        set_servo_2_angle(0);
        vTaskDelay(MOVE_DELAY); // Espera en 0°
    } else if (predicted_class == 0) { // CARTÓN (Servo 2)
        ESP_LOGI(TAG, "📦 Servo 2: CARTÓN. Movimiento 0° -> 90° -> 0°");

// IDA: Mover de 0 a 90
        set_servo_2_angle(90);
        vTaskDelay(MOVE_DELAY); 

// VUELTA: Mover de 90 a 0
        set_servo_2_angle(0);
        vTaskDelay(MOVE_DELAY);
    } else {
        ESP_LOGI(TAG, "Clase desconocida. Sin movimiento.");
// No se necesita llamar a servos_to_initial_position, ya se hizo al inicio.
    }
    // NOTA: EL BUCLE PRINCIPAL YA NO NECESITA LLAMAR A vTaskDelay O servos_to_initial_position.
}

// =========================================================
//                   FUNCIONES ESP-NOW (Recepción)
// =========================================================

// =========================================================
//                   FUNCIONES ESP-NOW (Recepción)
// =========================================================

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(class_data_t)) {
        class_data_t data;
        memcpy(&data, incomingData, sizeof(data));

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Intentamos enviar a la cola usando la versión segura para ISR
        if (xQueueSendFromISR(class_queue, &data.class_id, &xHigherPriorityTaskWoken) != pdTRUE) { 
            ESP_LOGW(TAG, "Cola llena o error de envío.");
        }

        // Si se despertó una tarea de mayor prioridad (como app_main), forzamos el cambio de contexto
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}
void init_espnow_slave() {
    // Crear la cola
    class_queue = xQueueCreate(5, sizeof(int)); 

    // 1. Inicializar NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // 2. Configurar Wi-Fi en modo Estación (necesario para ESP-NOW)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 3. Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando ESP-NOW");
        return;
    }

    // 4. Registrar Callback de recepción
    ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
    
    ESP_LOGI(TAG, "ESP-NOW Esclavo configurado. Listo para recibir.");
}


// =========================================================
//                          MAIN
// =========================================================

extern "C" void app_main(void) {
    // Inicialización del hardware de control
    ledc_init_servos(); 
    servos_to_initial_position();

    // Inicialización de la comunicación
    init_espnow_slave(); 
    
    int received_class;
    
    ESP_LOGI(TAG, "✅ Sistema de Control listo. Esperando clases (ESP-NOW)...");

    while (1) {
        // Esperar dato en la cola (espera infinita si no hay datos)
        if (xQueueReceive(class_queue, &received_class, portMAX_DELAY) == pdTRUE) {
            
            ESP_LOGI(TAG, "Clase recibida via ESP-NOW: %d", received_class);
            
            // 1. Mover los servos a la posición de descarte
            move_servos_based_on_class(received_class);
            
            // 2. Esperar un tiempo prudente para el descarte
            //vTaskDelay(pdMS_TO_TICKS(1500)); 
            
            // 3. Volver a la posición inicial (reposo)
            //servos_to_initial_position(); 
        }
    }
}