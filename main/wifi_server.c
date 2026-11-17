#include <stdio.h>
#include <string.h>
#include <ctype.h>             
#include "esp_wifi.h"          
#include "esp_event.h"         
#include "esp_log.h"           
#include "esp_http_server.h"   
#include "cJSON.h"             
#include "nvs_flash.h"
#include "wifi_server.h"       
#include "motor_logic.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WIFI_SERVER";

const char *html_page = 
"<!DOCTYPE html><html lang=\"es\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no\">"
"<title>Panel Grua</title>"
"<style>"
"body{background:#121212;color:#00ffcc;font-family:monospace;padding:15px;text-align:center; user-select: none; -webkit-user-select: none;}"
"h2{border-bottom:2px solid #00ffcc;padding-bottom:10px;margin-bottom:20px;letter-spacing:2px;}"
".c{background:#1e1e1e;border:1px solid #333;border-radius:12px;padding:20px 10px;margin-bottom:20px;box-shadow:0px 4px 15px rgba(0,255,204,0.1);}"
"label{font-size:1.2rem;font-weight:bold;display:block;margin-bottom:15px; color:#ffaa00;}"
"input[type=range]{-webkit-appearance:none;width:100%;background:transparent;}"
"input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;height:35px;width:35px;border-radius:50%;background:#00ffcc;margin-top:-12px;box-shadow:0px 0px 10px #00ffcc;}"
"input[type=range]::-webkit-slider-runnable-track{width:100%;height:10px;background:#444;border-radius:5px;}"
".btn{background:#00ffcc; color:#121212; border:none; padding:15px; font-size:1.1rem; font-weight:bold; border-radius:8px; margin:5px; width:45%; cursor:pointer;}"
".btn:active{background:#ffaa00; transform:scale(0.95);}"
".row{display:flex; justify-content:space-around; width:100%;}"
".input-num{background:#333; color:#00ffcc; border:1px solid #00ffcc; border-radius:5px; padding:5px; width:60px; text-align:center; font-weight:bold;}"
"</style></head><body><h2>⚡ CONTROL MAESTRO ⚡</h2>"
"<button class=\"btn\" style=\"width:100%; background:#ff0000; color:white; font-size:1.2rem; margin-bottom:20px; box-shadow:0px 0px 15px #ff0000;\" onclick=\"e()\">🛑 PARADA DE EMERGENCIA 🛑</button>"
"<div class=\"c\" style=\"border-color:#ffaa00;\"><label style=\"color:#ffaa00;\">⚠️ Requiere Calibracion</label>"
"<button class=\"btn\" style=\"width:90%; background:#ffaa00;\" onclick=\"h()\">REALIZAR HOMING</button></div>"
"<div class=\"c\"><label>Giro Manual (Z)</label><div class=\"row\">"
"<button class=\"btn\" onmousedown=\"d('Z',1)\" onmouseup=\"d('Z',0)\" onmouseleave=\"d('Z',0)\" ontouchstart=\"d('Z',1)\" ontouchend=\"d('Z',0)\" ontouchcancel=\"d('Z',0)\">Horario</button>"
"<button class=\"btn\" onmousedown=\"d('Z',-1)\" onmouseup=\"d('Z',0)\" onmouseleave=\"d('Z',0)\" ontouchstart=\"d('Z',-1)\" ontouchend=\"d('Z',0)\" ontouchcancel=\"d('Z',0)\">Antihorario</button></div></div>"
"<div class=\"c\"><label>Elevacion (Y): <span id=\"valY\">0</span>%</label><input type=\"range\" id=\"sliderY\" min=\"0\" max=\"100\" value=\"0\" onchange=\"p('Y',this.value)\" oninput=\"u(this)\"></div>"
"<div class=\"c\"><label>Extension (X): <span id=\"valX\">0</span>%</label><input type=\"range\" id=\"sliderX\" min=\"0\" max=\"100\" value=\"0\" onchange=\"p('X',this.value)\" oninput=\"u(this)\"></div>"
"<div class=\"c\" style=\"border: 1px solid #ff0055; box-shadow: 0px 4px 15px rgba(255,0,85,0.2);\"><label style=\"color:#ff0055;\">🧪 Test de Repetitividad</label>"
"<div style=\"text-align:left; font-size:1rem; color:#fff; display:flex; flex-direction:column; align-items:center; gap:10px;\">"
"<div>Eje: <select id=\"rE\" class=\"input-num\" style=\"width:50px;\"><option value=\"X\">X</option><option value=\"Y\">Y</option></select></div>"
"<div>Pos Inicial (%): <input type=\"number\" id=\"rP1\" class=\"input-num\" value=\"10\"></div>"
"<div>Pos Final (%): <input type=\"number\" id=\"rP2\" class=\"input-num\" value=\"90\"></div>"
"<div>Repeticiones: <input type=\"number\" id=\"rC\" class=\"input-num\" value=\"4\"></div>"
"</div><button class=\"btn\" onclick=\"rutina()\" style=\"width:90%; background:#ff0055; color:white; margin-top:20px;\">▶ INICIAR ENSAYO</button></div>"
"<script>"
"function e(){fetch('/api/v1/control',{method:'POST',body:JSON.stringify({emergencia:true})}).then(r=>{alert('EMERGENCIA ACTIVADA. SE REQUIERE HOMING PARA REANUDAR.');});}"
"function h(){fetch('/api/v1/control',{method:'POST',body:JSON.stringify({homing:true})}).then(r=>{if(r.ok)alert('Calibrando... Espere a que termine el movimiento.');});}"
"function u(el){document.getElementById(el.id.replace('slider','val')).innerText=el.value;}"
"function p(eje,val){fetch('/api/v1/control',{method:'POST',body:JSON.stringify({eje:eje,posicion:parseInt(val)})}).then(r=>{if(r.status===403)alert('Grua bloqueada. Haga Homing primero.');});}"
"function d(eje,dir){fetch('/api/v1/control',{method:'POST',body:JSON.stringify({eje:eje,direccion:dir})}).then(r=>{if(r.status===403)alert('Grua bloqueada. Haga Homing primero.');});}"
"function rutina(){"
"let ax=document.getElementById('rE').value;"
"let p1=parseInt(document.getElementById('rP1').value);"
"let p2=parseInt(document.getElementById('rP2').value);"
"let c=parseInt(document.getElementById('rC').value);"
"fetch('/api/v1/control',{method:'POST',body:JSON.stringify({eje:ax,rutina:true,pos1:p1,pos2:p2,ciclos:c})}).then(r=>{if(r.status===403)alert('Grua bloqueada. Haga Homing primero.');});"
"}"
"</script></body></html>";

esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
}

esp_err_t control_post_handler(httpd_req_t *req) {
    char content[150];
    int ret = httpd_req_recv(req, content, sizeof(content)-1);
    if (ret <= 0) return ESP_FAIL;
    content[ret] = '\0';

    cJSON *root = cJSON_Parse(content);
    if (root) {
        
        // --- 0. EVALUAR PARADA DE EMERGENCIA INMEDIATA ---
        if (cJSON_GetObjectItem(root, "emergencia")) {
            disparar_emergencia();
            httpd_resp_send(req, "{\"status\":\"emergencia\"}", HTTPD_RESP_USE_STRLEN);
            cJSON_Delete(root);
            return ESP_OK;
        }

        // --- 1. EVALUAR COMANDO ESPECIAL DE HOMING ---
        if (cJSON_GetObjectItem(root, "homing")) {
            // Permitimos el Homing incluso si estamos en Emergencia (para destrabar la máquina)
            motor_command_t cmd = { .eje = 'H', .posicion_destino = 0, .direccion = 0 };
            xQueueSend(xMotorQueue, &cmd, 0);
            httpd_resp_send(req, "{\"status\":\"homing_iniciado\"}", HTTPD_RESP_USE_STRLEN);
            cJSON_Delete(root);
            return ESP_OK;
        }

        // --- 2. LA BARRERA DE SEGURIDAD DE LA FSM ---
        if (estado_actual == ESTADO_EMERGENCIA) {
            ESP_LOGE(TAG, "Comando bloqueado: Sistema en EMERGENCIA.");
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Estado de Emergencia. Haga Homing para reanudar.");
            cJSON_Delete(root);
            return ESP_OK; 
        }
        else if (estado_actual != ESTADO_LISTO) {
            ESP_LOGE(TAG, "Comando bloqueado: La maquina no esta referenciada (Falta Homing).");
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Falta hacer Homing.");
            cJSON_Delete(root);
            return ESP_OK; 
        }

        // --- 3. PROCESAMIENTO NORMAL DE COMANDOS (Si pasó la barrera) ---
        cJSON *eje_json = cJSON_GetObjectItem(root, "eje");
        if (eje_json) {
            char eje_raw = toupper((unsigned char)eje_json->valuestring[0]);
            
            if (cJSON_GetObjectItem(root, "rutina")) {
                int p1 = 0, p2 = 0, c = 0;
                if (cJSON_GetObjectItem(root, "pos1")) p1 = cJSON_GetObjectItem(root, "pos1")->valueint;
                if (cJSON_GetObjectItem(root, "pos2")) p2 = cJSON_GetObjectItem(root, "pos2")->valueint;
                if (cJSON_GetObjectItem(root, "ciclos")) c = cJSON_GetObjectItem(root, "ciclos")->valueint;
                
                ESP_LOGI(TAG, "Lanzando Rutina Eje %c. P1:%d P2:%d Ciclos:%d", eje_raw, p1, p2, c);
                iniciar_rutina_automatica(eje_raw, p1, p2, c, 2000); 
                httpd_resp_send(req, "{\"status\":\"rutina_iniciada\"}", HTTPD_RESP_USE_STRLEN);
            } 
            else {
                motor_command_t cmd = { .eje = eje_raw, .posicion_destino = 0, .direccion = 0 };
                
                if (cJSON_GetObjectItem(root, "posicion")) {
                    cmd.posicion_destino = cJSON_GetObjectItem(root, "posicion")->valueint;
                    if (cmd.posicion_destino > 100) cmd.posicion_destino = 100;
                    if (cmd.posicion_destino < 0) cmd.posicion_destino = 0;
                }
                if (cJSON_GetObjectItem(root, "direccion")) {
                    cmd.direccion = cJSON_GetObjectItem(root, "direccion")->valueint;
                }

                if (xQueueSend(xMotorQueue, &cmd, 0) == pdPASS) {
                    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
                } else {
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cola llena.");
                }
            }
        }
        cJSON_Delete(root);
    }
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; 
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) return server;
    return NULL;
}

void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Grua_Mecatronica",
            .ssid_len = strlen("Grua_Mecatronica"),
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
}

void init_wifi(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_softap();
    vTaskDelay(pdMS_TO_TICKS(500)); 
    httpd_handle_t server = start_webserver();
    if (server != NULL) {
        httpd_uri_t uri_get = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_get);
        httpd_uri_t uri_post = { .uri = "/api/v1/control", .method = HTTP_POST, .handler = control_post_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_post);
    }
}