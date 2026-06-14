#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_netif.h"

// ======================================================================
// --- CONFIGURAÇÕES GERAIS ---
// ======================================================================

// Pinos I2C
#define I2C_SDA_PIN                 GPIO_NUM_6
#define I2C_SCL_PIN                 GPIO_NUM_7
#define I2C_PORT                    I2C_NUM_0

// Endereço do multiplexador I2C e canais dos sensores de luz
#define TCA9548A_ADDR               0x70
#define CH_MESA3_BH1750             3
#define CH_MESA4_BH1750             1

// Sensor de luz BH1750: endereço, comando e fatores de correção
#define BH1750_SENSOR_ADDR          0x23
#define BH1750_CMD_START            0x10
#define BH1750_CAL_FACTOR           1.12f   // Correção para o luxímetro de referência
#define BH1750_POS_FACTOR_M3        1.45f   // Correção de posição do sensor na Mesa 3
#define BH1750_POS_FACTOR_M4        1.35f   // Correção de posição do sensor na Mesa 4

// Pino do sensor PIR (movimento)
#define SENSOR_PIR_PIN              GPIO_NUM_8

// Pinos HC-SR04 Mesa 4 — Sensor A
#define HCSR04_M4A_TRIG             GPIO_NUM_4
#define HCSR04_M4A_ECHO             GPIO_NUM_5

// Pinos HC-SR04 Mesa 4 — Sensor B
// Atenção: GPIO0 é pino de strapping; trocar se causar problemas no boot
#define HCSR04_M4B_TRIG             GPIO_NUM_0
#define HCSR04_M4B_ECHO             GPIO_NUM_1

// Pinos HC-SR04 Mesa 3 — Sensor A
#define HCSR04_M3A_TRIG             GPIO_NUM_10
#define HCSR04_M3A_ECHO             GPIO_NUM_11

// Pinos HC-SR04 Mesa 3 — Sensor B
#define HCSR04_M3B_TRIG             GPIO_NUM_12
#define HCSR04_M3B_ECHO             GPIO_NUM_13

// Velocidade do som em cm/µs (~25°C)
#define SOUND_SPEED_CM_US           0.0343f

// Distâncias mínima e máxima válidas do HC-SR04
#define HCSR04_DIST_MIN_CM          2.0f
#define HCSR04_DIST_MAX_CM          160.0f

// Timeout aguardando o eco retornar
#define HCSR04_ECHO_TIMEOUT_US      25000

// Intervalo mínimo entre medições
#define HCSR04_MEASURE_INTERVAL_MS  60

// Quantidade de amostras por medição (para calcular a mediana)
#define HCSR04_NUM_AMOSTRAS         10

// Tamanho da janela da média móvel (número de medianas acumuladas)
#define HCSR04_MM_N                 10

// Confirmações consecutivas necessárias para mudar o estado da mesa
#define HCSR04_CONFIRM_OCUPADO       1
#define HCSR04_CONFIRM_LIVRE         1

// Intervalo entre disparos de sensores diferentes para evitar interferência
#define HCSR04_INTER_SENSOR_GUARD_US 30000

// Desvio padrão máximo aceito; acima disso a leitura é descartada
#define HCSR04_MAX_STD_DEV_CM        8.0f

// Faixa de distância que indica presença (em cm)
#define HCSR04_LIMIAR_MIN_CM        100.0f
#define HCSR04_LIMIAR_MAX_CM        159.0f

// Valores publicados no MQTT
#define HCSR04_OCUPADO               1
#define HCSR04_LIVRE                 0

// Credenciais Wi-Fi e endereço do broker MQTT
#define WIFI_SSID       "PROJETO"
#define WIFI_PASS       "12345678"
#define MQTT_URI        "mqtt://192.168.0.101:1884"

// ======================================================================
// --- VARIÁVEIS GLOBAIS ---
// ======================================================================

static const char *TAG = "SENSORES_MQTT";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool wifi_connected = false;
static bool mqtt_recently_connected = false;

// ======================================================================
// --- MQTT ---
// ======================================================================

// Aguarda 3 s após reconexão antes de retomar publicações
static void mqtt_reconnect_delay_task(void *pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    mqtt_recently_connected = false;
    ESP_LOGI(TAG, "MQTT pronto para publicação após reconexão.");
    vTaskDelete(NULL);
}

// Publica uma mensagem em um tópico MQTT
static void mqtt_publish(const char *topic, const char *msg)
{
    if (!mqtt_connected || !mqtt_client || mqtt_recently_connected) return;
    esp_mqtt_client_publish(mqtt_client, topic, msg, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT → [%s] = %s", topic, msg);
}

// Publica um valor float (2 casas decimais)
static void mqtt_publish_float(const char *topic, float value)
{
    char payload[32];
    snprintf(payload, sizeof(payload), "%.2f", value);
    mqtt_publish(topic, payload);
}

// Publica um valor inteiro
static void mqtt_publish_int(const char *topic, int value)
{
    char payload[8];
    snprintf(payload, sizeof(payload), "%d", value);
    mqtt_publish(topic, payload);
}

// Trata eventos do broker MQTT (conexão/desconexão)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            mqtt_recently_connected = true;
            ESP_LOGI(TAG, "✅ Conectado ao broker MQTT");
            esp_mqtt_client_publish(mqtt_client, "ambiente/status", "online", 0, 1, 1);
            xTaskCreate(mqtt_reconnect_delay_task, "mqtt_reconnect_delay", 2048, NULL, 5, NULL);
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            mqtt_recently_connected = false;
            ESP_LOGW(TAG, "⚠️  Desconectado do broker MQTT, tentando reconectar...");
            esp_mqtt_client_publish(mqtt_client, "ambiente/status", "offline", 0, 1, 1);
            esp_mqtt_client_reconnect(mqtt_client);
            break;

        default:
            break;
    }
}

// Inicializa e inicia o cliente MQTT
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .session.keepalive = 30,
        .session.disable_clean_session = false
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "Iniciando cliente MQTT...");
}

// ======================================================================
// --- WIFI ---
// ======================================================================

// Trata eventos Wi-Fi: conexão, desconexão e obtenção de IP
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGW(TAG, "Wi-Fi desconectado, tentando reconectar...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected = true;
        ESP_LOGI(TAG, "✅ Conectado à rede Wi-Fi");
    }
}

// Inicializa o Wi-Fi em modo estação (cliente)
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ======================================================================
// --- MULTIPLEXADOR I2C (TCA9548A) ---
// ======================================================================

// Seleciona qual canal do multiplexador estará ativo
static esp_err_t tca9548a_select_channel(uint8_t channel)
{
    uint8_t cmd = 1 << channel;
    esp_err_t ret = i2c_master_write_to_device(I2C_PORT, TCA9548A_ADDR, &cmd, 1, pdMS_TO_TICKS(200));
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

// ======================================================================
// --- HC-SR04 ---
// ======================================================================

// Configura os pinos TRIG e ECHO de um sensor HC-SR04
static void hcsr04_init(gpio_num_t trig_pin, gpio_num_t echo_pin)
{
    // TRIG inicia em HIGH para evitar problema de strapping no GPIO0
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << trig_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_cfg));
    gpio_set_level(trig_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(trig_pin, 0);

    // ECHO como entrada (sinal vem do divisor de tensão 1kΩ/2kΩ)
    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << echo_pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_cfg));

    ESP_LOGI(TAG, "HC-SR04 inicializado — TRIG: GPIO%d | ECHO: GPIO%d",
             trig_pin, echo_pin);
}

// Realiza uma única medição de distância com o HC-SR04
static bool hcsr04_measure_single(gpio_num_t trig_pin, gpio_num_t echo_pin, float *dist_cm)
{
    // Aguarda ECHO estar em LOW antes de disparar (evita eco residual)
    int64_t t_guard = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - t_guard) > HCSR04_ECHO_TIMEOUT_US) return false;
    }

    // Pulso de disparo
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // Aguarda início do eco
    int64_t t_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if ((esp_timer_get_time() - t_start) > HCSR04_ECHO_TIMEOUT_US) return false;
    }

    // Mede duração do eco e calcula a distância
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if ((esp_timer_get_time() - echo_start) > HCSR04_ECHO_TIMEOUT_US) return false;
    }
    int64_t echo_end = esp_timer_get_time();

    float dist = ((float)(echo_end - echo_start) * SOUND_SPEED_CM_US) / 2.0f;
    if (dist < HCSR04_DIST_MIN_CM || dist > HCSR04_DIST_MAX_CM) return false;

    *dist_cm = dist;
    return true;
}

// Coleta N amostras, calcula a mediana e descarta se o desvio padrão for alto
static bool hcsr04_measure_mediana(gpio_num_t trig_pin, gpio_num_t echo_pin, float *dist_cm)
{
    float amostras[HCSR04_NUM_AMOSTRAS];
    int validas = 0;

    for (int i = 0; i < HCSR04_NUM_AMOSTRAS; i++) {
        float d = 0.0f;
        if (hcsr04_measure_single(trig_pin, echo_pin, &d)) {
            amostras[validas++] = d;
        }
        vTaskDelay(pdMS_TO_TICKS(HCSR04_MEASURE_INTERVAL_MS));
    }

    if (validas == 0) return false;

    // Calcula desvio padrão das amostras
    float soma = 0.0f;
    for (int i = 0; i < validas; i++) soma += amostras[i];
    float media = soma / (float)validas;

    float soma_sq = 0.0f;
    for (int i = 0; i < validas; i++) {
        float diff = amostras[i] - media;
        soma_sq += diff * diff;
    }
    float std_dev = (validas > 1) ? sqrtf(soma_sq / (float)(validas - 1)) : 0.0f;

    // Descarta leitura se as amostras estiverem muito dispersas (eco indireto)
    if (std_dev > HCSR04_MAX_STD_DEV_CM) {
        ESP_LOGW("HCSR04", "TRIG:%d — desvio padrão %.1f cm > %.1f cm, descartado (multipath)",
                 trig_pin, std_dev, HCSR04_MAX_STD_DEV_CM);
        return false;
    }

    // Ordena e retorna a mediana
    for (int i = 1; i < validas; i++) {
        float key = amostras[i];
        int j = i - 1;
        while (j >= 0 && amostras[j] > key) {
            amostras[j + 1] = amostras[j];
            j--;
        }
        amostras[j + 1] = key;
    }

    *dist_cm = amostras[validas / 2];
    return true;
}

// ======================================================================
// --- MÉDIA MÓVEL POR SENSOR ---
// ======================================================================

// Buffer circular para suavizar as leituras ao longo do tempo
typedef struct {
    float   buf[HCSR04_MM_N];  // Histórico de medianas
    int     idx;                // Posição atual de escrita
    float   soma;               // Soma dos valores no buffer
} hcsr04_mm_t;

// Inicializa o buffer com distância máxima (→ mesa livre no início)
static void mm_init(hcsr04_mm_t *mm)
{
    mm->idx  = 0;
    mm->soma = HCSR04_DIST_MAX_CM * HCSR04_MM_N;
    for (int i = 0; i < HCSR04_MM_N; i++)
        mm->buf[i] = HCSR04_DIST_MAX_CM;
}

// Insere nova mediana e retorna a média móvel atualizada
static float mm_atualizar(hcsr04_mm_t *mm, float nova_dist)
{
    mm->soma         -= mm->buf[mm->idx];
    mm->buf[mm->idx]  = nova_dist;
    mm->soma         += nova_dist;
    mm->idx           = (mm->idx + 1) % HCSR04_MM_N;
    return mm->soma / (float)HCSR04_MM_N;
}

// Aplica mediana + média móvel e decide se há presença na zona do sensor
static int hcsr04_ocupancia(gpio_num_t trig_pin, gpio_num_t echo_pin,
                             hcsr04_mm_t *mm, float *dist_cm)
{
    float mediana = HCSR04_DIST_MAX_CM;

    if (!hcsr04_measure_mediana(trig_pin, echo_pin, &mediana)) {
        // Falha de leitura: usa valor neutro (mesa livre)
        mediana = HCSR04_DIST_MAX_CM;
    }

    // Atualiza média móvel
    *dist_cm = mm_atualizar(mm, mediana);

    if (*dist_cm >= HCSR04_DIST_MAX_CM)
        return HCSR04_LIVRE;

    return (*dist_cm >= HCSR04_LIMIAR_MIN_CM && *dist_cm < HCSR04_LIMIAR_MAX_CM)
           ? HCSR04_OCUPADO : HCSR04_LIVRE;
}

// Combina dois sensores: mesa ocupada se qualquer um detectar presença (lógica OR)
static int fusao_ocupancia(int ocup_a, int ocup_b)
{
    return (ocup_a == HCSR04_OCUPADO || ocup_b == HCSR04_OCUPADO) ? HCSR04_OCUPADO : HCSR04_LIVRE;
}

// ======================================================================
// --- DEBOUNCE DE ESTADO (HISTERESE) ---
// ======================================================================

// Estrutura de debounce: evita mudanças de estado por leituras isoladas
typedef struct {
    int estado_atual;   // Estado confirmado: OCUPADO ou LIVRE
    int cont_ocupado;   // Contagem de leituras consecutivas com presença
    int cont_livre;     // Contagem de leituras consecutivas sem presença
} hcsr04_debounce_t;

// Atualiza o debounce com a nova leitura e retorna o estado estável
static int debounce_atualizar(hcsr04_debounce_t *db, int leitura_bruta)
{
    if (leitura_bruta == HCSR04_OCUPADO) {
        db->cont_ocupado++;
        db->cont_livre = 0;
        if (db->cont_ocupado >= HCSR04_CONFIRM_OCUPADO) {
            db->estado_atual = HCSR04_OCUPADO;
            db->cont_ocupado = HCSR04_CONFIRM_OCUPADO;
        }
    } else {
        db->cont_livre++;
        db->cont_ocupado = 0;
        if (db->cont_livre >= HCSR04_CONFIRM_LIVRE) {
            db->estado_atual = HCSR04_LIVRE;
            db->cont_livre   = HCSR04_CONFIRM_LIVRE;
        }
    }
    return db->estado_atual;
}

// ======================================================================
// --- FUNÇÃO PRINCIPAL ---
// ======================================================================

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    mqtt_app_start();

    // Inicializa barramento I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Inicializa sensor PIR
    gpio_config_t pir_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SENSOR_PIR_PIN),
        .pull_up_en   = 0,
        .pull_down_en = 1
    };
    gpio_config(&pir_conf);

    // Inicializa os quatro sensores HC-SR04
    hcsr04_init(HCSR04_M3A_TRIG, HCSR04_M3A_ECHO);
    hcsr04_init(HCSR04_M3B_TRIG, HCSR04_M3B_ECHO);
    hcsr04_init(HCSR04_M4A_TRIG, HCSR04_M4A_ECHO);
    hcsr04_init(HCSR04_M4B_TRIG, HCSR04_M4B_ECHO);

    // Inicializa sensores de luz BH1750 das mesas 3 e 4
    uint8_t bh_cmd = BH1750_CMD_START;
    tca9548a_select_channel(CH_MESA3_BH1750);
    i2c_master_write_to_device(I2C_PORT, BH1750_SENSOR_ADDR, &bh_cmd, 1, pdMS_TO_TICKS(200));
    tca9548a_select_channel(CH_MESA4_BH1750);
    i2c_master_write_to_device(I2C_PORT, BH1750_SENSOR_ADDR, &bh_cmd, 1, pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Inicialização concluída.");

    // Últimas distâncias válidas — inicializadas como livre
    float dist3a = HCSR04_DIST_MAX_CM, dist3b = HCSR04_DIST_MAX_CM;
    float dist4a = HCSR04_DIST_MAX_CM, dist4b = HCSR04_DIST_MAX_CM;

    // Buffers de média móvel — um por sensor
    hcsr04_mm_t mm_m3a, mm_m3b, mm_m4a, mm_m4b;
    mm_init(&mm_m3a); mm_init(&mm_m3b);
    mm_init(&mm_m4a); mm_init(&mm_m4b);

    // Estruturas de debounce — uma por sensor
    hcsr04_debounce_t db_m3a = { .estado_atual = HCSR04_LIVRE, .cont_ocupado = 0, .cont_livre = 0 };
    hcsr04_debounce_t db_m3b = { .estado_atual = HCSR04_LIVRE, .cont_ocupado = 0, .cont_livre = 0 };
    hcsr04_debounce_t db_m4a = { .estado_atual = HCSR04_LIVRE, .cont_ocupado = 0, .cont_livre = 0 };
    hcsr04_debounce_t db_m4b = { .estado_atual = HCSR04_LIVRE, .cont_ocupado = 0, .cont_livre = 0 };

    while (1)
    {
        int pir = gpio_get_level(SENSOR_PIR_PIN);
        float lux3 = 0.0f, lux4 = 0.0f;
        uint8_t data[2];

        // Leitura de luminosidade — Mesa 3
        tca9548a_select_channel(CH_MESA3_BH1750);
        if (i2c_master_read_from_device(I2C_PORT, BH1750_SENSOR_ADDR, data, 2, pdMS_TO_TICKS(200)) == ESP_OK)
            lux3 = (((data[0] << 8) | data[1]) / 1.2f) * BH1750_CAL_FACTOR * BH1750_POS_FACTOR_M3;

        // Leitura de luminosidade — Mesa 4
        tca9548a_select_channel(CH_MESA4_BH1750);
        if (i2c_master_read_from_device(I2C_PORT, BH1750_SENSOR_ADDR, data, 2, pdMS_TO_TICKS(200)) == ESP_OK)
            lux4 = (((data[0] << 8) | data[1]) / 1.2f) * BH1750_CAL_FACTOR * BH1750_POS_FACTOR_M4;

        // Leitura de ocupância — Mesa 3 (fusão dos dois sensores)
        int ocup3a    = hcsr04_ocupancia(HCSR04_M3A_TRIG, HCSR04_M3A_ECHO, &mm_m3a, &dist3a);
        int db3a      = debounce_atualizar(&db_m3a, ocup3a);
        esp_rom_delay_us(HCSR04_INTER_SENSOR_GUARD_US);
        int ocup3b    = hcsr04_ocupancia(HCSR04_M3B_TRIG, HCSR04_M3B_ECHO, &mm_m3b, &dist3b);
        int db3b      = debounce_atualizar(&db_m3b, ocup3b);
        int ocupado_mesa3 = fusao_ocupancia(db3a, db3b);

        // Leitura de ocupância — Mesa 4 (fusão dos dois sensores)
        esp_rom_delay_us(HCSR04_INTER_SENSOR_GUARD_US);
        int ocup4a    = hcsr04_ocupancia(HCSR04_M4A_TRIG, HCSR04_M4A_ECHO, &mm_m4a, &dist4a);
        int db4a      = debounce_atualizar(&db_m4a, ocup4a);
        esp_rom_delay_us(HCSR04_INTER_SENSOR_GUARD_US);
        int ocup4b    = hcsr04_ocupancia(HCSR04_M4B_TRIG, HCSR04_M4B_ECHO, &mm_m4b, &dist4b);
        int db4b      = debounce_atualizar(&db_m4b, ocup4b);
        int ocupado_mesa4 = fusao_ocupancia(db4a, db4b);

        // Publicações MQTT
        mqtt_publish_float("mesa3/lux",         lux3);
        mqtt_publish_float("mesa3/distancia_a", dist3a);
        mqtt_publish_float("mesa3/distancia_b", dist3b);
        mqtt_publish_int  ("mesa3/ocupado",      ocupado_mesa3);

        mqtt_publish_float("mesa4/lux",         lux4);
        mqtt_publish_float("mesa4/distancia_a", dist4a);
        mqtt_publish_float("mesa4/distancia_b", dist4b);
        mqtt_publish_int  ("mesa4/ocupado",      ocupado_mesa4);

        mqtt_publish_int("ambiente/movimento",  pir);

        ESP_LOGI(TAG, "Mesa3 → Lux: %.2f lx | A: %.1fcm(db:%d) B: %.1fcm(db:%d) | Ocupada: %d",
                 lux3, dist3a, db3a, dist3b, db3b, ocupado_mesa3);
        ESP_LOGI(TAG, "Mesa4 → Lux: %.2f lx | A: %.1fcm(db:%d) B: %.1fcm(db:%d) | Ocupada: %d",
                 lux4, dist4a, db4a, dist4b, db4b, ocupado_mesa4);
        ESP_LOGI(TAG, "Ambiente → Movimento PIR: %d", pir);
        ESP_LOGI(TAG, "------------------------------------------------------");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}