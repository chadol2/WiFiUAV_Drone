#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"


/*====================================================
 * 사용자 설정 (여기만 채우면 됨)
 *===================================================*/
#define WIFI_SSID            "FLOW_54A663"   // 드론 AP 이름 (자동 검색 대상) -> 스마트폰 이용 접속한 드론의 AP 이름으로 변경해야 함
#define WIFI_PASS            ""              // 오픈 네트워크면 빈 문자열 유지

// 스틱 최대 편향값(raw ADC count, 중심값 기준 offset). 실측 후 조정할 것.
// 아래 DEBUG_PRINT_RAW_SWING = 1 로 두고 스틱을 끝까지 밀어서 로그로 확인.
#define R_X_MAX_SWING   3000
#define R_Y_MAX_SWING   3000
#define L_X_MAX_SWING   3000
#define L_Y_MAX_SWING   3000

#define DEBUG_PRINT_RAW_SWING   0   // 1로 켜면 캘리브레이션 offset 원시값을 계속 출력


/*====================================================
 * 하드웨어 핀 정의 (기존과 동일)
 *===================================================*/
#define I2C_SDA_GPIO         GPIO_NUM_6
#define I2C_SCL_GPIO         GPIO_NUM_7

#define JOYSTICK_SW_GPIO_L   GPIO_NUM_5    // 이착륙 토글 버튼 (좌측 토글 버튼)
#define JOYSTICK_SW_GPIO_R   GPIO_NUM_21   // Stop 비상정지 버튼 (우측 토글 버튼)

#define BUZZER_GPIO          GPIO_NUM_4    // 능동 부저 (기존 GPIO4 유지)
#define WIFI_RSSI_WEAK_DBM   (-75)         // 이 값보다 약하면 "약함" 경고음 시작 (1차: 조기 경고)

// 신호가 약해지는 순서: -75(경고음 삐 시작) → -85(강제 LAND) → -90(강제 STOP, 최후수단)
#define WIFI_RSSI_LAND_DBM   (-85)         // 이 이하로 떨어지면 강제 착륙 명령 (2차 대응)
#define WIFI_RSSI_STOP_DBM   (-90)         // 이 이하로 떨어지면 강제 STOP 명령 (최후수단)
#define RSSI_GRACE_PERIOD_MS  3000         // 연결 직후 이 시간 동안은 강제 land/stop 판단 보류
                                            // (막 붙은 직후 RSSI가 일시적으로 튀어 오탐하는 것 방지)

// buzzer_task(감시 스레드) → 메인 조종 루프로 "강제 명령 실행해줘" 요청하는 플래그.
// 메인 루프가 소비(consume)하고 다시 false로 되돌림.
static volatile bool g_force_land = false;
static volatile bool g_force_stop = false;

#define ADS1115_ADDR         0x48
#define ADS1115_REG_CONVERSION   0x00
#define ADS1115_REG_CONFIG       0x01

#define DEADZONE   100


/*====================================================
 * V888 프로토콜 상수 (문서 실측 기반)
 *===================================================*/
#define DRONE_IP     "192.168.169.1"
#define CTRL_PORT    8800
#define NEUTRAL      0x80

static const uint8_t HELLO[]     = {0xef,0x00,0x04,0x00};
static const uint8_t SHORT_CMD[] = {0xef,0x20,0x06,0x00,0x01,0x65};
static const uint8_t CMD2[] = {
    0xef,0x20,0x19,0x00,0x01,0x67,0x3c,0x69,0x3d,0x32,0x5e,0x62,
    0x66,0x5f,0x73,0x73,0x69,0x64,0x3d,0x63,0x6d,0x64,0x3d,0x32,0x3e
};
static const uint8_t CMD3[] = {
    0xef,0x20,0x19,0x00,0x01,0x67,0x3c,0x69,0x3d,0x32,0x5e,0x62,
    0x66,0x5f,0x73,0x73,0x69,0x64,0x3d,0x63,0x6d,0x64,0x3d,0x33,0x3e
};

#define TOGGLE_HOLD_MS   1500   // 실측 1.2초 → 여유 두고 1.5초


static const char *TAG = "V888";

/*----------------------------------------------------
 * I2C / ADS1115 관련 전역
 *---------------------------------------------------*/
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t ads1115_dev;

static int16_t r_x_center, r_y_center, l_x_center, l_y_center;

/*----------------------------------------------------
 * UDP 소켓
 *---------------------------------------------------*/
static int udp_sock = -1;
static struct sockaddr_in drone_addr;
static uint32_t rc_seq = 0;


/*====================================================
 * I2C / ADS1115
 *===================================================*/
static void i2c_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = 10000,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = false }
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &ads1115_dev));
}

static esp_err_t ads1115_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_transmit(ads1115_dev, data, sizeof(data), 1000);
}

static esp_err_t ads1115_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t rx_data[2];
    esp_err_t ret = i2c_master_transmit_receive(ads1115_dev, &reg, 1, rx_data, 2, 1000);
    if (ret != ESP_OK) return ret;
    *value = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    return ESP_OK;
}

static int16_t ads1115_read_channel(uint8_t channel)
{
    uint16_t config;
    switch (channel) {
        case 0: config = 0xC383; break;
        case 1: config = 0xD383; break;
        case 2: config = 0xE383; break;
        case 3: config = 0xF383; break;
        default: return -1;
    }

    if (ads1115_write_register(ADS1115_REG_CONFIG, config) != ESP_OK) return -1;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t dummy = 0;
    ads1115_read_register(ADS1115_REG_CONVERSION, &dummy);   // 크로스토크 제거용 더미 리드

    ads1115_write_register(ADS1115_REG_CONFIG, config);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t raw = 0;
    if (ads1115_read_register(ADS1115_REG_CONVERSION, &raw) != ESP_OK) return -1;

    return (int16_t)raw;
}

static void joystick_button_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_SW_GPIO_R) | (1ULL << JOYSTICK_SW_GPIO_L),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}


/*====================================================
 * 능동 부저 (GPIO4) — WiFi 신호 상태 알림음
 *===================================================*/
static EventGroupHandle_t wifi_event_group;   // WiFi 상태 비트 (buzzer_task, wifi_connect 등에서 공유)
#define WIFI_CONNECTED_BIT   BIT0

static void buzzer_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(BUZZER_GPIO, 0);
}

// 능동 부저라 GPIO를 켰다 끄기만 하면 됨. 이 함수는 소리 길이만큼 블로킹되므로
// 반드시 별도 태스크(buzzer_task)에서만 호출 — 메인 RC 전송 루프에서는 절대 호출 금지.
static void beep(int on_ms)
{
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    gpio_set_level(BUZZER_GPIO, 0);
}

typedef enum {
    WIFI_SIGNAL_UNKNOWN = 0,
    WIFI_SIGNAL_GOOD,
    WIFI_SIGNAL_WEAK,
    WIFI_SIGNAL_DISCONNECTED
} wifi_signal_state_t;

// WiFi 연결/신호 세기를 독립적으로 감시하며 부저를 울리는 태스크.
// 메인 조종 루프(RC 전송)와 완전히 분리되어 있어, beep() 안의 vTaskDelay가
// 40/50Hz 조종 패킷 전송을 방해하지 않음.
static void buzzer_task(void *arg)
{
    wifi_signal_state_t prev_state = WIFI_SIGNAL_UNKNOWN;
    int64_t last_weak_beep_ms = 0;

    bool ever_connected = false;      // 부팅 후 한 번이라도 실제 연결된 적 있는지
                                       // (최초 부팅/스캔 중을 "끊김"으로 오판하지 않기 위함)
    int64_t connected_since_ms = 0;   // 마지막으로 연결된 시각 (RSSI 안정화 유예 계산용)

    // 강제 land/stop은 임계값을 넘는 "순간"에 한 번만 요청을 세우고,
    // 신호가 회복되어야 다시 재무장(re-arm)되도록 래치 처리 (반복 스팸 방지)
    bool land_latched = false;
    bool stop_latched = false;

    while (1) {
        bool connected = (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (!connected) {
            // 한 번도 연결된 적 없는 상태(부팅 직후 스캔/연결 중)는 "끊김"이 아니라
            // "아직 연결 전"이므로 알람을 울리지 않고 조용히 대기
            if (!ever_connected) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            if (prev_state != WIFI_SIGNAL_DISCONNECTED) {
                // 방금 끊김 감지 → 1초 간격으로 딱 5번만 삐
                ESP_LOGW(TAG, "WiFi disconnected — buzzer alert x5");
                for (int i = 0; i < 5; i++) {
                    beep(150);
                    vTaskDelay(pdMS_TO_TICKS(850));   // 총 주기 약 1초
                    // 도중에 재연결되면 나머지 반복 중단
                    if ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) != 0) {
                        break;
                    }
                }
                prev_state = WIFI_SIGNAL_DISCONNECTED;
            }
            land_latched = false;   // 연결 끊기면 래치 리셋 (재연결 후 다시 판단)
            stop_latched = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // ---- 연결된 상태: RSSI 조회 ----
        wifi_ap_record_t ap_info;
        bool have_rssi = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
        int8_t rssi = have_rssi ? ap_info.rssi : 0;

        if (have_rssi) {
            ESP_LOGI(TAG, "WiFi RSSI = %d dBm", rssi);   // 요청하신 현재 dBm 로그 출력
        }

        // 부팅 후 최초 접속, 또는 끊겼다가 재접속된 순간 → 삐 1번
        if (prev_state == WIFI_SIGNAL_UNKNOWN || prev_state == WIFI_SIGNAL_DISCONNECTED) {
            beep(150);
            ESP_LOGI(TAG, "WiFi connected — buzzer beep once (RSSI=%d)", rssi);
            last_weak_beep_ms = now_ms;
            prev_state = WIFI_SIGNAL_GOOD;   // 아래에서 실제 신호세기로 재평가됨

            ever_connected = true;
            connected_since_ms = now_ms;     // RSSI 안정화 유예 타이머 시작
        }

        if (have_rssi && rssi < WIFI_RSSI_WEAK_DBM) {
            prev_state = WIFI_SIGNAL_WEAK;
            if (now_ms - last_weak_beep_ms >= 3000) {
                beep(150);
                last_weak_beep_ms = now_ms;
                ESP_LOGW(TAG, "WiFi weak signal (RSSI=%d dBm) — buzzer beep", rssi);
            }
        } else {
            prev_state = WIFI_SIGNAL_GOOD;
        }

        // ---- 강제 착륙 / 강제 정지 요청 (임계값 최초 진입 시 1회만 세팅) ----
        // 연결 직후 RSSI_GRACE_PERIOD_MS 동안은 판단 보류 (막 붙은 직후 값이 불안정할 수 있음 —
        // 이 유예 없이 즉시 판단하면 실제로는 신호 정상인데도 순간적인 튀는 값 때문에
        // 아밍 직전/직후에 잘못 STOP·LAND가 발동될 수 있었음)
        bool grace_period_active = (now_ms - connected_since_ms) < RSSI_GRACE_PERIOD_MS;

        if (have_rssi && !grace_period_active) {
            if (rssi <= WIFI_RSSI_STOP_DBM) {
                if (!stop_latched) {
                    g_force_stop = true;
                    stop_latched = true;
                    ESP_LOGE(TAG, "RSSI %d dBm <= %d dBm — 강제 STOP 요청", rssi, WIFI_RSSI_STOP_DBM);
                }
            } else {
                stop_latched = false;   // 신호 회복 시 재무장
            }

            if (rssi <= WIFI_RSSI_LAND_DBM) {
                if (!land_latched) {
                    g_force_land = true;
                    land_latched = true;
                    ESP_LOGW(TAG, "RSSI %d dBm <= %d dBm — 강제 LAND 요청", rssi, WIFI_RSSI_LAND_DBM);
                }
            } else {
                land_latched = false;   // 신호 회복 시 재무장
            }
        }

        vTaskDelay(pdMS_TO_TICKS(300));   // 상태 체크 주기 (약함 상태 삐 간격은 위에서 3초로 별도 관리)
    }
}

static void calibrate_joysticks()
{
    int32_t sum_rx=0, sum_ry=0, sum_lx=0, sum_ly=0;
    const int N = 10;

    ESP_LOGI(TAG, "Calibrating... keep sticks centered");

    for (int i = 0; i < N; i++) {
        sum_rx += ads1115_read_channel(2);   // 우측 X (A2)
        sum_ry += ads1115_read_channel(0);   // 우측 Y (A0)
        sum_lx += ads1115_read_channel(3);   // 좌측 X (A3, 스왑됨)
        sum_ly += ads1115_read_channel(1);   // 좌측 Y (A1, 스왑됨)
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    r_x_center = sum_rx / N;
    r_y_center = sum_ry / N;
    l_x_center = sum_lx / N;
    l_y_center = sum_ly / N;

    ESP_LOGI(TAG, "Center: R(%d,%d) L(%d,%d)",
             r_x_center, r_y_center, l_x_center, l_y_center);
}

// 정규화된 조이스틱 값(중심 보정 + 반전 + 데드존까지 적용된 최종값) 읽기
static void read_joysticks(int16_t *r_x, int16_t *r_y, int16_t *l_x, int16_t *l_y)
{
    int16_t raw_r_x = ads1115_read_channel(2) - r_x_center;
    int16_t raw_r_y = ads1115_read_channel(0) - r_y_center;
    int16_t raw_l_x = ads1115_read_channel(3) - l_x_center;
    int16_t raw_l_y = ads1115_read_channel(1) - l_y_center;

    *r_x = raw_r_x;
    *r_y = -raw_r_y;     // 위로 밀면 +
    *l_x = -raw_l_x;     // 180도 배치 보정
    *l_y = raw_l_y;      // 180도 보정 + 위=+ 보정 상쇄

    if (abs(*r_x) < DEADZONE) *r_x = 0;
    if (abs(*r_y) < DEADZONE) *r_y = 0;
    if (abs(*l_x) < DEADZONE) *l_x = 0;
    if (abs(*l_y) < DEADZONE) *l_y = 0;
}


/*====================================================
 * 정규화 값(raw offset) → V888 프로토콜 바이트(0~255, 중립 128)
 *===================================================*/
static uint8_t map_axis_to_byte(int16_t value, int16_t max_swing)
{
    if (value > max_swing)  value = max_swing;
    if (value < -max_swing) value = -max_swing;

    int32_t scaled = (int32_t)value * 127 / max_swing;   // -127 ~ 127
    int32_t out = 128 + scaled;

    if (out < 0)   out = 0;
    if (out > 255) out = 255;

    return (uint8_t)out;
}


/*====================================================
 * WiFi 초기화 (드론 AP에 station으로 접속)
 *===================================================*/
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP, ready to connect to drone");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// 1단계: WiFi 스택만 초기화하고 STA 모드로 시작 (아직 접속은 안 함)
static void wifi_stack_init()
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// 2단계: 주변 AP 스캔 후 리스트를 로그로 출력. target_ssid가 목록에 있으면 true 반환
static bool wifi_scan_and_print(const char *target_ssid)
{
    ESP_LOGI(TAG, "Scanning for WiFi APs...");

    wifi_scan_config_t scan_config = {};   // 전체 0으로 초기화 (모든 필드 커버)
    scan_config.ssid = NULL;
    scan_config.bssid = NULL;
    scan_config.channel = 0;
    scan_config.show_hidden = true;

    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);   // true = 블로킹, 스캔 끝날 때까지 대기
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(ret));
        return false;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);

    if (ap_count == 0) {
        ESP_LOGW(TAG, "No APs found");
        return false;
    }

    wifi_ap_record_t *ap_records =
        (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);

    if (ap_records == NULL) {
        ESP_LOGE(TAG, "malloc failed for AP list (count=%d)", ap_count);
        return false;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_records));

    printf("\n==== WiFi AP Scan Result (%d found) ====\n", ap_count);
    printf("%-3s %-32s %-6s %-4s\n", "No", "SSID", "RSSI", "CH");
    printf("-----------------------------------------------\n");

    bool found = false;

    for (int i = 0; i < ap_count; i++) {
        printf("%-3d %-32s %-6d %-4d\n",
               i + 1,
               (char *)ap_records[i].ssid,
               ap_records[i].rssi,
               ap_records[i].primary);

        if (strcmp((char *)ap_records[i].ssid, target_ssid) == 0) {
            found = true;
        }
    }
    printf("=================================================\n\n");

    free(ap_records);

    return found;
}

// 목표 SSID가 스캔 목록에 나타날 때까지 계속 재시도 (못 찾으면 무한 반복)
static void wifi_scan_until_found(const char *target_ssid)
{
    int attempt = 0;

    while (true) {
        attempt++;
        ESP_LOGI(TAG, "Scan attempt #%d — looking for \"%s\"", attempt, target_ssid);

        if (wifi_scan_and_print(target_ssid)) {
            ESP_LOGI(TAG, "Target AP \"%s\" found!", target_ssid);
            return;
        }

        ESP_LOGW(TAG, "\"%s\" not found, retrying in 2s...", target_ssid);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// 3단계: 지정된 SSID로 실제 접속
static void wifi_connect()
{
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = (strlen(WIFI_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_LOGI(TAG, "Connecting to drone AP: %s", WIFI_SSID);
    esp_wifi_connect();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}


/*====================================================
 * UDP 소켓 및 V888 패킷 빌드
 *===================================================*/
static void udp_init()
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    memset(&drone_addr, 0, sizeof(drone_addr));
    drone_addr.sin_family = AF_INET;
    drone_addr.sin_port = htons(CTRL_PORT);
    inet_pton(AF_INET, DRONE_IP, &drone_addr.sin_addr);

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void udp_send(const uint8_t *data, size_t len)
{
    sendto(udp_sock, data, len, 0, (struct sockaddr *)&drone_addr, sizeof(drone_addr));
}

// 핸드셰이크용 88바이트 상태 envelope (command_len=0, 즉 빈 envelope)
static void build_status_envelope(uint8_t *pkt, uint32_t counter)
{
    memset(pkt, 0, 88);
    uint8_t header[8] = {0xef,0x02,0x58,0x00,0x02,0x02,0x00,0x01};
    memcpy(pkt, header, 8);
    memcpy(pkt + 12, &counter, 4);   // LE, ESP32는 little-endian이라 그대로 memcpy 가능
    uint8_t quality[4] = {0x32,0x4b,0x14,0x2d};
    memcpy(pkt + 82, quality, 4);
}

// 88바이트 RC 제어 패킷 빌드 (체크섬/터미네이터까지 문서 스펙대로 정확히 채움)
static void build_rc_packet(uint8_t *pkt, uint32_t seq,
                             uint8_t roll, uint8_t pitch, uint8_t throttle, uint8_t yaw,
                             uint8_t flag, uint8_t mode, uint8_t camera_index)
{
    memset(pkt, 0, 88);

    uint8_t header[8] = {0xef,0x02,0x58,0x00,0x02,0x02,0x00,0x01};
    memcpy(pkt, header, 8);

    pkt[8] = 0x00;               // num_ack_slots = 0 (영상 ACK 미사용, 88바이트 고정)
    // 9-11 reserved = 0 (memset으로 이미 0)

    memcpy(pkt + 12, &seq, 4);   // command_seq (LE)

    pkt[16] = 0x14; pkt[17] = 0x00;   // command_len = 20
    pkt[18] = 0x66; pkt[19] = 0x14;   // RC 매직 마커

    pkt[20] = roll;
    pkt[21] = pitch;
    pkt[22] = throttle;
    pkt[23] = yaw;
    pkt[24] = flag;
    pkt[25] = mode;

    // 26-35 padding = 0 (memset)

    uint8_t checksum = roll ^ pitch ^ throttle ^ yaw ^ flag ^ mode;
    pkt[36] = checksum;
    pkt[37] = 0x99;   // 터미네이터

    // 38-81 padding = 0 (memset)

    uint8_t quality[4] = {0x32,0x4b,0x14,0x2d};
    memcpy(pkt + 82, quality, 4);

    pkt[86] = camera_index;
    // 87 reserved = 0
}


/*====================================================
 * 핸드셰이크
 *===================================================*/
static void v888_connect()
{
    udp_send(HELLO, sizeof(HELLO));
    udp_send(HELLO, sizeof(HELLO));

    for (int i = 0; i < 12; i++) {
        uint8_t env[88];
        build_status_envelope(env, i);
        udp_send(env, sizeof(env));
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    udp_send(SHORT_CMD, sizeof(SHORT_CMD));
    udp_send(CMD2, sizeof(CMD2));
    udp_send(CMD3, sizeof(CMD3));

    vTaskDelay(pdMS_TO_TICKS(1000));  // 드론이 RC 스트림 인식/정착할 시간
                                       // (Python 레퍼런스 실측: 0.5초는 부족해서 아밍 실패,
                                       //  1.0초에서 안정적으로 확인됨)

    ESP_LOGI(TAG, "V888 handshake complete, RC stream starting");
}


/*====================================================
 * main
 *===================================================*/
extern "C" void app_main(void)
{
    // 1. 조이스틱/ADS1115/부저 초기화
    i2c_init();
    joystick_button_init();
    buzzer_init();
    ESP_LOGI(TAG, "Calibrating joysticks — keep both sticks centered");
    calibrate_joysticks();

    // 2. WiFi: 스택 초기화 → 목표 드론 AP("FLOW_54A663")가 검색될 때까지 반복 스캔 → 접속
    wifi_stack_init();

    // wifi_event_group이 생성된 직후부터 독립 태스크로 WiFi 상태/신호세기 감시 + 부저 알림 시작
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 1, NULL);

    wifi_scan_until_found(WIFI_SSID);
    wifi_connect();

    // 3. UDP 소켓 준비 + 핸드셰이크
    udp_init();
    v888_connect();

    // 4. 버튼 엣지 감지 + flag 펄스 상태
    int prev_sw_r = 1, prev_sw_l = 1;   // pull-up: 안 눌림=1
    uint8_t control_flag = 0x00;
    int64_t flag_release_time_ms = 0;

    bool was_connected = true;   // 부팅 시점엔 이미 연결된 상태로 시작

    // ---- 이착륙 상태 추적 (드론 텔레메트리가 없어 자체 추적) ----
    bool is_flying = false;          // 마지막으로 우리가 보낸 토글 기준 비행 여부
    bool sending_paused = false;     // true면 RC 패킷 전송 자체를 건너뜀
    bool land_pending_pause = false; // "착륙" 펄스가 끝나는 순간 sending_paused=true로 전환 예약
    // ※ Python 레퍼런스 실측 경고: 착륙 성공 직후 중립 패킷을 계속 보내면 드론이
    //    저절로 재이륙하는 현상이 확인됨. 그래서 착륙 펄스가 끝나면 전송을 완전히
    //    멈추고, 다음 이착륙 버튼(재이륙)을 눌러야만 전송을 재개하도록 구현.

    ESP_LOGI(TAG, "RC control loop start (50Hz)");

    while (1) {
        // ---- WiFi 연결 상태 체크 (끊김 감지 → 재연결 시 핸드셰이크 재실행) ----
        bool is_connected =
            (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) != 0;

        if (!is_connected) {
            if (was_connected) {
                ESP_LOGW(TAG, "WiFi disconnected — pausing RC stream, waiting for reconnect");
            }
            was_connected = false;

            // 안전을 위해 조종값을 중립으로 리셋 (재연결 전까지 값 누적/오작동 방지)
            control_flag = 0x00;

            vTaskDelay(pdMS_TO_TICKS(200));
            continue;   // 연결 안 된 동안은 RC 패킷 전송/조이스틱 처리 건너뜀
        }

        if (!was_connected) {
            // 방금 재연결됨 → 드론 쪽 UDP 세션도 리셋됐을 가능성이 높으므로 핸드셰이크 재실행
            ESP_LOGI(TAG, "WiFi reconnected — redoing V888 handshake");
            v888_connect();
            was_connected = true;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        // ---- 조이스틱 읽기 ----
        int16_t r_x, r_y, l_x, l_y;
        read_joysticks(&r_x, &r_y, &l_x, &l_y);

#if DEBUG_PRINT_RAW_SWING
        printf("RAW  R(X=%d,Y=%d)  L(X=%d,Y=%d)\n", r_x, r_y, l_x, l_y);
#endif

        uint8_t roll     = map_axis_to_byte(r_x, R_X_MAX_SWING);
        uint8_t pitch    = map_axis_to_byte(r_y, R_Y_MAX_SWING);
        uint8_t throttle = map_axis_to_byte(l_y, L_Y_MAX_SWING);
        uint8_t yaw      = map_axis_to_byte(l_x, L_X_MAX_SWING);

        // ---- 버튼 엣지 감지 ----
        int sw_r = gpio_get_level(JOYSTICK_SW_GPIO_R);
        int sw_l = gpio_get_level(JOYSTICK_SW_GPIO_L);

        bool r_pressed_edge = (prev_sw_r == 1 && sw_r == 0);
        bool l_pressed_edge = (prev_sw_l == 1 && sw_l == 0);
        prev_sw_r = sw_r;
        prev_sw_l = sw_l;

        if (r_pressed_edge || g_force_stop) {
            // Stop이 이착륙 토글보다 우선순위 높음 (비상 상황) — WiFi 강제 STOP도 동일 우선순위
            control_flag = 0x02;
            flag_release_time_ms = now_ms + TOGGLE_HOLD_MS;
            is_flying = false;
            // emergency_stop()은 Python 레퍼런스에서도 펄스 후 전송을 멈추지 않음 —
            // 정지 후에도 중립 패킷 계속 전송(레퍼런스와 동일하게 유지)
            if (g_force_stop) {
                ESP_LOGE(TAG, "WiFi 신호 매우 약함 — 강제 STOP 명령 전송");
                g_force_stop = false;   // 요청 소비
            } else {
                ESP_LOGW(TAG, "STOP triggered");
            }
        } else if ((l_pressed_edge && control_flag == 0x00) ||
                   (g_force_land && is_flying && control_flag == 0x00)) {
            control_flag = 0x01;
            flag_release_time_ms = now_ms + TOGGLE_HOLD_MS;

            if (!is_flying) {
                // 지상 → 이륙 (버튼으로만 도달 가능 — g_force_land는 is_flying 조건으로 걸러짐)
                is_flying = true;
                sending_paused = false;   // 혹시 이전 착륙으로 정지 상태였다면 재개
                land_pending_pause = false;
                ESP_LOGI(TAG, "Takeoff toggle triggered");
            } else {
                // 공중 → 착륙: 펄스가 끝나는 시점에 전송을 멈추도록 예약
                is_flying = false;
                land_pending_pause = true;
                if (g_force_land) {
                    ESP_LOGW(TAG, "WiFi 신호 약함 — 강제 LAND 명령 전송");
                    g_force_land = false;   // 요청 소비
                } else {
                    ESP_LOGI(TAG, "Land toggle triggered");
                }
            }
        }

        if (control_flag != 0x00 && now_ms >= flag_release_time_ms) {
            control_flag = 0x00;

            if (land_pending_pause) {
                // 착륙 펄스 종료 시점 → 전송 완전 정지 (재이륙 방지, Python 레퍼런스와 동일)
                sending_paused = true;
                land_pending_pause = false;
                ESP_LOGI(TAG, "Landing pulse complete — RC stream paused (prevents auto re-takeoff)");
            }
        }

        // ---- RC 패킷 전송 (착륙 후 정지 상태면 아예 건너뜀) ----
        if (!sending_paused) {
            uint8_t pkt[88];
            rc_seq++;
            build_rc_packet(pkt, rc_seq, roll, pitch, throttle, yaw, control_flag, 0x02, 0x00);
            udp_send(pkt, sizeof(pkt));

            printf("R=%02X P=%02X T=%02X Y=%02X FLAG=%02X\n",
                   roll, pitch, throttle, yaw, control_flag);
        }

        vTaskDelay(pdMS_TO_TICKS(20));   // 50Hz (Python 레퍼런스와 동일 주기)
    }
}