#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Konfigurasi Pin BTS7960
#define R_EN_GPIO     16   // Right Enable
#define L_EN_GPIO     17   // Left Enable
#define RPWM_GPIO     5  // Right PWM
#define LPWM_GPIO     18  // Left PWM

// Konfigurasi PWM
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RESOL   LEDC_TIMER_10_BIT  // Resolusi 10-bit (0-1023)
#define LEDC_FREQUENCY    100              // Frekuensi 20kHz

void motor_init() {
    // Inisialisasi pin Enable
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << R_EN_GPIO) | (1ULL << L_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&en_conf);
    
    // Aktifkan kedua enable pin
    gpio_set_level(R_EN_GPIO, 1);
    gpio_set_level(L_EN_GPIO, 1);

    // Konfigurasi timer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RESOL,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Konfigurasi channel PWM untuk RPWM
    ledc_channel_config_t ledc_rpwm = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = RPWM_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_rpwm);

    // Konfigurasi channel PWM untuk LPWM
    ledc_channel_config_t ledc_lpwm = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LPWM_GPIO,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_lpwm);
}

void motor_control(int speed) {
    // Batasi kecepatan antara -255 sampai 255
    speed = (speed > 1023) ? 1023 : ((speed < -1023) ? -1023 : speed);

    if(speed > 0) { // Putar ke arah kanan
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, speed);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    }
    else if(speed < 0) { // Putar ke arah kiri
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, -speed);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    }
    else { // Berhenti
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    }
}

void app_main(void) {
    motor_init();

    while(1) {
        printf("Motor maju...\n");
        for(int i=0; i<=1023; i+=4) {
            motor_control(i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        printf("Motor mundur...\n");
        for(int i=0; i<=1023; i+=4) {
            motor_control(-i);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        printf("Motor stop\n");
        motor_control(0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        //hallo hallo nyoba
    }
}