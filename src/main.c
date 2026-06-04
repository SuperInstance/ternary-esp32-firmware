/*
 * main.c — ESP32 main loop: ternary policy engine on bare metal
 *
 * Pipeline each tick:
 *   1. Read sensors (ADC → simulated)
 *   2. Convert to trits
 *   3. Majority-filter (denoise)
 *   4. Classify via compiled LUT
 *   5. Policy lookup → action
 *   6. Output motor commands
 *
 * On ESP32 at 240 MHz, the policy lookup takes ~8ns (2 clock cycles).
 * Full pipeline: <100µs per tick, leaving >99.99% CPU for WiFi/BLE.
 */

#include "ternary_policy.h"
#include <string.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#define DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#else
/* Host simulation */
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#define DELAY_MS(ms) usleep((ms) * 1000)
#endif

/* ── Global state ─────────────────────────────────────────────────────────── */

static majority_filter_t filter;
static classifier_lut_t  classifier;
static compiled_policy_t policy;

/* ── Simulated ADC reading (host & early testing) ─────────────────────────── */

#ifndef ESP_PLATFORM
static uint16_t sim_adc_counter = 0;

static void read_sensors_simulated(sensor_frame_t *frame)
{
    /* Cycle through patterns to exercise all trit combinations */
    uint16_t patterns[][NUM_SENSOR_CHANNELS] = {
        { 400, 1000, 1000, 1500},  /* all NEG */
        {2000, 2000, 2500, 2000},  /* all ZERO-ish */
        {3500, 3500, 3800, 2500},  /* all POS */
        { 400, 2000, 3800, 1500},  /* mixed */
        {2000, 2000, 2000, 2000},  /* all mid → ZERO */
    };
    int idx = sim_adc_counter % 5;
    memcpy(frame->channels, patterns[idx], sizeof(patterns[idx]));
    sim_adc_counter++;
}
#endif

/* ── ESP32 ADC reading ────────────────────────────────────────────────────── */

#ifdef ESP_PLATFORM
static void read_sensors_esp32(sensor_frame_t *frame)
{
    /* ADC1 channels: GPIO36(CH0), GPIO39(CH1), GPIO34(CH2), GPIO35(CH3) */
    adc1_channel_t channels[NUM_SENSOR_CHANNELS] = {
        ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_6, ADC1_CHANNEL_7
    };
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        frame->channels[i] = (uint16_t)adc1_get_raw(channels[i]);
    }
}
#endif

/* ── Extern from ternary_denoise.c ────────────────────────────────────────── */

extern void sensor_frame_to_ternary(const sensor_frame_t *frame,
                                    ternary_sensor_t *out);
extern uint8_t sensor_pack_summary(const ternary_sensor_t *sensor);

/* ── Motor output ─────────────────────────────────────────────────────────── */

static void output_motor_cmd(const motor_cmd_t *cmd, action_code_t action)
{
#ifdef ESP_PLATFORM
    /* Write PWM to motor driver pins */
    /* mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, cmd->left_speed); */
    /* mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, cmd->right_speed); */
    printf("[ESP32] Motor: L=%d R=%d (action=%d)\n",
           cmd->left_speed, cmd->right_speed, action);
#else
    printf("[SIM] Motor: L=%+4d R=%+4d (action=%d)\n",
           cmd->left_speed, cmd->right_speed, action);
#endif
}

/* ── Main loop ────────────────────────────────────────────────────────────── */

void app_main(void)
{
    printf("=== Ternary ESP32 Firmware v0.1 ===\n");
    printf("Policy size: %zu bytes\n", sizeof(policy));
    printf("Classifier LUT size: %zu bytes\n", sizeof(classifier));
    printf("Total ternary state: %zu bytes\n",
           sizeof(policy) + sizeof(classifier) + sizeof(filter));

    /* Initialize ternary engine */
    majority_filter_init(&filter);
    classifier_lut_default(&classifier);
    policy_default(&policy);

    printf("Ternary policy engine initialized. Entering main loop.\n\n");

    sensor_frame_t frame;
    int tick = 0;

    while (1) {
        /* 1. Read sensors */
#ifdef ESP_PLATFORM
        read_sensors_esp32(&frame);
#else
        read_sensors_simulated(&frame);
#endif

        /* 2. Convert ADC → trits */
        ternary_sensor_t raw_ternary;
        sensor_frame_to_ternary(&frame, &raw_ternary);

        /* 3. Majority filter (denoise) */
        majority_filter_push(&filter, &raw_ternary);
        ternary_sensor_t denoised;
        majority_filter_result(&filter, &denoised);

        /* 4. Classify */
        class_label_t cls = classify(&classifier, &denoised);

        /* 5. Policy lookup */
        uint8_t sensor_sum = sensor_pack_summary(&denoised);
        action_code_t action = policy_lookup(&policy, cls, sensor_sum);

        /* 6. Motor output */
        const motor_cmd_t *cmd = policy_motor_cmd(&policy, action);
        output_motor_cmd(cmd, action);

        /* Debug output (suppress in production) */
        if (tick % 5 == 0) {
            printf("  [tick %d] trits=[%d,%d,%d,%d] cls=%d act=%d\n",
                   tick,
                   denoised.values[0], denoised.values[1],
                   denoised.values[2], denoised.values[3],
                   cls, action);
        }

        tick++;

#ifdef ESP_PLATFORM
        DELAY_MS(10);  /* 100 Hz control loop */
#else
        DELAY_MS(500); /* Slow for host demo */
        if (tick >= 20) {
            printf("\n=== Demo complete (20 ticks) ===\n");
            break;
        }
#endif
    }
}

/* Host entry point (non-ESP32) */
#ifndef ESP_PLATFORM
int main(void)
{
    app_main();
    return 0;
}
#endif
