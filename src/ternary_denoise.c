/*
 * ternary_denoise.c — Sensor denoising via ternary majority filter
 *
 * This module wraps the majority filter with ADC-to-trit conversion,
 * providing a clean interface for the main loop.
 */

#include "ternary_policy.h"
#include <string.h>

/* Default thresholds for ADC → trit conversion per channel.
 * Can be tuned per-application. Channel mapping:
 *   0: proximity sensor  (lo: 800,  hi: 3000)
 *   1: temperature       (lo: 1200, hi: 2800)
 *   2: battery voltage   (lo: 1500, hi: 3500)
 *   3: tilt/accelerometer (lo: 1800, hi: 2200)
 */
static const struct {
    uint16_t lo;
    uint16_t hi;
} default_thresholds[NUM_SENSOR_CHANNELS] = {
    { 800, 3000},
    {1200, 2800},
    {1500, 3500},
    {1800, 2200},
};

/* Convert raw sensor frame to ternary sensor state */
void sensor_frame_to_ternary(const sensor_frame_t *frame,
                             ternary_sensor_t *out)
{
    for (int ch = 0; ch < NUM_SENSOR_CHANNELS; ch++) {
        out->values[ch] = adc_to_trit(
            frame->channels[ch],
            default_thresholds[ch].lo,
            default_thresholds[ch].hi
        );
    }
}

/* Pack 4 sensor trits into a 3-bit summary for policy lookup.
 * Uses sign of the sum: mostly neg→0, mixed→1..6, mostly pos→7 */
uint8_t sensor_pack_summary(const ternary_sensor_t *sensor)
{
    int sum = 0;
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++)
        sum += (int)sensor->values[i];

    /* Map sum from [-4,+4] → [0,7] */
    int packed = sum + 4;  /* 0..8 */
    if (packed > 7) packed = 7;
    return (uint8_t)packed;
}
