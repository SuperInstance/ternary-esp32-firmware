/*
 * ternary_policy.h — Ternary policy types and lookup tables for ESP32 firmware
 *
 * This header defines the core data structures for compiled ternary policies.
 * A ternary system uses three-valued logic (−1, 0, +1) called "trits" instead
 * of bits, enabling richer classification with minimal memory footprint.
 *
 * The compiled-policy lookup tables are designed for <15KB binary size,
 * making them ideal for microcontroller deployment.
 */

#ifndef TERNARY_POLICY_H
#define TERNARY_POLICY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Trit encoding ────────────────────────────────────────────────────────── */

/* Trit values: −1 (negative/false), 0 (neutral/unknown), +1 (positive/true) */
typedef int8_t trit_t;

#define TRIT_NEG  ((trit_t)-1)
#define TRIT_ZERO ((trit_t) 0)
#define TRIT_POS  ((trit_t) 1)

/* Pack 5 trits into one int8_t using balanced-ternary base-3 encoding.
 * Range: 3^5 = 243 values, fits in [-121, +121], needs int8_t.
 * This gives us a compact lookup key from sensor readings. */
#define TRITS_PER_BYTE 5
#define TRIT_BASE      3

/* ── Sensor types ─────────────────────────────────────────────────────────── */

/* Simulated ADC channels — on ESP32 these map to GPIO 36-39 (ADC1) */
#define NUM_SENSOR_CHANNELS 4
#define ADC_RESOLUTION      4095  /* 12-bit ADC */

/* Raw sensor reading frame */
typedef struct {
    uint16_t channels[NUM_SENSOR_CHANNELS]; /* raw ADC values */
} sensor_frame_t;

/* Denoised ternary sensor state */
typedef struct {
    trit_t values[NUM_SENSOR_CHANNELS]; /* denoised trit values */
} ternary_sensor_t;

/* ── Majority filter (denoise) ────────────────────────────────────────────── */

/* Window size for majority filter — must be odd */
#define MAJORITY_FILTER_SIZE 5

typedef struct {
    trit_t buffer[NUM_SENSOR_CHANNELS][MAJORITY_FILTER_SIZE];
    uint8_t index;  /* circular buffer write position */
    uint8_t filled; /* how many samples have been recorded */
} majority_filter_t;

/* ── Classifier ───────────────────────────────────────────────────────────── */

/* Number of input trits feeding the classifier */
#define CLASSIFIER_INPUT_TRITS 4

/* Classification result codes */
typedef enum {
    CLASS_IDLE        = 0,
    CLASS_NORMAL      = 1,
    CLASS_ALERT       = 2,
    CLASS_EMERGENCY   = 3,
    CLASS_MAINTENANCE = 4,
    CLASS_COUNT       = 5
} class_label_t;

/* Lookup table for classification.
 * Index: packed trit key (0..80 for 4 trits, 3^4 = 81 entries).
 * Value: class label. Total: 81 bytes. */
#define CLASSIFIER_LUT_SIZE 81  /* 3^4 */

typedef struct {
    uint8_t entries[CLASSIFIER_LUT_SIZE];
} classifier_lut_t;

/* ── Compiled policy ──────────────────────────────────────────────────────── */

/* Motor command: speed + direction per motor */
typedef struct {
    int8_t left_speed;   /* −100..+100 */
    int8_t right_speed;  /* −100..+100 */
} motor_cmd_t;

/* Action codes from policy lookup */
typedef enum {
    ACTION_STOP        = 0,
    ACTION_FORWARD     = 1,
    ACTION_REVERSE     = 2,
    ACTION_TURN_LEFT   = 3,
    ACTION_TURN_RIGHT  = 4,
    ACTION_BRAKE       = 5,
    ACTION_FREEWHEEL   = 6,
    ACTION_COUNT       = 7
} action_code_t;

/* Compiled policy lookup table.
 * Key: (classification << 3) | sensor_packed → action.
 * Total entries: CLASS_COUNT * 8 = 40 bytes for action codes.
 * Plus motor parameters: ACTION_COUNT * sizeof(motor_cmd_t).
 * Grand total: ~54 bytes for full policy. Well under 15KB. */
#define POLICY_SENSOR_PACK_MAX 8  /* 3 bits from sensor summary */

typedef struct {
    action_code_t actions[CLASS_COUNT][POLICY_SENSOR_PACK_MAX];
    motor_cmd_t   motor_params[ACTION_COUNT];
} compiled_policy_t;

/* ── Ternary packing utilities ────────────────────────────────────────────── */

/* Pack an array of trits into a single integer key for LUT indexing.
 * Returns 0 on success, -1 if count exceeds TRITS_PER_BYTE.
 * Result written to *out. */
int trit_pack(const trit_t *trits, int count, uint8_t *out);

/* Unpack a key back into trit array. */
int trit_unpack(uint8_t key, trit_t *trits, int count);

/* Convert raw ADC value to trit using thresholds. */
trit_t adc_to_trit(uint16_t adc_val, uint16_t lo_thresh, uint16_t hi_thresh);

/* ── API functions ────────────────────────────────────────────────────────── */

/* Initialize the majority filter to TRIT_ZERO */
void majority_filter_init(majority_filter_t *f);

/* Push a new ternary sensor reading into the filter */
void majority_filter_push(majority_filter_t *f, const ternary_sensor_t *s);

/* Get the majority-filtered (denoised) sensor state */
void majority_filter_result(const majority_filter_t *f, ternary_sensor_t *out);

/* Build the default classifier LUT (demonstration policy) */
void classifier_lut_default(classifier_lut_t *lut);

/* Classify a denoised ternary sensor state using the LUT.
 * Returns a class label. */
class_label_t classify(const classifier_lut_t *lut, const ternary_sensor_t *sensor);

/* Build the default compiled policy (demonstration) */
void policy_default(compiled_policy_t *policy);

/* Look up the action for a given (classification, sensor_summary) pair.
 * Returns the action code. ~8ns on ESP32 at 240MHz. */
action_code_t policy_lookup(const compiled_policy_t *policy,
                            class_label_t classification,
                            uint8_t sensor_summary);

/* Get the motor command for an action code. */
const motor_cmd_t *policy_motor_cmd(const compiled_policy_t *policy,
                                     action_code_t action);

#ifdef __cplusplus
}
#endif

#endif /* TERNARY_POLICY_H */
