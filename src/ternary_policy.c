/*
 * ternary_policy.c — Pure C implementation of ternary policy engine
 *
 * No ESP-IDF dependencies — portable to any C99 platform.
 * The compiled-policy lookup tables total <15KB, suitable for MCU deployment.
 *
 * Pipeline: sensor → ADC-to-trit → majority filter → classify → policy lookup → motor cmd
 */

#include "ternary_policy.h"
#include <string.h>

/* ── Trit packing ─────────────────────────────────────────────────────────── */

int trit_pack(const trit_t *trits, int count, uint8_t *out)
{
    if (count > TRITS_PER_BYTE || count < 1)
        return -1;

    int key = 0;
    int power = 1;
    for (int i = 0; i < count; i++) {
        /* Map trit: −1→0, 0→1, +1→2 in base-3 */
        int digit = (int)trits[i] + 1;
        key += digit * power;
        power *= TRIT_BASE;
    }
    *out = (uint8_t)key;
    return 0;
}

int trit_unpack(uint8_t key, trit_t *trits, int count)
{
    if (count > TRITS_PER_BYTE || count < 1)
        return -1;

    for (int i = 0; i < count; i++) {
        int digit = key % TRIT_BASE;
        trits[i] = (trit_t)(digit - 1); /* 0→−1, 1→0, 2→+1 */
        key /= TRIT_BASE;
    }
    return 0;
}

trit_t adc_to_trit(uint16_t adc_val, uint16_t lo_thresh, uint16_t hi_thresh)
{
    if (adc_val <= lo_thresh)
        return TRIT_NEG;
    if (adc_val >= hi_thresh)
        return TRIT_POS;
    return TRIT_ZERO;
}

/* ── Majority filter (denoise) ────────────────────────────────────────────── */

void majority_filter_init(majority_filter_t *f)
{
    memset(f, 0, sizeof(*f));
    /* Pre-fill with TRIT_ZERO */
    for (int ch = 0; ch < NUM_SENSOR_CHANNELS; ch++) {
        for (int i = 0; i < MAJORITY_FILTER_SIZE; i++) {
            f->buffer[ch][i] = TRIT_ZERO;
        }
    }
}

void majority_filter_push(majority_filter_t *f, const ternary_sensor_t *s)
{
    for (int ch = 0; ch < NUM_SENSOR_CHANNELS; ch++) {
        f->buffer[ch][f->index] = s->values[ch];
    }
    f->index = (f->index + 1) % MAJORITY_FILTER_SIZE;
    if (f->filled < MAJORITY_FILTER_SIZE)
        f->filled++;
}

void majority_filter_result(const majority_filter_t *f, ternary_sensor_t *out)
{
    for (int ch = 0; ch < NUM_SENSOR_CHANNELS; ch++) {
        int sum = 0;
        for (int i = 0; i < MAJORITY_FILTER_SIZE; i++) {
            sum += (int)f->buffer[ch][i];
        }
        /* Majority: if sum > 0 → POS, sum < 0 → NEG, else ZERO */
        if (sum > 0)
            out->values[ch] = TRIT_POS;
        else if (sum < 0)
            out->values[ch] = TRIT_NEG;
        else
            out->values[ch] = TRIT_ZERO;
    }
}

/* ── Classifier LUT ───────────────────────────────────────────────────────── */

void classifier_lut_default(classifier_lut_t *lut)
{
    memset(lut, 0, sizeof(*lut));

    /*
     * Default demonstration classifier:
     *   - All NEG   → EMERGENCY
     *   - 3+ POS    → NORMAL
     *   - 3+ NEG    → ALERT
     *   - All ZERO  → IDLE
     *   - Mixed     → MAINTENANCE
     *
     * We enumerate all 81 possible 4-trit combinations and assign labels.
     */
    for (int key = 0; key < CLASSIFIER_LUT_SIZE; key++) {
        trit_t t[CLASSIFIER_INPUT_TRITS];
        trit_unpack((uint8_t)key, t, CLASSIFIER_INPUT_TRITS);

        int neg_count = 0, pos_count = 0, zero_count = 0;
        for (int i = 0; i < CLASSIFIER_INPUT_TRITS; i++) {
            if (t[i] == TRIT_NEG)  neg_count++;
            else if (t[i] == TRIT_POS) pos_count++;
            else zero_count++;
        }

        if (neg_count == CLASSIFIER_INPUT_TRITS)
            lut->entries[key] = CLASS_EMERGENCY;
        else if (pos_count >= 3)
            lut->entries[key] = CLASS_NORMAL;
        else if (neg_count >= 3)
            lut->entries[key] = CLASS_ALERT;
        else if (zero_count == CLASSIFIER_INPUT_TRITS)
            lut->entries[key] = CLASS_IDLE;
        else
            lut->entries[key] = CLASS_MAINTENANCE;
    }
}

class_label_t classify(const classifier_lut_t *lut, const ternary_sensor_t *sensor)
{
    uint8_t key;
    if (trit_pack(sensor->values, CLASSIFIER_INPUT_TRITS, &key) != 0)
        return CLASS_IDLE; /* fallback */
    return (class_label_t)lut->entries[key];
}

/* ── Compiled policy ──────────────────────────────────────────────────────── */

void policy_default(compiled_policy_t *policy)
{
    memset(policy, 0, sizeof(*policy));

    /* Default motor parameters per action */
    policy->motor_params[ACTION_STOP]      = (motor_cmd_t){  0,   0};
    policy->motor_params[ACTION_FORWARD]   = (motor_cmd_t){ 80,  80};
    policy->motor_params[ACTION_REVERSE]   = (motor_cmd_t){-60, -60};
    policy->motor_params[ACTION_TURN_LEFT]  = (motor_cmd_t){-40,  40};
    policy->motor_params[ACTION_TURN_RIGHT] = (motor_cmd_t){ 40, -40};
    policy->motor_params[ACTION_BRAKE]     = (motor_cmd_t){  0,   0};
    policy->motor_params[ACTION_FREEWHEEL] = (motor_cmd_t){  0,   0};

    /* Map (classification, sensor_summary) → action */
    for (int cls = 0; cls < CLASS_COUNT; cls++) {
        for (int s = 0; s < POLICY_SENSOR_PACK_MAX; s++) {
            action_code_t action;
            switch (cls) {
                case CLASS_IDLE:        action = ACTION_FREEWHEEL; break;
                case CLASS_NORMAL:
                    action = (s < 4) ? ACTION_FORWARD : ACTION_TURN_RIGHT;
                    break;
                case CLASS_ALERT:
                    action = (s % 2 == 0) ? ACTION_TURN_LEFT : ACTION_REVERSE;
                    break;
                case CLASS_EMERGENCY:   action = ACTION_BRAKE; break;
                case CLASS_MAINTENANCE: action = ACTION_STOP; break;
                default:                action = ACTION_STOP; break;
            }
            policy->actions[cls][s] = action;
        }
    }
}

action_code_t policy_lookup(const compiled_policy_t *policy,
                            class_label_t classification,
                            uint8_t sensor_summary)
{
    if (classification >= CLASS_COUNT)
        classification = CLASS_IDLE;
    if (sensor_summary >= POLICY_SENSOR_PACK_MAX)
        sensor_summary = 0;
    return policy->actions[classification][sensor_summary];
}

const motor_cmd_t *policy_motor_cmd(const compiled_policy_t *policy,
                                     action_code_t action)
{
    if (action >= ACTION_COUNT)
        action = ACTION_STOP;
    return &policy->motor_params[action];
}
