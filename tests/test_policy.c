/*
 * test_policy.c — Host-side tests for ternary policy engine
 *
 * Compile and run on development machine (not ESP32).
 * Tests cover: trit packing, majority filter, classifier LUT, policy lookup,
 * edge cases, and round-trip correctness.
 *
 * Build: make test
 * Run:   ./test_policy
 */

#include "ternary_policy.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) printf("  TEST %-50s ", name);
#define PASS() do { printf("PASS\n"); tests_passed++; } while(0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); tests_failed++; } while(0)
#define ASSERT_EQ(a, b, msg) do { \
    if ((a) != (b)) { FAIL(msg); return; } \
} while(0)

/* ── Test 1: Trit pack single trit ────────────────────────────────────────── */
static void test_pack_single_trit(void)
{
    TEST("pack single trit (NEG)");
    uint8_t key;
    trit_t t[] = { TRIT_NEG };
    int r = trit_pack(t, 1, &key);
    ASSERT_EQ(r, 0, "pack should succeed");
    ASSERT_EQ(key, 0, "NEG should pack to 0");
    PASS();
}

/* ── Test 2: Trit pack all zeros ──────────────────────────────────────────── */
static void test_pack_all_zero(void)
{
    TEST("pack all zeros");
    uint8_t key;
    trit_t t[] = { TRIT_ZERO, TRIT_ZERO, TRIT_ZERO, TRIT_ZERO };
    int r = trit_pack(t, 4, &key);
    ASSERT_EQ(r, 0, "pack should succeed");
    /* 1*1 + 1*3 + 1*9 + 1*27 = 40 */
    ASSERT_EQ(key, 40, "all zeros should pack to 40");
    PASS();
}

/* ── Test 3: Trit pack all positive ───────────────────────────────────────── */
static void test_pack_all_pos(void)
{
    TEST("pack all positive");
    uint8_t key;
    trit_t t[] = { TRIT_POS, TRIT_POS, TRIT_POS, TRIT_POS };
    int r = trit_pack(t, 4, &key);
    ASSERT_EQ(r, 0, "pack should succeed");
    /* 2*1 + 2*3 + 2*9 + 2*27 = 80 */
    ASSERT_EQ(key, 80, "all pos should pack to 80");
    PASS();
}

/* ── Test 4: Trit pack/unpack round-trip ──────────────────────────────────── */
static void test_pack_unpack_roundtrip(void)
{
    TEST("pack/unpack round-trip (4 trits)");
    trit_t original[] = { TRIT_NEG, TRIT_POS, TRIT_ZERO, TRIT_NEG };
    uint8_t key;
    trit_pack(original, 4, &key);

    trit_t restored[4];
    trit_unpack(key, restored, 4);

    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(restored[i], original[i], "round-trip mismatch");
    }
    PASS();
}

/* ── Test 5: Trit pack too many trits ─────────────────────────────────────── */
static void test_pack_too_many(void)
{
    TEST("pack rejects >5 trits");
    uint8_t key;
    trit_t t[6] = {0};
    int r = trit_pack(t, 6, &key);
    ASSERT_EQ(r, -1, "should reject 6 trits");
    PASS();
}

/* ── Test 6: ADC to trit conversion ───────────────────────────────────────── */
static void test_adc_to_trit(void)
{
    TEST("ADC to trit conversion");
    ASSERT_EQ(adc_to_trit(500, 1000, 3000), TRIT_NEG, "low ADC → NEG");
    ASSERT_EQ(adc_to_trit(2000, 1000, 3000), TRIT_ZERO, "mid ADC → ZERO");
    ASSERT_EQ(adc_to_trit(3500, 1000, 3000), TRIT_POS, "high ADC → POS");
    ASSERT_EQ(adc_to_trit(1000, 1000, 3000), TRIT_NEG, "at lo_thresh → NEG");
    ASSERT_EQ(adc_to_trit(3000, 1000, 3000), TRIT_POS, "at hi_thresh → POS");
    PASS();
}

/* ── Test 7: Majority filter initialization ───────────────────────────────── */
static void test_majority_filter_init(void)
{
    TEST("majority filter init");
    majority_filter_t f;
    majority_filter_init(&f);
    ASSERT_EQ(f.index, 0, "index should be 0");
    ASSERT_EQ(f.filled, 0, "filled should be 0");
    PASS();
}

/* ── Test 8: Majority filter all same ─────────────────────────────────────── */
static void test_majority_filter_all_same(void)
{
    TEST("majority filter all POS");
    majority_filter_t f;
    majority_filter_init(&f);
    ternary_sensor_t s = { .values = { TRIT_POS, TRIT_POS, TRIT_POS, TRIT_POS } };

    for (int i = 0; i < MAJORITY_FILTER_SIZE; i++)
        majority_filter_push(&f, &s);

    ternary_sensor_t result;
    majority_filter_result(&f, &result);

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++)
        ASSERT_EQ(result.values[i], TRIT_POS, "should be POS");
    PASS();
}

/* ── Test 9: Majority filter majority wins ────────────────────────────────── */
static void test_majority_filter_majority(void)
{
    TEST("majority filter: 3 NEG + 2 POS → NEG");
    majority_filter_t f;
    majority_filter_init(&f);
    ternary_sensor_t neg = { .values = { TRIT_NEG, TRIT_NEG, TRIT_NEG, TRIT_NEG } };
    ternary_sensor_t pos = { .values = { TRIT_POS, TRIT_POS, TRIT_POS, TRIT_POS } };

    /* Push: NEG, NEG, POS, NEG, POS → sum per channel = −1 → NEG */
    majority_filter_push(&f, &neg);
    majority_filter_push(&f, &neg);
    majority_filter_push(&f, &pos);
    majority_filter_push(&f, &neg);
    majority_filter_push(&f, &pos);

    ternary_sensor_t result;
    majority_filter_result(&f, &result);

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++)
        ASSERT_EQ(result.values[i], TRIT_NEG, "NEG should win (3 of 5)");
    PASS();
}

/* ── Test 10: Majority filter tie → ZERO ──────────────────────────────────── */
static void test_majority_filter_tie(void)
{
    TEST("majority filter: tie → ZERO");
    majority_filter_t f;
    majority_filter_init(&f);
    ternary_sensor_t neg = { .values = { TRIT_NEG, TRIT_NEG, TRIT_NEG, TRIT_NEG } };
    ternary_sensor_t pos = { .values = { TRIT_POS, TRIT_POS, TRIT_POS, TRIT_POS } };

    /* 2 NEG + 2 POS + 1 ZERO (initial) → 2(-1) + 2(+1) + 1(0) = 0 → ZERO */
    majority_filter_push(&f, &neg);
    majority_filter_push(&f, &pos);
    majority_filter_push(&f, &neg);
    majority_filter_push(&f, &pos);

    ternary_sensor_t result;
    majority_filter_result(&f, &result);

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++)
        ASSERT_EQ(result.values[i], TRIT_ZERO, "tie should resolve to ZERO");
    PASS();
}

/* ── Test 11: Classifier LUT completeness ─────────────────────────────────── */
static void test_classifier_lut_complete(void)
{
    TEST("classifier LUT has all 81 entries valid");
    classifier_lut_t lut;
    classifier_lut_default(&lut);

    for (int i = 0; i < CLASSIFIER_LUT_SIZE; i++) {
        ASSERT_EQ(lut.entries[i] < CLASS_COUNT, 1, "entry out of range");
    }
    PASS();
}

/* ── Test 12: Classifier: all NEG → EMERGENCY ────────────────────────────── */
static void test_classify_all_neg(void)
{
    TEST("classify: all NEG → EMERGENCY");
    classifier_lut_t lut;
    classifier_lut_default(&lut);

    ternary_sensor_t s = { .values = { TRIT_NEG, TRIT_NEG, TRIT_NEG, TRIT_NEG } };
    class_label_t cls = classify(&lut, &s);
    ASSERT_EQ(cls, CLASS_EMERGENCY, "all NEG should be EMERGENCY");
    PASS();
}

/* ── Test 13: Classifier: all ZERO → IDLE ─────────────────────────────────── */
static void test_classify_all_zero(void)
{
    TEST("classify: all ZERO → IDLE");
    classifier_lut_t lut;
    classifier_lut_default(&lut);

    ternary_sensor_t s = { .values = { TRIT_ZERO, TRIT_ZERO, TRIT_ZERO, TRIT_ZERO } };
    class_label_t cls = classify(&lut, &s);
    ASSERT_EQ(cls, CLASS_IDLE, "all ZERO should be IDLE");
    PASS();
}

/* ── Test 14: Policy lookup returns correct action ────────────────────────── */
static void test_policy_lookup_basic(void)
{
    TEST("policy lookup: EMERGENCY → BRAKE");
    compiled_policy_t policy;
    policy_default(&policy);

    action_code_t act = policy_lookup(&policy, CLASS_EMERGENCY, 0);
    ASSERT_EQ(act, ACTION_BRAKE, "EMERGENCY should BRAKE");
    PASS();
}

/* ── Test 15: Policy motor commands match action ──────────────────────────── */
static void test_policy_motor_cmd(void)
{
    TEST("policy motor cmd: FORWARD → positive speeds");
    compiled_policy_t policy;
    policy_default(&policy);

    const motor_cmd_t *cmd = policy_motor_cmd(&policy, ACTION_FORWARD);
    ASSERT_EQ(cmd->left_speed > 0, 1, "forward left should be positive");
    ASSERT_EQ(cmd->right_speed > 0, 1, "forward right should be positive");
    PASS();
}

/* ── Test 16: Policy lookup bounds safety ─────────────────────────────────── */
static void test_policy_lookup_bounds(void)
{
    TEST("policy lookup: out-of-bounds → safe fallback");
    compiled_policy_t policy;
    policy_default(&policy);

    /* classification out of range */
    action_code_t act = policy_lookup(&policy, (class_label_t)99, 0);
    ASSERT_EQ(act, ACTION_FREEWHEEL, "bad class → IDLE default → FREEWHEEL");

    /* sensor_summary out of range */
    act = policy_lookup(&policy, CLASS_NORMAL, 99);
    ASSERT_EQ(act == ACTION_FORWARD || act == ACTION_TURN_RIGHT, 1,
              "clamped sensor should be valid");
    PASS();
}

/* ── Test 17: Classifier LUT total size ───────────────────────────────────── */
static void test_binary_sizes(void)
{
    TEST("binary sizes within 15KB budget");
    size_t total = sizeof(compiled_policy_t) + sizeof(classifier_lut_t)
                 + sizeof(majority_filter_t);
    printf("[%zu bytes] ", total);
    ASSERT_EQ(total < 15360, 1, "total ternary state must be < 15KB");
    PASS();
}

/* ── Test 18: Pack every possible 4-trit key is unique ────────────────────── */
static void test_pack_unique_keys(void)
{
    TEST("all 81 4-trit combos produce unique keys");
    uint8_t seen[CLASSIFIER_LUT_SIZE];
    memset(seen, 0, sizeof(seen));

    for (int key = 0; key < CLASSIFIER_LUT_SIZE; key++) {
        trit_t t[4];
        trit_unpack((uint8_t)key, t, 4);

        uint8_t repacked;
        trit_pack(t, 4, &repacked);
        ASSERT_EQ(repacked, (uint8_t)key, "round-trip key mismatch");

        seen[repacked] = 1;
    }

    for (int i = 0; i < CLASSIFIER_LUT_SIZE; i++)
        ASSERT_EQ(seen[i], 1, "not all keys visited");
    PASS();
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("═══ Ternary Policy Tests ═══\n\n");

    test_pack_single_trit();
    test_pack_all_zero();
    test_pack_all_pos();
    test_pack_unpack_roundtrip();
    test_pack_too_many();
    test_adc_to_trit();
    test_majority_filter_init();
    test_majority_filter_all_same();
    test_majority_filter_majority();
    test_majority_filter_tie();
    test_classifier_lut_complete();
    test_classify_all_neg();
    test_classify_all_zero();
    test_policy_lookup_basic();
    test_policy_motor_cmd();
    test_policy_lookup_bounds();
    test_binary_sizes();
    test_pack_unique_keys();

    printf("\n═══ Results: %d passed, %d failed ═══\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
