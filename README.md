# Ternary ESP32 Firmware

Proof-of-concept: running the ternary decision system on bare metal (ESP32).

Compiled-policy lookup tables — generated on a Raspberry Pi — deployed as <15KB firmware on a microcontroller.

## The Pipeline

```
┌─────────────┐     ┌──────────────┐     ┌──────────────────┐
│  Raspberry  │     │ compiled-    │     │     ESP32        │
│     Pi      │────▶│ policy-c     │────▶│   firmware       │
│ (training)  │     │ (<15KB)      │     │  (lookup only)   │
└─────────────┘     └──────────────┘     └──────────────────┘
   Strategy          C header/byte           8ns lookup
   optimization      array compiled          per decision
   + evolution       from strategy           at 240 MHz
```

### How It Works

1. **On the Pi**: Strategies are evolved/optimized using ternary logic (three-valued: −1, 0, +1). The best strategy is compiled into a flat C lookup table — a `compiled_policy_t` struct.

2. **Compilation**: The `compiled-policy-c` tool converts the strategy into a byte array. Total binary footprint: under 15KB.

3. **On the ESP32**: The firmware does **no learning** — it's pure lookup:
   - Read sensors (ADC)
   - Convert to trits (threshold comparison)
   - Denoise (majority filter over 5 samples)
   - Classify (81-entry lookup table, 4 trits → class)
   - Decide action (policy table: class × sensor_summary → action)
   - Output motor commands (PWM)

Each decision takes ~8ns on the ESP32's 240 MHz processor. The full 100Hz control loop uses <100µs — leaving 99.9% CPU free for WiFi, BLE, or other tasks.

## Project Structure

```
src/
  main.c              ESP32 main loop (simulated sensors on host)
  ternary_policy.h    Types, trit encoding, API declarations
  ternary_policy.c    Pure C implementation (no ESP-IDF dependency)
  ternary_denoise.c   ADC→trit conversion + sensor packing
tests/
  test_policy.c       18 host-side tests
Makefile              Builds for host (test) and ESP32 (cross-compile)
```

## Building

### Host Tests

```bash
make test
```

This compiles and runs 18 tests covering trit packing, majority filtering, classification, policy lookup, edge cases, and binary size budgets.

### Host Demo

```bash
make demo
```

Runs the simulated sensor loop — cycles through patterns and prints classification/decisions.

### ESP32 Cross-Compile

```bash
# With ESP-IDF:
export ESP_IDF_PATH=/path/to/esp-idf
make esp32

# With standalone xtensa toolchain:
make esp32 ESP_TOOLCHAIN=xtensa-esp32-elf-gcc
```

## Ternary Concepts

### Trits
A **trit** is a ternary digit with three states: −1 (negative), 0 (neutral), +1 (positive). This gives richer signal representation than binary for sensor classification.

### Majority Filter (Denoising)
Raw sensor readings are noisy. The majority filter keeps a sliding window of 5 samples and outputs the trit with the highest vote count. Ties resolve to 0 (neutral). This eliminates transient spikes without complex DSP.

### Classifier LUT
4 sensor trits → packed into a base-3 key (0–80) → lookup 81-entry table → class label. The table is generated on the Pi from evolved strategies.

### Compiled Policy
A 2D lookup: (classification × sensor_summary) → action code → motor parameters. Total size: ~54 bytes. Lookup time: 2 CPU instructions.

## Memory Footprint

| Component | Size |
|-----------|------|
| `compiled_policy_t` | ~90 bytes |
| `classifier_lut_t` | 81 bytes |
| `majority_filter_t` | ~24 bytes |
| **Total ternary state** | **<200 bytes** |
| Full firmware binary | **<15KB** |

## License

MIT
