# Makefile — Ternary ESP32 Firmware
#
# Targets:
#   make test       Build and run host-side tests
#   make demo       Build and run host demo (simulated sensors)
#   make esp32      Build for ESP32 (requires ESP-IDF or xtensa toolchain)
#   make clean      Remove build artifacts
#   make size       Report binary sizes
#
# ESP32 cross-compilation:
#   Set ESP_IDF_PATH or ESP_TOOLCHAIN_PREFIX to enable xtensa build.
#   Without these, the esp32 target creates a stub showing required config.

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c99 -O2
SRC_DIR = src
BUILD   = build

# Source files
LIB_SRCS = $(SRC_DIR)/ternary_policy.c $(SRC_DIR)/ternary_denoise.c

# ── Host targets ────────────────────────────────────────────────────────────

.PHONY: test demo esp32 clean size all

all: test

test: $(BUILD)/test_policy
	@echo ""
	@echo "Running tests..."
	@./$(BUILD)/test_policy

$(BUILD)/test_policy: tests/test_policy.c $(LIB_SRCS) $(SRC_DIR)/ternary_policy.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ tests/test_policy.c $(LIB_SRCS)

demo: $(BUILD)/demo
	@echo ""
	@./$(BUILD)/demo

$(BUILD)/demo: $(SRC_DIR)/main.c $(LIB_SRCS) $(SRC_DIR)/ternary_policy.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $(SRC_DIR)/main.c $(LIB_SRCS)

# ── ESP32 cross-compile ────────────────────────────────────────────────────

ESP_TOOLCHAIN ?= xtensa-esp32-elf-gcc

esp32:
ifndef ESP_IDF_PATH
	@echo "╔══════════════════════════════════════════════════════════════╗"
	@echo "║  ESP32 Cross-Compile Placeholder                            ║"
	@echo "║                                                              ║"
	@echo "║  To build for ESP32, set ESP_IDF_PATH and run:               ║"
	@echo "║    export ESP_IDF_PATH=/path/to/esp-idf                     ║"
	@echo "║    idf.py build                                              ║"
	@echo "║                                                              ║"
	@echo "║  Or with standalone toolchain:                               ║"
	@echo "║    make esp32 ESP_TOOLCHAIN=xtensa-esp32-elf-gcc            ║"
	@echo "║    make esp32 ESP_IDF_PATH=/path/to/esp-idf                 ║"
	@echo "╚══════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Checking for ESP32 toolchain..."
	@which $(ESP_TOOLCHAIN) 2>/dev/null && \
		echo "Found! Building..." && \
		mkdir -p $(BUILD) && \
		$(ESP_TOOLCHAIN) $(CFLAGS) -I$(SRC_DIR) -DESP_PLATFORM \
			-o $(BUILD)/firmware.elf \
			$(SRC_DIR)/main.c $(LIB_SRCS) || \
		echo "ESP32 toolchain not found. Install ESP-IDF for full build."
else
	@echo "Building with ESP-IDF..."
	@cd $(ESP_IDF_PATH) && idf.py -C $(CURDIR) build
endif

# ── Utilities ───────────────────────────────────────────────────────────────

size: test
	@echo ""
	@echo "═══ Ternary State Sizes ═══"
	@echo "compiled_policy_t:  unknown (run test for details)"
	@echo ""

clean:
	rm -rf $(BUILD)
