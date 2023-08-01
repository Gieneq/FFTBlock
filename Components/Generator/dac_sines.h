#pragma once
#include <stdint.h>

#define DAC_SINES_COUNT 6
#define DAC_FREQ_STEPS 100
#define DAC_SAMPLES_VALUES_COUNT 4032

typedef struct sinewave_data_t {
	uint32_t start_idx;
	const uint16_t* mem_ptr;
	uint32_t samples_count;
	uint32_t divider;
  float frequency_hz;
} sinewave_data_t;

extern const uint16_t dac_samples_values[DAC_SAMPLES_VALUES_COUNT];
extern const sinewave_data_t dac_samples_data[DAC_FREQ_STEPS];
