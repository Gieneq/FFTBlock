#pragma once
#include "stm32g0xx.h"

#define SIG_GENERATOR_SINE_SAMPLES_COUNT  (1024)
#define SIG_GENERATOR_MAX_AMPLITUDE 3.3F
#define SIG_GENERATOR_MAX_DAC_VALUE 4095
#define SIG_GENERATOR_SAMPING_RATE 60000

typedef enum sig_generator_state_t {
	SIG_GENERATOR_OFF = 0,
	SIG_GENERATOR_ON
} sig_generator_state_t;



typedef struct sig_generator_t {
	DAC_HandleTypeDef* dac;
	TIM_HandleTypeDef* event_timer;
	float sig_amplitude;
	float sig_mean;
	float sig_frequency;
	sig_generator_state_t state;
	uint16_t dac_min_value;
	uint16_t dac_max_value;
	uint16_t sig_buffer[SIG_GENERATOR_SINE_SAMPLES_COUNT];
} sig_generator_t;

int sig_generator_init(sig_generator_t* generator, DAC_HandleTypeDef* dac_handle, TIM_HandleTypeDef* tim_handle);
