#pragma once
#include "stm32g0xx.h"
#include "mcp6s22.h"
#include "sig_generator.h"
#include "ts5a9411.h"



typedef enum fftslice_signal_source_t {
	FFTSLICE_SOURCE_EXTERNAL_0 = 0,
	FFTSLICE_SOURCE_EXTERNAL_1,
	FFTSLICE_SOURCE_SELF_DAC
} fftslice_signal_source_t;


typedef enum fftslice_gain_t {
	FFTSLICE_GAIN_1 = MCP6S22_GAIN_1,
} fftslice_gain_t;


typedef struct fftslice_config_t {
	GPIO_TypeDef* ts5a9411_select_sig_port;
	uint16_t ts5a9411_select_sig_pin;

	SPI_HandleTypeDef* mcp6s22_spi;
	GPIO_TypeDef* mcp6s22_ncs_port;
	uint16_t mcp6s22_ncs_pin;

} fftslice_config_t;


typedef struct fftslice_t {
	mcp6s22_t amplifier;
	ts5a9411_t selector;
	fftslice_signal_source_t source_signal;
	fftslice_gain_t gain;
	sig_generator_t generator;
} fftslice_t;


void fftslice_init(fftslice_t* fftslice, fftslice_config_t* config);

HAL_StatusTypeDef fftslice_signal_source(fftslice_t* fftslice, fftslice_signal_source_t source);

/* Set gain only for external sources. For self DAC set max gain to get max amplitude. */
HAL_StatusTypeDef fftslice_set_amplifier_gain(fftslice_t* fftslice, fftslice_gain_t gain);
