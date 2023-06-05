#include "fftslice.h"
#include "dac.h"
#include "spi.h"
#include "gpio.h"

void fftslice_init(fftslice_t* fftslice, fftslice_config_t* config) {
	/* Init HAL perips */
	MX_DAC1_Init();
	//todo

	/* Init drivers */
	TS6A9411_init(&fftslice->selector, config->ts5a9411_select_sig_port, config->ts5a9411_select_sig_pin);
	MCP6S22_init(&fftslice->amplifier, config->mcp6s22_spi, config->mcp6s22_ncs_port, config->mcp6s22_ncs_pin);
	sig_generator_init(&fftslice->generator, &hdac1);

	HAL_StatusTypeDef res;
	res = HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)fftslice->generator.sig_buffer, SIG_GENERATOR_SINE_SAMPLES_COUNT, DAC_ALIGN_12B_R);

	/* Init registers */
}


HAL_StatusTypeDef fftslice_select_signal_source(fftslice_t* fftslice, fftslice_signal_source_t source) {
	fftslice->source_signal = source;

	//stop DAC if not used
	return HAL_OK;
}


HAL_StatusTypeDef fftslice_set_amplifier_gain(fftslice_t* fftslice, fftslice_gain_t gain) {
	fftslice->gain = gain;
	/* is DAC selected divide samples ? */
	return HAL_OK;
}


