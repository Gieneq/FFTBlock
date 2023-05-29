#include "fftslice.h"

void fftslice_init(fftslice_t* fftslice, fftslice_config_t* config) {
	TS6A9411_init(&fftslice->selector, config->ts5a9411_select_sig_port, config->ts5a9411_select_sig_pin);
	MCP6S22_init(&fftslice->amplifier, config->mcp6s22_spi, config->mcp6s22_ncs_port, config->mcp6s22_ncs_pin);
}


HAL_StatusTypeDef fftslice_select_signal_source(fftslice_t* fftslice, fftslice_signal_source_t source) {
	fftslice->source_signal = source;

	//stop DAC if not used
}


HAL_StatusTypeDef fftslice_set_amplifier_gain(fftslice_t* fftslice, fftslice_gain_t gain) {
	fftslice->gain = gain;
	/* is DAC selected divide samples ? */
}


