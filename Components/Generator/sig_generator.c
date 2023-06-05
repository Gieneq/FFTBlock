#include <sig_generator.h>
#include <string.h>

void sig_generator_init(sig_generator_t* generator, DAC_HandleTypeDef* dac_handle) {
	generator->dac = dac_handle;
	memset(generator->sig_buffer, 0, sizeof(generator->sig_buffer[0]) * SIG_GENERATOR_SINE_SAMPLES_COUNT);

	for(int i=0; i<SIG_GENERATOR_SINE_SAMPLES_COUNT; ++i) {
		generator->sig_buffer[i] = 1024;
	}
}
