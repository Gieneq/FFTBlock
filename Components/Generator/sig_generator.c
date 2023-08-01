#include "sig_generator.h"
#include <string.h>
#include "stm32g0xx_hal_dac.h"
#include "stm32g0xx_hal_tim.h"
#include <math.h>
#include "dac_sines.h"

#define PI_TIMES_2 (6.28318530718F)

int sig_generator_init(sig_generator_t* generator, DAC_HandleTypeDef* dac_handle, TIM_HandleTypeDef* tim_handle) {
	HAL_StatusTypeDef res;
	generator->dac = dac_handle;
	generator->event_timer = tim_handle;

	/* DAC samples */
	memset(generator->sig_buffer, 0, sizeof(generator->sig_buffer[0]) * SIG_GENERATOR_SINE_SAMPLES_COUNT);
	for(int i=0; i<SIG_GENERATOR_SINE_SAMPLES_COUNT; ++i) {
		generator->sig_buffer[i] = (uint16_t)((1.0F + sinf(PI_TIMES_2 * 1.0F * i / SIG_GENERATOR_SINE_SAMPLES_COUNT)) * 4095.0F/2.0F);
	}

	sig_generator_set(generator, 0);

	return 0;
}

void sig_generator_set(sig_generator_t* generator, int frequency_index) {
	if((frequency_index < 0) || (frequency_index >= DAC_FREQ_STEPS)) {
		return;
	}

	//todo enque change for next DMA transfer end to avoid tearing

	const sinewave_data_t* sine_data = &dac_samples_data[frequency_index];

	HAL_StatusTypeDef res;
	res = HAL_DAC_Stop_DMA(generator->dac, DAC_CHANNEL_1);


	res = HAL_DAC_Start_DMA(generator->dac, DAC_CHANNEL_1, (uint32_t*)sine_data->mem_ptr, sine_data->samples_count, DAC_ALIGN_12B_R);

	__HAL_TIM_SET_AUTORELOAD(generator->event_timer, (sine_data->divider - 1));
	res = HAL_TIM_Base_Start(generator->event_timer);

	UNUSED(res);
}
