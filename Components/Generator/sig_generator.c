#include <sig_generator.h>
#include <string.h>
#include "stm32g0xx_hal_dac.h"
#include "stm32g0xx_hal_tim.h"
#include <math.h>

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


	/* DAC output */
	res = HAL_DAC_Start_DMA(generator->dac, DAC_CHANNEL_1, (uint32_t*)generator->sig_buffer, SIG_GENERATOR_SINE_SAMPLES_COUNT, DAC_ALIGN_12B_R);
	if(res != HAL_OK) {
		return -1;
	}


	//1024 sampkes, 313 div -> 199.7Hz

	/* Timer controling DAC output */
	__HAL_TIM_SET_AUTORELOAD(generator->event_timer, (313-1));
	res = HAL_TIM_Base_Start(generator->event_timer);
	if(res != HAL_OK) {
		return -1;
	}


	return 0;
}
