#pragma once
#include "stm32g0xx.h"


typedef enum ts5a9411_channel_t {
	TS6A9411_CHANNEL_0 = 0,
	TS6A9411_CHANNEL_1 = 1,
} ts5a9411_channel_t;

typedef struct ts5a9411_t {
	GPIO_TypeDef* select_sig_port;
	uint16_t select_sig_pin;
} ts5a9411_t;

void TS6A9411_init(ts5a9411_t* ts_switch, GPIO_TypeDef* select_sig_port, uint16_t select_sig_pin);
void TS6A9411_select_channel(ts5a9411_t* ts_switch, ts5a9411_channel_t channel);
