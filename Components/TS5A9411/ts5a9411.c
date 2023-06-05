#include <ts5a9411.h>

void TS6A9411_init(ts5a9411_t* ts_switch, GPIO_TypeDef* select_sig_port, uint16_t select_sig_pin) {
	ts_switch->select_sig_port = select_sig_port;
	ts_switch->select_sig_pin = select_sig_pin;
	TS6A9411_select_channel(ts_switch, TS6A9411_CHANNEL_0);
}

void TS6A9411_select_channel(ts5a9411_t* ts_switch, ts5a9411_channel_t channel) {
	GPIO_PinState pin_state = (channel == TS6A9411_CHANNEL_0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(ts_switch->select_sig_port, ts_switch->select_sig_pin, pin_state);
}
