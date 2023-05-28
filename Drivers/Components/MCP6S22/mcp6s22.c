#include "mcp6s22.h"

#define MCP6S22_WRITE_TO_CHANNEL 0x41
#define MCP6S22_WRITE_TO_GAIN 0x40

#define MCP6S22_MIN_GAIN MCP6S22_GAIN_1
#define MCP6S22_MAX_GAIN MCP6S22_GAIN_32

static HAL_StatusTypeDef MCP6S22_write_reg(mcp6s22_t* mcp, uint8_t instruction, uint8_t reg) {
	uint8_t transmit_data[2] = {instruction, reg};

	HAL_GPIO_WritePin(mcp->ncs_port, mcp->ncs_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef result_status = HAL_SPI_Transmit(mcp->spi_handler, transmit_data, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(mcp->ncs_port, mcp->ncs_pin, GPIO_PIN_SET);

	return result_status;
}

void MCP6S22_init(mcp6s22_t* mcp, SPI_HandleTypeDef* spi_handler, GPIO_TypeDef* ncs_port, uint16_t ncs_pin) {
	mcp->spi_handler = spi_handler;
	mcp->ncs_port = ncs_port;
	mcp->ncs_pin = ncs_pin;
	HAL_GPIO_WritePin(mcp->ncs_port, mcp->ncs_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef MCP6S22_select_channel(mcp6s22_t* mcp, mcp6s22_channel_t channel) {
	if(channel != MCP6S22_CHANNEL_0 || channel != MCP6S22_CHANNEL_1) {
		return HAL_ERROR;
	}
	return MCP6S22_write_reg(mcp, MCP6S22_WRITE_TO_CHANNEL, (uint8_t)channel);
}

HAL_StatusTypeDef MCP6S22_set_gain(mcp6s22_t* mcp, mcp6s22_gain_t gain) {
	if(gain < MCP6S22_MIN_GAIN || gain > MCP6S22_MAX_GAIN) {
		return HAL_ERROR;
	}
	return MCP6S22_write_reg(mcp, MCP6S22_WRITE_TO_GAIN, (uint8_t)gain);
}
