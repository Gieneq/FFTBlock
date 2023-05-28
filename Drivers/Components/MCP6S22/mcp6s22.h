#pragma once
#include "stm32g0xx.h"


typedef enum mcp6s22_channel_t {
	MCP6S22_CHANNEL_0 = 0,
	MCP6S22_CHANNEL_1 = 1,
} mcp6s22_channel_t;

typedef enum mcp6s22_gain_t {
	MCP6S22_GAIN_1 = 0,
	MCP6S22_GAIN_2 = 1,
	MCP6S22_GAIN_4 = 2,
	MCP6S22_GAIN_5 = 3,
	MCP6S22_GAIN_8 = 4,
	MCP6S22_GAIN_10 = 5,
	MCP6S22_GAIN_16 = 6,
	MCP6S22_GAIN_32 = 7,
} mcp6s22_gain_t;

typedef struct mcp6s22_t {
	SPI_HandleTypeDef* spi_handler;
	GPIO_TypeDef* ncs_port;
	uint16_t ncs_pin;
} mcp6s22_t;

void MCP6S22_init(mcp6s22_t* mcp, SPI_HandleTypeDef* spi_handler, GPIO_TypeDef* ncs_port, uint16_t ncs_pin);
HAL_StatusTypeDef MCP6S22_select_channel(mcp6s22_t* mcp, mcp6s22_channel_t channel);
HAL_StatusTypeDef MCP6S22_set_gain(mcp6s22_t* mcp, mcp6s22_gain_t gain);
