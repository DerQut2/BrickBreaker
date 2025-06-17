/*
 * brickbreakergui.c
 *
 *  Created on: Jun 18, 2025
 *      Author: babel
 */


#include "brickbreakergui.h"

void blit_palette(I2C_HandleTypeDef *hi2c, uint8_t x_cord, uint8_t y_cord) {
	for (int16_t x = ( (int16_t) x_cord)-8; x < x_cord+7; x++) {
		for (int16_t y = (int16_t) y_cord; y < y_cord+5; y++) {
			if (x>0 && x<64)
				ssd1306_DrawPixel((uint8_t)y, (uint8_t)x, White);
		}
	}
}
