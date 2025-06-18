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
			if (x>0 && x < SSD1306_WIDTH)
				ssd1306_DrawPixel((uint8_t)x, (uint8_t)y, White);
		}
	}
}

void blit_ball(I2C_HandleTypeDef *hi2c, uint8_t x_cord, uint8_t y_cord) {
	for (int16_t x = ( (int16_t) x_cord)-2; x < x_cord+2; x++) {
			for (int16_t y = (int16_t) y_cord-2; y < y_cord+2; y++) {
				if (x>0 && x< SSD1306_WIDTH && x>0 && y < SSD1306_HEIGHT)
					ssd1306_DrawPixel((uint8_t)x, (uint8_t)y, White);
			}
	}
}


void blit_splash_screen(I2C_HandleTypeDef *hi2c) {
	ssd1306_Fill(White);
}
