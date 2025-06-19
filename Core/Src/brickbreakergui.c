/*
 * brickbreakergui.c
 *
 *  Created on: Jun 18, 2025
 *      Author: babel
 */


#include "brickbreakergui.h"

void blit_palette(uint8_t x_cord, uint8_t y_cord) {
	for (int16_t x = ( (int16_t) x_cord)-8; x < x_cord+7; x++) {
		for (int16_t y = (int16_t) y_cord; y < y_cord+5; y++) {
			if (x>0 && x < SSD1306_WIDTH)
				ssd1306_DrawPixel((uint8_t)x, (uint8_t)y, White);
		}
	}
}

void blit_ball(uint8_t x_cord, uint8_t y_cord) {
	for (int16_t x = ( (int16_t) x_cord)-2; x < x_cord+2; x++) {
			for (int16_t y = (int16_t) y_cord-2; y < y_cord+2; y++) {
				if (x>0 && x< SSD1306_WIDTH && x>0 && y < SSD1306_HEIGHT)
					ssd1306_DrawPixel((uint8_t)x, (uint8_t)y, White);
			}
	}
}

void blit_bricks(uint8_t bricks[], uint8_t brick_count) {
	for (uint8_t i=0; i < brick_count; i++) {
		for (uint8_t j=0; j<8; j++) {
			if (bricks[i] & (1<<j)) {
				blit_brick((uint8_t) BRICK_X_SIZE * j, SSD1306_HEIGHT - (uint8_t) BRICK_Y_SIZE * i);
				//ssd1306_DrawPixel( (uint8_t) BRICK_X_SIZE * j, SSD1306_HEIGHT - (uint8_t) BRICK_Y_SIZE * i - 1, White);
			}
		}
	}
}

void blit_brick(uint8_t x, uint8_t y) {
	for (uint8_t i=1; i<BRICK_X_SIZE-1; i++) {
		for (uint8_t j=1; j<BRICK_Y_SIZE-1; j++) {
			ssd1306_DrawPixel(x+i, y+j, White);
		}
	}
}


void blit_main_splash_screen() {
	ssd1306_Fill(White);
}

void blit_secondary_splash_screen() {
	ssd1306_Fill(White);
	ssd1306_SetCursor(25, 0);
	ssd1306_WriteString("Marcel", Font_11x18, Black);

	ssd1306_SetCursor(9, 20);
	ssd1306_WriteString("Cholodecki", Font_11x18, Black);

	ssd1306_SetCursor(43, 50);
	ssd1306_WriteString("275818", Font_7x10, Black);
}
