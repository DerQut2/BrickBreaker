/*
 * brickbreakergui.c
 *
 *  Created on: Jun 18, 2025
 *      Author: babel
 */

#include <stdio.h>
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
	for (uint8_t brick_row=1; brick_row < brick_count; brick_row++) {
		if (!bricks[brick_row])
			continue;

		for (uint8_t brick=0; brick<8; brick++) {
			if (bricks[brick_row] & (1<<brick)) {
				blit_brick((uint8_t) BRICK_X_SIZE * brick, SSD1306_HEIGHT - (uint8_t) BRICK_Y_SIZE * brick_row);
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

void blit_score(uint16_t score) {
	char buffer[8];
	uint8_t characters = sprintf(buffer, "%d", score);
	ssd1306_SetCursor(SSD1306_WIDTH - 16 * (characters+1), 32);
	ssd1306_WriteString(buffer, Font_16x26, White);
}

void blit_multiplier(float multiplier) {
	char buffer[16];
	uint8_t characters = sprintf(buffer, "MULT: x%.1f", multiplier);
	ssd1306_SetCursor(SSD1306_WIDTH - 7 * (characters+1), 1);
	ssd1306_WriteString(buffer, Font_7x10, White);
}

void blit_hearts(uint8_t lives, uint8_t lives_max) {
	for (uint8_t heart = 0; heart < lives_max; heart++) {
		ssd1306_DrawPixel(2 + heart*14, 0, White);
		ssd1306_DrawPixel(3 + heart*14, 0, White);
		ssd1306_DrawPixel(7 + heart*14, 0, White);
		ssd1306_DrawPixel(8 + heart*14, 0, White);

		ssd1306_DrawPixel(1 + heart*14, 1, White);
		ssd1306_DrawPixel(4 + heart*14, 1, White);
		ssd1306_DrawPixel(6 + heart*14, 1, White);
		ssd1306_DrawPixel(9 + heart*14, 1, White);

		ssd1306_DrawPixel(0 + heart*14, 2, White);
		ssd1306_DrawPixel(5 + heart*14, 2, White);
		ssd1306_DrawPixel(10 + heart*14, 2, White);

		ssd1306_DrawPixel(0 + heart*14, 3, White);
		ssd1306_DrawPixel(10 + heart*14, 3, White);

		ssd1306_DrawPixel(0 + heart*14, 4, White);
		ssd1306_DrawPixel(10 + heart*14, 4, White);

		ssd1306_DrawPixel(0 + heart*14, 5, White);
		ssd1306_DrawPixel(10 + heart*14, 5, White);

		ssd1306_DrawPixel(1 + heart*14, 6, White);
		ssd1306_DrawPixel(9 + heart*14, 6, White);

		ssd1306_DrawPixel(2 + heart*14, 7, White);
		ssd1306_DrawPixel(8 + heart*14, 7, White);

		ssd1306_DrawPixel(3 + heart*14, 8, White);
		ssd1306_DrawPixel(7 + heart*14, 8, White);

		ssd1306_DrawPixel(4 + heart*14, 9, White);
		ssd1306_DrawPixel(6 + heart*14, 9, White);

		ssd1306_DrawPixel(5 + heart*14, 10, White);

		if (heart < lives) {
			ssd1306_DrawPixel(2 + heart*14, 1, White);
			ssd1306_DrawPixel(3 + heart*14, 1, White);
			ssd1306_DrawPixel(7 + heart*14, 1, White);
			ssd1306_DrawPixel(8 + heart*14, 1, White);

			ssd1306_DrawPixel(1 + heart*14, 2, White);
			ssd1306_DrawPixel(2 + heart*14, 2, White);
			ssd1306_DrawPixel(3 + heart*14, 2, White);
			ssd1306_DrawPixel(4 + heart*14, 2, White);
			ssd1306_DrawPixel(6 + heart*14, 2, White);
			ssd1306_DrawPixel(7 + heart*14, 2, White);
			ssd1306_DrawPixel(8 + heart*14, 2, White);
			ssd1306_DrawPixel(9 + heart*14, 2, White);

			ssd1306_DrawPixel(1 + heart*14, 3, White);
			ssd1306_DrawPixel(2 + heart*14, 3, White);
			ssd1306_DrawPixel(3 + heart*14, 3, White);
			ssd1306_DrawPixel(4 + heart*14, 3, White);
			ssd1306_DrawPixel(5 + heart*14, 3, White);
			ssd1306_DrawPixel(6 + heart*14, 3, White);
			ssd1306_DrawPixel(7 + heart*14, 3, White);
			ssd1306_DrawPixel(8 + heart*14, 3, White);
			ssd1306_DrawPixel(9 + heart*14, 3, White);

			ssd1306_DrawPixel(1 + heart*14, 4, White);
			ssd1306_DrawPixel(2 + heart*14, 4, White);
			ssd1306_DrawPixel(3 + heart*14, 4, White);
			ssd1306_DrawPixel(4 + heart*14, 4, White);
			ssd1306_DrawPixel(5 + heart*14, 4, White);
			ssd1306_DrawPixel(6 + heart*14, 4, White);
			ssd1306_DrawPixel(7 + heart*14, 4, White);
			ssd1306_DrawPixel(8 + heart*14, 4, White);
			ssd1306_DrawPixel(9 + heart*14, 4, White);

			ssd1306_DrawPixel(1 + heart*14, 5, White);
			ssd1306_DrawPixel(2 + heart*14, 5, White);
			ssd1306_DrawPixel(3 + heart*14, 5, White);
			ssd1306_DrawPixel(4 + heart*14, 5, White);
			ssd1306_DrawPixel(5 + heart*14, 5, White);
			ssd1306_DrawPixel(6 + heart*14, 5, White);
			ssd1306_DrawPixel(7 + heart*14, 5, White);
			ssd1306_DrawPixel(8 + heart*14, 5, White);
			ssd1306_DrawPixel(9 + heart*14, 5, White);

			ssd1306_DrawPixel(2 + heart*14, 6, White);
			ssd1306_DrawPixel(3 + heart*14, 6, White);
			ssd1306_DrawPixel(4 + heart*14, 6, White);
			ssd1306_DrawPixel(5 + heart*14, 6, White);
			ssd1306_DrawPixel(6 + heart*14, 6, White);
			ssd1306_DrawPixel(7 + heart*14, 6, White);
			ssd1306_DrawPixel(8 + heart*14, 6, White);

			ssd1306_DrawPixel(3 + heart*14, 7, White);
			ssd1306_DrawPixel(4 + heart*14, 7, White);
			ssd1306_DrawPixel(5 + heart*14, 7, White);
			ssd1306_DrawPixel(6 + heart*14, 7, White);
			ssd1306_DrawPixel(7 + heart*14, 7, White);

			ssd1306_DrawPixel(4 + heart*14, 8, White);
			ssd1306_DrawPixel(5 + heart*14, 8, White);
			ssd1306_DrawPixel(6 + heart*14, 8, White);

			ssd1306_DrawPixel(5 + heart*14, 9, White);
		}
	}
}

void blit_lose_message() {
	ssd1306_SetCursor(26, 27);
	ssd1306_WriteString("): peap noh", Font_7x10, White);
}
