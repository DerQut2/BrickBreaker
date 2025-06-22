/*
 * brickbreakergui.h
 *
 *  Created on: Jun 18, 2025
 *      Author: babel
 */

#ifndef INC_BRICKBREAKERGUI_H_
#define INC_BRICKBREAKERGUI_H_

#include "stm32f4xx_hal.h"
#include "ssd1306.h"

#define BRICK_X_SIZE 16
#define BRICK_Y_SIZE 6

void blit_main_splash_screen();
void blit_secondary_splash_screen();

void blit_palette(uint8_t x_cord, uint8_t y_cord);
void blit_ball(uint8_t x_cord, uint8_t y_cord);

void blit_bricks(uint8_t bricks[], uint8_t brick_count);

void blit_brick(uint8_t x, uint8_t y);

void blit_score(uint16_t score);
void blit_multiplier(float multiplier);
void blit_hearts(uint8_t lives, uint8_t lives_max);


#endif /* INC_BRICKBREAKERGUI_H_ */
