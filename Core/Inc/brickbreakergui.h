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

void blit_splash_screen(I2C_HandleTypeDef *hi2c);

void blit_palette(I2C_HandleTypeDef *hi2c, uint8_t x_cord, uint8_t y_cord);
void blit_ball(I2C_HandleTypeDef *hi2c, uint8_t x_cord, uint8_t y_cord);


#endif /* INC_BRICKBREAKERGUI_H_ */
