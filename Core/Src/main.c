/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include "fonts.h"
#include "string.h"
#include "brickbreakergui.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint8_t player_pos = 0;

volatile int16_t ball_x_pos = 0;
volatile int16_t ball_y_pos = 32;

volatile int8_t ball_x_speed = 0;
volatile int8_t ball_y_speed = -1;

volatile uint8_t time_since_last_x_bounce = 10;
volatile uint8_t time_since_last_y_bounce = 10;

volatile uint8_t time_since_last_brick_break = 10;

volatile uint16_t score = 0;
volatile float multiplier = 0.5f;

volatile uint8_t secondary_screen_changed = 1;

volatile uint8_t lives = LIFE_COUNT;

volatile uint8_t current_sound = NONE;

volatile uint8_t has_already_played_lose_sound = 0;

uint8_t bricks[BRICK_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  bricks[0] = 0x00;
  for (uint8_t i=1; i < BRICK_COUNT; i++)
	  bricks[i] = 0xFF;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (ssd1306_Init(&hi2c1) != 0 || ssd1306_Init(&hi2c2) != 0) {
      Error_Handler();
  }
  HAL_Delay(5);

  blit_main_splash_screen();
  ssd1306_UpdateScreen(&hi2c1);


  blit_secondary_splash_screen();
  ssd1306_UpdateScreen(&hi2c2);

  // Start ADC
  HAL_ADC_Start_IT(&hadc1);

  HAL_Delay(2000);

  ball_x_pos = player_pos;

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ball_x_pos += ball_x_speed;
	  ball_y_pos += ball_y_speed;

	  if (ball_x_pos < 0)
		  ball_x_pos = 0;

	  if (ball_x_pos > SSD1306_WIDTH)
		  ball_x_pos = SSD1306_WIDTH;

	  if (ball_y_pos < 0)
		  ball_y_pos = 0;

	  if (ball_y_pos > SSD1306_HEIGHT)
		  ball_y_pos = SSD1306_HEIGHT;

	  time_since_last_x_bounce++;
	  time_since_last_y_bounce++;

	  time_since_last_brick_break++;

	  calculate_ball_speed();
	  brick_check();
	  kill_check();

	  // Main screen drawing
	  ssd1306_Fill(Black);

	  blit_palette((uint8_t) (player_pos), 12);
	  blit_ball(ball_x_pos, ball_y_pos);

	  blit_bricks(bricks, BRICK_COUNT);

	  ssd1306_UpdateScreen(&hi2c1);

	  // Secondary screen drawing
	  if (secondary_screen_changed) {
		  ssd1306_Fill(Black);

		  blit_score(score);
		  blit_multiplier(multiplier);
		  blit_hearts(lives, LIFE_COUNT);

		  ssd1306_UpdateScreen(&hi2c2);
		  secondary_screen_changed = 0;
	  }

	  HAL_ADC_Start_IT(&hadc1);
	  HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 95999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 255;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 16000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Read & Update The ADC Result
    player_pos = (uint8_t) (HAL_ADC_GetValue(&hadc1)/2);
}


void calculate_ball_speed() {
	if (!lives)
		return;

	if (((ball_x_pos > SSD1306_WIDTH-4 && ball_x_speed > 0) || (ball_x_pos < 4 && ball_x_speed < 0)) && time_since_last_x_bounce > 5) {
		ball_x_speed = ball_x_speed * (-1);
		time_since_last_x_bounce = 0;
		current_sound = BOUNCE;
	}

	if (((ball_y_pos > SSD1306_HEIGHT-4 && ball_y_speed > 0) || (ball_y_pos < 21 && ball_y_speed < 0 && (ball_x_pos > player_pos-12 && ball_x_pos < player_pos+12))) && time_since_last_y_bounce > 5) {
		ball_y_speed = ball_y_speed * (-1);
		time_since_last_y_bounce = 0;
		current_sound = BOUNCE;

		if (ball_y_pos < 21) {
			ball_x_speed = (uint8_t) ((ball_x_pos - player_pos)/2);
			if (ball_x_speed > 4)
				ball_x_speed = 4;

			if (ball_x_speed < 3)
				ball_y_speed = 2;
			else
				ball_y_speed = 1;

			ball_x_speed = ball_x_speed * multiplier;
			current_sound = BOUNCE;

			if (ball_x_speed == 0) {
				ball_x_speed = 1;
			}
		}
	}
}

void kill_check() {
	if ((ball_y_pos < 17 && !(ball_x_pos > player_pos-16 && ball_x_pos < player_pos+16)) || (ball_y_pos < 10)) {

		if (lives > 0) {
			lives--;
			secondary_screen_changed = 1;
		}

		if (lives > 0) {
			ball_y_pos = 32;
			ball_x_pos = player_pos;
			ball_x_speed = 0;
			ball_y_speed = -1;

			// Play the DEATH sound
			current_sound = DEATH;
		} else {
			ball_x_speed = 0;
			ball_y_speed = 0;

			// Play the LOSE sound
			if (!has_already_played_lose_sound) {
				current_sound = LOSE;
				has_already_played_lose_sound = 1;
			}
		}

	}
}

void brick_check() {

	// Sweep all brick rows
	for (uint8_t brick_row=0; brick_row<BRICK_COUNT; brick_row++) {

		// Check if brick row contains any bricks
		if (!bricks[brick_row])
			continue;

		// Sweep all bricks in a given row
		for (uint8_t brick=0; brick<8; brick++) {

			// Check if a given brick exists
			if (!(bricks[brick_row] & (1 << brick)))
				continue;

			// Check if ball x position aligns with the brick
			if (!(ball_x_pos+2 > brick * BRICK_X_SIZE && ball_x_pos < (brick+1) * BRICK_X_SIZE+2))
				continue;

			// Check if ball y position aligns with the brick
			if (!(ball_y_pos-2 < SSD1306_HEIGHT - brick_row * BRICK_Y_SIZE && ball_y_pos+2 > SSD1306_HEIGHT - (brick_row+1) * BRICK_Y_SIZE))
				continue;

			// Check if the ball can break the brick (cool-down)
			if (!(time_since_last_brick_break>5 || (ball_y_speed < 0 && time_since_last_brick_break > 0)))
				continue;

			// Remove the brick (XOR with 1 sets to 0- the given value was already asserted to be 1)
			bricks[brick_row] = bricks[brick_row] ^ (1<<brick);

			// Increase score
			score += BRICK_REWARD * multiplier;
			secondary_screen_changed = 1;

			// Reset the cool-down
			time_since_last_brick_break = 0;

			// Play the brick sounds
			current_sound = BRICK;

			// Check for win
			win_check();

			// Check if the ball is not falling and can be made to fall
			if (!(time_since_last_y_bounce > 2 && ball_y_speed > 0))
				continue;

			// Force the ball to start falling (trigger a y-bounce)
			time_since_last_y_bounce = 0;
			ball_y_speed = ball_y_speed * (-1);
		}
	}
}

void win_check() {

	// Sweep all brick rows
	for (uint8_t brick_row = 1; brick_row < BRICK_COUNT; brick_row++) {

		// Check if any row has at least one brick- if yes, stop checking for win
		if (bricks[brick_row] != 0x00) {
			return;
		}
	}

	// Rebuild the bricks
	for (uint8_t brick_row = 1; brick_row < BRICK_COUNT; brick_row++) {
		bricks[brick_row] = 0xFF;
	}

	// Increase the score/speed multiplier
	multiplier += 0.5f;

	// Reset the ball position and speed
	ball_x_pos = player_pos;
	ball_y_pos = 32;
	ball_x_speed = 0;
	ball_y_speed = -1;

	// Reward the player
	score += 100;

	// Play the win sound
	current_sound = WIN;

	// Refresh the secondary screen
	secondary_screen_changed = 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Decrement score each second
	if (htim == &htim2) {
		if (multiplier >= 1 && lives > 0) {
			if (score >= multiplier * TIME_PENALTY)
				score -= multiplier * TIME_PENALTY;
			else
				score = 0;
		}

		secondary_screen_changed = 1;
	}

	// Play the given sound
	if (htim == &htim3) {
		switch (current_sound) {

		case BOUNCE:
			TIM4->ARR = 61184;
			TIM4->CCR1 = 30592;
			TIM4->PSC = 2;
			break;

		case BRICK:
			TIM4->ARR = 54513;
			TIM4->CCR1 = 27257;
			TIM4->PSC = 2;
			break;

		case WIN:
			TIM4->ARR = 48581;
			TIM4->CCR1 = 24291;
			TIM4->PSC = 1;
			break;

		case LOSE:
			TIM4->ARR = 65039;
			TIM4->CCR1 = 32520;
			TIM4->PSC = 17;
			break;

		case DEATH:
			TIM4->ARR = 65305;
			TIM4->CCR1 = 32653;
			TIM4->PSC = 9;
			break;

		default:
			TIM4->ARR = 65535;
			TIM4->CCR1 = 0;
			TIM4->PSC = 0;
		}

		current_sound = NONE;

	}
}

// External interrupt handler (user button)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_7) // If The INT Source Is EXTI Line7 (C7 Pin)
    {
    	// Reset values shown on the secondary screen
    	multiplier = 0.5f;
    	lives = LIFE_COUNT;
    	score = 0;
    	secondary_screen_changed = 1;

    	// Reset ball parameters
    	ball_x_pos = player_pos;
		ball_y_pos = 32;
		ball_x_speed = 0;
		ball_y_speed = -1;

		// Reset the bricks
		bricks[0] = 0x00;
		for (uint8_t i=1; i < BRICK_COUNT; i++)
		  bricks[i] = 0xFF;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
