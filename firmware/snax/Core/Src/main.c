/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct Command {
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t throwerSpeed;
  uint16_t throwerAngle;
  uint16_t throwerGrab;
  uint16_t delimiter;
} Command;

typedef struct Feedback {
  int16_t ballDetected;
  uint16_t delimiter;
} Feedback;

int16_t IRsensor;

typedef struct MotorControl {
	int16_t gainP;
	int16_t gainI;
	int16_t gainD;
	int32_t integral;
	int16_t positionChange;
	int16_t position;
} MotorControl;

MotorControl motorControl1 = {.position = 0, .positionChange = 0, .gainP = 1000, .gainI = 50, .gainD = 10, .integral = 0};
MotorControl motorControl2 = {.position = 0, .positionChange = 0, .gainP = 1000, .gainI = 50, .gainD = 10, .integral = 0};
MotorControl motorControl3 = {.position = 0, .positionChange = 0, .gainP = 1000, .gainI = 50, .gainD = 10, .integral = 0};

Command command = {
		.speed1 = 0,
		.speed2 = 0,
		.speed3 = 0,
		.throwerSpeed = 0,
		.throwerAngle = 6900,
		.throwerGrab = 4800,
		.delimiter = 0};

volatile uint8_t isCommandReceived = 0;

void CDC_On_Receive(uint8_t* buffer, uint32_t* length) {
  if (*length == sizeof(Command)) {
    memcpy(&command, buffer, sizeof(Command));

    if (command.delimiter == 0xAAAA) {
      isCommandReceived = 1;
    }
  }
}

int32_t ClampValueI32(int32_t val, int32_t min, int32_t max) {
	if (val < min) {
		return min;
	}
	if (val > max) {
		return max;
	}
	return val;
}

uint16_t ClampValueU16(uint16_t val, uint16_t min, uint16_t max) {
	if (val < min) {
		return min;
	}
	if (val > max) {
		return max;
	}
	return val;
}

int32_t ControlMotor(MotorControl* motorControl, int16_t position, int16_t set_speed) {
	motorControl->positionChange = (position-motorControl->position); //Calculates the position change
	int16_t error = (set_speed-motorControl->positionChange); //Calculates the error
	motorControl->integral+=error;
	motorControl->integral=ClampValueI32(motorControl->integral, -65535/motorControl->gainI, 65535/motorControl->gainI); //Adds the errors together and clamps the value
	motorControl->position = position; //Previous position is now the current position
	return (error*motorControl->gainP + motorControl->gainI*motorControl->integral + motorControl->positionChange*motorControl->gainD);
}

void EnableMotorDrivers() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	for (int i = 0; i < 350; i++) {
	  __asm("nop");
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	for (int i = 0; i < 350; i++) {
	  __asm("nop");
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void UpdateServosAndThrower() {
	//Start thrower speed < 3200
	//Thrower speed between 3200 and 6400
	TIM15->CCR2 = command.throwerSpeed+3150;
	//Thrower angle
	TIM8->CCR2 = ClampValueU16(command.throwerAngle, 2700, 6900);
	//Grabber
	TIM16->CCR1 = ClampValueU16(command.throwerGrab, 2700, 6900);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//Reading the motor encoders and calculating motor pwm
	int32_t motor1PWM = ClampValueI32(ControlMotor(&motorControl1, (int16_t)TIM2->CNT, command.speed1), -65535, 65535);
	//Direction M1
	if (motor1PWM < 0) {
		HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, 0);
	} else {
		HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, 1);
	}
	int32_t motor2PWM = ClampValueI32(ControlMotor(&motorControl2, (int16_t)TIM3->CNT, command.speed2), -65535, 65535);
	//Direction M2
	if (motor2PWM < 0) {
		HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, 0);
	} else {
		HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, 1);
	}
	int32_t motor3PWM = ClampValueI32(ControlMotor(&motorControl3, (int16_t)TIM4->CNT, command.speed3), -65535, 65535);
	//Direction M3
	if (motor3PWM < 0) {
		HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, 0);
	} else {
		HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, 1);
	}
	//Changing motor pwm
	TIM1->CCR2 = abs(motor1PWM);
	TIM1->CCR1 = abs(motor2PWM);
	TIM17->CCR1 = abs(motor3PWM);
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_Device_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  Feedback feedback = {
		.ballDetected = 0,
        .delimiter = 0xAAAA
    };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M1_DIR_Pin|M1_PWM_Pin|M2_DIR_Pin|M2_PWM_Pin
                          |M3_DIR_Pin|M3_PWM_Pin|TIM1_CH1_Pin|TIM1_CH2_Pin
                          |USB_DM_Pin|USB_DP_Pin|SYS_JTCK_SWCLK_Pin|SYS_JTCK_SWCLKA14_Pin
                          |TIM8_CH1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TIM3_CH1_Pin|TIM3_CH2_Pin|TIM4_CH1_Pin|TIM4_CH2_Pin
                          |THROWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_DIR_Pin M1_PWM_Pin M2_DIR_Pin M2_PWM_Pin
                           M3_DIR_Pin M3_PWM_Pin TIM1_CH1_Pin TIM1_CH2_Pin
                           USB_DM_Pin USB_DP_Pin SYS_JTCK_SWCLK_Pin SYS_JTCK_SWCLKA14_Pin
                           TIM8_CH1_Pin */
  GPIO_InitStruct.Pin = M1_DIR_Pin|M1_PWM_Pin|M2_DIR_Pin|M2_PWM_Pin
                          |M3_DIR_Pin|M3_PWM_Pin|TIM1_CH1_Pin|TIM1_CH2_Pin
                          |USB_DM_Pin|USB_DP_Pin|SYS_JTCK_SWCLK_Pin|SYS_JTCK_SWCLKA14_Pin
                          |TIM8_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TIM3_CH1_Pin TIM3_CH2_Pin TIM4_CH1_Pin TIM4_CH2_Pin
                           THROWER_Pin */
  GPIO_InitStruct.Pin = TIM3_CH1_Pin|TIM3_CH2_Pin|TIM4_CH1_Pin|TIM4_CH2_Pin
                          |THROWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
