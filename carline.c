/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ================= DEFINE ================= */
// Motor L298N
#define IN1_PORT  GPIOC
#define IN1_PIN   GPIO_PIN_9
#define IN2_PORT  GPIOC
#define IN2_PIN   GPIO_PIN_10
#define IN3_PORT  GPIOC
#define IN3_PIN   GPIO_PIN_11
#define IN4_PORT  GPIOC
#define IN4_PIN   GPIO_PIN_12

// PWM
#define PWM_TIM         htim1
#define PWM_CH_RIGHT    TIM_CHANNEL_1
#define PWM_CH_LEFT     TIM_CHANNEL_2
// Sensor
#define SENSOR_PORT GPIOB
#define S1_PIN GPIO_PIN_0
#define S2_PIN GPIO_PIN_1
#define S3_PIN GPIO_PIN_2
#define S4_PIN GPIO_PIN_3
#define S5_PIN GPIO_PIN_4

// Speed
#define SPEED_MAX   850
#define SPEED_MID   450
#define SPEED_BRAKE 150

#define LIMIT_TOP   999
#define LIMIT_BOT  -850

// PID
#define KP 70.0f
#define KI 0.0001f
#define KD 35.0f

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* ========================================================================= */
/* ================= STRUCT ================= */
struct DataLine {
    uint8_t s1:1, s2:1, s3:1, s4:1, s5:1;
};

union MapLine {
    struct DataLine data;
    uint8_t state;
} raw;

struct CarControl {
    float P, I, D;
    float PID_value;
    int errorNow;
    int errorPrev;
    bool lastDirection;
} car = {0};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * CHÂN IN1 V�?I PINC9
 * CHÂN IN2 V�?I PINC10
 * CHÂN IN3 V�?I PINC11
 * CHÂN IN4 V�?I PINC12
 */
void init_pwm() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}
/* ========================================================================= */
/* ================= FUNCTION ================= */
int constrain(int x, int min, int max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void go_custom(int speedL, int speedR) {
    // RIGHT
    if (speedR >= 0) {
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, 1);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, 0);
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_RIGHT, speedR);
    } else {
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, 0);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, 1);
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_RIGHT, -speedR);
    }

    // LEFT
    if (speedL >= 0) {
        HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, 1);
        HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, 0);
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_LEFT, speedL);
    } else {
        HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, 0);
        HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, 1);
        __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_LEFT, -speedL);
    }
}
void motor_control(void) {
    // đọc sensor (đảo nếu cần: !HAL_GPIO_ReadPin)
    raw.data.s1 = HAL_GPIO_ReadPin(SENSOR_PORT, S1_PIN);
    raw.data.s2 = HAL_GPIO_ReadPin(SENSOR_PORT, S2_PIN);
    raw.data.s3 = HAL_GPIO_ReadPin(SENSOR_PORT, S3_PIN);
    raw.data.s4 = HAL_GPIO_ReadPin(SENSOR_PORT, S4_PIN);
    raw.data.s5 = HAL_GPIO_ReadPin(SENSOR_PORT, S5_PIN);

    // 2. Xử lý mất vạch (Search Mode - Tăng mạnh lực xoay)
        if (raw.state == 0) {
            // Tăng tốc độ lên mức 700-800 để đảm bảo thắng được ma sát sàn
            int search_speed_boost = 750; // Bánh tiến
            int search_speed_back  = -700; // Bánh lùi (phanh gắt)

            if (car.lastDirection) {
                // Lệch phải -> Xoay trái: Bánh phải tiến, bánh trái lùi
                go_custom(search_speed_boost, search_speed_back);
            }
            else {
                // Lệch trái -> Xoay phải: Bánh trái tiến, bánh phải lùi
                go_custom(search_speed_back, search_speed_boost);
            }
            return;
        }

        // mapping error (ĐÃ ĐẢO DẤU ĐỂ CHUẨN HÓA)
            switch (raw.state) {
                // ---- VẠCH BÊN TRÁI (Xe đang lệch Phải -> Cần rẽ Trái) ----
                case 12: car.errorNow = -1; car.lastDirection = 1; break;
                case 8:  car.errorNow = -2; car.lastDirection = 1; break;
                case 24: car.errorNow = -3; car.lastDirection = 1; break;
                case 16: car.errorNow = -5; car.lastDirection = 1; break;

                // ---- VẠCH BÊN PHẢI (Xe đang lệch Trái -> Cần rẽ Phải) ----
                case 6:  car.errorNow = 1;  car.lastDirection = 0; break;
                case 2:  car.errorNow = 2;  car.lastDirection = 0; break;
                case 3:  car.errorNow = 3;  car.lastDirection = 0; break;
                case 1:  car.errorNow = 5;  car.lastDirection = 0; break;

                case 4:  car.errorNow = 0;  car.I = 0; break;

                // ---- GÓC VUÔNG BÊN PHẢI (Vạch nằm ngang bên Phải) ----
                case 7:
                case 15:
                    car.errorNow = 6;  // Trị dương để rẽ Phải
                    car.lastDirection = 0;
                    break;

                // ---- GÓC VUÔNG BÊN TRÁI (Vạch nằm ngang bên Trái) ----
                case 28:
                case 30:
                    car.errorNow = -6; // Trị âm để rẽ Trái
                    car.lastDirection = 1;
                    break;

                case 31: car.errorNow = car.errorPrev; break;
                default: car.errorNow = car.errorPrev; break;
            }

            // PID calculation
            car.P = car.errorNow;
            car.I = constrain(car.I + car.P, -200, 200);
            car.D = car.P - car.errorPrev;
            car.errorPrev = car.errorNow;

            car.PID_value = KP*car.P + KI*car.I + KD*car.D;

            // Dynamic Speed
            int base;
            if (abs(car.errorNow) == 0) base = SPEED_MAX;
            else if (abs(car.errorNow) <= 2) base = SPEED_MID;
            else if (abs(car.errorNow) <= 4) base = SPEED_BRAKE;
            else base = 0;

            // ĐẢO DẤU TẠI ĐÂY ĐỂ PHÙ HỢP VỚI HƯỚNG MOTOR CỦA CẬU
           // Nếu sL = base - PID làm xe chạy ngược, hãy đổi thành sL = base + PID
            int sL = base + (int)car.PID_value;
            int sR = base - (int)car.PID_value;
         //  int sL = base - car.PID_value;
      //     int sR = base + car.PID_value;
            go_custom(constrain(sL, LIMIT_BOT, LIMIT_TOP),
                      constrain(sR, LIMIT_BOT, LIMIT_TOP));
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
  /* USER CODE BEGIN 2 */
  // Khởi động phát PWM trên TIM1 cho ENA và ENB
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  motor_control();
	  HAL_Delay(2); // hoặc 2
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
