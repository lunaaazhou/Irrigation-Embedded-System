/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
uint8_t pwm_option;
uint8_t start_time;
uint8_t stop_time;
uint8_t zone_number;
} PipelineConfig_t;

/* ---- Added per your change ---- */
typedef struct {
int pipe;
int pwm_percent;
int pwm_option_raw;
} ActiveHourCfg;
/* -------------------------------- */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint8_t clock_secs = 0;
volatile uint8_t wall_clock_hr_update_flag = 0;
volatile uint8_t mode = 0; //0 = setup mode; 1 = run mode

volatile uint8_t rxd_data = 0;
volatile uint8_t uart_rx_flag = 0;
uint8_t txd_msg_buffer[128] = {0};
volatile uint32_t rpm_tick_count = 0;
volatile uint8_t hcsr04_Rx_flag = 0;
volatile uint8_t first_edge = 0;
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;
PipelineConfig_t zone_config[4] = {0};

volatile uint8_t reservoir_empty_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
void newline(void);
uint8_t UART_GetSingleDigit(uint8_t min, uint8_t max);
uint8_t UART_GetDoubleDigit(uint8_t min, uint8_t max);
void turnOnLed(char c);
void skipSetUp(void);
void enterSetUpMode(void);
void HCSR04_TRIG_PULSE(void);
int getHCSR04(void);
void waitForRun(void);
void digitDisplay(int depth);
void special(void);

static int map_pwm_percent(uint8_t pwm_sel);
static void PIPE0_FromADC(ActiveHourCfg *cfg_out);
static void PIPE_Idle(ActiveHourCfg *cfg_out);
static void SelectConfigForUpcomingHour(int hour, ActiveHourCfg *cfg_out);

static void ZONE_PIPE(PipelineConfig_t *pconfig, ActiveHourCfg *cfg_out);

void run(void);
/* -------------------------------------------- */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ADC_Select_CH(int CH){
ADC_ChannelConfTypeDef sConfig = {0};
switch(CH)
{
case 0:
sConfig.Channel = ADC_CHANNEL_0;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 1:
sConfig.Channel = ADC_CHANNEL_1;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 2:
sConfig.Channel = ADC_CHANNEL_2;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 3:
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 4:
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 5:
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 6:
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 7:
sConfig.Channel = ADC_CHANNEL_7;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 8:
sConfig.Channel = ADC_CHANNEL_8;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 9:
sConfig.Channel = ADC_CHANNEL_9;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 10:
sConfig.Channel = ADC_CHANNEL_10;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 11:
sConfig.Channel = ADC_CHANNEL_11;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 12:
sConfig.Channel = ADC_CHANNEL_12;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 13:
sConfig.Channel = ADC_CHANNEL_13;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 14:
sConfig.Channel = ADC_CHANNEL_14;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
case 15:
sConfig.Channel = ADC_CHANNEL_15;
sConfig.Rank = 1;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){ Error_Handler(); }
break;
}
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
uint8_t DIGITA_VAL = DIGIT_A & 0x0F;
uint8_t DIGITB_VAL = DIGIT_B & 0x0F;

int Abit0 = DIGITA_VAL & 1;
int Abit1 = (DIGITA_VAL >> 1) & 1;
int Abit2 = (DIGITA_VAL >> 2) & 1;
int Abit3 = (DIGITA_VAL >> 3) & 1;

int Bbit0 = DIGITB_VAL & 1;
int Bbit1 = (DIGITB_VAL >> 1) & 1;
int Bbit2 = (DIGITB_VAL >> 2) & 1;
int Bbit3 = (DIGITB_VAL >> 3) & 1;

// 个位 (A) ：PB5–PB9
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, Abit0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, Abit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, Abit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, Abit3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

// 十位 (B) ：PC10–PC12, PC3
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, Bbit0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, Bbit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, Bbit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, Bbit3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


/* USER CODE END 0 */

/**
* @brief The application entry point.
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
MX_USART2_UART_Init();
MX_TIM3_Init();
MX_ADC1_Init();
MX_TIM2_Init();
MX_TIM5_Init();
MX_TIM4_Init();
/* USER CODE BEGIN 2 */
HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(TIM3_IRQn);
HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(TIM4_IRQn);

// HCSR04
HAL_TIM_Base_Init(&htim5);
HAL_TIM_Base_Start(&htim5);
HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);

// PWM Motor
HAL_TIM_Base_Init(&htim3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

TIM3->PSC = 0;
TIM3->ARR = 100000;
TIM3->CCR1 = 0;
TIM3->CCR3 = 0;

// Servo Motor
HAL_TIM_Base_Start(&htim2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
TIM2->PSC = 16-1;

// Wall Clock
clock_hours = 0;
clock_mins = 0;
clock_secs = 0;
wall_clock_hr_update_flag = 0;

HAL_TIM_Base_Start_IT(&htim4);
__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

enterSetUpMode();
waitForRun();
run();
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
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
hadc1.Init.Resolution = ADC_RESOLUTION_8B;
hadc1.Init.ScanConvMode = ENABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
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
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN ADC1_Init 2 */

/* USER CODE END ADC1_Init 2 */

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
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN TIM2_Init 1 */

/* USER CODE END TIM2_Init 1 */
htim2.Instance = TIM2;
htim2.Init.Prescaler = 16-1;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 20000-1;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 500-1;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM2_Init 2 */

/* USER CODE END TIM2_Init 2 */
HAL_TIM_MspPostInit(&htim2);

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
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN TIM3_Init 1 */

/* USER CODE END TIM3_Init 1 */
htim3.Instance = TIM3;
htim3.Init.Prescaler = 16-1;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 2000-1;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 1200-1;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
sConfigOC.Pulse = 0;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

/* USER CODE END TIM3_Init 2 */
HAL_TIM_MspPostInit(&htim3);

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

/* USER CODE BEGIN TIM4_Init 1 */

/* USER CODE END TIM4_Init 1 */
htim4.Instance = TIM4;
htim4.Init.Prescaler = 53 - 1;
htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
htim4.Init.Period = 1000-1;
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
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM4_Init 2 */

/* USER CODE END TIM4_Init 2 */

}

/**
* @brief TIM5 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM5_Init(void)
{

/* USER CODE BEGIN TIM5_Init 0 */

/* USER CODE END TIM5_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_IC_InitTypeDef sConfigIC = {0};

/* USER CODE BEGIN TIM5_Init 1 */

/* USER CODE END TIM5_Init 1 */
htim5.Instance = TIM5;
htim5.Init.Prescaler = 16-1;
htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
htim5.Init.Period = 65536-1;
htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
sConfigIC.ICFilter = 0;
if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM5_Init 2 */

/* USER CODE END TIM5_Init 2 */

}

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

/* USER CODE BEGIN USART2_Init 0 */

/* USER CODE END USART2_Init 0 */

/* USER CODE BEGIN USART2_Init 1 */

/* USER CODE END USART2_Init 1 */
huart2.Instance = USART2;
huart2.Init.BaudRate = 115200;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN USART2_Init 2 */

/* USER CODE END USART2_Init 2 */

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
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, LD2_Pin|BLU_Pin|GRN_Pin|RED_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|HCSR04_TRIG_Pin
|DIGIT_A3_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin : PC13 */
GPIO_InitStruct.Pin = GPIO_PIN_13;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : DIGIT_B3_Pin DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin */
GPIO_InitStruct.Pin = DIGIT_B3_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/*Configure GPIO pins : LD2_Pin BLU_Pin GRN_Pin RED_Pin */
GPIO_InitStruct.Pin = LD2_Pin|BLU_Pin|GRN_Pin|RED_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pin : RPM_TICK_Pin */
GPIO_InitStruct.Pin = RPM_TICK_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : DIGIT_A0_Pin DIGIT_A1_Pin DIGIT_A2_Pin HCSR04_TRIG_Pin
DIGIT_A3_Pin */
GPIO_InitStruct.Pin = DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|HCSR04_TRIG_Pin
|DIGIT_A3_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* EXTI interrupt init*/
HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void newline(void){
sprintf((char*)txd_msg_buffer, "\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}

uint8_t UART_GetSingleDigit(uint8_t min, uint8_t max)
{
uint8_t value = 0;
while(1)
{
uart_rx_flag = 0;
HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxd_data, 1);
while(uart_rx_flag == 0) {}

if(rxd_data >= '0' && rxd_data <= '9')
{
value = (uint8_t)(rxd_data - '0');
if(value >= min && value <= max)
{
sprintf((char*)txd_msg_buffer, "\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
return value;
}
else
{
sprintf((char*)txd_msg_buffer, " - Invalid! Enter %d-%d: ", min, max);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}
}
else
{
sprintf((char*)txd_msg_buffer, "\r\nInvalid input! Enter a digit (%d-%d): ", min, max);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}
}
}

uint8_t UART_GetDoubleDigit(uint8_t min, uint8_t max)
{
uint8_t tens = 0, ones = 0, value = 0;
while(1)
{
uart_rx_flag = 0;
HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxd_data, 1);
while(uart_rx_flag == 0) {}
if(rxd_data >= '0' && rxd_data <= '9'){ tens = (uint8_t)(rxd_data - '0'); }
else{
sprintf((char*)txd_msg_buffer, "\r\nInvalid! Enter two digits (%02d-%02d): ", min, max);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
continue;
}

uart_rx_flag = 0;
HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxd_data, 1);
while(uart_rx_flag == 0) {}
if(rxd_data >= '0' && rxd_data <= '9'){
ones = (uint8_t)(rxd_data - '0');
value = (uint8_t)(tens * 10 + ones);
if(value >= min && value <= max){
sprintf((char*)txd_msg_buffer, "\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
return value;
}else{
sprintf((char*)txd_msg_buffer, " - Invalid range! Enter %02d-%02d: ", min, max);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}
}else{
sprintf((char*)txd_msg_buffer, "\r\nInvalid! Enter two digits (%02d-%02d): ", min, max);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}
}
}

void turnOnLed(char c){
switch(c) {
case 'r': // red
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // RED
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // GRN
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // BLU
break;
case 'g': // green
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
break;
case 'b': // blue
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
break;
case 'p': // purple
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
break;
case 'w': // white
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
break;
case 'x': // off
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
break;
default: break;
}
}

void skipSetUp(void){
zone_config[0].zone_number = 0;
zone_config[0].pwm_option = 0;
zone_config[0].start_time = 0;
zone_config[0].stop_time = 3;

zone_config[1].zone_number = 1;
zone_config[1].pwm_option = 3;
zone_config[1].start_time = 4;
zone_config[1].stop_time = 6;

zone_config[2].zone_number = 2;
zone_config[2].pwm_option = 1;
zone_config[2].start_time = 10;
zone_config[2].stop_time = 13;

zone_config[3].zone_number = 3;
zone_config[3].pwm_option = 2;
zone_config[3].start_time = 16;
zone_config[3].stop_time = 20;
}

void enterSetUpMode(void){
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

TIM3->CCR1 = 0;
TIM3->CCR3 = 0;

sprintf((char*)txd_msg_buffer, "\r\n>>> SETUP MODE <<<\r\n\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
HAL_Delay(200);

sprintf((char*)txd_msg_buffer, "Enter SETUP Parameters\r\n");
newline();
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

sprintf((char*)txd_msg_buffer, "PIPELINE (options: 0 to 3):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[0].zone_number = UART_GetSingleDigit(0, 3);

sprintf((char*)txd_msg_buffer, "PUMP PWM (options: 0 to 3):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[0].pwm_option = UART_GetSingleDigit(0, 3);

for(int i = 1; i< 4; i++){
sprintf((char*)txd_msg_buffer, "PIPELINE (options: 0 to 3):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[i].zone_number = UART_GetSingleDigit(1, 3);

sprintf((char*)txd_msg_buffer, "PUMP PWM (options: 0 to 3):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[i].pwm_option = UART_GetSingleDigit(1, 3);
}
newline();

sprintf((char*)txd_msg_buffer, "Pipeline 0 Pump FIRST HOUR (options: 00 to 07):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[0].start_time = UART_GetDoubleDigit(0, 7);

sprintf((char*)txd_msg_buffer, "Pipeline 0 Pump LAST HOUR (options: 00 to 07):");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[0].stop_time = UART_GetDoubleDigit(0, 7);

for(int i = 1; i< 4; i++){
sprintf((char*)txd_msg_buffer, "Pipeline %d Pump FIRST HOUR (options: 08 to 23):", (int)i);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[i].start_time = UART_GetDoubleDigit(8, 23);

sprintf((char*)txd_msg_buffer, "Pipeline %d Pump LAST HOUR (options: 09 to 23):", (int)i);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
zone_config[i].stop_time = UART_GetDoubleDigit(9, 23);
}

newline();
sprintf((char*)txd_msg_buffer, "\r\n>>> Printing SETUP Parameter <<<\r\n\r\n");
newline();

HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
newline();

sprintf((char*)txd_msg_buffer, "CURRENT WALL CLOCK HOUR 0\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

for(int i = 0; i< 4; i++){
sprintf((char*)txd_msg_buffer, "PIPLINE: %d Pump PWM: %d Pump FIRST HOUR: %d Pump LAST HOUR: %d\r\n",
zone_config[i].zone_number, zone_config[i].pwm_option, zone_config[i].start_time, zone_config[i].stop_time);
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
newline();
}
newline();

sprintf((char*)txd_msg_buffer, "SETUP is done. Press Blue Button for RUN MODE\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}

void HCSR04_TRIG_PULSE(void)
{
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
for(int j = 0; j < 15; j++) { __NOP(); }
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
//sprintf((char*)txd_msg_buffer, "Debug: Trigger pulse sent\r\n");
//HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}

int getHCSR04(void)
{
hcsr04_Rx_flag = 0;
first_edge = 0;
time_edge1 = 0;
time_edge2 = 0;
__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
__HAL_TIM_SET_COUNTER(&htim5, 0);

HCSR04_TRIG_PULSE();
HAL_Delay(10);

uint32_t timeout = 30000;
while (hcsr04_Rx_flag == 0 && timeout-- > 0) { HAL_Delay(1); }

if (hcsr04_Rx_flag) {
uint32_t time_diff;
if (time_edge2 >= time_edge1) {
time_diff = time_edge2 - time_edge1;
} else {
time_diff = (0xFFFF - time_edge1 + time_edge2);
}
float distance = (time_diff * 0.0343f) / 2.0f; // Distance in cm

//sprintf((char*)txd_msg_buffer, "Debug: time_edge1: %d, time_edge2: %d, time_diff: %lu\r\n", time_edge1, time_edge2, time_diff);
//HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

int retVal = (distance > 90.0f) ? 90 : (int)distance;

int depthVal = 90 - retVal;
if (depthVal < 0) {
depthVal = 0;
}
if (depthVal > 90) {
depthVal = 90;
}
return depthVal;
} else {
return 0;
}
}

void waitForRun(void){
while(1){
if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
break;
} else {
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
HAL_Delay(150);
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
HAL_Delay(150);
}
}
mode = 1;
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
sprintf((char*)txd_msg_buffer, "\r\n>>> RUN MODE <<<\r\n\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
}

void digitDisplay(int depth){
uint8_t tens = (uint8_t)(depth / 10);
uint8_t ones = (uint8_t)(depth % 10);
DIGITS_Display(tens, ones);
}

void special(void){
sprintf((char*)txd_msg_buffer, "RESERVOIR IS EMPTY\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

TIM3->CCR1 = 0;
TIM3->CCR3 = 0;

digitDisplay(0);

while(1){
turnOnLed('w');
HAL_Delay(100);
turnOnLed('x');
HAL_Delay(100);
}
}

static int map_pwm_percent(uint8_t pwm_sel)
{
if (pwm_sel == 1) return 70;
if (pwm_sel == 2) return 85;
if (pwm_sel == 3) return 99;
return 0;
}

static void ZONE_PIPE(PipelineConfig_t *pconfig, ActiveHourCfg *cfg_out)
{
int zone = pconfig->zone_number;
int pwm_percent = map_pwm_percent(pconfig->pwm_option);

TIM2->CCR1 = (uint32_t)(700 - 300 * zone);

if (zone == 1) turnOnLed('r');
else if (zone == 2) turnOnLed('g');
else if (zone == 3) turnOnLed('b');

int PWM;
if (pwm_percent == 99)
PWM = (int)(23300.0f * 0.99f);
else
PWM = (int)(23300.0f * (pwm_percent / 100.0f) * 0.9f);

if (PWM < 0) PWM = 0;
TIM3->CCR1 = (uint32_t)PWM;
TIM3->CCR3 = 0;

cfg_out->pipe = zone;
cfg_out->pwm_percent = pwm_percent;
cfg_out->pwm_option_raw = pconfig->pwm_option;
}

static void PIPE0_FromADC(ActiveHourCfg *cfg_out)
{
ADC_Select_CH(4);
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1, 1000);
uint16_t adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
HAL_ADC_Stop(&hadc1);

int PWM = 8500 + (int)((43300.0f * adc) / 255.0f);
if (PWM < 0) PWM = 0;
if (PWM > 53000) PWM = 53000;

TIM2->CCR1 = 700;
TIM3->CCR1 = (uint32_t)PWM;
TIM3->CCR3 = 0;
turnOnLed('p');

int pwm_percent = (int)(PWM / 53000.0f * 100.0f);
if (pwm_percent < 0) pwm_percent = 0;
if (pwm_percent > 99) pwm_percent = 99;

cfg_out->pipe = 0;
cfg_out->pwm_percent = pwm_percent;
cfg_out->pwm_option_raw = zone_config[0].pwm_option;
}

static void PIPE_Idle(ActiveHourCfg *cfg_out)
{
TIM3->CCR1 = 0;
TIM3->CCR3 = 0;
turnOnLed('x');

cfg_out->pipe = -1;
cfg_out->pwm_percent = -1;
cfg_out->pwm_option_raw = -1;
}

static void SelectConfigForUpcomingHour(int hour, ActiveHourCfg *cfg_out)
{
if (hour >= zone_config[0].start_time && hour <= zone_config[0].stop_time)
{
PIPE0_FromADC(cfg_out);
return;
}

for (int i = 1; i < 4; i++)
{
PipelineConfig_t *z = &zone_config[i];
if (hour >= z->start_time && hour <= z->stop_time)
{
ZONE_PIPE(z, cfg_out);
return;
}
}

PIPE_Idle(cfg_out);
}

void run(void)
{
sprintf((char*)txd_msg_buffer, " CLOCK : PIPE : PWM : RPM : DEPTH : \r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
sprintf((char*)txd_msg_buffer, "--------------------------------------\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

ActiveHourCfg active = {0, 0};
SelectConfigForUpcomingHour(0, &active);

while (1)
{
if (active.pipe == 0) {
PIPE0_FromADC(&active);
}

//HAL_UART_Transmit(&huart2, (uint8_t*)"waiting hour flag...\r\n", 23, 100);
while (wall_clock_hr_update_flag == 0) { HAL_Delay(1); }
//HAL_UART_Transmit(&huart2, (uint8_t*)"hour flag set!\r\n", 16, 100);
wall_clock_hr_update_flag = 0;

int clkhr = (int)clock_hours - 1;
if (clkhr < 0) continue;

uint32_t rpm = (uint32_t)((60 * rpm_tick_count) / (20 * 12));
rpm_tick_count = 0;
int depth = getHCSR04();

char pipe_field[4];
char pwm_field[5];
if (active.pipe < 0) {
strcpy(pipe_field, " ");
strcpy(pwm_field, " ");
} else {
snprintf(pipe_field, sizeof(pipe_field), "%d", active.pipe);
snprintf(pwm_field, sizeof(pwm_field), "%d", active.pwm_option_raw);
}

sprintf((char*)txd_msg_buffer,
" %d : %s : %s : %lu : %d :\r\n",
clkhr,
pipe_field,
pwm_field,
(unsigned long)rpm,
depth);

HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

digitDisplay(depth);

if (depth == 0 && !reservoir_empty_flag) {
reservoir_empty_flag = 1;
special();
}

int next_hour = (int)clock_hours;
if (next_hour > 24) break;

SelectConfigForUpcomingHour(next_hour, &active);
}

// After 24 hours
PIPE_Idle(&active);
turnOnLed('x');
sprintf((char*)txd_msg_buffer, "\r\nIRRIGATION COMPLETED\r\n");
HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

while (1) {
HAL_Delay(1000);
}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if(huart->Instance == USART2)
{
HAL_UART_Transmit(&huart2, (uint8_t*)&rxd_data, 1, 100);
uart_rx_flag = 1;
}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
{
if (first_edge == 0)
{
// Capture the first rising edge
time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
first_edge = 1;

__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

//sprintf((char*)txd_msg_buffer, "Debug: First edge captured, time_edge1: %d\r\n", time_edge1);
//HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}
else
{
// Capture the second edge (falling edge)
time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
first_edge = 0;
hcsr04_Rx_flag = 1;

__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

//sprintf((char*)txd_msg_buffer, "Debug: Second edge captured, time_edge2: %d\r\n", time_edge2);
//HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}
}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == RPM_TICK_Pin)
{
rpm_tick_count += 1;
//sprintf((char*)txd_msg_buffer, "Debug: RPM tick count: %lu\r\n", rpm_tick_count);
//HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
if(htim->Instance == TIM4)
{
if(mode == 0) {
return;
}
//HAL_UART_Transmit(&huart2, (uint8_t*)"tick\r\n", 6, 100);
clock_secs += 1;
if(clock_secs == 60)
{
clock_mins += 1;
clock_secs = 0;

if(clock_mins == 60 )
{
clock_hours += 1;
clock_mins = 0;
wall_clock_hr_update_flag = 1;
}
}
}
}
/* USER CODE END 4 */

/**
* @brief This function is executed in case of error occurrence.
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
#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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
