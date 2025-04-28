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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../ECUAL/LCD16X2/LCD16X2.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MyLCD LCD16X2_1

#define spark_row 12
#define spark_col 16


#define inject_row 12
#define inject_col 16

#define map_vector_size 12
#define rpm_vector_size 11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for get_input */
osThreadId_t get_inputHandle;
const osThreadAttr_t get_input_attributes = {
  .name = "get_input",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for calculate_efi */
osThreadId_t calculate_efiHandle;
uint32_t calculate_efiBuffer[ 1023 ];
osStaticThreadDef_t calculate_efiControlBlock;
const osThreadAttr_t calculate_efi_attributes = {
  .name = "calculate_efi",
  .cb_mem = &calculate_efiControlBlock,
  .cb_size = sizeof(calculate_efiControlBlock),
  .stack_mem = &calculate_efiBuffer[0],
  .stack_size = sizeof(calculate_efiBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for display_data */
osThreadId_t display_dataHandle;
const osThreadAttr_t display_data_attributes = {
  .name = "display_data",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for set_output */
osThreadId_t set_outputHandle;
const osThreadAttr_t set_output_attributes = {
  .name = "set_output",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for inject1_control */
osThreadId_t inject1_controlHandle;
const osThreadAttr_t inject1_control_attributes = {
  .name = "inject1_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inject2_control */
osThreadId_t inject2_controlHandle;
const osThreadAttr_t inject2_control_attributes = {
  .name = "inject2_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inject3_control */
osThreadId_t inject3_controlHandle;
const osThreadAttr_t inject3_control_attributes = {
  .name = "inject3_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inject4_control */
osThreadId_t inject4_controlHandle;
const osThreadAttr_t inject4_control_attributes = {
  .name = "inject4_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for spark1_control */
osThreadId_t spark1_controlHandle;
const osThreadAttr_t spark1_control_attributes = {
  .name = "spark1_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for spark2_control */
osThreadId_t spark2_controlHandle;
const osThreadAttr_t spark2_control_attributes = {
  .name = "spark2_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for spark3_control */
osThreadId_t spark3_controlHandle;
const osThreadAttr_t spark3_control_attributes = {
  .name = "spark3_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for spark4_control */
osThreadId_t spark4_controlHandle;
const osThreadAttr_t spark4_control_attributes = {
  .name = "spark4_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

uint16_t rpm_value[10];
uint32_t current_rpm = 900; //least rpm value
uint8_t current_map = 20;  //least map value(common sensors start to measure from20kp tp 100kp)


uint8_t spark_advance[spark_row][spark_col] = {
//8  16  24   33  41  49  58 66 74 83 91 100 --MAP RPM|
{114,119,131,122,116,106,95,79,63,63,63,63}, //400
{114,119,131,122,116,106,95,79,63,63,63,63}, //773
{118,123,132,127,119,111,100,90,74,47,47,47}, //1146
{120,125,135,132,122,111,103,95,84,71,71,71}, //1519
{131,137,146,133,122,114,106,98,90,82,82,82}, //1893
{141,145,154,148,133,125,111,105,95,95,95,95},//2266
{149,154,159,154,138,130,123,111,103,88,88,88},//2640
{153,158,160,149,136,127,122,116,106,90,90,90},//3013
{158,163,165,156,148,135,130,124,119,109,109,109},//3386
{157,163,165,159,151,138,132,124,119,119,119,119},//3760
{163,164,165,159,148,138,132,127,116,111,111,111},//4133
};

uint8_t injection[inject_row][inject_col] = {
//8  16  24   33  41  49  58 66 74 83 91 100 --MAP RPM|
{121,121,126,131,134,135,137,138,138,138,138,138}, //400
{121,121,126,131,134,135,137,138,138,138,138,138}, //773
{118,118,122,125,127,128,130,131,132,132,132,132}, //1146
{115,115,123,125,127,128,129,130,131,131,131,131}, //1519
{116,116,123,127,129,131,132,133,133,133,133,133}, //1893
{116,116,120,127,128,130,131,131,131,148,148,148}, //2266
{125,125,126,129,130,131,131,131,131,148,148,148}, //2640
{144,144,132,130,130,130,130,130,130,142,142,142}, //3013
{126,126,125,127,128,129,130,130,131,134,134,134}, //3386
{120,120,121,125,126,127,128,129,129,129,129,129}, //3760
{125,125,131,131,131,131,131,131,132,131,131,131}, //4133

};

uint8_t map_vector[map_vector_size] = {8,16,24,33,41,49,58,66,74,83,91,100};
uint16_t rpm_vector[rpm_vector_size] = {400,773,1146,1519,1893,2266,2640,3013,3386,3760,4133};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void get_input_function(void *argument);
void calculate_efi_func(void *argument);
void display_data_func(void *argument);
void set_output_func(void *argument);
void inject1_control_function(void *argument);
void inject2_control_function(void *argument);
void inject3_control_function(void *argument);
void inject4_control_function(void *argument);
void spark1_control_function(void *argument);
void spark2_control_function(void *argument);
void spark3_control_function(void *argument);
void spark4_control_function(void *argument);

/* USER CODE BEGIN PFP */
void get_map();  //read  map from gpio, convert it to value between 0 to 100
uint8_t spark_advance_timing(uint8_t map_input , uint16_t rpm); //calculate spark advance timing
uint8_t injection_timing(uint8_t map_input , uint16_t rpm); //calculate injection timing
uint8_t rpm_vector_map(uint16_t rpm); // map rpm  pr MAP sensor value to saved valued in the code
uint8_t map_vector_map(uint8_t map);  //map MAP sensor value to saved value in the code
uint8_t current_spark=0;
uint8_t current_inject =0;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rpm_value, 5); //16bit we selected!(half of a word)


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of get_input */
  get_inputHandle = osThreadNew(get_input_function, NULL, &get_input_attributes);

  /* creation of calculate_efi */
  calculate_efiHandle = osThreadNew(calculate_efi_func, NULL, &calculate_efi_attributes);

  /* creation of display_data */
  display_dataHandle = osThreadNew(display_data_func, NULL, &display_data_attributes);

  /* creation of set_output */
  set_outputHandle = osThreadNew(set_output_func, NULL, &set_output_attributes);

  /* creation of inject1_control */
  inject1_controlHandle = osThreadNew(inject1_control_function, NULL, &inject1_control_attributes);

  /* creation of inject2_control */
  inject2_controlHandle = osThreadNew(inject2_control_function, NULL, &inject2_control_attributes);

  /* creation of inject3_control */
  inject3_controlHandle = osThreadNew(inject3_control_function, NULL, &inject3_control_attributes);

  /* creation of inject4_control */
  inject4_controlHandle = osThreadNew(inject4_control_function, NULL, &inject4_control_attributes);

  /* creation of spark1_control */
  spark1_controlHandle = osThreadNew(spark1_control_function, NULL, &spark1_control_attributes);

  /* creation of spark2_control */
  spark2_controlHandle = osThreadNew(spark2_control_function, NULL, &spark2_control_attributes);

  /* creation of spark3_control */
  spark3_controlHandle = osThreadNew(spark3_control_function, NULL, &spark3_control_attributes);

  /* creation of spark4_control */
  spark4_controlHandle = osThreadNew(spark4_control_function, NULL, &spark4_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, inject1_Pin|spark1_Pin|inject2_Pin|spark2_Pin
                          |inject3_Pin|spark3_Pin|inject4_Pin|spark4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : map_inc_Pin map_dec_Pin */
  GPIO_InitStruct.Pin = map_inc_Pin|map_dec_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : inject1_Pin spark1_Pin inject2_Pin spark2_Pin
                           inject3_Pin spark3_Pin inject4_Pin spark4_Pin */
  GPIO_InitStruct.Pin = inject1_Pin|spark1_Pin|inject2_Pin|spark2_Pin
                          |inject3_Pin|spark3_Pin|inject4_Pin|spark4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD14 PD15 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void get_map(){

	//current_map = (map_value[0] * 3.3) /4095;
	if(HAL_GPIO_ReadPin(map_inc_GPIO_Port, map_inc_Pin) == 0) //key pressed
		current_map +=10;
	if(current_map>100)
		current_map=100;


	if(HAL_GPIO_ReadPin(map_inc_GPIO_Port, map_dec_Pin) == 0) //key pressed
			current_map -=10;
		if(current_map<20)
			current_map=20;
}

uint8_t spark_advance_timing(uint8_t map_input , uint16_t rpm){

	uint8_t rpm_index = rpm_vector_map(rpm);
	uint8_t map_index = map_vector_map(map_input);

	uint8_t spark_advance_degree = spark_advance[rpm_index][map_index];

	float degree_to_time = ((float)spark_advance_degree /360 );

    uint8_t spark_advance_time = degree_to_time * (60000/rpm);

	return spark_advance_time;
}

uint8_t injection_timing(uint8_t map_input , uint16_t rpm){

	uint8_t rpm_index = rpm_vector_map(current_rpm);
	uint8_t map_index = map_vector_map(map_input);

	return injection[rpm_index][map_index];
}

uint8_t rpm_vector_map(uint16_t rpm){

	int min_value=0;

	uint8_t result=0;

	char data[5];

	min_value = abs(rpm - rpm_vector[0]);

	result = 0;

	for(int i=1; i<rpm_vector_size; i++){

		if(abs(rpm - rpm_vector[i])< min_value){
			min_value = abs(rpm - rpm_vector[i]);
				result = i;

		}

		itoa(abs(rpm - rpm_vector[i]) , data , 10);
		strcat(data , "\n");
		//HAL_UART_Transmit(&huart1, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	}



	//itoa(result , data , 10);
	//strcat(data , "\n");

	//HAL_UART_Transmit(&huart1, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);

	return result;

}

uint8_t map_vector_map(uint8_t map){

	int min_value=0;

	uint8_t result=0;

	min_value = abs(map - map_vector[0]);

	result = 0;

	for(int i=1; i<map_vector_size; i++){

		if(abs(map - map_vector[i])< min_value){
			min_value = abs(map - map_vector[i]);
				result = i;
		}

	}

	return result;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_get_input_function */
/**
* @brief Function implementing the get_input thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_get_input_function */
void get_input_function(void *argument)
{
  /* USER CODE BEGIN get_input_function */
  /* Infinite loop */
  for(;;)
  {
	 get_map();
   //  HAL_UART_Transmit(&huart1, (uint8_t *)"getinput\n", 10, HAL_MAX_DELAY);
	// HAL_Delay(500); //debouncing
     vTaskDelay(pdMS_TO_TICKS(100));
    osDelay(1);
  }
  /* USER CODE END get_input_function */
}

/* USER CODE BEGIN Header_calculate_efi_func */
/**
* @brief Function implementing the calculate_efi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_calculate_efi_func */
void calculate_efi_func(void *argument)
{
  /* USER CODE BEGIN calculate_efi_func */
  /* Infinite loop */
  for(;;)
  {
	current_spark = spark_advance_timing(current_map ,  rpm_value[0]);

	current_inject = injection_timing(current_map , rpm_value[0]);

    osDelay(1);
  }
  /* USER CODE END calculate_efi_func */
}

/* USER CODE BEGIN Header_display_data_func */
/**
* @brief Function implementing the display_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_display_data_func */
void display_data_func(void *argument)
{
  /* USER CODE BEGIN display_data_func */
	    LCD16X2_Init(MyLCD);
	    LCD16X2_Clear(MyLCD);
  /* Infinite loop */
  for(;;)
  {
	       char rpm_string[10];
	       char map_string[10];

	       LCD16X2_Set_Cursor(MyLCD, 1, 1);
	       LCD16X2_Write_String(MyLCD, "MAP:");

	       LCD16X2_Set_Cursor(MyLCD, 2, 1);
	       LCD16X2_Write_String(MyLCD, "RPM:");

	       itoa(current_map ,map_string , 10);
	       itoa(rpm_value[0] ,rpm_string , 10);

	       LCD16X2_Set_Cursor(MyLCD, 2, 6);
	       LCD16X2_Write_String(MyLCD, rpm_string);

	       LCD16X2_Set_Cursor(MyLCD, 1, 5);
	       LCD16X2_Write_String(MyLCD, map_string);
           //HAL_UART_Transmit(&huart1, (uint8_t *)map_string, 10, HAL_MAX_DELAY);

	    //  uint8_t res = spark_advance_timing(current_map , rpm_value[0]);

	    //  uint8_t res2 = injection_timing(current_map , rpm_value[0]);

	      char data[5];
	      itoa(current_inject ,data , 10);

	      char data2[5];
	  	   itoa(current_spark ,data2 , 10);

	      LCD16X2_Set_Cursor(MyLCD, 2, 11);
	      LCD16X2_Write_String(MyLCD, "IJ");
	      LCD16X2_Set_Cursor(MyLCD, 2, 14);
	      LCD16X2_Write_String(MyLCD, data);

	      LCD16X2_Set_Cursor(MyLCD, 1, 9);
	      LCD16X2_Write_String(MyLCD, "SP");
	      LCD16X2_Set_Cursor(MyLCD, 1, 13);
	      LCD16X2_Write_String(MyLCD, data2);

	      vTaskDelay(pdMS_TO_TICKS(500));
	 	  LCD16X2_Clear(MyLCD);



    osDelay(1);
  }
  /* USER CODE END display_data_func */
}

/* USER CODE BEGIN Header_set_output_func */
/**
* @brief Function implementing the set_output thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_set_output_func */
void set_output_func(void *argument)
{
  /* USER CODE BEGIN set_output_func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END set_output_func */
}

/* USER CODE BEGIN Header_inject1_control_function */
/**
* @brief Function implementing the inject1_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inject1_control_function */
void inject1_control_function(void *argument)
{
  /* USER CODE BEGIN inject1_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(inject1_GPIO_Port , inject1_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_inject));
    osDelay(1);
  }
  /* USER CODE END inject1_control_function */
}

/* USER CODE BEGIN Header_inject2_control_function */
/**
* @brief Function implementing the inject2_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inject2_control_function */
void inject2_control_function(void *argument)
{
  /* USER CODE BEGIN inject2_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(inject2_GPIO_Port , inject2_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_inject));
    osDelay(1);
  }
  /* USER CODE END inject2_control_function */
}

/* USER CODE BEGIN Header_inject3_control_function */
/**
* @brief Function implementing the inject3_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inject3_control_function */
void inject3_control_function(void *argument)
{
  /* USER CODE BEGIN inject3_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(inject3_GPIO_Port , inject3_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_inject));
    osDelay(1);
  }
  /* USER CODE END inject3_control_function */
}

/* USER CODE BEGIN Header_inject4_control_function */
/**
* @brief Function implementing the inject4_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inject4_control_function */
void inject4_control_function(void *argument)
{
  /* USER CODE BEGIN inject4_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(inject4_GPIO_Port , inject4_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_inject));
    osDelay(1);
  }
  /* USER CODE END inject4_control_function */
}

/* USER CODE BEGIN Header_spark1_control_function */
/**
* @brief Function implementing the spark1_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spark1_control_function */
void spark1_control_function(void *argument)
{
  /* USER CODE BEGIN spark1_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(spark1_GPIO_Port , spark1_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_spark));
    osDelay(1);
  }
  /* USER CODE END spark1_control_function */
}

/* USER CODE BEGIN Header_spark2_control_function */
/**
* @brief Function implementing the spark2_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spark2_control_function */
void spark2_control_function(void *argument)
{
  /* USER CODE BEGIN spark2_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(spark2_GPIO_Port , spark2_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_spark));
    osDelay(1);
  }
  /* USER CODE END spark2_control_function */
}

/* USER CODE BEGIN Header_spark3_control_function */
/**
* @brief Function implementing the spark3_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spark3_control_function */
void spark3_control_function(void *argument)
{
  /* USER CODE BEGIN spark3_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(spark2_GPIO_Port , spark2_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_spark));
    osDelay(1);
  }
  /* USER CODE END spark3_control_function */
}

/* USER CODE BEGIN Header_spark4_control_function */
/**
* @brief Function implementing the spark4_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spark4_control_function */
void spark4_control_function(void *argument)
{
  /* USER CODE BEGIN spark4_control_function */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(spark4_GPIO_Port , spark4_Pin);
	  vTaskDelay(pdMS_TO_TICKS(current_spark));
    osDelay(1);
  }
  /* USER CODE END spark4_control_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
