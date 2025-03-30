#include "main.h"
#include "stm32f4xx_hal.h"
#include "Herkulex.h"
#include "time.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define YELLOW_LED                             	GPIO_PIN_13
#define YELLOW_GPIO_PORT                       	GPIOC
#define RED_LED                                	GPIO_PIN_14
#define RED_GPIO_PORT                          	GPIOC
#define BLUE_LED                               	GPIO_PIN_15
#define BLUE_GPIO_PORT                         	GPIOB
#define SHIELD_STAT_LED													GPIO_PIN_14
#define SHIELD_STAT_GPIO_PORT										GPIOB
#define EYE_1_GPIO_PORT													GPIOA
#define EYE_1_LED																GPIO_PIN_2
#define EYE_2_GPIO_PORT													GPIOC
#define EYE_2_LED																GPIO_PIN_3

#define I2C_TIMEOUT 1000

#define ZPOSL 0x01
#define ZPOSR 0x02
#define MPOSL 0x03
#define MPOSR 0x04
#define MANGL 0x05
#define MANGR 0x06
#define RAW_ANGLE_L 0x0C
#define RAW_ANGLE_R 0x0D
#define ANGLE_L 0x0E
#define ANGLE_R 0x0F

#define SERVO_ID 253

#define MASTER_CAN_ID 0x30

//---------------------------------------------------------------------------------
#define N_JOINTS 12 // IMPORTANT
//---------------------------------------------------------------------------------

#define SEA_DATA_SIZE 2
#define POSITION_DATA_SIZE 2
#define SERIAL_ENCODE_MASK 0b10000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef rxHeader; // CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader; // CAN Bus Transmit Header
uint8_t CAN_RX_buffer[8];  // CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; // CAN Bus Filter
uint32_t canMailbox; // CAN Bus Mail box variable

static uint8_t encoder_address = 0b01101100; 	// 0b01101100 left-shifted by 1 - not sure why
uint8_t I2C_buffer[1];
uint8_t position_data_array[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void Error_Handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t left;
uint16_t right;
uint8_t csend[(SEA_DATA_SIZE+POSITION_DATA_SIZE)]; // CAN Tx Buffer
const uint8_t request_packet[] = {0xFF}; // request byte from master
static int my_command = 0;
int const nums[] = {362, 379, 399, 420, 441, 461, 481, 502, 522, 543, 563, 583, 604, 625, 645, 662};

uint32_t last_check;
uint32_t check_period = 5000;

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

	canfil.FilterBank = 0;
	canfil.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil.FilterIdHigh = 0;
	canfil.FilterIdLow = 0;
	canfil.FilterMaskIdHigh = 0;
	canfil.FilterMaskIdLow = 0;
	canfil.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil.FilterActivation = ENABLE;
	canfil.SlaveStartFilterBank = 0;

	txHeader.DLC = 8; // number of bytes to be transmitted - max 8
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;

	//---------------------------------------------------------------------------------
	txHeader.StdId = 0x06; // IMPORTANT
	//---------------------------------------------------------------------------------

	txHeader.ExtId = 0x02;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK) // initialize CAN filter
	{
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) // initialize CAN bus
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // initialize CAN bus Rx interrupt
	{
		Error_Handler();
	}

	HAL_Delay(2000);
	herkulex_init();
	move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
	HAL_Delay(2000);

	// get encoder initial reading and use to calibrate
	// if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, RAW_ANGLE_L, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	// 	Error_Handler();
	// left = I2C_buffer[0];
	// if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, RAW_ANGLE_R, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	// 	Error_Handler();
	// right = I2C_buffer[0];

	// I2C_buffer[0] = left;
	// if (HAL_I2C_Mem_Write(&hi2c2, encoder_address, ZPOSL, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	// 	Error_Handler();
	// I2C_buffer[0] = right;
	// if (HAL_I2C_Mem_Write(&hi2c2, encoder_address, ZPOSR, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
	// 	Error_Handler();
	// HAL_Delay(50);


  /* USER CODE END 2 */
  last_check = HAL_GetTick();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	// shield status LED
  	HAL_GPIO_WritePin(SHIELD_STAT_GPIO_PORT, SHIELD_STAT_LED, GPIO_PIN_SET);

  	// eye code
  	// if (txHeader.StdId == N_JOINTS-1)
  	// {
  	// 	HAL_GPIO_WritePin(EYE_1_GPIO_PORT, EYE_1_LED, GPIO_PIN_SET);
  	// 	HAL_GPIO_WritePin(EYE_2_GPIO_PORT, EYE_2_LED, GPIO_PIN_SET);
  	// 	HAL_Delay(rand() % (5000 - 200 + 1) + 200);
  	// 	HAL_GPIO_WritePin(EYE_1_GPIO_PORT, EYE_1_LED, GPIO_PIN_RESET);
  	// 	HAL_GPIO_WritePin(EYE_2_GPIO_PORT, EYE_2_LED, GPIO_PIN_RESET);
  	// 	HAL_Delay(100);
  	// }
    /* USER CODE END WHILE */
    if((HAL_GetTick() - last_check) > check_period)
    {
      last_check = HAL_GetTick();
      if(get_status(SERVO_ID)){
        herkulex_init();
      }
      // herkulex_init();
    }
    // HAL_Delay(5000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ; // 10 and 5 gives 250k, 5 and 2 gives 500k, 2 and 1 gives 1M
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	// __disable_irq();
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, CAN_RX_buffer) != HAL_OK) // receive CAN bus message in CAN Rx buffer
	{
		Error_Handler();
	}

	// if master sends message over CAN:
	// determine if it is my command or a request
	if (rxHeader.StdId == MASTER_CAN_ID)
	{
		if (CAN_RX_buffer[0] == request_packet[0]) // if state data request
		{
	  	// get position data (2 bytes each) and add to packet
	  	// get_position_bytes(SERVO_ID, position_data_array);
	  	// csend[0] = position_data_array[0];
	  	// csend[1] = (position_data_array[1] | SERIAL_ENCODE_MASK);

	  	// get encoder reading (2 bytes each) and add to packet
			// if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, ANGLE_L, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
			// 	Error_Handler();
			// left = I2C_buffer[0];
			// if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, ANGLE_R, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
			// 	Error_Handler();
			// right = I2C_buffer[0];
			// csend[0] = right;
			// csend[1] = left;

			// HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);

			// // send over CAN back to master
			// if (HAL_CAN_AddTxMessage(hcan1, &txHeader, csend, &canMailbox) != HAL_OK) // send message
			// {
			// 	Error_Handler();
			// }

			// HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);
		} else {
      // txHeader.StdId
			HAL_GPIO_WritePin(BLUE_GPIO_PORT, BLUE_LED, GPIO_PIN_SET);
			// grab command and execute

      if ((txHeader.StdId % 2) == 1) {
        my_command = nums[(CAN_RX_buffer[(txHeader.StdId/2)]&0b11110000)>>4];
      } else {
        my_command = nums[(CAN_RX_buffer[(txHeader.StdId/2)-1]&0b1111)];
      }

			// my_command = ((CAN_RX_buffer[2] & 0x03) << 8) | CAN_RX_buffer[1];
			move_positional(SERVO_ID, my_command, 100, H_LED_WHITE);
			HAL_GPIO_WritePin(BLUE_GPIO_PORT, BLUE_LED, GPIO_PIN_RESET);
		}
  }
	// __enable_irq();
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
  HAL_GPIO_WritePin(RED_GPIO_PORT, RED_LED, GPIO_PIN_SET);
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
