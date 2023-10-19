#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "Herkulex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define YELLOW_LED                             GPIO_PIN_13
#define YELLOW_GPIO_PORT                       GPIOC
#define RED_LED                                GPIO_PIN_14
#define RED_GPIO_PORT                          GPIOC
#define BLUE_LED                               GPIO_PIN_15
#define BLUE_GPIO_PORT                         GPIOB

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

#define N_JOINTS 3
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
uint8_t CAN_RX_buffer[(N_JOINTS-1) * (POSITION_DATA_SIZE + SEA_DATA_SIZE)]; // CAN Bus Receive Buffer
//uint8_t CAN_RX_buffer[8];
CAN_FilterTypeDef canfil; // CAN Bus Filter
uint32_t canMailbox; // CAN Bus Mail box variable

uint8_t encoder_address = 0b01101100; 	// left-shifted by 1 - not sure why :)
uint8_t I2C_buffer[1];
uint8_t usb_in[N_JOINTS * POSITION_DATA_SIZE];
uint8_t usb_out[N_JOINTS * (POSITION_DATA_SIZE + SEA_DATA_SIZE)];
uint16_t packet_len = sizeof(usb_out) / sizeof(usb_out[0]);
uint8_t position_data_array[2];

uint8_t state_buffer[(N_JOINTS-1)*(SEA_DATA_SIZE + POSITION_DATA_SIZE)];
uint8_t my_state_buffer[(SEA_DATA_SIZE + POSITION_DATA_SIZE)];

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

extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
void Error_Handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t left;
uint16_t right;
uint8_t csend[8]; // CAN Tx Buffer
//uint8_t request_packet[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t request_packet[] = {0xFF};

// reset and zero servo if error flag is raised
void reset_and_zero_pos()
{
	if(get_status(SERVO_ID)) {
		clear_error(SERVO_ID);
		torque_on(SERVO_ID);
		move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
		HAL_Delay(50);
	}
}

// echo first character of string received over USB
//void echo(void)
//{
//	if(usb_in[0] != '\0')
//	{
//		usb_out[0] = usb_in[0];
//		HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);
//		HAL_Delay(10);
//		HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);
//		CDC_Transmit_FS(usb_out, 1);
//		memset(usb_in, '\0', 64); // clear buffer
//	}
//}

// oscillate between +/- angle deg and send a '<' or '>' over serial
//void oscillate_and_send(int angle, int period)
//{
//	// check if error is raised and reset if so
//	reset_and_zero_pos();
//	int i = 0;
//	while (1) {
//		if (i==period)
//		{
//			usb_out[0] = '<';
//			CDC_Transmit_FS(usb_out, 1);
//			HAL_Delay(50);
//			move_angle(SERVO_ID, -angle, 000, H_LED_WHITE);
//		}
//		if (i==period*2) {
//			usb_out[0] = '>';
//			CDC_Transmit_FS(usb_out, 1);
//			HAL_Delay(50);
//			move_angle(SERVO_ID, angle, 000, H_LED_BLUE);
//			i = 0;
//		}
//		HAL_Delay(50);
//		i++;
//	}
//}

// oscillate between +/- angle
//void oscillate(int angle, int period)
//{
//	// check if error is raised and reset if so
//	reset_and_zero_pos();
//	int i = 0;
//	while (1) {
//		if (i==period)
//		{
//			move_angle(SERVO_ID, -angle, 000, H_LED_WHITE);
//		}
//		if (i==period*2) {
//			move_angle(SERVO_ID, angle, 000, H_LED_BLUE);
//			i = 0;
//		}
//		HAL_Delay(50);
//		i++;
//	}
//}

// rotate servo left and right for '<' and '>' received over serial
//void serial_pos_command()
//{
//	// check if error is raised and reset if so
//	reset_and_zero_pos();
//	while (1)
//	{
//		// rotate -45 deg if < is received from PC
//		if (usb_in[0] == '<')
//		{
//			move_angle(SERVO_ID, -45, 000, H_LED_WHITE);
//			memset(usb_in, '\0', 64);
//		}
//		// rotate 45 deg if > is received from PC
//		if (usb_in[0] == '>')
//		{
//			move_angle(SERVO_ID, 45, 000, H_LED_BLUE);
//			memset(usb_in, '\0', 64);
//		}
//		HAL_Delay(50);
//	}
//}

// get the servo position (raw, bytes) and send over serial
// must be converted to raw 10-bit or angle on the receiving end
//void serial_send_pos()
//{
//	uint8_t position_data_array[2]; // position data is contained in 2 bytes
//	get_position_bytes(SERVO_ID, position_data_array);
//	usb_out[0] = position_data_array[0];
//	usb_out[1] = position_data_array[1];
//	CDC_Transmit_FS(usb_out, 2);
//}

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
  MX_USB_DEVICE_Init();
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
	txHeader.StdId = 0x030;
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
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // initialize CAN bus RX interrupt
	{
		Error_Handler();
	}

	HAL_Delay(2000);
	herkulex_init();
	move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
	HAL_Delay(2000);

	int my_command = 0;
	int i = 0;
  int index = 0;
  int iter = 0;
  uint8_t request_packet[] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// get encoder initial reading and use to calibrate
	if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, RAW_ANGLE_L, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	left = I2C_buffer[0];
	if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, RAW_ANGLE_R, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	right = I2C_buffer[0];

	I2C_buffer[0] = left;
	if (HAL_I2C_Mem_Write(&hi2c2, encoder_address, ZPOSL, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	I2C_buffer[0] = right;
	if (HAL_I2C_Mem_Write(&hi2c2, encoder_address, ZPOSR, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
		Error_Handler();
	HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  	// check if error is raised and reset if so
  	reset_and_zero_pos();

  	// send request to other segments over CAN for their data
		HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);
		if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, request_packet, &canMailbox) != HAL_OK) // Send Message
		{
			Error_Handler();
		}
		HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);

		// receive state data delay
		HAL_Delay(25);

		__disable_irq();
		for (i = 0; i < sizeof state_buffer; i++)
		{
			state_buffer[i] = CAN_RX_buffer[i];
		}
		__enable_irq();

  	// add own data
  	// get position data (2 bytes each) and add to packet
  	get_position_bytes(SERVO_ID, position_data_array);
  	my_state_buffer[0] = position_data_array[0];
  	my_state_buffer[1] = (position_data_array[1] | SERIAL_ENCODE_MASK); // encode servo data MSB to create contrast to encoder data

  	// get encoder reading (2 bytes each) and add to packet
		if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, ANGLE_L, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		left = I2C_buffer[0];
		if (HAL_I2C_Mem_Read(&hi2c2, encoder_address, ANGLE_R, 1, I2C_buffer, 1, HAL_MAX_DELAY) != HAL_OK)
			Error_Handler();
		right = I2C_buffer[0];
		my_state_buffer[2] = right;
		my_state_buffer[3] = left;

//		for (int iter = 0; iter > sizeof my_state_buffer; iter++)
//    {
//      usb_out[index] = my_state_buffer[iter];
//      index++;
//    }
//    for (int iter = 0; iter > sizeof state_buffer; iter++)
//    {
//      usb_out[index] = my_state_buffer[iter];
//      index++;
//    }
//    index = 0;
		// memcpy(&usb_out, &my_state_buffer, sizeof my_state_buffer);
		// memcpy(&usb_out + sizeof my_state_buffer, &state_buffer, sizeof state_buffer);

    for (iter = 0; iter < sizeof my_state_buffer; iter++)
    {
      usb_out[index] = my_state_buffer[iter];
      index++;
    }
    for (iter = 0; iter < sizeof state_buffer; iter++)
    {
      usb_out[index] = state_buffer[iter];
      index++;
    }
    index = 0;

		// send state data to PC
		CDC_Transmit_FS(usb_out, packet_len);

		// memset(my_state_buffer, 0x0, sizeof(my_state_buffer));
		// memset(state_buffer, 0x0, sizeof(state_buffer));

		// check if command is received and if so, execute it
  	if (usb_in[0] != 0 && usb_in[1] != 0) // i.e. if usb_in not empty
  	{
  		HAL_GPIO_WritePin(BLUE_GPIO_PORT, BLUE_LED, GPIO_PIN_SET);
  		// grab own command
  		my_command = ((usb_in[1] & 0x03) << 8) | usb_in[0];

  		// send rest over CAN
  		for (i = 2; i < ((N_JOINTS*POSITION_DATA_SIZE)-1); i += 2)
  		{
  			uint8_t csend[] = {i/2, usb_in[i], usb_in[i+1]};
				if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) != HAL_OK) // send message
				{
					Error_Handler();
				}
  		}
			move_positional(SERVO_ID, my_command, 100, H_LED_WHITE);
			memset(usb_in, '\0', sizeof usb_in);
			HAL_GPIO_WritePin(BLUE_GPIO_PORT, BLUE_LED, GPIO_PIN_RESET);
  	}

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
static void MX_CAN1_Init(void) {
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
		Error_Handler();
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
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, CAN_RX_buffer) != HAL_OK) // receive CAN bus message in CAN Rx buffer
	{
		Error_Handler();
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
