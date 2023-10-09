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
CAN_RxHeaderTypeDef rxHeader; //CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Transmit Header
uint8_t CAN_RX_buffer[8];  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

uint8_t usb_in[1];
uint8_t usb_out[18];
uint16_t len = sizeof(usb_out) / sizeof(usb_out[0]);
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

//uint8_t IMU_address =  0b01101100; //left shifted by 1 - not sure why
//uint8_t I2C_buffer[1];

//uint16_t left;
//uint16_t right;
//uint16_t offset;
//int raw;
//int raw_angle;
//int deg_angle;
//int dec_angle;

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

	txHeader.DLC = 8; // Number of bytes to be transmitted max- 8
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x030;
	txHeader.ExtId = 0x02;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_ConfigFilter(&hcan1,&canfil) != HAL_OK) //Initialize CAN Filter
	{
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) //Initialize CAN Bus
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// Initialize CAN Bus Rx Interrupt
	{
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//--------------------------------------------------------------------------------

//		if(usb_in[0] == 'x')
//		{
//			usb_out[0] = usb_in[0];
//			HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);
//			HAL_Delay(10);
//			HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);
//			CDC_Transmit_FS(usb_out, 1);
//			memset(usb_in, '\0', 64); // clear buffer
//		}

//--------------------------------------------------------------------------------

//		HAL_Delay(100); // let the serial monitor catch up
//		herkulex_init();
//
//		if(get_status(SERVO_ID)) {
//			clear_error(SERVO_ID);
//			torque_on(SERVO_ID);
//			move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
//			HAL_Delay(50);
//		}
//
//		int i = 0;
//
//		while (1) {
//
//			if (i==50)
//			{
//				usb_out[0] = '<';
//				CDC_Transmit_FS(usb_out, 1);
//				HAL_Delay(50);
//				move_angle(SERVO_ID, -45, 000, H_LED_WHITE);
//			}
//			if (i==100) {
//				usb_out[0] = '>';
//				CDC_Transmit_FS(usb_out, 1);
//				HAL_Delay(50);
//				move_angle(SERVO_ID, 45, 000, H_LED_BLUE);
//				i = 0;
//			}
//
//			HAL_Delay(50);
//			i++;
//		}

	  //--------------------------------------------------------------------------------

		HAL_Delay(100); // let the serial monitor catch up
		herkulex_init();

		if(get_status(SERVO_ID)) {
			clear_error(SERVO_ID);
			torque_on(SERVO_ID);
			move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
			HAL_Delay(50);
		}

		while (1)
		{

			if (usb_in[0] == '<')
			{
				move_angle(SERVO_ID, -45, 000, H_LED_WHITE);
				memset(usb_in, '\0', 64);
			}

			if (usb_in[0] == '>')
			{
				move_angle(SERVO_ID, 45, 000, H_LED_BLUE);
				memset(usb_in, '\0', 64);
			}

			HAL_Delay(50);
		}

	  //--------------------------------------------------------------------------------
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
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, CAN_RX_buffer) != HAL_OK) //Receive CAN bus message to canRX buffer
	{
		Error_Handler();
	}

//	CDC_Transmit_FS("ID: ", 4);
//	while (CDC_Transmit_FS("ID: ", 4) != USBD_OK);
//	while (CDC_Transmit_FS(rxHeader.StdId, 1) != USBD_OK);
//	while (CDC_Transmit_FS("| ", 2) != USBD_OK);
//	while (CDC_Transmit_FS(canRX, sizeof(canRX)/sizeof(canRX[0])) != USBD_OK);
//	char buf[8];
	if (rxHeader.StdId > 9)
	{
		rxHeader.StdId += 55;
	} else {
		rxHeader.StdId += 48;
	}
//	sprintf(buf, "ID: %lu\r\n", rxHeader.StdId);
//	char val[] = " ";
//	val[0] = (char) rxHeader.StdId;
//	strcat(buf, val);
//	CDC_Transmit_FS((uint8_t*)buf, 8);
//	rxHeader.StdId += 48;
	CDC_Transmit_FS((uint8_t*)&rxHeader.StdId, 1);
//	CDC_Transmit_FS("1", 1);
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
