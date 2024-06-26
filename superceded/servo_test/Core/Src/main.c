#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "Herkulex.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

void Error_Handler(void);

CAN_RxHeaderTypeDef rxHeader; //CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader; //CAN Bus Transmit Header
uint8_t CAN_RX_buffer[8];  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

uint8_t usb_in[1];
uint8_t usb_out[18];
uint16_t len = sizeof(usb_out) / sizeof(usb_out[0]);
//uint8_t buf[] = "test";
//uint16_t len = sizeof(buf)/sizeof(buf[0]);

uint8_t check = 0;

#define YELLOW_LED                             GPIO_PIN_13
#define YELLOW_GPIO_PORT                       GPIOC
#define RED_LED                                GPIO_PIN_14
#define RED_GPIO_PORT                          GPIOC
#define BLUE_LED                               GPIO_PIN_15
#define BLUE_GPIO_PORT                         GPIOB
#define SERVO_ID							   253

int main(void) {
 	HAL_Init();

	SystemClock_Config();
	SystemCoreClockUpdate();

	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_USB_DEVICE_Init();
	MX_UART4_Init();

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

	if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK) //Initialize CAN Filter
		Error_Handler();

	if (HAL_CAN_Start(&hcan1) != HAL_OK) //Initialize CAN Bus
		Error_Handler();
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) // Initialize CAN Bus Rx Interrupt
		Error_Handler();

//	uint8_t csend[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }; // Tx Buffer
//
//	uint16_t count = 0;
	HAL_Delay(5000);
	herkulex_init();
	int i = 0;
	move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
	HAL_Delay(1000);
	while (1) {
//		uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer
//		if (HAL_CAN_AddTxMessage(&hcan1,&txHeader,csend,&canMailbox) != HAL_OK) // Send Message
//		{
//			Error_Handler();
//		}
//		if (check)
//		{
////			if (canRX[1] == 2)
////			{
//		while (CDC_Transmit_FS(usb_out, len) != USBD_OK);
////			}
//			while (CDC_Transmit_FS(canRX, sizeof(canRX)/sizeof(canRX[0])) != USBD_OK);
//			check = 0;
//		}
//		if(usb_in[0] == 'x')
//		{
//			HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);
////			uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer
//			if (HAL_CAN_AddTxMessage(&hcan1,&txHeader,csend,&canMailbox) != HAL_OK) // Send Message
//			{
//				Error_Handler();
//			}
//			HAL_Delay(10);
//			HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);
//			memset(usb_in, '\0', 64); // clear buffer
//		}

//		if (HAL_CAN_AddTxMessage(&hcan1,&txHeader,csend,&canMailbox) != HAL_OK) // Send Message
//		{
//			Error_Handler();
//		}
//		if (++count == 1000){
//			HAL_GPIO_TogglePin(BLUE_GPIO_PORT, BLUE_LED);
		//			count = 0;
//		}
//		move_continuous(4, 500, H_LED_BLUE);
//		HAL_Delay(1000);

		if(get_status(SERVO_ID)) {
			clear_error(SERVO_ID);
			torque_on(SERVO_ID);
			move_angle(SERVO_ID, 0, 0, H_LED_GREEN);
			HAL_Delay(1000);
		}

		if (i==80)
		{
			usb_out[0] = '<';
			CDC_Transmit_FS(usb_out, 3);
			HAL_Delay(50);
			move_angle(SERVO_ID, -40, 000, H_LED_WHITE);
		}
		if (i==160) {
			usb_out[0] = '>';
			CDC_Transmit_FS(usb_out, 3);
			HAL_Delay(50);
			move_angle(SERVO_ID, 40, 000, H_LED_BLUE);
			i = 0;
		}

		HAL_Delay(50);
		i++;
	}
//		get_position(253);



}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	uint8_t buf[20];
//	buf = *huart4.pRxBuffPtr;
//	if (HAL_UART_Receive_IT(&huart4, UART_RX_buffer, UART_RX_buffer_size) != HAL_OK)
//		Error_Handler();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, CAN_RX_buffer)
			!= HAL_OK) //Receive CAN bus message to canRX buffer
		Error_Handler();
//	CDC_Transmit_FS("ID: ", 4);
//	while (CDC_Transmit_FS("ID: ", 4) != USBD_OK);
//	while (CDC_Transmit_FS(rxHeader.StdId, 1) != USBD_OK);
//	while (CDC_Transmit_FS("| ", 2) != USBD_OK);
//	while (CDC_Transmit_FS(canRX, sizeof(canRX)/sizeof(canRX[0])) != USBD_OK);
//	char buf[8];
//	if (rxHeader.StdId > 9)
//	{
//		rxHeader.StdId += 55;
//	} else {
//		rxHeader.StdId += 48;
//	}
//	sprintf(buf, "ID: %lu\r\n", rxHeader.StdId);
//	char val[] = " ";
//	val[0] = (char) rxHeader.StdId;
//	strcat(buf, val);
//	CDC_Transmit_FS((uint8_t*)buf, 8);
//	rxHeader.StdId += 48;
//	CDC_Transmit_FS((uint8_t*)&rxHeader.StdId, 1);
//	CDC_Transmit_FS("1", 1);
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
		Error_Handler();

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();

}

static void MX_CAN1_Init(void) {
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 9;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_2TQ; //10 and 5 gives 250k, 5 and 2 gives 500k, 2 and 1 gives 1M
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_UART4_Init(void) {
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
		Error_Handler();

}

static void MX_GPIO_Init(void) {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct_BLUE;
	GPIO_InitStruct_BLUE.Pin = BLUE_LED;
	GPIO_InitStruct_BLUE.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_BLUE.Pull = GPIO_PULLUP;
	GPIO_InitStruct_BLUE.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(BLUE_GPIO_PORT, &GPIO_InitStruct_BLUE);

	GPIO_InitTypeDef GPIO_InitStruct_YELLOW;
	GPIO_InitStruct_YELLOW.Pin = YELLOW_LED;
	GPIO_InitStruct_YELLOW.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_YELLOW.Pull = GPIO_PULLUP;
	GPIO_InitStruct_YELLOW.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(YELLOW_GPIO_PORT, &GPIO_InitStruct_YELLOW);

	GPIO_InitTypeDef GPIO_InitStruct_RED;
	GPIO_InitStruct_RED.Pin = RED_LED;
	GPIO_InitStruct_RED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct_RED.Pull = GPIO_PULLUP;
	GPIO_InitStruct_RED.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(RED_GPIO_PORT, &GPIO_InitStruct_RED);
}

void Error_Handler(void) {
	__disable_irq();
	HAL_GPIO_WritePin(RED_GPIO_PORT, RED_LED, GPIO_PIN_SET);
	while (1) {
	}
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
