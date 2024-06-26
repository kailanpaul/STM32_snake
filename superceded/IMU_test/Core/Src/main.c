#include "stm32f4xx_hal.h"
#include "usb_device.h"

CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C3_Init(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

void Error_Handler(void);

CAN_RxHeaderTypeDef rxHeader; // CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader; // CAN Bus Transmit Header
uint8_t canRX[8];             // CAN Bus Receive Buffer
CAN_FilterTypeDef canfil;     // CAN Bus Filter
uint32_t canMailbox;          // CAN Bus Mail box variable

uint8_t usb_in[1];
uint8_t usb_out[18];
uint16_t len = sizeof(usb_out) / sizeof(usb_out[0]);

uint8_t IMU_address =  0b11010000; //left shifted by 1 - not sure why
uint8_t I2C_buffer[1];

uint16_t x;
uint16_t y;
uint16_t z;

#define BLK_SEL_W 0x79
#define MADDR_W 0x7A
#define M_W 0x7B
#define BLK_SEL_R 0x7C
#define MADDR_R 0x7D
#define M_R 0x7E
#define PWR_MGMT0 0x1F
#define ACCEL_DATA_X1 0x0B
#define ACCEL_DATA_X0 0x0C
#define ACCEL_DATA_Y1 0x0D
#define ACCEL_DATA_Y0 0x0E
#define ACCEL_DATA_Z1 0x0F
#define ACCEL_DATA_Z0 0x10
#define I2C_TIMEOUT 1000

#define YELLOW_LED GPIO_PIN_13
#define YELLOW_GPIO_PORT GPIOC
#define RED_LED GPIO_PIN_14
#define RED_GPIO_PORT GPIOC
#define BLUE_LED GPIO_PIN_15
#define BLUE_GPIO_PORT GPIOB

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USB_DEVICE_Init();
    MX_I2C3_Init();

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

    if (HAL_CAN_ConfigFilter(&hcan1, &canfil) != HAL_OK) // Initialize CAN Filter
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) // Initialize CAN Bus
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // Initialize CAN Bus Rx Interrupt
    {
        Error_Handler();
    }

//    uint8_t csend[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // Tx Buffer


    //Turn on IMU accelerometer
    I2C_buffer[0] = 0b11;
	if (HAL_I2C_Mem_Write(&hi2c3, IMU_address, PWR_MGMT0, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);


    while (1)
    {
//        if (usb_in[0] == 'x')
//        {
//            HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_SET);
//            //			uint8_t csend[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; // Tx Buffer
//            if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox) != HAL_OK) // Send Message
//            {
//                Error_Handler();
//            }
//            HAL_Delay(10);
//            HAL_GPIO_WritePin(YELLOW_GPIO_PORT, YELLOW_LED, GPIO_PIN_RESET);
//            memset(usb_in, '\0', 64); // clear buffer
//        }
//    	HAL_Delay(2000);
//    	uint8_t buffer[1];
//    	buffer[0] = BLK_SEL_W;
//    	buffer[1] = 0x00;
//    	HAL_I2C_Master_Transmit(&hi2c3, IMU_address, buffer, sizeof buffer, I2C_TIMEOUT);
//    	buffer[0] = MADDR_R;
//		buffer[1] = 0x14;
//		HAL_I2C_Master_Transmit(&hi2c3, IMU_address, buffer, sizeof buffer, I2C_TIMEOUT);
//		HAL_Delay(1);
////		buffer[0] = M_R;
////		buffer[1] = 0;
//    	HAL_I2C_Master_Receive(&hi2c3, IMU_address, M_R, 1, I2C_TIMEOUT);

    	HAL_Delay(1);


    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_X1, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
    		Error_Handler();
    	x = I2C_buffer[0] << 8;
    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_X0, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
			Error_Handler();
    	x |= I2C_buffer[0];
    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_Y1, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
			Error_Handler();
    	y = I2C_buffer[0] << 8;
    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_Y0, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
			Error_Handler();
    	y |= I2C_buffer[0];
    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_Z1, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
			Error_Handler();
    	z = I2C_buffer[0] << 8;
    	if (HAL_I2C_Mem_Read(&hi2c3, IMU_address, ACCEL_DATA_Z0, 1, I2C_buffer, 1, I2C_TIMEOUT) != HAL_OK)
			Error_Handler();
    	z |= I2C_buffer[0];

//    	CDC_Transmit_FS("1", 1);

    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK) // Receive CAN bus message to canRX buffer
    {
        Error_Handler();
    }
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_CAN1_Init(void)
{
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
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{
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
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
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

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    	HAL_GPIO_TogglePin(RED_GPIO_PORT, RED_LED);
    	HAL_Delay(500);
    }
}

#ifdef USE_FULL_ASSERT
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
