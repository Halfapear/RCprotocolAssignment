#include "stm32f4xx_hal.h"

// 定义指令的CAN ID
#define MOTOR_ON_CMD_ID        (0x200 + BOARD_ID)
#define VELCFG_CMD_ID          (0x200 + BOARD_ID)
#define VELCTRL_CMD_ID         (0x200 + BOARD_ID)

// 定义boardID和motorID
#define BOARD_ID 1
#define MOTOR_ID 1

// 速度环模式配置
void sendVelCfgCommand(CAN_HandleTypeDef *hcan) {
    uint8_t data[8] = {0}; // 初始化为0
    data[0] = MOTOR_ID;    // D0为MOTOR_ID
    data[1] = MOTOR_ID;    // D1为MOTOR_ID
    data[3] = 3; 
    data[2] = 3; 
    sendCanCommand(hcan, VELCFG_CMD_ID, data, 8); // 发送数据长度为8
}

// 使能电机
void enableMotor(CAN_HandleTypeDef *hcan) {
    uint8_t data[8] = {0}; // 初始化为0
    data[0] = MOTOR_ID;    // D0为MOTOR_ID
    data[1] = MOTOR_ID;    // D1为MOTOR_ID
    data[3] = 1; 
    data[2] = 1; 
    sendCanCommand(hcan, MOTOR_ON_CMD_ID, data, 8); // 发送数据长度为8
}

// 控制电机速度
void controlMotorVelocity(CAN_HandleTypeDef *hcan, int16_t vel) {
    uint8_t data[8] = {0}; // 初始化为0
    //我不能太确定 0 1 都是MOTOR_ID是啥意思——我忘记它为1了
    data[0] = MOTOR_ID;    // D0为MOTOR_ID
    data[1] = MOTOR_ID;    // D1为MOTOR_ID
    // data[0] = (uint8_t)(speed >> 8); // D1为速度的高字节
    // data[1] = (uint8_t)(speed & 0xFF); // D2为速度的低字节
    data[3] = 6; 
    data[2] = 6; 

    data[4] = vel; 
    data[5] = vel; 
    data[6] = vel; 
    data[7] = vel; 

    sendCanCommand(hcan, VELCTRL_CMD_ID, data, 8); // 发送数据长度为8
}

void sendCanCommand(CAN_HandleTypeDef *hcan, uint32_t cmdID, uint8_t *cmdData, uint8_t dataSize) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    txHeader.StdId = cmdID; // 标准ID
    txHeader.ExtId = 0x00; // 扩展ID未使用
    txHeader.RTR = CAN_RTR_DATA; // 数据帧
    txHeader.IDE = CAN_ID_STD; // 使用标准ID
    txHeader.DLC = dataSize; // 数据长度
    txHeader.TransmitGlobalTime = DISABLE; // 不使用全局时间

    if (HAL_CAN_AddTxMessage(hcan, &txHeader, cmdData, &txMailbox) != HAL_OK) {
        // 如果消息添加失败，进行错误处理
        Error_Handler();
    }
}

int main(void) {
    // HAL初始化和配置代码
    HAL_Init(); // 初始化HAL库
    SystemClock_Config(); // 配置系统时钟
    MX_CAN1_Init(); // 初始化CAN1
    HAL_CAN_Start(&hcan1); // 启动CAN1
    
    //下面这个函数在另一个文件夹里面
    my_can_filter_init_recv_all(&hcan1); // 初始化CAN过滤器，接收所有报文
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 激活CAN消息挂起中断通知
    
    // 使能电机
    enableMotor(&hcan1);
    
    // 发送velCfg指令
    sendVelCfgCommand(&hcan1);
    
    
    // 主循环
    while (1) {
        // 控制电机速度
        controlMotorVelocity(&hcan1, vel);
        
        // 延时，根据实际需求调整时间
        HAL_Delay(10);
    }
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

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}