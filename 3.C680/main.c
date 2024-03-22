#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    // 开启CAN接收中断和过滤器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterBank = 0;  // 用于CAN1的滤波器组编号
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterIdHigh = (0x200 << 5);  // ID为0x200，左移5位因为ID高位在FilterIdHigh中
    canFilter.FilterIdLow = 0;
    canFilter.FilterMaskIdHigh = 0xFFF << 5;
    canFilter.FilterMaskIdLow = 0;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &canFilter);
    HAL_CAN_Start(&hcan1);

    while (1) {
        // 循环体内不需要做任何事情，所有操作都在中断回调函数中完成
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    char uartBuf[50];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        if ((RxHeader.StdId >= 0x201) && (RxHeader.StdId <= 0x20F)) {  // 确保ID在0x201到0x20F范围内
            uint16_t rotorAngle = (RxData[0] << 8) | RxData[1];
            uint16_t rotorSpeed = (RxData[2] << 8) | RxData[3];
            int16_t torqueCurrent = (RxData[4] << 8) | RxData[5];
            uint8_t motorTemp = RxData[6];
            // 格式化为字符串
            sprintf(uartBuf, "Angle: %u, Speed: %u, Torque: %d, Temp: %u\r\n", rotorAngle, rotorSpeed, torqueCurrent, motorTemp);
            // 通过UART发送
            HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
        }
    }
}

// 以下是HAL库函数的配置，具体实现取决于您的硬件配置和STM32CubeMX工具的设置
void SystemClock_Config(void) {
    // 此处添加系统时钟配置代码
}

static void MX_GPIO_Init(void) {
    // 初始化GPIO
}

static void MX_CAN1_Init(void) {
    // CAN初始化代码
}

static void MX_USART2_UART_Init(void) {
    // UART初始化代码
}
