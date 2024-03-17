#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
        HAL_Delay(500); // 500ms延时，产生大约1Hz的闪烁频率
    }
}

void SystemClock_Config(void) {
    // 这里应配置系统时钟，具体配置依赖于您的需求
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOF_CLK_ENABLE(); // 使能GPIOF时钟

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9; // 设置要配置的引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 设置速度
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); // 初始化GPIO
}
