#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
        HAL_Delay(500); // 500ms��ʱ��������Լ1Hz����˸Ƶ��
    }
}

void SystemClock_Config(void) {
    // ����Ӧ����ϵͳʱ�ӣ�����������������������
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOF_CLK_ENABLE(); // ʹ��GPIOFʱ��

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9; // ����Ҫ���õ�����
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // �������
    GPIO_InitStruct.Pull = GPIO_NOPULL; // ��ʹ��������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // �����ٶ�
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); // ��ʼ��GPIO
}
