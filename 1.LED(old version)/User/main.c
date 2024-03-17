#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Blink_2Hz(void);
void LED_Breathe(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
//    GPIO_Init();

    while (1) {
//        LED_On();
//        HAL_Delay(1000); // ����ʾ��ʵ��ʹ���п���ȥ����ʱ
//        
//        LED_Off();
//        HAL_Delay(1000); // ����ʾ
//        
//        LED_Blink_2Hz();
        
//        LED_Breathe();
			//����Hal��Ĳ��ԣ�û�ɹ���
			GPIOF->MODER &= ~(3UL << (9 * 2));  // ���֮ǰ������
			GPIOF->MODER |= (1UL << (9 * 2));   // ����Ϊ���ģʽ	
			GPIOF->ODR |= (1UL << 9);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // ʹ�ܵ�Դ����ʱ��
    __HAL_RCC_PWR_CLK_ENABLE();

    // ���õ�ѹ�����������ѹ����1����֧��оƬ���������
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // ������PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8; // Ԥ��Ƶ���ӣ�����ȷ��VCO����Ƶ�ʷ�Χ
    RCC_OscInitStruct.PLL.PLLN = 336; // VCO���Ƶ�� = PLL����Ƶ�� * PLLN = 8MHz * 336 = 336MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // ϵͳʱ�� = VCO���Ƶ�� / PLLP = 336MHz / 2 = 168MHz
    RCC_OscInitStruct.PLL.PLLQ = 7; // USB��SDIO�������������Ƶ�� = VCO���Ƶ�� / PLLQ = 336MHz / 7 = 48MHz

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
				LED_Off();
        // ��ʼ��������
    }

    // ��ʼ��CPU��AHB��APB����ʱ��
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB��Ƶ��Ϊ1��AHBʱ�� = ϵͳʱ�� = 168MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // APB1��Ƶ��Ϊ4��APB1ʱ�� = AHBʱ�� / 4 = 42MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // APB2��Ƶ��Ϊ2��APB2ʱ�� = AHBʱ�� / 2 = 84MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        // ��ʼ��������
    }
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

void LED_On(void) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
}

void LED_Off(void) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
}

void LED_Blink_2Hz(void) {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
        HAL_Delay(500);
    
}

void LED_Breathe(void) {
    for(int i = 0; i < 3; ++i) { // ��ʾ3�κ�������
        for(int j = 0; j < 5; ++j) { // �𽥼�����ʱ��ģ�����Ч��
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(400 - (j * 80)); // �𽥼��ٵ���ʱ��
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(100 + (j * 80)); // ������Ϩ��ʱ��
        }
        for(int j = 4; j >= 0; --j) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(400 - (j * 80));
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(100 + (j * 80));
        }
    }
}
