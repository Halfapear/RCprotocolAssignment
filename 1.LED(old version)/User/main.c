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
//        HAL_Delay(1000); // 仅演示，实际使用中可以去掉延时
//        
//        LED_Off();
//        HAL_Delay(1000); // 仅演示
//        
//        LED_Blink_2Hz();
        
//        LED_Breathe();
			//舍弃Hal库的测试（没成功）
			GPIOF->MODER &= ~(3UL << (9 * 2));  // 清除之前的设置
			GPIOF->MODER |= (1UL << (9 * 2));   // 设置为输出模式	
			GPIOF->ODR |= (1UL << 9);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // 使能电源控制时钟
    __HAL_RCC_PWR_CLK_ENABLE();

    // 设置电压调节器输出电压级别1，以支持芯片的最大性能
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // 配置主PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8; // 预分频因子，用于确保VCO输入频率范围
    RCC_OscInitStruct.PLL.PLLN = 336; // VCO输出频率 = PLL输入频率 * PLLN = 8MHz * 336 = 336MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 系统时钟 = VCO输出频率 / PLLP = 336MHz / 2 = 168MHz
    RCC_OscInitStruct.PLL.PLLQ = 7; // USB、SDIO、随机数生成器频率 = VCO输出频率 / PLLQ = 336MHz / 7 = 48MHz

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
				LED_Off();
        // 初始化错误处理
    }

    // 初始化CPU、AHB和APB总线时钟
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB分频器为1，AHB时钟 = 系统时钟 = 168MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // APB1分频器为4，APB1时钟 = AHB时钟 / 4 = 42MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // APB2分频器为2，APB2时钟 = AHB时钟 / 2 = 84MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        // 初始化错误处理
    }
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
    for(int i = 0; i < 3; ++i) { // 演示3次呼吸周期
        for(int j = 0; j < 5; ++j) { // 逐渐减少延时，模拟呼吸效果
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(400 - (j * 80)); // 逐渐减少点亮时间
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(100 + (j * 80)); // 逐渐增加熄灭时间
        }
        for(int j = 4; j >= 0; --j) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
            HAL_Delay(400 - (j * 80));
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
            HAL_Delay(100 + (j * 80));
        }
    }
}
