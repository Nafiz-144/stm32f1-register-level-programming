#include "stm32f10x.h"

void PWM_Init(void)
{
    // 1. Enable GPIOA and TIM2 clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // GPIOA clock enable
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // TIM2 clock enable

    // 2. Configure PA0 as Alternate Function Push-Pull (TIM2_CH1)
    GPIOA->CRL &= ~(0xF << 0);    // Clear CNF0[1:0] and MODE0[1:0]
    GPIOA->CRL |=  (0xB << 0);    // MODE0=11 (50MHz output), CNF0=10 (AF Push-Pull)

    // 3. Configure TIM2 for PWM mode
    TIM2->PSC = 72 - 1;        // Prescaler: 72 MHz / 72 = 1 MHz timer clock
    TIM2->ARR = 1000 - 1;      // Auto-reload value ? 1 kHz PWM frequency
    TIM2->CCR1 = 0;            // Initial duty cycle = 0%

    TIM2->CCMR1 &= ~(7 << 4);  // Clear OC1M bits
    TIM2->CCMR1 |= (6 << 4);   // OC1M = 110 ? PWM mode 1
    TIM2->CCMR1 |= (1 << 3);   // OC1PE = 1 (preload enable)

    TIM2->CCER  |= (1 << 0);   // Enable output for channel 1 (CC1E = 1)

    TIM2->CR1   |= (1 << 7);   // ARPE = 1 (Auto-reload preload enable)
    TIM2->EGR   |= (1 << 0);   // UG = 1 (Generate update event)
    TIM2->CR1   |= (1 << 0);   // CEN = 1 (Start the counter)
}

void delay(volatile uint32_t time)
{
    while (time--);
}

int main(void)
{
    PWM_Init();

    while (1)
    {
        // Fade In (0% ? 100%)
        for (int i = 0; i < 1000; i++)
        {
            TIM2->CCR1 = i;     // Change duty cycle
            delay(100000);
        }

        // Fade Out (100% ? 0%)
        for (int i = 1000; i > 0; i--)
        {
            TIM2->CCR1 = i;     // Change duty cycle
            delay(100000);
        }
    }
}
