#include "stm32f10x.h"

volatile uint16_t adc_value;
volatile uint8_t tx_buffer[50]; // USART transmit buffer

void rcc_config(void){
    RCC->APB1ENR |=((1<<0)|(1<<1)); //TIM2 & TIM3 CLOCK ENABLE 
    RCC->APB2ENR |=((1<<2)|(1<<9)|(1<<14)); //GPIOA, ADC1 & USART1 CLOCK ENABLE
    RCC->AHBENR |=(1<<0); //DMA1 CLOCK ENABLE
}

void gpio_config(void){
    // PA0 – Ultrasonic Trigger Pin ? Output Push-Pull
    GPIOA->CRL &= ~((1<<3)|(1<<2)|(1<<0));
    GPIOA->CRL |= (1<<0);

    // PA1 – Ultrasonic Echo Pin ? Input Floating
    GPIOA->CRL &= ~((1<<7)|(1<<6)|(1<<5)|(1<<4));
    GPIOA->CRL |= (1<<6);

    // PA2 – USART1 TX ? AF Push-Pull
    GPIOA->CRL &= ~((1<<11)|(1<<10)|(1<<9)|(1<<8));
    GPIOA->CRL |= ((1<<11)|(1<<8));

    // PA3 – USART1 RX ? Input Floating
    GPIOA->CRL &= ~((1<<15)|(1<<14)|(1<<13)|(1<<12));
    GPIOA->CRL |= (1<<14);

    // PA4 – ADC Input
    GPIOA->CRL &= ~((1<<19)|(1<<18)|(1<<17)|(1<<16));

    // PA5 – LED / Buzzer Output Push-Pull
    GPIOA->CRL &= ~((1<<23)|(1<<22)|(1<<21)|(1<<20));
    GPIOA->CRL |= (1<<21);
}

void tim2_input_capture_config(void){
    TIM2->PSC = 36-1;
    TIM2->ARR = 0xFFFF;
    TIM2->CCMR1 &= ~((1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<12)|(1<<13)|(1<<14)|(1<<15));
    TIM2->CCMR1 |= (1<<8);
    TIM2->CCER &= ~((1<<4)|(1<<5));
    TIM2->CCER |= (1<<5);
    TIM2->DIER |= (1<<2);
    TIM2->CR1 |= (1<<0);
}

void tim3_wakeup_config(void){
    TIM3->PSC=36000-1;
    TIM3->ARR =1000-1;
    TIM3->DIER |=(1<<0);
    TIM3->CR1 |=(1<<0);
}

void adc_dma_config(void){
    // DMA1 Channel1 ? ADC1
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)&adc_value;
    DMA1_Channel1->CNDTR = 1;
    DMA1_Channel1->CCR = (1<<7); // CIRC=1
    DMA1_Channel1->CCR |= (1<<0); // Enable DMA

    ADC1->SQR3 = 4;
    ADC1->CR2 |= (1<<8) | (1<<1); // DMA=1, ADON=1
    ADC1->CR2 |= (1<<0);           // SWSTART
}

void usart_dma_config(void){
    USART1->BRR = 0x0EA6;       // 9600 bps
    USART1->CR1 |= (1<<3) | (1<<2);  // TE, RE
    USART1->CR1 |= (1<<13);          // UE
    USART1->CR3 |= (1<<7);           // DMAT

    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uint32_t)tx_buffer;
    DMA1_Channel4->CNDTR = 0;
    DMA1_Channel4->CCR = (1<<7) | (1<<4); // CIRC=1, MINC=1
}

void nvic_config(void){
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void usart_dma_send(uint8_t *data, uint16_t size){
    for(uint16_t i=0;i<size;i++)
        tx_buffer[i] = data[i];
    DMA1_Channel4->CNDTR = size;
    DMA1_Channel4->CCR |= (1<<0);
}

void enter_sleep_mode(void){
    SCB->SCR &= ~(1<<2); // normal sleep
    __enable_irq();
    __asm volatile("WFI");
}
int main(void){
    rcc_config();
    gpio_config();
    tim2_input_capture_config();
    tim3_wakeup_config();
    adc_dma_config();
    usart_dma_config();
    nvic_config();

    while(1){
        enter_sleep_mode(); // CPU sleeps, wakes on interrupt
    }
}/*
#include<stdio.h>
#include "stm32f10x.h"

volatile uint16_t adc_value = 0;

void rcc_gpio_adc_init(void){
    // 1. Enable clocks
    RCC->APB2ENR |= (1<<2) | (1<<9); // GPIOA + ADC1

    // 2. Configure PA4 as Analog Input
    GPIOA->CRL &= ~((1<<19)|(1<<18)|(1<<17)|(1<<16)); // MODE=00, CNF=00
    // ADC clock (PCLK2 / 6) default ~6MHz (safe)
    
    // 3. ADC1 Configuration
    ADC1->SQR3 = 4;           // First conversion = channel 4 (PA4)
    ADC1->CR2 |= (1<<0);      // ADON = 1 (turn on ADC)
}

uint16_t adc_read(void){
    ADC1->CR2 |= (1<<0);      // ADON again to start conversion (first time)
    ADC1->CR2 |= (1<<22);     // SWSTART
    while(!(ADC1->SR & (1<<1))); // Wait for EOC
    return ADC1->DR;
}

void rcc_gpio_usart_init(void){
    RCC->APB2ENR |= (1<<2) | (1<<14);  // GPIOA & USART1 Clock
    // PA2 -> TX
    GPIOA->CRL &= ~((1<<11)|(1<<10)|(1<<9)|(1<<8));
    GPIOA->CRL |= ((1<<11)|(1<<8)); 
    // PA3 -> RX
    GPIOA->CRL &= ~((1<<15)|(1<<14)|(1<<13)|(1<<12));
    GPIOA->CRL |= (1<<14);           

    USART1->BRR = 0x0EA6;     // 9600 @ 36MHz
    USART1->CR1 |= (1<<3) | (1<<2); // TE, RE
    USART1->CR1 |= (1<<13);         // UE
}

void usart_send_string(char *str){
    while(*str){
        while(!(USART1->SR & (1<<7)));
        USART1->DR = *str++;
    }
}

int main(void){
    char buffer[30];
    rcc_gpio_adc_init();
    rcc_gpio_usart_init();

    while(1){
        adc_value = adc_read(); // Read ADC
        int temp_c = (adc_value * 3300 / 4095) / 10; // LM35: 10mV/°C, 3.3V ref
        int len = sprintf(buffer, "Temp = %d C\r\n", temp_c);
        usart_send_string(buffer);

        for(volatile int i=0;i<1000000;i++); // Delay
    }
}
*/

