/*#include "stm32f10x.h"                  // Device header

void gpio_init(){
  RCC->APB2ENR |=(1<<2);
	GPIOA->CRL &=~((1<<7)|(1<<6)|(1<<5)|(1<<4));
	GPIOA->CRL |=(1<<4); //OUTPUT PUSH PULL(PA1)
  GPIOA->CRL &=~((1<<11)|(1<<10)|(1<<9)|(1<<8));
	GPIOA->CRL |=(1<<8); //INPUT FLOATING(PA2)(for Timer Input Capture)
	GPIOA->CRL &=~((1<<23)|(1<<22)|(1<<21)|(1<<20));
	GPIOA->CRL |=(1<<20); //OUTPUT PUSH PULL (LED)(PA5)
	GPIOA->CRH &=~((1<<7)|(1<<6)|(1<<5)|(1<<4));
	GPIOA->CRH |=((1<<7)|(1<<5)|(1<<4)); //Alternate Function Output Push-Pull (USART1 TX)(PA9)
   
}
void timer2_input_capture_init(void){
  RCC->APB1ENR |=(1<<1);//ENABLE TIM2
	TIM2->PSC =72-1; // Prescaler  72 MHz / 72 = 1 MHz
	TIM2->ARR =65535; // Max auto-reload (16-bit full range)
	
	




}


int main(){}
*/

#include "stm32f10x.h"

#define LOG_BUFFER_SIZE 100
char logBuffer[LOG_BUFFER_SIZE];

void gpio_init(void) {
    RCC->APB2ENR |= (1 << 2); // Enable GPIOA clock

    // PA1: Output Push-Pull (TRIG)
    GPIOA->CRL &= ~(0xF << (1 * 4));  // Clear bits for PA1
    GPIOA->CRL |=  ((1 << 0) | (0 << 2)); // MODE1=01 (Output 10MHz), CNF1=00 (Push-Pull)

    // PA2: Input Floating (TIM2_CH3)
    GPIOA->CRL &= ~(0xF << (2 * 4));  // Clear bits for PA2
    GPIOA->CRL |=  ((0 << 0) | (1 << 2)); // MODE2=00 (Input), CNF2=01 (Floating)

    // PA5: Output Push-Pull (LED)
    GPIOA->CRL &= ~(0xF << (5 * 4));  // Clear bits for PA5
    GPIOA->CRL |=  ((1 << 0) | (0 << 2)); // MODE5=01 (Output 10MHz), CNF5=00 (Push-Pull)

    // PA9: Alternate Function Push-Pull (USART1 TX)
    RCC->APB2ENR |= (1 << 14);         // Enable USART1 clock (needed for AFIO)
    GPIOA->CRH &= ~(0xF << ((9 - 8) * 4));
    GPIOA->CRH |=  ((1 << 0) | (1 << 2)); // MODE9=10 (Output 2MHz), CNF9=10 (AF Push-Pull)
}

void usart1_init(void) {
    RCC->APB2ENR |= (1 << 14);   // USART1 clock enable

    // Baud rate = 115200 (assuming 72MHz PCLK2)
    USART1->BRR = 0x271; // 72000000/115200 = 625 (0x271)

    USART1->CR1 = (1 << 3) | (1 << 13); // TE=1 (Transmitter enable), UE=1 (USART enable)
    USART1->CR3 = (1 << 7);              // DMAT=1 (DMA transmit enable)
}

void dma1_channel4_init(uint8_t *buffer, uint16_t length) {
    RCC->AHBENR |= (1 << 0); // DMA1 clock enable

    DMA1_Channel4->CCR &= ~(1 << 0); // Disable DMA channel

    DMA1_Channel4->CPAR = (uint32_t) &USART1->DR; // Peripheral address
    DMA1_Channel4->CMAR = (uint32_t) buffer;       // Memory address
    DMA1_Channel4->CNDTR = length;                  // Number of data items

    DMA1_Channel4->CCR = (1 << 7)  | // Memory increment mode
                        (0 << 6)  | // Peripheral size 8-bit
                        (0 << 5)  | // Memory size 8-bit
                        (0 << 4)  | // Circular mode disabled
                        (1 << 1);   // Direction: Memory to Peripheral

    DMA1_Channel4->CCR |= (1 << 0); // Enable DMA channel
}

void timer2_input_capture_init(void) {
    RCC->APB1ENR |= (1 << 0); // Enable TIM2 clock

    TIM2->PSC = 72 - 1;        // Prescaler: 72MHz/72=1MHz timer clock
    TIM2->ARR = 0xFFFF;        // Max auto-reload value

    TIM2->CCMR2 &= ~(0xFF);    // Clear CC3 channel config
    TIM2->CCMR2 |= (1 << 0);   // CC3S = 01 (input, mapped to TI3)
    // IC3F=0000 (no filter), IC3PSC=00 (capture every edge) are default cleared

    TIM2->CCER |= (1 << 8);    // CC3E = 1 (enable capture)
    TIM2->CCER &= ~(1 << 9);   // CC3P = 0 (rising edge)

    TIM2->DIER |= (1 << 13);   // CC3DE = 1 enable DMA request on capture

    TIM2->CR1 |= (1 << 0);     // CEN = 1, enable TIM2
}

void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) {
        __NOP();
    }
}

void send_trigger_pulse(void) {
    GPIOA->BSRR = (1 << 1);  // PA1 high
    delay_us(10);            // 10 microseconds delay
    GPIOA->BRR = (1 << 1);   // PA1 low
}

int main(void) {
    gpio_init();
    usart1_init();
    timer2_input_capture_init();

    while (1) {
        send_trigger_pulse();

        // ????? ???? Timer2->CCR3 ???? ???????? ?????? ???? distance ????? ????
        // ????? UART ???? ???? ?????? ????? ????? ??? ???? ?????? ????

        char msg[] = "Distance measurement\r\n";
        dma1_channel4_init((uint8_t *)msg, sizeof(msg) - 1);

        // Delay ???????? (Avoid continuous flooding)
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}

