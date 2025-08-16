//Real-Time Voltage Monitoring using ADC + DMA + USART
#include "stm32f10x.h"                  // Device header
#define ADC_BUF_LEN 1
uint16_t adc_buffer[ADC_BUF_LEN];
#include<stdio.h>

void gpio_init(){
	RCC->APB2ENR |=(1<<2); //ENABLE CLK FOR A
	RCC->APB2ENR |=(1<<4); //ENABLE CLK FOR C
	GPIOA->CRH &=~((1<<7)|(1<<6)|(1<<5)|(1<<4));
	GPIOA->CRH |=((1<<7)|(1<<6)|(1<<5));  //ALTENATE FUNCTION PUSH PULL (PA9)
	
	GPIOC->CRH &=~ ((1<<23)|(1<<22)|(1<<21)|(1<<20));
	GPIOC->CRH |=(1<<21); //OUTPUT PUSH PULL(PC13)
	
}
void usart_init(){

  RCC->APB2ENR |=(1<<14); //ENABLE USART CLK
	USART1->BRR = 0x1D4C; // For 9600 baud at 72MHz
	USART1->CR1 |=(1<<3); //ENABLE TRANSMITTER 
	USART1->CR1 |=(1<<13); // ENABLE USART
}
void adc_and_dma_config(){
	RCC->AHBENR |= (1 << 0);  // DMA1 clock enable
	RCC->APB2ENR |=(1<<9); //ENABLE ADC1 CLK
	RCC->APB2ENR |=(1<<2); //ENABLE CLK FOR A 
	GPIOA->CRL &=~((1<<3)|(1<<2)|(1<<1)|(1<<0)); //RESET PA0(ANALOG INPUT)
	ADC1->SQR3 = 0; // Channel 0 selected
	ADC1->SMPR2 |=  (0x7 << 0); 
	ADC1->CR2 |=(1<<1); //CONVERSION MODE ON
	ADC1->CR2 |=(1<<8); //ENABLE DMA
	ADC1->CR2 |=(1<<0); //ENABLE ADC
	for(volatile int i=0; i<10000; i++);
	ADC1->CR2 |= (1 << 22); // START CONVERSION
	
	

}
void dma_config(){
	DMA1_Channel1->CCR &= ~(1<<0); // Disable channel before config

	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;       // ADC data register address
	DMA1_Channel1->CMAR = (uint32_t)adc_buffer;      // Memory buffer address
	DMA1_Channel1->CNDTR = ADC_BUF_LEN;              // Number of data to transfer
	DMA1_Channel1->CCR = (1<<5)  // MINC
                   | (1<<6)  // PSIZE = 16-bit 
                   | (1<<8)  // MSIZE = 16-bit 
                   | (1<<7); // Circular mode
	DMA1_Channel1->CCR |= (1<<0); // Finally, enable DMA

}
void usart_send_string(char *str){
	while(*str){
		while(!(USART1->SR & (1<<7))); // Wait until TXE is empty
		USART1->DR = *str;
		str++;
	}
}

int main(){
	char msg[30];

	gpio_init();
	usart_init();
	dma_config();
	adc_and_dma_config();

	while(1){
		sprintf(msg, "ADC: %d\r\n", adc_buffer[0]);
		usart_send_string(msg);
		for(volatile int i=0; i<100000; i++); // Delay
	}
}
