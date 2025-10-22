/*
 * main.c - Bare-metal STM32F103 Blink Test for PC13
 * --------------------------------------------------
 * Purpose: Blinks the built-in blue/green LED on PC13.
 * This is the "Hello, World!" of microcontrollers.
 */
#include "stm32f10x.h"
#include <stdint.h>

// A simple software delay function
void delay(volatile uint32_t count) {
    while(count--);
}

int main(void) {
    // 1. Enable the clock for GPIO Port C in the APB2 peripheral clock register.
    // The bit for IOPCEN (I/O Port C Enable) is the 4th bit.
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // 2. Configure PC13 as a general-purpose push-pull output.
    // We need to set the MODE and CNF bits for pin 13.
    // Pin 13 is controlled by the CRH (Control Register High).
    
    // First, clear the 4 bits for PC13 (bits 20-23)
    GPIOC->CRH &= ~(0xF << 20);
    
    // Then, set MODE13 to '01' (Output mode, max speed 10 MHz)
    // and CNF13 to '00' (General purpose output push-pull).
    // The combined value is 0b0001 = 0x1.
    GPIOC->CRH |= (0x1 << 20);


    // 3. Infinite loop to blink the LED
    while (1) {
        // Turn the LED ON.
        // The PC13 LED is "active-low", meaning setting the pin LOW turns it ON.
        // We write a '1' to the corresponding bit in the BRR (Bit Reset Register) to pull the pin low.
        GPIOC->BRR = (1 << 13);
        
        delay(300000); // Wait for a moment

        // Turn the LED OFF.
        // Setting the pin HIGH turns it OFF.
        // We write a '1' to the corresponding bit in the BSRR (Bit Set/Reset Register) to pull the pin high.
        GPIOC->BSRR = (1 << 13);

        delay(300000); // Wait for a moment
    }
}