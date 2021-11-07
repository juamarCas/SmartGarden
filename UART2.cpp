/*
 * UART2.cpp
 *
 *  Created on: Nov 7, 2021
 *      Author: Juan
 */
#include "UART2.h"
void UART2_init(){
	RCC->APB1ENR  |= RCC_APB1ENR_USART2EN;
	GPIOA->MODER  |= (1U << 5U); //gpioa2 alternate mode
	GPIOA->MODER  |= (1U << 7U); //gpioa3 alternate mode
	GPIOA->AFR[0] |= (7U << 8U); //put af7 pin 2
	const std::uint32_t baud = 8000000/9600;
	USART2->BRR = 0x0341;
	USART2->CR1 |= USART_CR1_TE;
	USART2->CR3 = 0;

	USART2->CR1 |= USART_CR1_UE;

}

int UART2_write(int ch){
	while(!(USART2->ISR & USART_ISR_TXE)){}
		USART2->TDR = ch & 0xFF;
		return ch;

}

/* The code below is the interface to the C standard I/O library.
 * All the I/O are directed to the console.
 */
namespace std{

	struct __FILE { int handle; };
	FILE __stdin;
	FILE __stdout;
	FILE __stderr;


/*int fgetc(FILE *f) {
    int c;

    c = USART2_read();

    if (c == '\r') {
        UART2_write(c);
        c = '\n';
    }

    USART2_write(c);

    return c;
}*/


/* Called by C library console/file output */
	int fputc(int c, FILE *f) {
			return UART2_write(c);  /* write the character to console */
	}
}



