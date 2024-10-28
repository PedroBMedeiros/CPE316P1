/*
 * dac.c
 *
 *  Created on: Oct 26, 2024
 *      Author: pedro
 */

#include "main.h"
#include "dac.h"

void Config_PA5_SCLK(void) {
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= (GPIO_MODER_MODE5_1); // alternate function mode

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5); // push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5); // very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5_0); // very high speed

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5); // no pull up or pull down

	GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL5_Pos); // alternate function 5 - SPI1SCLK
	GPIOA->AFR[0] |= (0x0005 << GPIO_AFRL_AFSEL5_Pos); // alternate function 5 - SPI1SCLK
}

void Config_PA7_MOSI(void) {
	GPIOA->MODER &= ~(GPIO_MODER_MODE7);
	GPIOA->MODER |= (2 << GPIO_MODER_MODE7_Pos); // alternate function mode

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT7); // push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7); // very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7_0); // very high speed

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7); // no pull up or pull down

	GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL7_Pos); // alternate function 5 - SPI1SCLK
	GPIOA->AFR[0] |= (0x0005 << GPIO_AFRL_AFSEL7_Pos); // alternate function 5 - SPI1SCLK
}

void Config_PA4_CS(void) {
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |= (2 << GPIO_MODER_MODE4_Pos); // alternate function mode

	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4); // push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4); // very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4_0); // very high speed

	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4); // no pull up or pull down

	GPIOA->AFR[0] &= ~(0x000F << GPIO_AFRL_AFSEL4_Pos); // alternate function 5 - SPI1SCLK
	GPIOA->AFR[0] |= (0x0005 << GPIO_AFRL_AFSEL4_Pos); // alternate function 5 - SPI1SCLK
}

void DAC_init(void) { // initialize the SPI peripheral to communicate with the DAC
	// Configure GPIO pins - SCLK (PA5), MISO (PA6), CS (PA4)
	// Configure GPIO for MOSI, SCK
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // enable GPIOA
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);

	Config_PA5_SCLK();
	Config_PA7_MOSI();
	Config_PA4_CS();

	//GPIOA->ODR |= (GPIO_ODR_OD4); // set CS high initially
	//GPIOA->ODR &= ~(GPIO_ODR_OD5); // sets clock initially to 0

	// Configuring CR1
	SPI1->CR1 &= ~(SPI_CR1_SPE); // disable SPI for configure

	SPI1->CR1 &= ~(SPI_CR1_BR); // baud rate = 000 (f/2)
	SPI1->CR1 &= ~(SPI_CR1_CPOL); // CPOL = 0; clock 0 when idle
	SPI1->CR1 &= ~(SPI_CR1_CPHA); // CPHA = 0; rising edge transition
	SPI1->CR1 &= ~(SPI_CR1_RXONLY); // half-duplex; transmit and receive
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE); // RXONLY only
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); // MSB first
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);
	SPI1->CR1 &= ~(SPI_CR1_SSM);
	SPI1->CR1 &= ~(SPI_CR1_SSI);
	SPI1->CR1 |= (SPI_CR1_MSTR); // set as controller

	// Configuring CR2
	SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
	SPI1->CR2 &= ~(SPI_CR2_FRF);
	SPI1->CR2 |= ((0xF << SPI_CR2_DS_Pos) | SPI_CR2_SSOE | SPI_CR2_NSSP);

	SPI1->CR1 |= (SPI_CR1_SPE);
}

// receives 16 bit value that represents a voltage in DAC terms and puts it into data register to be passed on
void DAC_Write(uint16_t DACValue) {
	while (!(SPI1->SR & SPI_SR_TXE)) {}
	SPI1->DR = DACValue; // write DAC value into data register
}


uint16_t DAC_volt_conv(uint16_t Volts) { // takes in three digits in integer format and transforms it into a DAC readable value for voltage
	uint16_t ReturnValue = 0;
	// ReturnValue = (mV value)/0.809
	// number taken in is from 0 to 3000
	ReturnValue = Volts/DACSCALER;
	if (ReturnValue >= OUTOFDACRANGE) {
		ReturnValue = MAXDACVALUE;
	}
	ReturnValue |= (HEADERVALUE << HEADERPOSITION);
	return ReturnValue;
}



