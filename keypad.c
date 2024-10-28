/*
 * keypad.c
 *
 *  Created on: Oct 26, 2024
 *      Author: pedro
 */

#include "main.h"
#include "keypad.h"

void Config_Keypad_Rows() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Enable GPIOB
	GPIOB->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);  // Input mode for PB1, PB13-PB15
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD13_0 | GPIO_PUPDR_PUPD14_0 | GPIO_PUPDR_PUPD15_0;  // Enable pull-up resistors
}

void Config_Keypad_Columns() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // Enable GPIOC
	// Configure PC5, PC6, and PC8 as output for keypad columns
	GPIOC->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER8);  // Clear modes for PC5, PC6, PC8
	GPIOC->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER8_0;  // Set PC5, PC6, PC8 to output mode
}

void Config_LEDs() {
	// LED setup
	GPIOC->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);  // Clear modes for PC0-PC3
	GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0;
}

uint8_t scan_keypad(void) {
    // Adjusted column and row pin mappings
    const uint16_t column_pins[3] = {GPIO_ODR_OD5, GPIO_ODR_OD6, GPIO_ODR_OD8}; // PC5, PC6, PC8 for columns
    const uint16_t row_pins[4] = {GPIO_IDR_ID1, GPIO_IDR_ID13, GPIO_IDR_ID14, GPIO_IDR_ID15}; // PB1, PB13, PB14, PB15 for rows
    // Iterate over each column
    for (uint8_t col = 0; col < 3; col++) {
        // Set all columns high (inactive)
        GPIOC->ODR |= (GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD8);
        // Activate the current column by setting it low
        GPIOC->ODR &= ~column_pins[col];
        // Check each row for the key press
        for (uint8_t row = 0; row < 4; row++) {
            if (!(GPIOB->IDR & row_pins[row])) { // If the row pin is low, key is pressed
                // Reset columns high before returning the key
                GPIOC->ODR |= (GPIO_ODR_OD5 | GPIO_ODR_OD6 | GPIO_ODR_OD8);
                // Return unique key codes based on the column and row
                if (col == 0) { // Column 1
                    if (row == 0) return 1; // Key "1"
                    if (row == 1) return 4; // Key "4"
                    if (row == 2) return 7; // Key "7"
                    if (row == 3) return 10; // Key "*"
                } else if (col == 1) { // Column 2
                    if (row == 0) return 2; // Key "2"
                    if (row == 1) return 5; // Key "5"
                    if (row == 2) return 8; // Key "8"
                    if (row == 3) return 0; // Key "0"
                } else if (col == 2) { // Column 3
                    if (row == 0) return 3; // Key "3"
                    if (row == 1) return 6; // Key "6"
                    if (row == 2) return 9; // Key "9"
                    if (row == 3) return 11; // Key "#"
                }
            }
        }
        // Reset the current column before moving to the next one
        GPIOC->ODR |= column_pins[col];
    }
    // No key press detected, return 0xFF
    return 0xFF;
}

void set_pins(uint8_t value) {
    // Clear PC0-PC3 and set them according to the 'value'
    GPIOC->ODR &= ~(0xF);         // Clear PC0-PC3 (lower 4 bits)
    GPIOC->ODR |= (value & 0xF);  // Set PC0-PC3 to 'value' (0 to 15)
}
