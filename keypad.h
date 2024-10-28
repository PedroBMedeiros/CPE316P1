/*
 * keypad.h
 *
 *  Created on: Oct 26, 2024
 *      Author: pedro
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

void Config_Keypad_Rows();
void Config_Keypad_Columns();
uint8_t scan_keypad(void);
void set_pins(uint8_t value);
void Config_LEDs();

#endif /* INC_KEYPAD_H_ */
