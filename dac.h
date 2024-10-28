/*
 * dac.h
 *
 *  Created on: Oct 26, 2024
 *      Author: pedro
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#define MAXDACVALUE 4095
#define OUTOFDACRANGE 4096
#define DACSCALER 0.809
#define HEADERPOSITION 12
#define HEADERVALUE 3

void Config_PA5_SCLK(void);
void Config_PA7_MOSI(void);
void Config_PA4_CS(void);
void DAC_init(void);
void DAC_Write(uint16_t DACValue);
uint16_t DAC_volt_conv(uint16_t Volts);

#endif /* INC_DAC_H_ */
