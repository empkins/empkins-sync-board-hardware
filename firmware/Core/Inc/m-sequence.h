/*
 * m-sequence.h
 *
 *  Created on: Nov 9, 2022
 *      Author: el77enuv
 */

#ifndef INC_M_SEQUENCE_H_
#define INC_M_SEQUENCE_H_

//includes
#include "stm32l1xx_hal.h"

//functions
void mls(uint8_t n, uint8_t *out);
void lfsr(uint8_t n, uint8_t *out, uint8_t *polynom, uint8_t *tab);

#endif /* INC_M_SEQUENCE_H_ */
