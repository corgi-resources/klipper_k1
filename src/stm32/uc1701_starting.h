#ifndef __STM32_UC1701_STARTING_H
#define __STM32_UC1701_STARTING_H

#include <stdint.h> // uint32_t

void UC1701_begin(void);
void UC1701_clear(void);
void UC1701_setCursor(uint8_t column, uint8_t line);
void UC1701_transfer(uint8_t isCMD, uint8_t data);
void UC1701_write(uint8_t chr);
void UC1701_string(uint8_t *str);
void UC1701_Show(void);
void UC1701_neopixelSetColor(uint8_t r, uint8_t g, uint8_t b);
void UC1701_neopixelBegin(void);

void BIQU_HurakanStarting(void);

#endif // uc1701_starting.h
