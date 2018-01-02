/*
 * usart_hlf.h
 *
 * Created: 07.12.2013 19:44:25
 *  Author: calm
 */ 

// High Level Functions for µC's (this time AVRs)

#ifndef USART_HLF_H_
#define USART_HLF_H_

#include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "./usart.h"

void usart_puts (char *s);

void usart_putp(const char *s);

void usart_pute (const char *s);

void usart_puti(unsigned int c);

void usart_puth(unsigned char c);


#endif /* USART_HLF_H_ */