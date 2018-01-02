#include <avr/pgmspace.h>
#include <stdint.h>
#include <avr/eeprom.h>
//USART Daten Senden
#ifndef USART_H_
#define USART_H_
#define BUFFERED_TX 0

typedef enum {Five,Six,Seven,Eight,Nine} DataBits_t;
typedef enum {None,Even,Odd} Parity_t;
typedef enum {One,Two} StopBits_t;

typedef struct
{        
	int16_t BaudrateRegister;
	uint8_t Doublespeed;
	DataBits_t NoOfBits;
	Parity_t Paritytype;
	StopBits_t NoOfStopbits;
} usartparam_t;



void usart_init(usartparam_t* Param);
void usart_putc(unsigned char c);

void Set_RX_Hook(void*);
void Delete_RX_Hook(void);


#endif

