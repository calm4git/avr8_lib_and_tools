#include <avr/io.h> 		//IO Map für den Mega8
#include <avr/interrupt.h> 	//Interrupts für den Mega8
#include <inttypes.h>  		//wie der Name schon sagt
#include <stdlib.h> 		//das übliche c gerumpel

#include "./usart.h"
//So einmal USART Routunen für die uController
//USART Daten Senden

#ifdef USART_UDRE_vect
#define UDRE_vect USART_UDRE_vect
#endif

#ifdef USART0_UDRE_vect
#define UDRE_vect USART0_UDRE_vect
#endif

#ifdef USART_RXC_vect
#define RXC_vect USART_RXC_vect
#else
#define RXC_vect USART0_RX_vect
#endif
#if BUFFERED_TX > 0
#include "./fifo.h"
uint8_t buffer0[BUF_SIZE0];
fifo_t tx0_fifo;

void usart_putc(unsigned char c) {

	#ifdef UCSRB //Wenn es ein alter Atmega ist
	if((UCSRB&(1<<(UDRIE)))==0)
	{
		UCSRB |= (1<<(UDRIE));
	}
	
	while(fifo_put(&tx0_fifo,c)==0)
	{
		UCSRB |= (1<<(UDRIE));
	}
	#endif
	
	
	#ifdef UCSR0B
	if((UCSR0B&(1<<(UDRIE0)))==0)
	{
		UCSR0B |= (1<<(UDRIE0));
	}
	
	while(fifo_put(&tx0_fifo,c)==0)
	{
		UCSR0B |= (1<<(UDRIE0));
	}	
	
	#endif
}



ISR(UDRE_vect)
{
	//Okay dann einmal das nächste Byte nachlegen
	int from_buffer=fifo_get_nowait(&tx0_fifo);
	
	if(from_buffer>-1)
	{
		#ifdef UDR
		UDR=(unsigned char)(from_buffer);
		#else
		UDR0=(unsigned char)(from_buffer);
		#endif
	}
	else
	{
		//Okay der Fifo ist leer also ISR aus
		#ifdef UCSRB
		UCSRB &= ~(1<<(UDRIE));
		#else
		UCSR0B &= ~(1<<(UDRIE0));
		#endif
	}
}
#else

void usart_putc(unsigned char c) {
   // wait until UDR ready
  	#ifdef UCSRA
	  while(!(UCSRA & (1 << UDRE)));
		UDR = c;    // send character
   #endif
   
   #ifdef UCSR0A
	while(!(UCSR0A & (1 << UDRE0)));
		UDR0 = c;    // send character
   #endif
}

#endif

void(*fptr)(void);
void Set_RX_Hook(void* funkptr)
{
fptr=funkptr;

#ifdef UBRRL
UCSRB|=(1<<RXCIE);
#elif defined UBRR0L
UCSR0B|=(1<<RXCIE0);
#endif

}

void Delete_RX_Hook(void)
{
	#ifdef UBRRL
	UCSRB&=~(1<<RXCIE);
	#elif defined UBRR0L
	UCSR0B&=~(1<<RXCIE0);
	#endif
fptr=0;
}


ISR(RXC_vect)
{
	if(fptr!=0)
	{
		(*fptr)();	
	}
	else
	{
		#ifdef UDR
		uint8_t temp=UDR;
		#else
		uint8_t temp=UDR0;
		#endif
	}
}


#ifdef UDR
void usart_init(usartparam_t* Param) 
{

#if BUFFERED_TX > 0
fifo_init (&tx0_fifo, buffer0, BUF_SIZE0);
#endif


unsigned int br=Param->BaudrateRegister;

UBRRL=(unsigned char)br;
UBRRH=(unsigned char)(br>>8);
UCSRB =(1<<RXEN)|(1<<TXEN);
//Parity einstellen:
//Einmal die Bits löschen (default = None Parity) also einen definierten Zustand schaffen
//Transmittermode auf USART 
//Einmal einen definierten Zustand schaffen
//Einstellung laut Datenblatt Atmel
UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UCPOL); //Muss laut Datenblatt eins sein um das Register zu schreiben
//Setzt den Baustein in den Asynchronen Modus

uint8_t UCSRC_Temp=(1<<URSEL); //Die einstellungen werden in eine temporäre Variable geschrieben, das das UCSRC und UBRRH die gleiche Speicheradresse haben
//Mit dem Zug umgeht man das eine oder andere Problem :-(

switch(Param->Paritytype)
{
	case None:
	{
		UCSRC_Temp&=~( (1<<UPM0)||(1<<UPM1) );		
	}
	break;
	
	case Even:
	{
		UCSRC_Temp|=(1<<UPM1);		
	}
	break;
	
	case Odd:
	{
		UCSRC_Temp|=(1<<UPM0)||(1<<UPM1);
	}
	break;
	
	default:
	{
		UCSRC_Temp&=~( (1<<UPM0)||(1<<UPM1) );
	}
	break;
}

switch(Param->NoOfBits)
{
	case Nine:
	{
		UCSRC_Temp |=( (1<<UCSZ2) | (1<<UCSZ1) | (1<<UCSZ0));
	}
	
	case Eight:
	{
		UCSRC_Temp |=( (0<<UCSZ2) | (1<<UCSZ1) | (1<<UCSZ0));
	}
	break;
	
	case Seven:
	{
		UCSRC_Temp |=( (0<<UCSZ2) | (1<<UCSZ1) | (0<<UCSZ0));
	}
	break;
	
	case Six:
	{
		UCSRC_Temp |=( 0<<UCSZ2) | (0<<UCSZ1) | (1<<UCSZ0);	
	}
	break;
	
	case Five:
	{
		UCSRC_Temp |=( (0<<UCSZ2) | (0<<UCSZ1) | (0<<UCSZ0));	
	}
	break;
	
	default:
	{
		UCSRC_Temp |=( (0<<UCSZ2) | (1<<UCSZ1) | (1<<UCSZ0));
	}
	break;
}

UCSRC=UCSRC_Temp;

if(Param->Doublespeed>0)
{
	UCSRA |= (1<<U2X); //Die Baudrate Verdoppeln auf 9600 Baud
}
else
{
	UCSRA &= ~(1<<U2X); //Die Baudrate Verdoppeln auf 9600 Baud
}

#if BUFFERED_TX > 0
UCSRB |= (1<<(UDRIE));
#endif

}
#endif

#ifdef UDR0
void usart_init(usartparam_t* Param)
{

	#if BUFFERED_TX > 0
	fifo_init (&tx0_fifo, buffer0, BUF_SIZE0);
	#endif


	unsigned int br=Param->BaudrateRegister;

	UBRR0L=(unsigned char)br;
	UBRR0H=(unsigned char)(br>>8);
	UCSR0B =(1<<RXEN0)|(1<<TXEN0);
	//Parity einstellen:
	//Einmal die Bits löschen (default = None Parity) also einen definierten Zustand schaffen
	//Transmittermode auf USART
	//Einmal einen definierten Zustand schaffen
	//Einstellung laut Datenblatt Atmel
	
	//Setzt den Baustein in den Asynchronen Modus

	uint8_t UCSRC_Temp=0; //Die einstellungen werden in eine temporäre Variable geschrieben, das das UCSRC und UBRRH die gleiche Speicheradresse haben
	//Mit dem Zug umgeht man das eine oder andere Problem :-(

	switch(Param->Paritytype)
	{
		case None:
		{
			UCSRC_Temp&=~( (1<<UPM00)||(1<<UPM01) );
		}
		break;
		
		case Even:
		{
			UCSRC_Temp|=(1<<UPM01);
		}
		break;
		
		case Odd:
		{
			UCSRC_Temp|=(1<<UPM00)||(1<<UPM01);
		}
		break;
		
		default:
		{
			UCSRC_Temp&=~( (1<<UPM00)||(1<<UPM01) );
		}
		break;
	}

	switch(Param->NoOfBits)
	{
		case Nine:
		{
			UCSRC_Temp |=( (1<<UCSZ02) | (1<<UCSZ01) | (1<<UCSZ00));
		}
		
		case Eight:
		{
			UCSRC_Temp |=( (0<<UCSZ02) | (1<<UCSZ01) | (1<<UCSZ00));
		}
		break;
		
		case Seven:
		{
			UCSRC_Temp |=( (0<<UCSZ02) | (1<<UCSZ01) | (0<<UCSZ00));
		}
		break;
		
		case Six:
		{
			UCSRC_Temp |=( 0<<UCSZ02) | (0<<UCSZ01) | (1<<UCSZ00);
		}
		break;
		
		case Five:
		{
			UCSRC_Temp |=( (0<<UCSZ02) | (0<<UCSZ01) | (0<<UCSZ00));
		}
		break;
		
		default:
		{
			UCSRC_Temp |=( (0<<UCSZ02) | (1<<UCSZ01) | (1<<UCSZ00));
		}
		break;
	}

	UCSR0C=UCSRC_Temp;

	if(Param->Doublespeed>0)
	{
		UCSR0A |= (1<<U2X0); //Verdoppel der Baudrate
	}
	else
	{
		UCSR0A &= ~(1<<U2X0); // Keine verdoppelung
	}

	#if BUFFERED_TX > 0
	UCSR0B |= (1<<(UDRIE0));
	#endif

}
#endif