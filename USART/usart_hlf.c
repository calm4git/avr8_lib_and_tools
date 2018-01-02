/*
 * usart_hlf.c
 *
 * Created: 07.12.2013 19:44:11
 *  Author: calm
 */ 

#include "./usart_hlf.h"

void usart_puts (char *s) {
	//  loop while *s != NULL
	while (*s) {
		usart_putc(*s);
		s++;
	}
}

void usart_pute (const char *s)
{
	char zeichen= (char) eeprom_read_byte((const uint8_t*)s);
	s++;
	while(zeichen!='\0')
	{
		usart_putc(zeichen);
		zeichen=(char) eeprom_read_byte((const uint8_t*)s);
		s++;
	}
}

// Einen 0-terminierten String senden, der im Flash steht.
void usart_putp ( const char *s)
{
	while (1)
	{
		unsigned char c = pgm_read_byte (s);
		s++;
		if ('\0' == c)
		break;
		usart_putc(c);
	}
}

void usart_puti_help(unsigned char* zehntausend,unsigned char* tausend,unsigned char* hundert,unsigned char* zehn,unsigned int* c)
{

	while(*c>9999) //max 3 durchläufe
	{
		*zehntausend=*zehntausend+1;
		*c-=10000L;
	}

	while(*c>999) //max 9 durchläufe
	{
		*tausend=*tausend+1;
		*c-=1000L;
	}

	while(*c>99) //max 9 durchläufe
	{
		*hundert=*hundert+1;
		*c-=100;
	}

	while(*c>9) //max 9 durchläufe
	{
		*zehn=*zehn+1;
		*c-=10;
	}

}
void usart_puti(unsigned int c)
{
	unsigned char zehntausend=0;
	unsigned char tausend=0;
	unsigned char hundert=0;
	unsigned char zehn=0;
	usart_puti_help(&zehntausend,&tausend,&hundert,&zehn,&c);

	if(zehntausend!=0)
	usart_putc(zehntausend+48);


	if((tausend!=0)||(zehntausend!=0))
	usart_putc(tausend+48);


	if((hundert!=0)||(tausend!=0)||(zehntausend!=0))
	usart_putc(hundert+48);

	if((zehn!=0)||(hundert!=0)||(tausend!=0)||(zehntausend!=0))
	usart_putc(zehn+48);

	usart_putc(c+48);

}


void usart_puth_help(unsigned char* low_nibble,unsigned char* high_nibble, unsigned char* c)
{
	*low_nibble=(*c & 0x0F); //Die oberen 4 Bit Maskieren
	*high_nibble=(*c & 0xF0);
	*high_nibble=(*high_nibble>>4);

	if(*low_nibble>9)
	{
		*low_nibble=*low_nibble+55;
	}
	else
	{
		*low_nibble=*low_nibble+48;
	}


	if(*high_nibble>9)
	{
		*high_nibble=*high_nibble+55;
	}
	else
	{
		*high_nibble=*high_nibble+48;
	}


}

void usart_puth(unsigned char c)
{
	unsigned char low_nibble=0;
	unsigned char high_nibble=0;
	usart_puth_help(&low_nibble,&high_nibble,&c);
	usart_putc('0');
	usart_putc('x');
	// Zerlegen des Byte in 2 Nibbel
	usart_putc(high_nibble);
	usart_putc(low_nibble);
}

