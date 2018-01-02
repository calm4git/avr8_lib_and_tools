/*	
 * usart_tools.c
 *
 * Created: 07.12.2013 19:52:14
 *  Author: calm
 */ 
#include <stdint.h>
#include "./usart.h"

void BaudrateRegisterCalc(usartparam_t* Param, long DesiredBaudrate)
{
	unsigned int br=0;
	//Okay das hier ist ein Problem da keine passende Runding vorgenommen wird
	DesiredBaudrate=((F_CPU/DesiredBaudrate)>>3); //Einmal durch 8 Teilen
	//Nun noch mal durch 2 Teilen
	if(DesiredBaudrate % 2 == 0)
	{
		DesiredBaudrate=DesiredBaudrate>>1;
		br=(int)(DesiredBaudrate);
		br=br-1;
	}
	else
	{
		DesiredBaudrate=DesiredBaudrate>>1;
		br=(int)(DesiredBaudrate);
	}
	Param->BaudrateRegister=br;
}

uint16_t BaudrateRegisterCalcOptimalValue(usartparam_t* Param, long DesiredBaudrate)
{
	//This Calculates the best value depending on System
	unsigned int br=0;
	long speed=DesiredBaudrate;
	
	unsigned int br1=0;
	long speed1=DesiredBaudrate;
	
	speed=((F_CPU/speed)>>3);
	if(speed % 2== 0){
		speed=speed>>1;
		br=(int)(speed);
		br=br-1;
	} else {
		speed=speed>>1;
		br=(int)(speed);
	}
	
		
	speed1=((F_CPU/speed1)>>2);
	if(speed1 % 2 == 0)	{
		speed1=speed1>>1;
		br1=(int)(speed1);
		br1=br1-1;
	} else {
		speed1=speed1>>1;
		br1=(int)(speed1);
	}
	
	
	
	//Okay we got a Baudratevalue for our Register, lets see how good we are
	long CalculatedBaudrate=(long)(F_CPU/((br+1)*16));
	long CalculatedBaudrate1=(long)(F_CPU/((br1+1)*8));
	//Okay mal sehen wie weit wir danebenliegen
	long Error = DesiredBaudrate - CalculatedBaudrate; 
	long Error1 = DesiredBaudrate - CalculatedBaudrate1; 
	if(Error<0)
	{
		Error=Error*-1;
	}
	if(Error1<0)
	{
		Error1=Error1*-1;
	}
	//Okay nun kann ausgewähklt werden welches setting besser ist
	if(Error<=Error1)
	{
		//Okay es muss das 2X bit nicht gesetzt werden
		//Damit erreicht der USART ein 16fach Oversampling
		//was der Daetnrueckgewinnung zur gute kommt
		Param->BaudrateRegister=br;
		Param->Doublespeed=0;
		return Error;
	}
	else
	{
		Param->BaudrateRegister=br1;
		Param->Doublespeed=1;
		return Error1;
	}
	//Okay das beste Setting ist angewendet...
	 
}