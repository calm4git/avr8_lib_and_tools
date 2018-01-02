/*
 * usart_tools.h
 *
 * Created: 07.12.2013 19:51:59
 *  Author: calm
 */ 


#ifndef USART_TOOLS_H_
#define USART_TOOLS_H_
#include "./usart.h"

void BaudrateRegisterCalc(usartparam_t* Param, long DesiredBaudrate);
uint16_t BaudrateRegisterCalcOptimalValue(usartparam_t* Param, long DesiredBaudrate);


#endif /* USART_TOOLS_H_ */