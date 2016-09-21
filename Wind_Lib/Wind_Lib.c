/*
 * ADC_Lib.c
 *
 *  Created on: 19 lip 2016
 *      Author: Jacek
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "Wind_Lib.h"


void ADC_Init(void){

	ADMUX|=(1<<REFS0); // napiecie odniesienia VCC
	ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

void ADC_Start(void){
	ADCSRA|=(1<<ADSC);
}

int16_t ADC_Wynik (void){
	if (ADCSRA&ADSC){
		switch(ADCW){
		case 786 ... 790:
		return 0;
		case 402 ... 408:
		return 22;
		case 460 ... 466:
		return 45;
		case 76 ... 82:
		return 67;
		case 85 ... 91:
		return 90;
		case 57 ... 63:
		return 112;
		case 178 ... 183:
		return 135;
		case 118 ... 124:
		return 157;
		case 283 ... 288:
		return 180;
		case 239 ... 245:
		return 202;
		case 629 ... 635:
		return 225;
		case 598 ... 604:
		return 247;
		case 943 ... 949:
		return 270;
		case 826 ... 832:
		return 292;
		case 885 ... 891:
		return 315;
		case 700 ... 708:
		return 337;
		default: return -2;
		}
	}
		else {
			return -1;
		}
}


