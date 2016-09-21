	/*
	 * AS3935.c
	 *
	 *  Created on: 16 lip 2015
	 *      Author: Jacek
	 */
	#include <util/delay.h>
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <util/atomic.h>
	#include "AS3935.h"
	#include "../UART/uart.h"
	#include "../Interrupt_Lib/Interrupt_Lib.h"
	#define _INDOR 0x12
	#define _OUTDOOR 0x0E
	#define CS_HIGH RF_PORT|=(1<<CS)
	#define CS_LOW RF_PORT&=~(1<<CS)
	#define ICP PD6

#define T0_ON TCCR0B |= (1<<CS02);   // wlacza timer0 i prescaler 256
#define T0_OFF TCCR0B &= ~((1<<CS02) | (1<<CS01) | (1<<CS00));   // wylacza timer0 i prescaler 256
void start(void);
volatile  uint16_t currentcount =0;
volatile uint16_t Overflow_cnt=0;
volatile  uint32_t ms_count=0;

/*
	void Counter_init(void)
	{
	TCCR1A = 0; // normal mode
	TCCR1B|= (1<<ICES1)| (1<<CS10);  //  no prescaling, rising edge,
	TIMSK1 |= (1<<TOIE1); // input capture interrupt enable, timer1 overflow interrupt enable
	TIFR1 |= (1<<TOV1);
	TCNT1=0;
	}
*/

	void SPI_Init(void)
	{

	RF_DDR  |= (1<<SDI)|(1<<SCK)|(1<<CS);//MOSI, SCK, CS' jako wyjœcia
	CS_HIGH;
	RF_DDR  &= ~(1<<SDO); // MISO jako wejœcie
	//SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
	SPCR = ( 1 << SPE ) | ( 1 << MSTR ) | ( 1 << SPR1 )| (1 << CPHA) ;   //W³¹czamy SPI,    }
	}

	void SPI_Write(uint8_t bajt){
	SPDR = bajt;                    //Wysy³amy zawartoœæ zmiennej bajt
	while(!(SPSR & (1<<SPIF)));        //Oczekujemy na zakoñczenie transmisji ( do ustawienia SPIF ) przez sprzêt}

	}

	uint8_t  SPI_Read(void){                            //Czekamy na koniec transmisji danych
		while(!(SPSR & (1<<SPIF)));  // ( a¿ do ustawienie flagi SPIF    )
		return SPDR;                    //Zwracamy to co dostaliœmy do SPDR}
	}

	uint8_t SPI_Transfer(uint8_t bajt)
	{
		SPDR=bajt;
		while(!(SPSR & (1<<SPIF)));
		return SPDR;
	}

	uint8_t SPI_Transfer2(uint8_t high,uint8_t low)
	{
	CS_LOW;
	SPI_Transfer(high);
	uint8_t regval = SPI_Transfer(low);
	CS_HIGH;
	return regval;
	}

	/*******************************************************************************
	* Function Thunder_Write(unsigned short address, unsigned short data1)
	* ------------------------------------------------------------------------------
	* Overview: Function writes desired byte into specified register address
	* Input: register address, byte
	* Output: Nothung
	*******************************************************************************/
	void Thunder_Write(uint8_t  address, uint8_t  data1)  {
		address &= ~(1 << 7);
		address &= ~(1 << 6);
		CS_LOW;
		SPI_Write(address);
		SPI_Write(data1);
		CS_HIGH;
	}


	/*******************************************************************************
	* Function _rawRegisterRead(unsigned short address)
	* ------------------------------------------------------------------------------
	* Overview: Function reads byte from specified address
	* Input: register address
	* Output: desired byte
	*******************************************************************************/
	uint8_t  _rawRegisterRead(uint8_t  address) {
	  return SPI_Transfer2((address & 0x3F) | 0x40, 0);
	}

	uint8_t _ffsz(uint8_t mask)
	{
		uint8_t i = 0;
		if (mask)
			for (i = 1; ~mask & 1; i++)
				mask >>= 1;
		return i;
	}


	uint8_t registerRead(uint8_t reg, uint8_t mask)
	{
		uint8_t regval =_rawRegisterRead(reg);
		regval = regval & mask;
		if (mask)
			regval >>= (_ffsz(mask)-1);
		return regval;
	}

	void registerWrite(uint8_t reg, uint8_t mask, uint8_t data)
	{
		uint8_t regval = _rawRegisterRead(reg);
		regval &= ~(mask);
		if (mask)
			regval |= (data << (_ffsz(mask)-1));
		else
			regval |= data;
		SPI_Transfer2(reg & 0x3F, regval);
	}

	void Thunder_Init(void) {
		uint8_t  temp;


		 Thunder_Write(0x3C, 0x96);           // set all registers in default mode
		  Thunder_Write(0x3D, 0x96);           // calibrate internal oscillator

		  temp = _rawRegisterRead(0x00) & 0xC1;
		  Thunder_Write(0x00, ((_INDOR << 1) | temp)); // set to indoor

		  temp = _rawRegisterRead(0x01) & 0x80;
		  Thunder_Write(0x01, 0x44 | temp);    // set NFL and WDTreshold

		  temp = _rawRegisterRead(0x02) & 0x80;    // clear statistics, min number of ligtning, spike rejection
		  Thunder_Write(0x02, 0x40 | temp);

		  temp = _rawRegisterRead(0x03) & 0x1F;    // Frequency division ratio(antenna),mask disturber, interrupt
		  Thunder_Write(0x03, 0x00 | temp);

		  Thunder_Write(0x08, 0x00);
	}






	uint8_t lightningDistanceKm()
	{
		return registerRead(AS3935_DISTANCE);
	}

	void powerDown(void)
	{
		registerWrite(AS3935_PWD,1);
	}

	void  powerUp(void)
	{
		registerWrite(AS3935_PWD,0);
		SPI_Transfer2(0x3D, 0x96);
		_delay_ms(3);
	}

	uint8_t interruptSource(void)
	{
		return registerRead(AS3935_INT);
	}

	void disableDisturbers(void)
	{
		registerWrite(AS3935_MASK_DIST,1);
	}

	void enableDisturbers(void)
	{
		registerWrite(AS3935_MASK_DIST,0);
	}

	uint8_t getMinimumLightnings()
	{
		return registerRead(AS3935_MIN_NUM_LIGH);
	}

	uint8_t setMinimumLightnings(uint8_t minlightning)
	{
		registerWrite(AS3935_MIN_NUM_LIGH,minlightning);
		return getMinimumLightnings();
	}


	void setIndoors()
	{
		registerWrite(AS3935_AFE_GB,AS3935_AFE_INDOOR);
	}

	void setOutdoors()
	{
		registerWrite(AS3935_AFE_GB,AS3935_AFE_OUTDOOR);
	}

	uint8_t getNoiseFloor()
	{
		return registerRead(AS3935_NF_LEV);
	}

	uint8_t setNoiseFloor(uint8_t noisefloor)
	{
		registerWrite(AS3935_NF_LEV,noisefloor);
		return getNoiseFloor();
	}

	uint8_t getSpikeRejection(void)
	{
		return registerRead(AS3935_SREJ);
	}

	uint8_t setSpikeRejection(uint8_t srej)
	{
		registerWrite(AS3935_SREJ, srej);
		return getSpikeRejection();
	}

	uint8_t getWatchdogThreshold()
	{
		return registerRead(AS3935_WDTH);
	}

	uint8_t setWatchdogThreshold(uint8_t wdth)
	{
		registerWrite(AS3935_WDTH,wdth);
		return getWatchdogThreshold();
	}

	void clearStats()
	{
		registerWrite(AS3935_CL_STAT,1);
		registerWrite(AS3935_CL_STAT,0);
		registerWrite(AS3935_CL_STAT,1);
	}

	uint16_t lightningEnergy(void)
	{
		uint16_t v = 0;
		char bits8[4];

		// Energy_u e;
		// REG_u reg4, reg5, reg6;

		bits8[3] = 0;
		bits8[2] = registerRead(AS3935_ENERGY_3);
		bits8[1] = registerRead(AS3935_ENERGY_2);
		bits8[0] = registerRead(AS3935_ENERGY_1);

		v = bits8[2]*65536 + bits8[1]*256 + bits8[0];

		return v;
	}


	uint8_t tuneAntena (void)
	{

		uint16_t stop_count=0;
		uint16_t target = 3125;
		int bestdiff = 32767;
		int currdiff = 0;
		uint8_t bestTune = 0;
		uint8_t currTune = 0;
	/*	cli();*/
		int32_t setupTime;

		// set lco_fdiv divider to 0, which translates to 16
		// so we are looking for 31250Hz on irq pin
		// and since we are counting for 100ms that translates to number 3125
		// each capacitor changes second least significant digit
		// using this timing so this is probably the best way to go
	/*	sei();
		start();*/
		registerWrite(AS3935_LCO_FDIV,0);
		registerWrite(AS3935_DISP_LCO,1);
		// tuning is not linear, can't do any shortcuts here
		// going over all built-in cap values and finding the best
		   for (currTune = 0; currTune <= 0x0F ; currTune++)
		      {


				registerWrite(AS3935_TUN_CAP,currTune);
		         // wait to settle
				_delay_ms(10);
				currentcount=0;
				stop_count=0;
				setupTime=millis()+100;
				// wait 100ms for measurments
				while ((long int)millis() - setupTime<0)
				{
					//
				}
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
					stop_count=currentcount;
					}
				currentcount=0;
		 		currdiff = target - stop_count;
		 		// don't look at me, abs() misbehaves
		 		if(currdiff < 0)
		 			currdiff = -currdiff;
		 		if(bestdiff > currdiff)
		 		{
		 			bestdiff = currdiff;
		 			bestTune = currTune;
		 		}
		      }


		if (bestdiff<109)
		{
		   registerWrite(AS3935_TUN_CAP,bestTune);
			_delay_ms(2);
			registerWrite(AS3935_DISP_LCO,0);
			// and now do RCO calibration
			powerUp();
			uart_putint(bestTune,10);
	 	    uart_puts(" ");
	 	   uart_putint(bestdiff,10);
	 	    uart_puts("\r\n");
		return (1);
		}
		// if error is over 109, we are outside allowed tuning range of +/-3.5%
		else
		{
			powerUp();
			return (0);

		}
	}


	void start(void)
	{
		T0_ON;
	}

	void Timer0_init(void)
	{
		TCCR0A |= (1<<WGM01); //CTC
		TCCR0B |= (1<<CS02); //256
		OCR0A = 71;
		TIMSK0 |= (1<<OCIE0A);
	}


	uint32_t millis(void){
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		return ms_count;
		}
	}



// Przerwanie INT2 od AS3935
	ISR(INT2_vect)
	{
		currentcount++;
	}



	ISR(TIMER0_COMPA_vect)
	{
	ms_count++;
	}
