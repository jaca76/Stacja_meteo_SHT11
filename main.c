/*
 * main.c
 *
 *  Created on: 7 lip 2016
 *      Author: Jacek
 */


#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UART/uart.h"
#include "Wind_Lib/Wind_Lib.h"
#include "AS3935/AS3935.h"
#include "SHTxx.h"

#define speed_pin (1<<PD2)
#define rain_pin (1<<PD3)
// SHT11 definicje
uint16_t wynik;
uint32_t time;

uint8_t flag;
uint8_t Distance;
uint8_t Thunder_dane;
uint8_t min_distance=99;

uint32_t lastSecond;
uint8_t seconds; //When it hits 60, increase the current minute
uint8_t seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
uint8_t minutes; //Keeps track of where we are in various arrays of data
uint8_t minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile uint8_t windClicks = 0;
//Timery programowe
volatile uint16_t Timer1, Timer2;
volatile uint16_t sht_timeout;
//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

uint8_t windspdavg[120]; //120 bytes to keep track of 2 minute average
#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile uint32_t rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain
//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0; // [%]
float tempf = 0; // [temperature F]
float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile uint32_t dailyrainin = 0; // [rain inches so far today in local time]
//float baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0;
// volatiles are subject to modification by IRQs
volatile uint32_t raintime, rainlast, raininterval, rain;
volatile uint16_t flaga;

// SHT11 deklaracje


// Deklaracje funkcji
uint16_t get_wind_speed();
void INT_Init(void);
void Timer1_Init(void);

void wait()
{
	int i;

	for(i = 0; i < 1000; i++)
		_delay_ms(1);

}



int main(void)
{
	value humid_val,temp_val;
	unsigned char checksum, error = 0;

	USART_Init(__UBRR);
    INT_Init();
    Timer1_Init();
    sei();
    lastSecond=millis();

	while (1)
	{
		if (millis()-lastSecond>=1000)
				{
			uart_puts("Tick");
			lastSecond += 1000;
			s_connectionreset();

			error+=s_measure((unsigned char*) &temp_val.i,&checksum,TEMP);
			error+=s_measure((unsigned char*) &humid_val.i,&checksum,HUMID);

			if(error != 0)
			{
			   s_connectionreset();
			   uart_puts("MEAS ERROR");
			   wait();
			}
			else
				{
				   humid_val.f=(float)humid_val.i;
				   temp_val.f=(float)temp_val.i;
				   calc_sht11(&humid_val.f,&temp_val.f);
				   temp_val.i=(int32_t)(temp_val.f*10);
					uart_puts("temp:");
					uart_puts("\r\n");
					   if(temp_val.i < 0)
						   {
						   		uart_puts("-");
							}
					   temp_val.i = abs(temp_val.i);
					   uart_putint(temp_val.i/10,10);
				}
			}
	}
}



uint16_t get_wind_speed()
{
	float deltaTime = millis() - lastWindCheck; //750ms
	deltaTime /= 1000.0; //Covert to second
	float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4
	windClicks = 0; //Reset and start watching for new wind
	lastWindCheck = millis();
	windSpeed *= 667; //4 * 1.492 = 5.968MPH
	return(windSpeed);
}

void INT_Init(void){
/* Ustawienie pinu jako wejœcie i podciagniecie do vcc*/
    DDRD &= ~speed_pin;
    PORTD |= speed_pin;
    DDRD &= ~rain_pin;
    PORTD |= rain_pin;
    // AS IRQ
    DDRB &= ~(1<<PB2);
    /* Ustawienie przerwania od INT0 (predkosc) i INT1 (deszcz)*/
    EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10)|(1<<ISC21);
    EIMSK|=(1<<INT0)|(1<<INT1)|(1<<INT2);
}
//Knfiguracja timera1 na 10ms
void Timer1_Init(void){
	TCCR1B |= (1<<CS10)|(1<<CS12)|(1<<WGM12); //CTC 1024
	OCR1A = 179;
	TIMSK1 |= (1<<OCIE1A);
}

ISR (TIMER1_COMPA_vect){
	uint16_t n;
	n = Timer1;		/* 100Hz Timer1 */
	if (n) Timer1 = --n;
	n = Timer2;		/* 100Hz Timer2 */
	if (n) Timer2 = --n;
	n=sht_timeout;
	if (n) sht_timeout = --n;
}


ISR( INT0_vect ) {


	if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
	{
		lastWindIRQ = millis(); //Grab the current time
		windClicks++; //There is 1.492MPH for each click per second.

	}

}

ISR (INT1_vect)
{
	raintime = millis(); // grab current time
	raininterval = raintime - rainlast; // calculate interval between this and last event

	if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
	{
		dailyrainin = dailyrainin+2794; //Each dump is 0,2794 mm of water
		rainHour[minutes] += 2794; //Increase this minute's amount of rain
		rainlast = raintime; // set up for next event
	}
}

