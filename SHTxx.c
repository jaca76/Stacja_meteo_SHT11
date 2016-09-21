/** \file SHTxx.c
	\author Tomasz Ostrowski
	\date August 2007
	\brief Routines for Sensirion SHTxx sensor family
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/delay.h>
#include <stdlib.h>

#include "SHTxx.h"

//#define DEBUG_SHT
#ifdef DEBUG_SHT
	#include "usart.h"
#endif
#ifdef DEBUG_SHT_CRC
	#include "usart.h"
#endif

uint8_t error = 0;
uint8_t CRC = 0x00;		///< initial CRC value, bit-reversed value of status register, default 0x00;                           

#define DELAY(x) (_delay_us(x*5))

//-----------------------------------------------------------------------------
/** \brief Generate a transmission start
	\code
       _____         ______
 DATA:      |_______|
           ___     ___
 SCK:  ___|   |___|   |____
	\endcode	
*/
void s_transstart (void)
{
	SWITCH_SCK_OUT;
	SWITCH_DATA_OUT;
    SET_DATA;
    CLEAR_SCK;
	DELAY(1);
    SET_SCK;;
	DELAY(1);
    CLEAR_DATA;
	DELAY(1);
    CLEAR_SCK;
	DELAY(2);
    SET_SCK;
	DELAY(1);
    SET_DATA;
	DELAY(1);
    CLEAR_SCK;
}

void clear_crc(void)
{
	CRC = 0x00;		// according to status register
#ifdef DEBUG_SHT_CRC
	usart_puts("CRC = 0x00\r\n");
#endif
}

void update_crc(uint8_t nextbit)
{
#ifdef DEBUG_SHT_CRC
	char buf[10];
	itoa(nextbit, buf, 16);
	usart_puts("nextbit: ");
	usart_puts(buf);
	usart_puts("\r\n");
#endif

	if( ((nextbit!=0) && !(CRC & 0x80)) || ((nextbit==0) && (CRC & 0x80)) )
	{
		CRC = CRC << 1;
		CRC = CRC ^ 0x31;
	#ifdef DEBUG_SHT_CRC
		usart_puts("bit = 1, CRC updated to 0x");
	#endif			
	}
	else
	{
		CRC = CRC << 1;
	#ifdef DEBUG_SHT_CRC
		usart_puts("bit = 0, CRC updated to 0x");
	#endif			
	}
#ifdef DEBUG_SHT_CRC	
	itoa(CRC, buf, 16);
	usart_puts(buf);
	usart_puts("\r\n");
#endif
}

//-----------------------------------------------------------------------------
/** \brief write a byte and checks the acknowledge
*/
unsigned char s_write_byte(unsigned char value)
{
#ifdef DEBUG_SHT
	char buf[10];
	itoa(value, buf, 16);
	usart_puts("Sending 0x");
	usart_puts(buf);
	usart_puts("\r\n");
#endif

    SWITCH_DATA_OUT;                  //Make data pin an output
    for(uint8_t i=0x80; i>0; i/=2)
    {
		if (i & value)SET_DATA;
		else CLEAR_DATA;
		SET_SCK;
		DELAY(1);
		CLEAR_SCK;
		DELAY(1);
		update_crc(i & value);
    }
    SET_DATA;                          //Release the data line
    SWITCH_DATA_IN;                   
    SET_DATA;                          //enable pull-up
	DELAY(1);
    SET_SCK;                           //Clock pulse #9 for ack
	DELAY(1);
    CLEAR_SCK;
    return DATA;                      //error=1 if no ACK from sensor
}

//-----------------------------------------------------------------------------

/** \brief read a byte and give an ack in case of 'ack=1'
*/
unsigned char s_read_byte(unsigned char ack)
{
    uint8_t val=0;

    SWITCH_DATA_OUT;                  //Make data pin an output
    SET_DATA;                          //Release the data line
    SWITCH_DATA_IN;                   //Make data pin an input
    SET_DATA;                          //Enable internal pull-up resistor
	DELAY(1);
    for(uint8_t i=0x80; i>0; i/=2)
    {
		SET_SCK;
		DELAY(1);     
		if (DATA)
		{
			val=(val | i);
			update_crc(1);
		}
		else
			update_crc(0);
		CLEAR_SCK;
		DELAY(1);
    }
    SWITCH_DATA_OUT;
    CLEAR_DATA;                        //In case 'ack==1' pull down DATA line
	DELAY(1);
    SET_SCK;                           //Clock pulse #9 for ack
    DELAY(1);
    CLEAR_SCK;
    DELAY(1);
    SET_DATA;                          //Release the data line

#ifdef DEBUG_SHT	
	char buf[10];
	itoa(val, buf, 16);
	usart_puts("Received 0x");
	usart_puts(buf);
	usart_puts("\r\n");
#endif	
    return val;
}



//-----------------------------------------------------------------------------
/** \brief communication reset: DATA=1 and at least 9 SCK's followed by transstart

	communication reset: DATA=1 and at least 9 SCK's followed by transstart
	\code	
       ____________________________________________         ______
 DATA:                                             |_______|
           _   _   _   _   _   _   _   _   _      ___     ___
 SCK:  ___| |_| |_| |_| |_| |_| |_| |_| |_| |____|   |___|   |____
	\endcode 
*/
void s_connectionreset (void)
{
	error = 0;
	SWITCH_SCK_OUT;    
	SWITCH_DATA_OUT;
    SET_DATA;
    CLEAR_SCK;
    for(uint8_t i=0; i<9; i++)
    {
		SET_SCK;
		DELAY(1);
		CLEAR_SCK;
		DELAY(1);
    }
    s_transstart();
}

//-----------------------------------------------------------------------------
/** \brief sensor software reset
*/
unsigned char s_softreset(void)
{
    error=0;
    s_connectionreset();               //communication reset
    error = s_write_byte(RESET);       //send reset command to sensor
    return error;                      //error if no response from sensor
}

//-----------------------------------------------------------------------------
/** \brief read the status register with 8-bit checksum value
*/
unsigned char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{
    error=0;
    s_transstart();                    //transmission start
    error=s_write_byte(STATUS_REG_R);  //send command to sensor
    *p_value=s_read_byte(ACK);         //read status register (8-bit)
    *p_checksum=s_read_byte(noACK);    //read checksum (8-bit)
    return error;                      //error if no response from sensor
}

//-----------------------------------------------------------------------------
/** \brief write the status register with 8-bit checksum value
*/
unsigned char s_write_statusreg(unsigned char *p_value)
{
    error=0;
    s_transstart();                    //transmission start
    error+=s_write_byte(STATUS_REG_W); //send command to sensor
    error+=s_write_byte(*p_value);     //send command to sensor
    return error;                      //error if no response from sensor
}

//-----------------------------------------------------------------------------
/** \brief take a measurement (temperature/humidity) with checksum
*/
unsigned char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
    uint8_t crc_val;
	error=0;
	clear_crc();
    s_transstart();                    //transmission start
    switch (mode) 
    {

		case TEMP : error += s_write_byte(MEASURE_TEMP); break;
		case HUMID: error += s_write_byte(MEASURE_HUMI); break;
		default   : break;
    }

#ifdef DEBUG_SHT	
	if(error)
		usart_puts("s_measure: error from s_write_byte\r\n");
#endif
	
    SWITCH_DATA_IN;                   //Make data pin an input
    SET_DATA;                          //Enable internal pull-up resistor
    uint8_t i=0;
    while((DATA) && (i++<100))
    {
		_delay_ms(20);
    }
    if (i>=100) error+=1;             //timeout ~2s
#ifdef DEBUG_SHT	
	if(error)
		usart_puts("s_measure: read timeout (2s)\r\n");
#endif		
    *(p_value+1) = s_read_byte(ACK);   //read MSB
#ifdef DEBUG_SHT
	if(error)
		usart_puts("s_measure: read MSB error\r\n");
#endif
    *(p_value)   = s_read_byte(ACK);   //read LSB
#ifdef DEBUG_SHT
	if(error)
		usart_puts("s_measure: read LSB error\r\n");
#endif
    //bit-reverse CRC
	crc_val = 0;
	for(uint8_t i=0; i<8; i++)
	{
		if(CRC & (1<<i))
			crc_val += (1<<(7-i)); 
	}
	*p_checksum  = s_read_byte(noACK); //read checksum, noACK to end transmission
#ifdef DEBUG_SHT
	if(error)
		usart_puts("s_measure: read checksum error\r\n");		
	char buf[10];
	usart_puts("CRC calculated: 0x");
	itoa(crc_val, buf, 16);
	usart_puts(buf);
	usart_puts(", CRC received: 0x");
	itoa(*p_checksum, buf, 16);
	usart_puts(buf);
	usart_puts("\r\n");
#endif
	if(crc_val != *p_checksum)
		error++;
    return error;
}

//-----------------------------------------------------------------------------
/** \brief calculate temperature degreesC and humidity %RH
	\param *p_humidity pointer to RH to recalculate (12 bit ticks to %RH)
	\param *p_temperature pointer to temp. to recalculate (14 bit ticks to deg)
*/
void calc_sht11(float *p_humidity, float *p_temperature)
{
    const float C1=-4.0;               //for 12 bit
    const float C2= 0.0405;            //for 12 bit
    const float C3=-0.0000028;         //for 12 bit
    const float T1= 0.01;              //for 14 bit at 5V
    const float T2= 0.00008;           //for 14 bit at 5V

    float rh=*p_humidity;              // rh:      humidity [Ticks] 12 bits
    float t =*p_temperature;           // t:       temperature [Ticks] 14 bits
    float rh_lin;                      // rh_lin:  humidity linear
    float rh_true;                     // rh_true: temperature compensated humidity
    float t_C;                         // t_C:     temperature degrees C

    t_C = (t*0.01) - 40.0;             // ticks -> degrees for Vcc = 5V
    rh_lin=C3*rh*rh + C2*rh + C1;  	// calc humidity from ticks
    rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;// calc temperature compensated humidity
    if(rh_true>100)rh_true=100;        // upper display bounds limit
    if(rh_true<0.1)rh_true=0.1;        // lower display bounds limit

    *p_temperature=t_C;                //return temperature
    *p_humidity=rh_true;               //return humidity
}


