/** \file SHTxx.c
	\author Tomasz Ostrowski
	\date August 2007
	\brief Routines for Sensirion SHTxx sensor family
*/

#ifndef SHTxx_H
#define SHTxx_H


#define NOP __asm__ __volatile__ ("nop")

// define electric connections according to your circuit, DATA line
#define DATA_PORT		PORTA
#define DATA_DDR		DDRA
#define DATA_PINPORT	PINA
#define DATA_PIN		2
#define SWITCH_DATA_IN  DATA_DDR &= ~_BV(DATA_PIN)
#define SWITCH_DATA_OUT DATA_DDR |= _BV(DATA_PIN); NOP	///< \todo is NOP needed?
#define CLEAR_DATA      DATA_PORT &= ~_BV(DATA_PIN)
#define SET_DATA        DATA_PORT |= _BV(DATA_PIN)
#define DATA            (DATA_PINPORT & _BV(DATA_PIN))

// define electric connections according to your circuit, SCK line
#define SCK_PORT		PORTA
#define SCK_DDR			DDRA
#define SCK_PINPORT		PINA
#define SCK_PIN			1
#define SWITCH_SCK_OUT  SCK_DDR  |= _BV(SCK_PIN)
#define SWITCH_SCK_IN	SCK_DDR  |= ~_BV(SCK_PIN)
#define CLEAR_SCK       SCK_PORT &= ~_BV(SCK_PIN)
#define SET_SCK         SCK_PORT |= _BV(SCK_PIN)


typedef union
{
    int32_t i;
    float f;
} value;


enum{TEMP,HUMID};

#define noACK 0
#define ACK   1    

//Sensor  opcodes               adr command r/w
#define STATUS_REG_W 0x06   //  000  0011   0
#define STATUS_REG_R 0x07   //  000  0011   1
#define MEASURE_TEMP 0x03   //  000  0001   1
#define MEASURE_HUMI 0x05   //  000  0010   1
#define RESET        0x1E   //  000  1111   0

//Function definitions
unsigned char s_write_byte (unsigned char value);
unsigned char s_read_byte (unsigned char ack);
void s_transstart (void);
void s_connectionreset (void);
unsigned char s_softreset (void);
unsigned char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum);
unsigned char s_write_statusreg(unsigned char *p_value);
unsigned char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode);
void calc_sht11(float *p_humidity, float *p_temperature);

#endif
