/*
 * AS3935.h
 *
 *  Created on: 7 lip 2016
 *      Author: Jacek
 */

#ifndef AS3935_AS3935_H_
#define AS3935_AS3935_H_


#define RF_PORT		PORTB
#define RF_DDR		DDRB
#define RF_PIN		PINB
#define SDI		5	// MOSI	- wyjœcie
#define SCK		7	// SCK
#define CS		4	// SS
#define SDO		6	// MISO	- wejœcie
void CS_HIGH();
void CS_LOW();
void SPI_Write(uint8_t);
void Timer0_init(void);
uint8_t SPI_Transfer(uint8_t);
uint8_t SPI_Read();
void SPI_Init();
void Thunder_Init(void);
uint8_t  Thunder_Read(uint8_t);
uint8_t lightningDistanceKm(void);
uint8_t setSpikeRejection(uint8_t);
uint8_t getSpikeRejection(void);
uint8_t getNoiseFloor(void);
uint8_t setNoiseFloor(uint8_t);
uint8_t getMinimumLightnings(void);
uint16_t lightningEnergy(void);
uint8_t interruptSource(void);
uint8_t tuneAntena (void);
uint32_t millis(void);

#define IRQ_PORT	PORTB
#define IRQ_DIR		DDRB
#define IRQ_PIN 	(1<<PB2)
// register access macros - register address, bitmask
// register access macros - register address, bitmask
#define AS3935_AFE_GB		0x00, 0x3E
#define AS3935_PWD		0x00, 0x01
#define AS3935_NF_LEV		0x01, 0x70
#define AS3935_WDTH		0x01, 0x0F
#define AS3935_CL_STAT		0x02, 0x40
#define AS3935_MIN_NUM_LIGH	0x02, 0x30
#define AS3935_SREJ		0x02, 0x0F
#define AS3935_LCO_FDIV	0x03, 0xC0
#define AS3935_MASK_DIST	0x03, 0x20
#define AS3935_INT		0x03, 0x0F
#define AS3935_DISTANCE	0x07, 0x3F
#define AS3935_DISP_LCO	0x08, 0x80
#define AS3935_DISP_SRCO	0x08, 0x40
#define AS3935_DISP_TRCO	0x08, 0x20
#define AS3935_TUN_CAP		0x08, 0x0F
#define AS3935_ENERGY_1		0x04, 0xFF
#define AS3935_ENERGY_2		0x05, 0xFF
#define AS3935_ENERGY_3		0x06, 0x1F

// other constants
#define AS3935_AFE_INDOOR	0x12
#define AS3935_AFE_OUTDOOR	0x0E


#endif /* AS3935_AS3935_H_ */
