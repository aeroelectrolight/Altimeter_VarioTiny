/*
 * VarioTinyC.c
 *
 * Created: 03/05/2020 09:11:19
 * Author : ludom
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <math.h>
#include "AsmTinySerial.h"
#include "bmp280.h"


#define FRSKY_USERDATA_BARO_ALT_B	0x10
#define FRSKY_USERDATA_BARO_ALT_A	0x21
#define FRSKY_USERDATA_ALT_MIN      0x31 // open9x Vario Mode Only
#define FRSKY_USERDATA_ALT_MAX      0x32 // open9x Vario Mode Only

#define SDA			0
#define LED_PIN		1
#define SCL			2
#define UART_PIN	3
#define RC_PIN		4

/********************************* Event REGistry *******************************/
#define _MIF_		0 // Timer Measure Interval Flag
#define _BMPEF_		1 // BMP error flag
#define _I2CANF_	2 // I2C Ack/Nack Flag, 0 - ACK, 1 - NACK
#define _I2CERF_	3 // I2C ERror Flag
#define _RCESF_		4 // RC event serial Flag
#define _RCEE_		5 // RC event Error Flag

volatile uint8_t pulseT = 0x00;
volatile uint8_t cmpt0	= 0x0e;
volatile register uint8_t flags asm("r16");
//float seaLevel;

inline void setCmpt0(){
	cmpt0	= 0x07; // toute les 450 ms //0x0e = toute les 950 ms 
}

/************************************************************************/
/* INTERRUPTION                                                         */
/************************************************************************/

ISR (TIMER0_OVF_vect) {
	if (!(--cmpt0))
	{
		setCmpt0();
		//blink();
		// met à 1 le flag lecture 
		flags |= (1<<_MIF_);
	}
	if (flags & (1<<_BMPEF_)){
		blink();
	}
}

ISR (TIMER1_OVF_vect) {
	pulseT++;
	TCNT1 = 0xFE;
}

/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/

#define UART_BUFLEN 10
void uart_print(char *name, long val)
{
	char debug_buffer[UART_BUFLEN];

	SerialTxChar(name);
	SerialTxChar(" = ");

	ltoa((val), debug_buffer, UART_BUFLEN);
	SerialTxChar(debug_buffer);
	SerialTxChar("\n");
}
/*
void serialOut()
{
	seaLevel = 1013.25;
	bmp280_measure();
	uint16_t centimeter = (uint16_t)(abs(bmp280_getaltitude(seaLevel))%100);
	int32_t meter;
	if (bmp280_getaltitude(seaLevel)>0){
		meter = bmp280_getaltitude(seaLevel)-centimeter;
	}else{
		meter = -1*(abs(bmp280_getaltitude(seaLevel))+centimeter);
	}
	meter = meter/100;
	SendValue(FRSKY_USERDATA_BARO_ALT_B,(int16_t)meter);
	SendValue(FRSKY_USERDATA_BARO_ALT_A, centimeter);
	SerialTx(0x5E);// End of Frame
	blink(); 
}
*/
void serialOut()
{
	bmp280_measure();
	uint16_t centimeter = (uint16_t)(abs(bmp280_getaltitude())%100);
	int32_t meter;
	if (bmp280_getaltitude()>0){
		meter = bmp280_getaltitude()-centimeter;
		}else{
		meter = -1*(abs(bmp280_getaltitude())+centimeter);
	}
	meter = meter/100;
	SendValue(FRSKY_USERDATA_BARO_ALT_B,(int16_t)meter);
	SendValue(FRSKY_USERDATA_BARO_ALT_A, centimeter);
	SerialTx(0x5E);// End of Frame
	blink();
}

/*
void serialOut()
{
	//seaLevel = 1013.25;
	bmp280_measure();
	uint16_t centimeter = (uint16_t)(abs(bmp280_getpressure())%100);
	int32_t meter;
	if (bmp280_getpressure()>0){
		meter = bmp280_getpressure()-centimeter;
		}else{
		meter = -1*(abs(bmp280_getpressure())+centimeter);
	}
	meter = meter/100;
	SendValue(FRSKY_USERDATA_BARO_ALT_B,(int16_t)meter);
	SendValue(FRSKY_USERDATA_BARO_ALT_A, centimeter);
	SerialTx(0x5E);// End of Frame
	blink();
}
*/

void blink(){
	PORTB ^= (1<<PB1);
}

/**********************************************************/
/* SendValue => send a value as FrSky sensor hub data     */
/**********************************************************/
void SendValue(uint8_t ID, uint16_t Value)
{
	uint8_t tmp1 = Value & 0x00ff;
	uint8_t tmp2 = (Value & 0xff00)>>8;

	SerialTx(0x5E);
	SerialTx(ID);
	if(tmp1 == 0x5E) {
		SerialTx(0x5D);
		SerialTx(0x3E);
	}
	else if(tmp1 == 0x5D) {
		SerialTx(0x5D);
		SerialTx(0x3D);
	}
	else {
		SerialTx(tmp1);
	}

	if(tmp2 == 0x5E) {
		SerialTx(0x5D);
		SerialTx(0x3E);
	}
	else if(tmp2 == 0x5D) {
		SerialTx(0x5D);
		SerialTx(0x3D);
	}
	else {
		SerialTx(tmp2);
	}
}

int main(void)
{
	/************************************************************************/
	/*                             INIT                                     */
	/************************************************************************/
	// CLOCK //
	CLKPR = (1<<CLKPCE); // en premier 
	CLKPR = (1<<CLKPS0); // puis dans les 4 cycles suivant
	// PORTS //
	DDRB	= (1<<UART_PIN) | (1<<LED_PIN);
	PORTB	= 0b00000000;
	// flags //
	flags = 0x00;
	
	/* Timer 0 and timer 1  overflow *************/
	/*     (1/4000000)*1024*256 = 65.536ms      */
	TCCR0B = (1<<CS02)|(1<<CS00); // 1024
	TCNT0 = 0x00;
	/*     (1/4000000)*128*1 = 0.032ms      */
	TCCR1 = (1<<CS12)|(1<<CS11)|(1<<CS10); // 64 /256
	TCNT1 = 0xFE;
	/* mise en route */
	TIMSK = (1<<TOIE0) | (1<<TOIE1);
	
	SerialInit( PB3  , 9600 );
	
	if (!bmp280_init())
	{
		flags |= (1<<_BMPEF_);
	}
	
	
	sei();
	
    /************************************************************************/
    /* MAIN                                                                 */
    /************************************************************************/
    for(;;) 
    {
		if (flags & (1<<_MIF_))
		{
			flags &= ~(1<<_MIF_); 
			/*
			//uart_print("status", bmp280_get_status());
			bmp280_measure();
			//uart_print("temperature x 100", bmp280_gettemperature());
			//uart_print("pressure x 100   ", bmp280_getpressure());
			uart_print("", bmp280_getaltitude(1024));
			SerialTxChar("\n");
			*/
			serialOut();
		}
		
    }
}

