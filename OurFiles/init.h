#ifndef __INIT__
#define __INIT__

#include <p33FJ128MC804.h>

#define LIGHT   0x01
#define SWITCH  0x02

#define RECBUFFER 		50			// Taille du buffer reception UART
#define FCY             8000000		// Nombre d'instructions par secondes (IPS)
#define BAUDRATE        115200		// Debit UART (RS232)
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 // Precalcul pour le baudrate generator

#define INT 			LATAbits.LATA3
#define SERVO_ON		LATAbits.LATA8 
#define LED 			LATCbits.LATC7 
#define LASER_ON		LATAbits.LATA7 
#define PWMH			LATBbits.LATB14
#define	PWML			LATBbits.LATB15
#define LASER_1 		PORTCbits.RC0
#define LASER_2 		PORTCbits.RC1
#define IDCAPTEUR_1 	0
#define IDCAPTEUR_2 	1
#define IDFIN_DE_TOUR	0xE3
#define IDREQUEST		0xE4
#define BALISE 			4

void Init(void);
void Initpwm(void);
void InitT2(void);


#endif