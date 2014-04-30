#include "init.h"

void Init(void)
{
		TRISAbits.TRISA0=1; // VADCV1 - Gestion d'alim
	TRISAbits.TRISA1=1; // VADCV2 - Gestion d'alim
	TRISAbits.TRISA2=1; // Oscillateur 8MHz - Oscillateur
//	TRISAbits.TRISA3=0; // SERVO_1 - Servo (contrôle d'assiette)
//	TRISAbits.TRISA4=0; // CS - Miwi
	TRISAbits.TRISA7=0; // SHD_+18V - Gestion d'alim
	TRISAbits.TRISA8=0; // 5V_ON/OFF - Gestion d'alim
//	TRISAbits.TRISA9=0; // E2 - Gestion d'alim
//	TRISAbits.TRISA10=0; // KILL - Gestion d'alim
	TRISBbits.TRISB0=1; // PGD - JTAG & Leds RGB
	TRISBbits.TRISB1=1; // PGC - JTAG & Leds RGB
	TRISBbits.TRISB2=1; // ADDR_H - Adressage carte
	TRISBbits.TRISB3=1; // ADDR_L - Adressage carte
	TRISBbits.TRISB4=1; // - - -
	TRISBbits.TRISB5=1; // SDO - Miwi
	TRISBbits.TRISB6=1; // INT - Miwi
	TRISBbits.TRISB7=1; // WAKE - Miwi
	TRISBbits.TRISB8=1; // SCL - I²C
	TRISBbits.TRISB9=1; // SDA - I²C
	TRISBbits.TRISB10=1; // TOP_TOUR - Capteur effet hall
	TRISBbits.TRISB11=1; // SDI1 - Codeur magnétique & Leds RGB
//	TRISBbits.TRISB12=0; // SCK1 - Codeur magnétique & Leds RGB
//	TRISBbits.TRISB13=0; // SDO1 - Codeur magnétique & Leds RGB
	TRISBbits.TRISB14=0; // RIN - Pilotage moteur
	TRISBbits.TRISB15=0; // FIN - Pilotage moteur
	TRISCbits.TRISC0=1; // LR61_1 - Detecteur laser
	TRISCbits.TRISC1=1; // LR61_2 - Detecteur laser
//	TRISCbits.TRISC2=0; // SERVO_2 - Servo (contrôle d'assiette)
//	TRISCbits.TRISC3=0; // E1 - Gestion d'alim
	TRISCbits.TRISC4=1; // SDI - Miwi
	TRISCbits.TRISC5=1; // SCK - Miwi
//	TRISCbits.TRISC6=0; // RESET - Miwi
	TRISCbits.TRISC7=0; // LED - Debug
	TRISCbits.TRISC8=1; // INT - Miwi
//	TRISCbits.TRISC9=0; // CS1 - Codeur magnétique

	CNPU2bits.CN16PUE = 1;  // TOP_TOUR - Capteur effet hall
	// A propos du top tour: cette fois ce n'est pas une bascule : etat bas quand champ magnétique > seuil
	CNPU1bits.CN6PUE = 1; // ADDR_H - Adressage carte
	CNPU1bits.CN7PUE = 1; // ADDR_L - Adressage carte
	CNPU1bits.CN8PUE = 1; // LR61_1 - Detecteur laser
	CNPU1bits.CN9PUE = 1; // LR61_2 - Detecteur laser
	
	SERVO_ON = 0;	// 5V OFF
	LED = 0; 		// LED OFF 
	LASER_ON = 0;	// LASER_OFF
	PWMH=0;
	PWML=0;
	LED=1;
	AD1PCFGL = 0x1FC;	// All pins except VADCV1 & VADCV2		

    Initpwm();		// Configuration du module PWM 
    InitT2();		// Configuration du timer 2	
	
	RPINR7bits.IC1R = 10;  // Capteur effet hall
	RPINR7bits.IC2R = 16;  // Capteur laser 1 (bas)
	RPINR10bits.IC7R = 17; // Capteur laser 2 (haut)
	
	// Initialize the Input Capture Module
	IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC1CONbits.ICI = 0b00; // Interrupt on every capture event
	IC1CONbits.ICM = 0b011; // Generate capture event on every Rising edge
	// Enable Capture Interrupt And Timer2
	IPC0bits.IC1IP = 3; // Setup IC1 interrupt priority level
	IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC2CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC2CONbits.ICI = 0b00; // Interrupt on every capture event
	IC2CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer2
	IPC1bits.IC2IP = 2; // Setup IC1 interrupt priority level
	IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC2IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC7CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC7CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC7CONbits.ICI = 0b00; // Interrupt on every capture event
	IC7CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer2
	IPC5bits.IC7IP = 1; // Setup IC1 interrupt priority level
	IFS1bits.IC7IF = 0; // Clear IC1 Interrupt Status Flag
	IEC1bits.IC7IE = 1; // Enable IC1 interrupt
	

}