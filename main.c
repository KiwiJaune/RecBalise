#include "UserAckMiwi.h"
#include "init.h"
#include <p33FJ128MC804.h>


_FOSCSEL(FNOSC_PRI);                                    //primary osc
_FOSC(OSCIOFNC_ON & POSCMD_EC);                        // EC Osc - PUTAIN D'XT DE MERDE !!!!! 
_FWDT(FWDTEN_OFF & WDTPOST_PS2);                         // Disable Watchdog timer
_FICD(JTAGEN_OFF & ICS_PGD1);
// JTAG should be disabled as well


/************************** VARIABLES ************************************/
#define CARTE_MIWI 0x00C2

#define SERVO_ON	LATAbits.LATA8 
#define LED 		LATCbits.LATC7 
#define LASER_ON	LATAbits.LATA7 
#define PWMH		LATBbits.LATB14
#define	PWML		LATBbits.LATB15
#define LASER_1 	PORTCbits.RC0
#define LASER_2 	PORTCbits.RC1

#define FALLING_EDGE 		0
#define RISING_EDGE 		1
#define SIGNAL_SERVO1 		LATAbits.LATA3
#define SIGNAL_SERVO2		LATCbits.LATC2
#define CPT_PERIODE_20MS	6250

#define BALISE 			4

#define  MAX_CHNUM	 			7		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			8		// Number of channels enabled for channel scan

unsigned int  Cpt_Tmr_Periode = 0,Periode_Servo1 = 0,Periode_Servo2=0;

unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int ADC_Results[8],DmaBuffer = 0,tension[2];
float tensionf[2];

unsigned char watchdog=0;
unsigned int periode_tour;
char Fin_de_tour = 0;
char capteurHautPrec = 0;
char capteurBasPrec = 0;
char oldMagnet = 0;
unsigned char nombre_angles[2]; // n angles (8 maxi)
unsigned char buffer_angles[2][2*8]; // debut_1_MSB; debut_1_LSB; fin_1_MSB; fin_1_LSB; debut_2_MSB; debut_2_LSB; fin_2_MSB; fin_2_LSB; ...;debut_n_MSB; debut_n_LSB; fin_n_MSB; fin_n_LSB; 
unsigned char nombre_fronts[2]; // n fronts (8 maxi)
unsigned int buffer_fronts[2][8]; // Valeurs bruts TMR2
unsigned int buffer_fronts_temp[2][8]; // Valeurs bruts TMR2
unsigned int ptr_fronts_bas; // ptr pour l'enregistrement
unsigned int ptr_fronts_haut; // 
unsigned int hall_front;
unsigned int motor_speed;
float angle;

unsigned char idbalise;

char pwm(unsigned char motor, float value);
void Initpwm(void);
void calcul_angles(void);

#define CAPTEUR_HAUT PORTCbits.RC2 // LATA de merde
#define CAPTEUR_BAS PORTCbits.RC1
//#define CAPTEUR_HAUT PORTAbits.RA10 // LATA de merde
//#define CAPTEUR_BAS PORTAbits.RA7
#define MAGNET PORTCbits.RC5
#define IDCAPTEUR_BAS 0
#define IDCAPTEUR_HAUT 1
#define IDFIN_DE_TOUR	0xE3
#define IDREQUEST	0xE4

#define FRONT_MONTANT 0x10
#define FRONT_DESCENDANT 0x11

void InitCapteurs()
{
	//TRISCbits.TRISC2 = 1;	// Capteur haut en entrée
	//TRISCbits.TRISC1 = 1;	// Capteur bas en entrée
	//TRISAbits.TRISA10 = 1;	// Capteur haut en entrée
	//TRISAbits.TRISA7 = 1;	// Capteur bas en entrée
}

void FonctionDebug(int numDebug)
{
}

#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{ 
	TRISCbits.TRISC7 = 0;
    BYTE i;
	char testConn = 0;

	BYTE indiceTabTrameMiwi = 0;

	//Trame Miwi envoi
	Trame trameMiwiTx;
	static BYTE messMiwiTx[50];
	trameMiwiTx.message = messMiwiTx;
	
	//Trame Miwi reception
	Trame trameMiwiRx;
	static BYTE messMiwiRx[50];
	trameMiwiRx.message = messMiwiRx;
	
	TRISAbits.TRISA0=1; // VADCV1 - Gestion d'alim
	TRISAbits.TRISA1=1; // VADCV2 - Gestion d'alim
	TRISAbits.TRISA2=1; // Oscillateur 8MHz - Oscillateur
	TRISAbits.TRISA3=0; // SERVO_1 - Servo (contrôle d'assiette)
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
	TRISCbits.TRISC2=0; // SERVO_2 - Servo (contrôle d'assiette)
//	TRISCbits.TRISC3=0; // E1 - Gestion d'alim
	TRISCbits.TRISC4=1; // SDI - Miwi
	TRISCbits.TRISC5=1; // SCK - Miwi
	TRISCbits.TRISC6=0; // RESET - Miwi
	TRISCbits.TRISC7=0; // LED - Debug
	TRISCbits.TRISC8=1; // INT - LTC2955
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
	AD1PCFGL = 0x1FC;	// All pins except VADCV1 & VADCV2		
	
    while(1)
	{
	    /*******************************************************************/
	    // Initialize the system
	    /*******************************************************************/
	    Initpwm();		// Configuration du module PWM 
		pwm(BALISE, 0);    
	    
		idbalise = 0xB3;
		if(PORTBbits.RB3 == 1)
			idbalise = 0xB2;	
		if(PORTBbits.RB2 == 1)
			idbalise = 0xB1;
	
		//Init Miwi
		InitMiwi(idbalise);

		RPINR7bits.IC1R = 10;  // Capteur effet hall
		RPINR7bits.IC2R = 16;  // Capteur laser 1 (bas)
		RPINR10bits.IC7R = 17; // Capteur laser 2 (haut)
		
		// Initialize the Input Capture Module
		IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
		IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
		IC1CONbits.ICI = 0b00; // Interrupt on every capture event
		IC1CONbits.ICM = 0b010; // Generate capture event on every Falling edge
		// Enable Capture Interrupt And Timer2
		IPC0bits.IC1IP = 5; // Setup IC1 interrupt priority level
		IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
		IEC0bits.IC1IE = 1; // Enable IC1 interrupt
		
		// Initialize the Input Capture Module
		IC2CONbits.ICM = 0b00; // Disable Input Capture 1 module
		IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
		IC2CONbits.ICI = 0b00; // Interrupt on every capture event
		IC2CONbits.ICM = 0b011; // Generate capture event on every Rising edge
		// Enable Capture Interrupt And Timer2
		IPC1bits.IC2IP = 6; // Setup IC1 interrupt priority level
		IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
		IEC0bits.IC2IE = 1; // Enable IC1 interrupt
		
		// Initialize the Input Capture Module
		IC7CONbits.ICM = 0b00; // Disable Input Capture 1 module
		IC7CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
		IC7CONbits.ICI = 0b00; // Interrupt on every capture event
		IC7CONbits.ICM = 0b011; // Generate capture event on every Rising edge
		// Enable Capture Interrupt And Timer2
		IPC5bits.IC7IP = 6; // Setup IC1 interrupt priority level
		IFS1bits.IC7IF = 0; // Clear IC1 Interrupt Status Flag
		IEC1bits.IC7IE = 1; // Enable IC1 interrupt
		
	
		// Init ADC
	
		AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
		AD1CON1bits.SSRC   = 7;		// Sample Clock Source: Conversion autostart
		AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
		AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation
	
		AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
		AD1CON2bits.CHPS  = 0;		// Converts CH0
	
		AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
		AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
									// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us	
		
		AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
		AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 6 ADC Channel is scanned
		AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words
	
		//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
		AD1CSSLbits.CSS0=1;		// Enable AN0 for channel scan
		AD1CSSLbits.CSS1=1;		// Enable AN1 for channel scan
		AD1CSSLbits.CSS2=0;		// Enable AN2 for channel scan
		AD1CSSLbits.CSS3=0;		// Enable AN3 for channel scan
		AD1CSSLbits.CSS6=0;		// Enable AN6 for channel scan
		AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
		AD1CSSLbits.CSS8=0;		// Enable AN8 for channel scan
		
	 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
		AD1PCFGL=0xFFFF;
		AD1PCFGLbits.PCFG0 = 0;	// AN0 as Analog Input
		AD1PCFGLbits.PCFG1 = 0;	// AN1 as Analog Input
	 	AD1PCFGLbits.PCFG2 = 1;	// AN2 as Digital Input
		AD1PCFGLbits.PCFG3 = 1;	// AN3 as Digital Input 
		AD1PCFGLbits.PCFG6 = 1;	// AN6 as Digital Input
		AD1PCFGLbits.PCFG7 = 1;	// AN7 as Digital Input
		AD1PCFGLbits.PCFG8 = 1;	// AN8 as Digital Input 
		
		IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
		IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt 
		AD1CON1bits.ADON = 1;		// Turn on the A/D converter
	
		// Init DMA
	
		DMA5CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
		DMA5CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
		DMA5PAD=(int)&ADC1BUF0;
		DMA5CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
		DMA5REQ = 13;					// Select ADC1 as DMA Request source
	
		DMA5STA = __builtin_dmaoffset(BufferA);		
		DMA5STB = __builtin_dmaoffset(BufferB);
	
		IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
		IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
		
		DMA5CONbits.CHEN=1;				// Enable DMA
	
		
		// RECBUN : RB2 = 1 RB3 = 0;
		// RECBEU : RB2 = 0 RB3 = 1;
		// RECBOI : RB2 = 0 RB3 = 0;

		// Signale sa présence
		trameMiwiTx.message[0] = idbalise;
		trameMiwiTx.message[1] = 0xF5;
		
		tensionf[0] = ADC_Results[0]*0.322667695;
		tensionf[1] = ADC_Results[1]*0.322667695;
	
		tension[0] = (unsigned int)tensionf[0];
		tension[1] = (unsigned int)tensionf[1];
		
		trameMiwiTx.message[2] = tension[0]>>8;
		trameMiwiTx.message[3] = tension[0]&0x00FF;
	     		
		trameMiwiTx.message[4] = tension[1]>>8;
		trameMiwiTx.message[5] = tension[1]&0x00FF;
		
		MiwiTasks();
		trameMiwiTx.nbChar = 6;
		EnvoiMiwi(CARTE_MIWI, BUFFER, trameMiwiTx);
		//EnvoiDebug(MY_SHORT_ADDRESS >> 8, MY_SHORT_ADDRESS, 10, 12);
		//EnvoiDebug(PORTBbits.RB3, PORTBbits.RB2, 3, 4);
		motor_speed = 0;
		
		// Init T4 (servo)
		T4CONbits.TON 	= 0;	//Stops the timer
		T4CONbits.TSIDL = 0;
		T4CONbits.TGATE = 0;
		T4CONbits.TCS	= 0;
		T4CONbits.T32	= 0;
		T4CONbits.TCKPS = 0b00;//10; //Prescaler set to 1:64
		
		TMR4 = 0; 				//Clear timer register
		PR4  = 40;				//Full Period = 20.5 µs
	
		IPC6bits.T4IP = 5; 		//Set Timer2 Interrupt Priority Level
		IFS1bits.T4IF = 0; 		//Clear Timer2 Interrupt Flag
		IEC1bits.T4IE = 1; 		//Enable Timer2 interrupt
		T4CONbits.TON = 1;		//Timer enabled
	
		InitT2();

		while(watchdog<10)
	    {	
			if(Fin_de_tour == 1)
			{
				Fin_de_tour = 0;
				
				for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
				{
					angle = (float)(buffer_fronts[IDCAPTEUR_HAUT][i]) / (float)(periode_tour) * 36000;
					buffer_angles[IDCAPTEUR_HAUT][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
					buffer_angles[IDCAPTEUR_HAUT][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
				}
				for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
				{
					angle = (float)(buffer_fronts[IDCAPTEUR_BAS][i]) / (float)(periode_tour) * 36000;
					buffer_angles[IDCAPTEUR_BAS][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
					buffer_angles[IDCAPTEUR_BAS][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
				}
	
				trameMiwiTx.message[0] = idbalise;
				trameMiwiTx.message[1] = IDREQUEST;
	
				trameMiwiTx.message[2] = (periode_tour >> 8) & 0x00FF;
				trameMiwiTx.message[3] = periode_tour & 0x00FF;
				
				
				trameMiwiTx.message[4] = nombre_angles[IDCAPTEUR_HAUT];
				trameMiwiTx.message[5] = nombre_angles[IDCAPTEUR_BAS];
	       		
				indiceTabTrameMiwi = 6;
				
	       		for(i = 0; i < nombre_angles[IDCAPTEUR_HAUT]; i++)
				{
					trameMiwiTx.message[indiceTabTrameMiwi+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i];	//MSB
	       			trameMiwiTx.message[indiceTabTrameMiwi+1+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i+1]; //LSB
				}
				
				indiceTabTrameMiwi += nombre_angles[IDCAPTEUR_HAUT]*2;
				
				for(i = 0; i < nombre_angles[IDCAPTEUR_BAS]; i++)
				{
					trameMiwiTx.message[indiceTabTrameMiwi+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i];	 //MSB
	       			trameMiwiTx.message[indiceTabTrameMiwi+1+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i+1]; //LSB
				}
				
				indiceTabTrameMiwi += nombre_angles[IDCAPTEUR_BAS]*2;
				
				trameMiwiTx.nbChar = indiceTabTrameMiwi;
				EnvoiMiwi(CARTE_MIWI,BUFFER,trameMiwiTx);
				indiceTabTrameMiwi = 0;
			}		
	
			//--Miwi Reception
	        MiwiTasks(); 
			
	        if(MiwiIsDataReady())
	        {
				trameMiwiRx = MiwiGetData();
				
				if(trameMiwiRx.message[0] == idbalise)
				{
					if(trameMiwiRx.message[1] == 0x01)
					{
						motor_speed = trameMiwiRx.message[2] * 256 + trameMiwiRx.message[3];
						if(motor_speed == 0)	LASER_ON = 0;
						else					LASER_ON = 1;
						pwm(BALISE, trameMiwiRx.message[2] * 256 + trameMiwiRx.message[3]);
						watchdog=0;
					}
					else if(trameMiwiRx.message[1] == 0x10)
					{
						Periode_Servo1 = trameMiwiRx.message[2] * 256 + trameMiwiRx.message[3];
						if(Periode_Servo1 == 0) SERVO_ON = 0;
						else					SERVO_ON = 1;
					}
					else if(trameMiwiRx.message[1] == 0x11)
					{
						Periode_Servo2 = trameMiwiRx.message[2] * 256 + trameMiwiRx.message[3];
						if(Periode_Servo2 == 0) SERVO_ON = 0;
						else					SERVO_ON = 1;
					}
					else if(trameMiwiRx.message[1] == 0xF0)
					{
						testConn  = 1;
					}
					else if(trameMiwiRx.message[1] == 0xF1) // RESET
					{
						Reset();
					}
					else if(trameMiwiRx.message[1] == 0xEE) // RESET
					{
						FonctionDebug(trameMiwiRx.message[2]);
					}
				}
	        }
			else if(testConn)
			{
				testConn = 0;
				
				trameMiwiTx.message[0] = idbalise;
				trameMiwiTx.message[1] = 0xF5;
	
				tensionf[0] = ADC_Results[0]*0.322667695;
				tensionf[1] = ADC_Results[1]*0.322667695;
	
				tension[0] = (unsigned int)tensionf[0];
				tension[1] = (unsigned int)tensionf[1];
	
				trameMiwiTx.message[2] = tension[0]>>8;
				trameMiwiTx.message[3] = tension[0]&0x00FF;
	
				trameMiwiTx.message[4] = tension[1]>>8;
				trameMiwiTx.message[5] = tension[1]&0x00FF;
	
				trameMiwiTx.nbChar = 6;
				EnvoiMiwi(CARTE_MIWI,BUFFER,trameMiwiTx);
			}
	    }//while(watchdog)
	}//while(1)
}

char pwm(unsigned char motor, float value) // Value = +/- 4000
{
	if(value >  4095) value =  4095;
	if(value < -4095) value = -4095;
	
	/*if(value >  2000) value =  2000; // config de test, faible puissance
	if(value < -2000) value = -2000;*/
	
	switch(motor)
	{
		case BALISE: 
		//		if(value >  500) value =  500; // config de test, faible puissance
		//		if(value < -500) value = -500;
				if(value > 0)	// Moteur Balise
				{
					//DIRB  = 1;		// Position incremente
					P1DC1 = (unsigned int)(4095- value);		
				}
				else
				{
					//DIRB  = 0;		// Position decremente
					P1DC1 = (unsigned int)(4095 + value); // 15/04/2012		
				}
				break;
		default : return -1;
	}
	return 0;
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void)
{
	unsigned char i;
	TMR2=0;
	IC2CONbits.ICM = 0b011;
	IC7CONbits.ICM = 0b011;
	periode_tour = IC1BUF;
	
	IFS0bits.IC1IF=0;
	LASER_ON=1;

	nombre_angles[IDCAPTEUR_HAUT] = ptr_fronts_haut;
	nombre_angles[IDCAPTEUR_BAS] = ptr_fronts_bas;

	for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
	{
		buffer_fronts[IDCAPTEUR_HAUT][i] = 	buffer_fronts_temp[IDCAPTEUR_HAUT][i];
	}
	for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
	{
		buffer_fronts[IDCAPTEUR_BAS][i] = 	buffer_fronts_temp[IDCAPTEUR_BAS][i];
	}
	
	ptr_fronts_haut=0;
	ptr_fronts_bas=0;
	//if(watchdog++>30) Reset(); // protection ultime en cas de miwi + boucle principale bloquée
	Fin_de_tour=1;
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas]=IC2BUF;
	if(ptr_fronts_bas < 8) ptr_fronts_bas++;
	IC2CONbits.ICM = 0b001;
	IFS0bits.IC2IF=0;
}

void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut]=IC7BUF;		
	if(ptr_fronts_haut < 8) ptr_fronts_haut++;
	IC7CONbits.ICM = 0b001;
	IFS1bits.IC7IF=0;
}

void calcul_angles(void)
{
	
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
	unsigned char i;
	ADC_Results[0]=0;
	ADC_Results[1]=0;
	ADC_Results[2]=0;
	ADC_Results[3]=0;
	ADC_Results[4]=0;
	ADC_Results[5]=0;
	if(DmaBuffer == 0)
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferA[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferA[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferA[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferA[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferA[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferA[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	else
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferB[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferB[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferB[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferB[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferB[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferB[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	
	DmaBuffer ^= 1;

	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}


void __attribute__((__interrupt__,__auto_psv__)) _T4Interrupt(void)
{	
	IFS1bits.T4IF = 0; 		//Clear Timer1 Interrupt flag
	
	Cpt_Tmr_Periode++;
	
	if(Cpt_Tmr_Periode == Periode_Servo1)
	{
		SIGNAL_SERVO1 = FALLING_EDGE;
	}
		
	if(Cpt_Tmr_Periode == Periode_Servo2)
	{
		SIGNAL_SERVO2 = FALLING_EDGE;
	}
	if(Cpt_Tmr_Periode == 2000) // 20 ms periode
	{
		SIGNAL_SERVO1 = RISING_EDGE;
		SIGNAL_SERVO2 = RISING_EDGE;
		Cpt_Tmr_Periode = 0;
	}		
}
