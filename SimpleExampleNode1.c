/****************************************************************************
* FileName:		main.c
* Dependencies: none   
* Processor:	PIC18, PIC24F, PIC32, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher	
*               Microchip C32 v1.02 or higher
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice for MiWi DE Software:
*
* Copyright © 2007-2012 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute 
* Software only when embedded on a Microchip microcontroller or digital 
* signal controller and used with a Microchip radio frequency transceiver, 
* which are integrated into your product or third party product (pursuant 
* to the terms in the accompanying license agreement).   
*
* You should refer to the license agreement accompanying this Software for 
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
* ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
* LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
* TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT 
* NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
****************************************************************************
* File Description:
*
*  This is the simple example that demonstrate the simple programming
*  interface of MiWi Development Environment (DE). As you may see in
*  the demo code, besides application specific code, all wireless
*  communicate code is less than 30 lines. This simple example must be 
*  used with node 2 of simple example.
*  In this simple example, following features have been demonstrated:
*   - Hand Shake
*       A hand-shaking process has been demonstrated by establishing
*       connection with a peer device.
*   - Receiving Message
*       This example demonstrate how to check received message and 
*       available information for the received data. Finally, this
*       example also show how to process the message
*   - Transmitting Message
*       This example demonstrate how to transmit message by broadcast
*       or unicast
*   - Security
*       This example demonstrate how to require the protocol stack
*       to encrypt the outgoing message. It also shows how the 
*       protocol stack automatically decrypt the incoming message
*       and provide the security status to the application layer.
*
* Detailed demo flow chart and execution instructions can be found in
*   MiWi DE help file located at directory <MLA Install Directory>/
*   Microchip/Help. From the content tab, find the document about
*   Simple Example at <ROOT> -> "Demos" -> "Running Demos" -> 
*   "Basic Demos" -> "Simple Example".
*
* Change History:
*  Rev   Date         Author    Description
*  0.1   1/03/2008    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
*  3.1   5/28/2010    yfy       MiWi DE 3.1
*  4.1   1/31/2012    yfy       MiWi DE 4.2, simplified demo interface
**************************************************************************/

/************************ HEADERS ****************************************/
#include "ConfigApp.h"
#include "WirelessProtocols/MCHP_API.h"
#include "WirelessProtocols/Console.h"
//#include "DemoOutput.h"
#include "HardwareProfile.h"
#include "DefineBalise.h"

/************************** VARIABLES ************************************/
#define LIGHT   0x01
#define SWITCH  0x02


#define RECBUFFER 		50			// Taille du buffer reception UART
#define FCY             40000000	// Nombre d'instructions par secondes (IPS)
#define BAUDRATE        115200		// Debit UART (RS232)
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 // Precalcul pour le baudrate generator
#define N				2			// Nombre de moteurs
#define DEFAULT_KP		1//10//70			// Coefficient proporionnel 
#define DEFAULT_KI		50//20//10			// Coefficient integral (inverse !)
#define DEFAULT_KD		0//90//110			// Coefficient derive
#define CODEUR			500 		// Nombre de pas par tour moteur (sans le ratio x4)
#define REDUCTEUR		1	// Reducteur utilise en sortie d'arbre moteur (=1 si roue codeuse indépendante)
#define DIAMETRE_ROUE 	35*1.071//1.064019			// Diametre de la roue motrice (ou roue codeuse si indépendante) en mm 
#define PI 				3.1416		// Ben pi quoi
#define VOIE			280*1.014//1.0344827586206896551724137931034//1.006	//1.0344827586206896551724137931034			// Distance entre les deux roues en mm
#define COEFF_ROUE		1.0000		// Coeff d'ajustement pour le diametre de la roue
#define COEFF_VOIE		1.0000		// Coeff d'ajustement pour la voie
#define MM_SCALER		COEFF_ROUE*DIAMETRE_ROUE*PI/(4*CODEUR*REDUCTEUR) // Formule de conversion [pas]<==>[mm]
#define MM_INVSCALER	4*CODEUR*REDUCTEUR/(COEFF_ROUE*DIAMETRE_ROUE*PI)
#define DEFAULT_SPEED	500			// Vitesse par défaut en mm/s
#define	MAX_SPEED		500
#define DEFAULT_ACCEL	500			// Acceleration par défaut en mm/s^2
#define ERROR_ALLOWED	1			// En cas de sifflement moteur intempestif (en pas)
#define KP	0
#define KI	1
#define KD	2


#define SERVO_ON	LATAbits.LATA8 
#define LED 		LATCbits.LATC7 
#define LASER_ON	LATAbits.LATA7 
#define PWMH		LATBbits.LATB14
#define	PWML		LATBbits.LATB15
#define LASER_1 	PORTCbits.RC0
#define LASER_2 	PORTCbits.RC1


#define INT 	LATAbits.LATA3
#define STOP			0x01
#define VITESSE			0x02
#define ACCELERATION	0x03
#define AVANCE			0x04
#define PIVOT			0x05
#define VIRAGE			0x06
#define DISTANCE		0x08
#define ARRIVE			0x10
#define PIVOTG			0x11
#define PIVOTD			0x12
#define RECULE			0x13
#define COUPURE			0x66
#define COEFF_P			0x20
#define COEFF_I			0x21
#define COEFF_D			0x22
#define ROULEAU_AV		0x81
#define ROULEAU_AR		0x82

#define FALSE			0x00
#define TRUE			0x01


#define AVANT 			0			// Convention 
#define ARRIERE 		1
#define GAUCHE 			2			
#define DROITE 			3
#define ON				1
#define OFF				0
#define FREELY			0
#define SMOOTH			1
#define ABRUPT			2

#define BALISE 			4


/*************************************************************************/
// AdditionalNodeID variable array defines the additional 
// information to identify a device on a PAN. This array
// will be transmitted when initiate the connection between 
// the two devices. This  variable array will be stored in 
// the Connection Entry structure of the partner device. The 
// size of this array is ADDITIONAL_NODE_ID_SIZE, defined in 
// ConfigApp.h.
// In this demo, this variable array is set to be empty.
/*************************************************************************/
#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {LIGHT};
#endif

/*************************************************************************/
// The variable myChannel defines the channel that the device
// is operate on. This variable will be only effective if energy scan
// (ENABLE_ED_SCAN) is not turned on. Once the energy scan is turned
// on, the operating channel will be one of the channels available with
// least amount of energy (or noise).
/*************************************************************************/
BYTE myChannel = 24;

unsigned char watchdog;
unsigned char chksum;
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

/*********************************************************************
* Function:         void main(void)
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    none
*
* Side Effects:	    none
*
* Overview:		    This is the main function that runs the simple 
*                   example demo. The purpose of this example is to
*                   demonstrate the simple application programming
*                   interface for the MiWi(TM) Development 
*                   Environment. By virtually total of less than 30 
*                   lines of code, we can develop a complete 
*                   application using MiApp interface. The 
*                   application will first try to establish a
*                   link with another device and then process the 
*                   received information as well as transmit its own 
*                   information.
*                   MiWi(TM) DE also support a set of rich 
*                   features. Example code FeatureExample will
*                   demonstrate how to implement the rich features 
*                   through MiApp programming interfaces.
*
* Note:			    
**********************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{   
    BYTE i;
    BYTE TxSynCount = 0;
    BYTE TxSynCount2 = 0;
    BYTE TxNum = 0;
    BYTE RxNum = 0;
	char recu = 0;
	char testConn = 0;
	unsigned int wait;

	
	
	/*for(wait=0;wait<65000;wait++) // Tempo : 60 nop x 65000 ~100ms min (@40MIPS)
	{
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
		Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();
	}*/

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
	
    while(1)
	{
    /*******************************************************************/
    // Initialize the system
    /*******************************************************************/
    Initpwm();		// Configuration du module PWM 
	pwm(BALISE,0);
	BoardInit();      
    ConsoleInit(); 
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
	

	
	
    //LED_1 = 0;
    //LED_2 = 0;

    /*******************************************************************/
    // Initialize Microchip proprietary protocol. Which protocol to use
    // depends on the configuration in ConfigApp.h
    /*******************************************************************/
    /*******************************************************************/
    // Function MiApp_ProtocolInit initialize the protocol stack. The
    // only input parameter indicates if previous network configuration
    // should be restored. In this simple example, we assume that the 
    // network starts from scratch.
    /*******************************************************************/
    MiApp_ProtocolInit(FALSE);

    // Set default channel
    if( MiApp_SetChannel(myChannel) == FALSE )
    {
        //DemoOutput_ChannelError(myChannel);
        #if defined(__18CXX)
            return;
        #else
            return 0;
        #endif
    }
    
    /*******************************************************************/
    // Function MiApp_ConnectionMode defines the connection mode. The
    // possible connection modes are:
    //  ENABLE_ALL_CONN:    Enable all kinds of connection
    //  ENABLE_PREV_CONN:   Only allow connection already exists in 
    //                      connection table
    //  ENABL_ACTIVE_SCAN_RSP:  Allow response to Active scan
    //  DISABLE_ALL_CONN:   Disable all connections. 
    /*******************************************************************/
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    //DemoOutput_Channel(myChannel, 0);
    
    /*******************************************************************/
    // Function MiApp_EstablishConnection try to establish a new 
    // connection with peer device. 
    // The first parameter is the index to the active scan result, 
    //      which is acquired by discovery process (active scan). If 
    //      the value of the index is 0xFF, try to establish a 
    //      connection with any peer.
    // The second parameter is the mode to establish connection, 
    //      either direct or indirect. Direct mode means connection 
    //      within the radio range; indirect mode means connection 
    //      may or may not in the radio range. 
    /*******************************************************************/
    i = MiApp_EstablishConnection(0xFF, CONN_MODE_DIRECT);
    
    /*******************************************************************/
    // Display current opertion on LCD of demo board, if applicable
    /*******************************************************************/
    if( i != 0xFF )
    {
		//LED_VERTE = LED_ON;
		//LED_ROUGE = LED_OFF;
        //DemoOutput_Channel(myChannel, 1);
    }
    else
    {
		//LED_VERTE = LED_OFF;
		//LED_ROUGE = LED_ON;
        /*******************************************************************/
        // If no network can be found and join, we need to start a new 
        // network by calling function MiApp_StartConnection
        //
        // The first parameter is the mode of start connection. There are 
        // two valid connection modes:
        //   - START_CONN_DIRECT        start the connection on current 
        //                              channel
        //   - START_CONN_ENERGY_SCN    perform an energy scan first, 
        //                              before starting the connection on 
        //                              the channel with least noise
        //   - START_CONN_CS_SCN        perform a carrier sense scan 
        //                              first, before starting the 
        //                              connection on the channel with 
        //                              least carrier sense noise. Not
        //                              supported for current radios
        //
        // The second parameter is the scan duration, which has the same 
        //     definition in Energy Scan. 10 is roughly 1 second. 9 is a 
        //     half second and 11 is 2 seconds. Maximum scan duration is 
        //     14, or roughly 16 seconds.
        //
        // The third parameter is the channel map. Bit 0 of the 
        //     double word parameter represents channel 0. For the 2.4GHz 
        //     frequency band, all possible channels are channel 11 to 
        //     channel 26. As the result, the bit map is 0x07FFF800. Stack 
        //     will filter out all invalid channels, so the application 
        //     only needs to pay attention to the channels that are not 
        //     preferred.
        /*******************************************************************/
        MiApp_StartConnection(START_CONN_DIRECT, 10, 0);
    }
	
    /*******************************************************************/
    // Function DumpConnection is used to print out the content of the
    //  Connection Entry on the hyperterminal. It may be useful in 
    //  the debugging phase.
    // The only parameter of this function is the index of the  
    //  Connection Entry. The value of 0xFF means to print out all
    //  valid Connection Entry; otherwise, the Connection Entry
    //  of the input index will be printed out.
    /*******************************************************************/
    DumpConnection(0xFF);

    // Turn on LED 1 to indicate connection established
    //LED_1 = 1;
    //DemoOutput_Instruction();

	// Signale sa présence
	MiApp_FlushTx();
                
	MiApp_WriteData(IDBALISE);
	MiApp_WriteData(0XF0);
                
   	MiApp_BroadcastPacket(FALSE);
	
	motor_speed = 0;

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

			MiApp_FlushTx();
			MiApp_WriteData(IDBALISE);
			chksum = IDBALISE;
			MiApp_WriteData(IDREQUEST); // Answer ID
			chksum ^= IDREQUEST;

			MiApp_WriteData((periode_tour >> 8 ) & 0x00FF); // MSB
       		chksum ^= (periode_tour >> 8 ) & 0x00FF;
			MiApp_WriteData((periode_tour      ) & 0x00FF); // LSB
			chksum ^= (periode_tour      ) & 0x00FF;

			MiApp_WriteData(nombre_angles[IDCAPTEUR_HAUT]);
			chksum ^= nombre_angles[IDCAPTEUR_HAUT];
       		MiApp_WriteData(nombre_angles[IDCAPTEUR_BAS]);
			chksum ^= nombre_angles[IDCAPTEUR_BAS];
       		
       		for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
			{
				MiApp_WriteData(buffer_angles[IDCAPTEUR_HAUT][2*i]); 	// MSB
				chksum ^= buffer_angles[IDCAPTEUR_HAUT][2*i];
       			MiApp_WriteData(buffer_angles[IDCAPTEUR_HAUT][2*i+1]);	// LSB
				chksum ^= buffer_angles[IDCAPTEUR_HAUT][2*i+1];
			}
			
			for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
			{
				MiApp_WriteData(buffer_angles[IDCAPTEUR_BAS][2*i]); 	// MSB
				chksum ^= buffer_angles[IDCAPTEUR_BAS][2*i];
				MiApp_WriteData(buffer_angles[IDCAPTEUR_BAS][2*i+1]);	// LSB
				chksum ^= buffer_angles[IDCAPTEUR_BAS][2*i+1];
			}

			MiApp_WriteData(chksum);	
       		
		   	MiApp_BroadcastPacket(FALSE);
		}		

        /*******************************************************************/
        // Function MiApp_MessageAvailable returns a boolean to indicate if 
        // a packet has been received by the transceiver. If a packet has 
        // been received, all information will be stored in the rxFrame, 
        // structure of RECEIVED_MESSAGE.
        /*******************************************************************/
        if( MiApp_MessageAvailable() )
        {
            /*******************************************************************/
            // If a packet has been received, handle the information available 
            // in rxMessage.
            /*******************************************************************/
            //DemoOutput_HandleMessage();
            //DemoOutput_UpdateTxRx(TxNum, ++RxNum);
            					
            // Toggle LED2 to indicate receiving a packet.

			if(rxMessage.Payload[0] == IDBALISE)
			{
				if(rxMessage.Payload[1] == 0x01)
				{
					motor_speed = rxMessage.Payload[2] * 256 + rxMessage.Payload[3];
					if(motor_speed == 0)	LASER_ON = 0;
					else					LASER_ON = 1;
					pwm(BALISE, rxMessage.Payload[2] * 256 + rxMessage.Payload[3]);
					watchdog=0;
				}

				else if(rxMessage.Payload[1] == IDREQUEST)
				{
					
					MiApp_FlushTx();
					MiApp_WriteData(IDBALISE);
					MiApp_WriteData(IDREQUEST); // Answer ID
										
					MiApp_WriteData(nombre_angles[IDCAPTEUR_HAUT]);
	        		MiApp_WriteData(nombre_angles[IDCAPTEUR_BAS]);

					MiApp_WriteData((periode_tour >> 8 ) & 0x00FF); // MSB
	        		MiApp_WriteData((periode_tour      ) & 0x00FF); // LSB

	        		for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
					{
						MiApp_WriteData(buffer_angles[IDCAPTEUR_HAUT][2*i]); 	// MSB
						MiApp_WriteData(buffer_angles[IDCAPTEUR_HAUT][2*i+1]);	// LSB
					}
					
					for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
					{
						MiApp_WriteData(buffer_angles[IDCAPTEUR_BAS][2*i]); 	// MSB
						MiApp_WriteData(buffer_angles[IDCAPTEUR_BAS][2*i+1]);	// LSB
					}
	        		
				   	MiApp_BroadcastPacket(FALSE);
					
				}

				else if(rxMessage.Payload[1] == 0xF0)
				{
					testConn  = 1;
				}
				else if(rxMessage.Payload[1] == 0xF1) // RESET
				{
					Reset();
				}
			}
            
            /*******************************************************************/
            // Function MiApp_DiscardMessage is used to release the current 
            //  received packet.
            // After calling this function, the stack can start to process the
            //  next received frame 
            /*******************************************************************/        
            MiApp_DiscardMessage();
        }
		else if(testConn)
		{
			testConn = 0;
			MiApp_FlushTx();
			MiApp_WriteData(IDBALISE);
			MiApp_WriteData(0XF0);			
			MiApp_BroadcastPacket(FALSE);
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
	
	if(motor==GAUCHE) value = -value;

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

void Initpwm(void)
{
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output 

	P1DC1 = 0xFFFF;
	// 0xFFFF =   0.00% Power
}

void __attribute__ ((interrupt, no_auto_psv)) _T2Interrupt(void) 
{
	IFS0bits.T2IF = 0;	
}

void __attribute__((__interrupt__)) _IC1Interrupt(void)
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
	if(watchdog++>30) Reset(); // protection ultime en cas de miwi + boucle principale bloquée
	Fin_de_tour=1;
}

void __attribute__((__interrupt__)) _IC2Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas]=IC2BUF;
	if(ptr_fronts_bas < 8) ptr_fronts_bas++;
	IC2CONbits.ICM = 0b001;
	IFS0bits.IC2IF=0;
}

void __attribute__((__interrupt__)) _IC7Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut]=IC7BUF;		
	if(ptr_fronts_haut < 8) ptr_fronts_haut++;
	IC7CONbits.ICM = 0b001;
	IFS1bits.IC7IF=0;
}
void InitT2(void)
{
	T2CONbits.TCKPS = 3;	// 1:256 Prescaler
	PR2 = 0xFFFF;			// Time to autoreload
	IFS0bits.T2IF = 0;		// Interrupt flag cleared
	IEC0bits.T2IE = 0;		// Interrupt disabled
	T2CONbits.TON = 1;		// Timer enabled
}

void calcul_angles(void)
{
	
}

