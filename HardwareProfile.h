/********************************************************************
* FileName:		HardwareProfile.h
* Dependencies:    
* Processor:	PIC18, PIC24, PIC32, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher	
*               Microchip C32 v1.02 or higher	
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
*
* Copyright © 2007-2010 Microchip Technology Inc.  All rights reserved.
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
*********************************************************************
* File Description:
*
*  This file defines functions used for demo board hardware
*
* Change History:
*  Rev   Date         Author    Description
*  1.0   2/15/2009    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
********************************************************************/

#ifndef _HARDWARE_PROFILE_H
    #define _HARDWARE_PROFILE_H
    
    #include "GenericTypeDefs.h"
    #include "ConfigApp.h"
    #include <p33FJ128MC804.h>
    
    // there are three ways to use NVM to store data: External EPROM, Data EEPROM and 
    // programming space, with following definitions:
    //  #define USE_EXTERNAL_EEPROM
    //  #define USE_DATA_EEPROM
    //  #define USE_PROGRAMMING_SPACE  
    // Each demo board has defined the method of using NVM, as
    // required by Network Freezer feature.
    
    #define RFIF            IFS1bits.INT1IF
    #define RFIE            IEC1bits.INT1IE

    #define CLOCK_FREQ      8000000
          
    #define RF_INT_PIN      PORTBbits.RB7
    #define RF_INT_TRIS     TRISBbits.TRISB7
      
    #define USE_EXTERNAL_EEPROM
           
    // Transceiver Configuration

	#define PHY_CS              LATAbits.LATA9 
	#define PHY_CS_TRIS         TRISAbits.TRISA9 
	#define PHY_RESETn          LATAbits.LATA4
	#define PHY_RESETn_TRIS     TRISAbits.TRISA4

	#define SPI_SDI             PORTBbits.RB4
	#define SDI_TRIS            TRISBbits.TRISB4
	#define SPI_SDO             LATBbits.LATB2 
	#define SDO_TRIS            TRISBbits.TRISB2
	#define SPI_SCK             LATBbits.LATB3 
	#define SCK_TRIS            TRISBbits.TRISB3
	
	#define PHY_WAKE        	LATAbits.LATA1
	#define PHY_WAKE_TRIS   	TRISAbits.TRISA1
	
	#define PUSH_BUTTON_1       PORTBbits.RB14
	#define PUSH_BUTTON_2       PORTBbits.RB15
	#define LED_1               LATBbits.LATB13
	#define LED_2               LATBbits.LATB12

	#define LED_VERTE			LED_1
	#define LED_ROUGE			LED_2
	#define LED_ON				0
	#define LED_OFF				1
	
	#define BUTTON_1_TRIS       TRISBbits.TRISB14
	#define BUTTON_2_TRIS       TRISBbits.TRISB15
	#define LED_1_TRIS          TRISBbits.TRISB13
	#define LED_2_TRIS          TRISBbits.TRISB12
	
	// Define SUPPORT_TWO_SPI if external EEPROM use the second SPI
	// port alone, not sharing SPI port with the transceiver
	#define SUPPORT_TWO_SPI
	
	// External EEPROM SPI chip select pin definition
	#define EE_nCS_TRIS         TRISBbits.TRISB1
	#define EE_nCS              LATBbits.LATB1
	
	#define TMRL TMR2

    #define GetInstructionClock()	(CLOCK_FREQ/2)
   
    void BoardInit(void);

#endif

