/*****************************************************************************
 *
 *              HardwareProfile.c -- Hardware Profile
 *
 *****************************************************************************
 * FileName:        HardwareProfile.c
 * Dependencies:
 * Processor:       PIC18, PIC24, PIC32, dsPIC30, dsPIC33
 * Compiler:        C18 02.20.00 or higher
 *                  C30 02.03.00 or higher
 *                  C32 01.00.02 or higher
 * Linker:          MPLINK 03.40.00 or higher
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
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
 *****************************************************************************
 * File Description:
 *
 *   This file provides configuration and basic hardware functionality 
 *   based on chosen hardware demo boards.
 *
 * Change History:
 *  Rev   Date         Description
 *  0.1   2/17/2009    Initial revision
 *  3.1   5/28/2010    MiWi DE 3.1
 *****************************************************************************/
#include "SystemProfile.h"
#include "Compiler.h"
#include "WirelessProtocols/Console.h"
#include "WirelessProtocols/LCDBlocking.h"
#include "TimeDelay.h"
#include "HardwareProfile.h"


    _FOSCSEL(FNOSC_PRI);                                    //primary osc
    _FOSC(OSCIOFNC_OFF & POSCMD_XT)                         // XT Osc
    _FWDT(FWDTEN_OFF & WDTPOST_PS2)                         // Disable Watchdog timer
	_FICD(JTAGEN_OFF & ICS_PGD1)
    // JTAG should be disabled as well

#define DEBOUNCE_TIME 0x00003FFF

BOOL PUSH_BUTTON_pressed;
MIWI_TICK PUSH_BUTTON_press_time;

/*********************************************************************
 * Function:        void BoardInit( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Board is initialized for P2P usage
 *
 * Overview:        This function configures the board 
 *
 * Note:            This routine needs to be called before the function 
 *                  to initialize P2P stack or any other function that
 *                  operates on the stack
 ********************************************************************/

void BoardInit(void)
{
    
    //TRISBbits.TRISB8 = 1;
	//TRISBbits.TRISB9 = 0;
	//TRISCbits.TRISC0 = 0;

	//RPOR4bits.RP9R     = 0b01011;	//SCK2			<==> RB9
	//RPINR22bits.SDI2R = 0b01000;	//SDI2			<==> RB8
	//RPOR8bits.RP16R   = 0b01010;	//SDO2			<==> RC0
        
    PHY_CS_TRIS = 0;
    PHY_CS = 1;
    PHY_RESETn_TRIS = 0;
    PHY_RESETn = 1;

    RF_INT_TRIS = 1;
	//RPINR0bits.INT1R = 0b00111; 	// INT1			<==> RP7   
		   
    SDI_TRIS = 1;
    SDO_TRIS = 0;
    SCK_TRIS = 0;
    SPI_SDO = 0;        
    SPI_SCK = 0;  
                   
    RPOR10bits.RP21R    = 0b01000;	// SCK1	<==> RC5 ; RP21
	RPINR20bits.SDI1R = 20; 		// SDI1	<==> RC4 ; RP20
	RPOR2bits.RP5R    = 0b00111;	// SDO 	<==> RB5 ; RP5
        
    PHY_WAKE_TRIS = 0;
    PHY_WAKE = 1;

    #if defined(HARDWARE_SPI)
      	SPI1CON1 = 0b0000000100111110;
        SPI1STAT = 0x8000;
 
        SPI2CON1 = 0b0000000100111110;
        SPI2STAT = 0x8000;
    #endif

    INTCON2bits.INT1EP = 1;

    #if defined(ENABLE_NVM)

         EE_nCS_TRIS = 0;
	     EE_nCS = 1;
        
    #endif

    RFIF = 0;
    if( RF_INT_PIN == 0 )
    {
        RFIF = 1;
    }
}
