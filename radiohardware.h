// (c) 2010-2012 nerdfever.com
// Modified and adapted to PIC24 by Ruben Jönsson 2012
// This file configures the radio hardware (2.4 GHz RF pictail board) 
// for an Explorer 16 development board.

// Change inside /*** Config start ... /*** Config end to match
// the hardware connection between the PIC and the MRF24J40

#ifndef _RADIOHARDWARE_H
#define _RADIOHARDWARE_H

#include "Compiler.h"
#include "GenericTypeDefs.h"


/*** Config start ***************************************************************************************/
// Set hardware interface pins to use.
// If a PIC with Peripheral Pin Select is used, also make sure to set up
// Peripheral Pin Select registers to the correct pins (Interrupt pin and SPI pins if HW SPI is uses).
// This setting is for the Explorer 16 development board

#define RADIO_INT_PIN      	PORTBbits.RB6
#define RADIO_INT_TRIS     	TRISBbits.TRISB6

#define RADIO_CS              	LATAbits.LATA4
#define RADIO_CS_TRIS         	TRISAbits.TRISA4
#define RADIO_RESETn          	LATCbits.LATC6
#define RADIO_RESETn_TRIS     	TRISCbits.TRISC6
#define RADIO_WAKE       		LATBbits.LATB7
#define RADIO_WAKE_TRIS   		TRISBbits.TRISB7

#define RADIO_SPI_SDI          	PORTCbits.RC4
#define RADIO_SDI_TRIS         	TRISCbits.TRISC4
#define RADIO_SPI_SDO          	LATBbits.LATB5
#define RADIO_SDO_TRIS         	TRISBbits.TRISB5
#define RADIO_SPI_SCK          	LATCbits.LATC5
#define RADIO_SCK_TRIS         	TRISCbits.TRISC5

// Set which external interrupt to use
#define RADIO_INT				1				// External interrupt input 1

// Hardware or software SPI.
// Comment out HARDWARE_SPI to use software bit-banged SPI
// If hardware SPI is used, also set USE_SPI to correct channel. SPICON1 and SPISTAT must also 
// be set to SPICON1_VAL and SPISTAT_VAL before radio is used.
// If using a PIC with reconfigurable I/O (Peripheral Pin Select) also make sure
// to set up Peripheral Pin Select Registers to the correct pin.

#define HARDWARE_SPI								// vs. software bit-bang (slower)
#define USE_SPI		1							// Use SPI channel 2
#define	SPI_CLKFREQ	(0x1e)							// Lowest 4 bits of SPICON1 that sets SPI clock speed
                                                                                // 0x1e = FCY/4
/*** Config end ************************************************************************************************/	

#if (RADIO_INT==0)
	#define RADIO_IF         		IFS0bits.INT0IF
	#define RADIO_IE         		IEC0bits.INT0IE
	#define RADIO_INTPOL			INTCON2bits.INT0EP
#elif (RADIO_INT==1)
	#define RADIO_IF         		IFS1bits.INT1IF
	#define RADIO_IE         		IEC1bits.INT1IE
	#define RADIO_INTPOL			INTCON2bits.INT1EP
#elif (RADIO_INT==2)
	#define RADIO_IF         		IFS1bits.INT2IF
	#define RADIO_IE         		IEC1bits.INT2IE
	#define RADIO_INTPOL			INTCON2bits.INT2EP
#else
	#error "Illegal value for RADIO_INT. Must be 0, 1 or 2."
#endif

#ifdef HARDWARE_SPI
	#if (USE_SPI==1)
		#define	SPICON1		SPI1CON1	
		#define SPISTAT		SPI1STAT
		#define	SPIBUF		SPI1BUF
		#define SPI_IF		IFS0bits.SPI1IF
	#elif (USE_SPI==2)
		#define	SPICON1		SPI2CON1	
		#define SPISTAT		SPI2STAT
		#define SPIBUF		SPI2BUF
		#define	SPI_IF		IFS2bits.SPI2IF
	#else
		#error "Illegal value for USE_SPI. Must be 1 or 2 (=spi channel to use)"
	#endif
	// SPI settings (minus clock speed) for use with 2.4GHz RF pictail board.
	#define SPICON1_VAL		((0b0000000100100000) | (SPI_CLKFREQ & 0x1F))
	#define SPISTAT_VAL		(0x8000)
#endif

void	RadioHW_Init(BOOL powerOn);
void	RadioHW_PowerOff();
void	RadioHW_PowerOn();

#endif	// _RADIOHARDWARE_H
