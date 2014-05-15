/*
	Hardware specific code for the MRF24J40 radio on a 2.4GHz
	RF Pictail board on an Explorer16 board.
*/

#include "radiohardware.h"

void	RadioHW_PowerOn()
// Add code here to power on the radio. It might be using an 
// external P-channel MOS transistor to power the radio or it
// might be using the built in wake functionality.
// If power is always going to be on (as here) - do everything in
// RadioHW_Init() and leave RadioHW_PowerOn() and RadioHW_PowerOff()
// empty.
{
	
}

void	RadioHW_PowerOff()
// Power always on in this democode. Otherwise put code here to power
// the radio down. Also disable interrupts and shut down the SPI port
// if hardware SPI is used.
{
	
}	

void	RadioHW_Init(BOOL powerOn)
// If called with powerOn==FALSE, RadioHW_PowerOn() must be called 
// explicitly before the radio is initiated and used.
{

    RADIO_CS_TRIS = 0;
    RADIO_CS = 1;

    RADIO_RESETn_TRIS = 0;
    RADIO_RESETn = 1;

    RADIO_INT_TRIS = 1;
    RADIO_SDI_TRIS = 1;
    RADIO_SDO_TRIS = 0;
    RADIO_SCK_TRIS = 0;

    RADIO_SPI_SDO = 0;        
    RADIO_SPI_SCK = 0;

    RADIO_WAKE_TRIS = 0;
    RADIO_WAKE = 1;

    // if a PIC with Peripheral Pin Select is used - Set up
    // Peripheral Pin Select registers here.
    RPINR0bits.INT1R = 0b0110;         //RP9

    AD1PCFGL=0xFFFF;

    RPOR10bits.RP21R    = 0b01000;	// SCK1	<==> RC5 ; RP21
	RPINR20bits.SDI1R = 20; 		// SDI1	<==> RC4 ; RP20
	RPOR2bits.RP5R    = 0b00111;	// SDO 	<==> RB5 ; RP5
/*
    RPOR2bits.RP5R    = 0b01011;        //SCK2
    RPOR4bits.RP8R    = 0b01010;        //SDO2
    RPINR22bits.SDI2R = 0b00110;        //SDI2*/

    #if defined(HARDWARE_SPI)
        // See radiohardware.h on how to configure SPICON1_VAL
    	SPICON1 = SPICON1_VAL;
    	SPISTAT = SPISTAT_VAL;
        
    #endif
    
    // Always powered on so we don't need to test powerOn here.
}
