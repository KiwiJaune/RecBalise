// (c) 2010-2012 nerdfever.com

// Address for IEEE 802.15.4 radio

#if !defined(_RADIOADDRESS_H_)
#define _RADIOADDRESS_H_

#include <p33FJ128MC804.h>

#define MY_LONG_ADDRESS 	(0xA00DEADBEEF0000ull)		// device MAC address (8 bytes, 64 bit)

//#define MY_SHORT_ADDRESS ((int)(0xB3-PORTBbits.RB3-2*PORTBbits.RB2))
//#define MY_SHORT_ADDRESS 0xB2

#define DEBUG_ADDRESS 0xE2

#define MY_PAN_ID			(0xA0A0)					// PAN identifier

#endif // _RADIOADDRESS_H_
