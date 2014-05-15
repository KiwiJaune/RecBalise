// (c) 2010-2012 nerdfever.com

// Address for IEEE 802.15.4 radio

#if !defined(_RADIOADDRESS_H_)
#define _RADIOADDRESS_H_

#include <p33FJ128MC804.h>

#define CARTE_BALISE_2

#define MY_LONG_ADDRESS 	(0xA00DEADBEEF0000ull)		// device MAC address (8 bytes, 64 bit)

//#define MY_SHORT_ADDRESS ((int)(0xB3-PORTBbits.RB3-2*PORTBbits.RB2))
//#define MY_SHORT_ADDRESS 0xB2

#ifdef CARTE_MIWI
#define MY_SHORT_ADDRESS	(0x00C2)					// short (2 byte) 802.15.4 address
#endif
#ifdef CARTE_MIWI_PI
#define MY_SHORT_ADDRESS	(0x00C3)					// short (2 byte) 802.15.4 address
#endif
#ifdef CARTE_BALISE_1
#define MY_SHORT_ADDRESS	(0x00B1)					// short (2 byte) 802.15.4 address
#endif
#ifdef CARTE_BALISE_2
#define MY_SHORT_ADDRESS	(0x00B2)					// short (2 byte) 802.15.4 address
#endif
#ifdef CARTE_BALISE_3
#define MY_SHORT_ADDRESS	(0x00B3)					// short (2 byte) 802.15.4 address
#endif

#define DEBUG_ADDRESS 0xE2

#define MY_PAN_ID			(0xA0A0)					// PAN identifier

#endif // _RADIOADDRESS_H_
