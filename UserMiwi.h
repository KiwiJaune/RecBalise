#ifndef __USER_MIWI__
#define __USER_MIWI__

#include "MRF24J40.h"
#include "UserTrame.h"
#include "radiohardware.h"
#include "radioAddress.h"
#include "UserAckMiwi.h"

void Envoi(char destinataire, Trame trame);
int CalculChecksum(Trame trame, char checksumPresent);
void RadioInitP2P(void);
void Init_Timer5(void);
void InitMiwi(int address);
void EnvoiAck(char destinataire, int myAddress, Trame trame);
void EnvoiDebug(char a, char b, char c, char d);

#define	DEFAULT_PAYLOAD 30
#define MY_CHANNEL 23
#define DEST_SHORT_ADDR 0xC200

#endif
