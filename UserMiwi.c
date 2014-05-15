#include "UserMiwi.h"


UINT8 txPayload[TX_PAYLOAD_SIZE];

void InitMiwi(void)
{
	
	InitAckMiwi();		
	RadioHW_Init(TRUE);
    RadioInit();
    RadioSetChannel(MY_CHANNEL);
    RadioInitP2P();
}

void RadioInitP2P(void)
{
	Tx.frameControl.frameType = PACKET_TYPE_DATA;
	Tx.frameControl.securityEnabled = 0;
	Tx.frameControl.framePending = 0;
	Tx.frameControl.ackRequest 	 = 0;
	Tx.frameControl.panIDcomp 	 = 1;
	Tx.frameControl.dstAddrMode  = SHORT_ADDR_FIELD;
	Tx.frameControl.frameVersion = 0;
	Tx.frameControl.srcAddrMode  = NO_ADDR_FIELD;
	Tx.dstPANID = RadioStatus.MyPANID;
	Tx.dstAddr  = DEST_SHORT_ADDR;		
	Tx.payload  = txPayload;
	Tx.payloadLength = DEFAULT_PAYLOAD;
}


void Envoi(char destinataire, Trame trame)
{
	int iChar;
	
	for(iChar = 0; iChar < trame.nbChar; iChar++)
	{
		txPayload[iChar] = trame.message[iChar];
	}
	Tx.dstAddr = 0x00FF & destinataire;
	Tx.payloadLength = trame.nbChar + 1;
	txPayload[trame.nbChar] = CalculChecksum(trame, FALSE);
	RadioTXPacket();
}



void EnvoiDebug(char a, char b, char c, char d)
{	
	txPayload[0] = a;
	txPayload[1] = b;
	txPayload[2] = c;
	txPayload[3] = d;
	
	Tx.dstAddr = DEBUG_ADDRESS;
	Tx.payloadLength = 4;
	
	RadioTXPacket();
}

void EnvoiAck(char destinataire, Trame trame)
{
	int iChar;
	
	for(iChar = 0; iChar < trame.nbChar; iChar++)
	{
		txPayload[iChar] = trame.message[iChar];
	}
	txPayload[1] = MY_SHORT_ADDRESS;
	Tx.dstAddr = 0x00FF & destinataire;
	Tx.payloadLength = trame.nbChar+1;
	RadioTXPacket();
}

int CalculChecksum(Trame trame, char checksumPresent)
{
	unsigned char checksum = 0;
	unsigned char iChar;
	
	for(iChar = 0; iChar <= (trame.nbChar - 1 - checksumPresent); iChar++)
		checksum ^= trame.message[iChar];
		
	return checksum;
}
