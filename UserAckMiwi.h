#ifndef __USER_ACK_MIWI__
#define __USER_ACK_MIWI__

#include "UserMiwi.h"
#include "UserTrame.h"
#include "init.h"

#define TRUE  1
#define FALSE 0

#define UP 1
#define DOWN 0

#define BUFFER 0
#define BUFFER_A 0x20
#define BUFFER_B 0x21
#define ACK_BUFFER 0x02

#define BLOQUANT 1
#define BLOQUANT_A 0x10
#define BLOQUANT_B 0x11
#define ACK_BLOQUANT 0x12

#define TAILLE_BUFFER 10
#define TIMEOUT_VALUE 50
#define TAILLE_MAX_TRAME 50
#define LONGEUR_TRAME_ACK 2

#define TIMER_PR_VALUE 500

//Define canal
#define NB_CANAUX 1
#define CANAL_VIDE 0

//Trame
#define ADDR_TRAME_RX 1
#define ADDR_TRAME_TX 0

void MiwiTasks(void);

void InitTimer5(void);
void InitAckMiwi(void);
void InitAckBuffer(void);
void InitAckBloquant(void);

char VerifChecksum(Trame trame);
void ReceptionMiwi(char expediteur,Trame trame);
void ReceptionAck(char expediteur,char bloquant);
void EnvoiMiwi(char destinataire, char bloquant, Trame trame);

char MiwiIsAck(void);
Trame MiwiGetData(void);
char MiwiIsDataReady(void);

unsigned char CanalAttribution(unsigned char indentifiant);


#endif
