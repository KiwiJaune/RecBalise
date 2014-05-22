/***********************
Bloquant
10 | Trame | CRC
11...

Buffer
00 | Trame | CRC

ACK Bloquant
12

ACK Buffer
02
*************************/
#include "UserAckMiwi.h"

//Indice du message bloquant 10 ou 11 pour l'envoi
unsigned char indiceBloquantAlt[NB_CANAUX];

//Indice du message buffer 10 ou 11 pour l'envoi
unsigned char indiceBufferAlt[NB_CANAUX];

//Indice du message bloquant 10 ou 11 pour la reception
unsigned char previousIndiceBloquantAltRecu[NB_CANAUX];

//Indice du message buffer 10 ou 11 pour la reception
unsigned char previousIndiceBufferAltRecu[NB_CANAUX];

//Flag de renvoi messages non acquite
char flagEnvoiBuffer[NB_CANAUX];
char flagEnvoiBloquant[NB_CANAUX];

//Flag indique que les donnees sont pretes pour etre traite
char flagDataIsReady = DOWN;

//Flag ack bloquant recu
char flagAckBloquantRecu[NB_CANAUX];

//Compteur de ms utilise pour dater les messages
int now = 100;
//Compteur de seconde dans le timer du ms
int periode = 0;

// Trames a envoyer
Trame trameBloquant[NB_CANAUX][TAILLE_BUFFER];
// Date de tentative d'envoi
int dateEnvoiBloquant[NB_CANAUX];
// Prochain indice libre pour ajouter une trame dans la liste
unsigned char indiceLibreBloquant[NB_CANAUX];
// Indice du message en cours de tentative de transmission
unsigned char indiceEnvoiBloquant[NB_CANAUX];
// Destinataire message bloquant
unsigned char destinataireBloquant[NB_CANAUX][TAILLE_BUFFER];

// Trame tampon a envoyer
Trame trameBuffer[NB_CANAUX];
static BYTE messageBuffer[NB_CANAUX][TAILLE_MAX_TRAME];

// Date de tentative d'envoi
int dateEnvoiBuffer[NB_CANAUX];
// Destinataire message buffer(A enlever)
unsigned char destinataireBuffer[NB_CANAUX];

//Buffer/bloquant qui stock les messages non acquite
Trame trameAckBuffer;
BYTE messageAckBuffer[LONGEUR_TRAME_ACK];

Trame trameAckBloquant;
BYTE messageAckBloquant[1];

static BYTE messages[NB_CANAUX][TAILLE_BUFFER][TAILLE_MAX_TRAME];

//Trame a traiter par l'utilisateur
Trame trameATraiter;
static BYTE messageATraiter[TAILLE_MAX_TRAME];

//Gestion canal
static BYTE canaux[NB_CANAUX];

void InitAckMiwi(void)
{
	int iCanal = 0;
	int iBuffer = 0;
	InitAckBuffer();
	InitAckBloquant();
	InitTimer5();
	
	trameATraiter.message = messageATraiter;		
	
	for(iCanal = 0; iCanal < NB_CANAUX ; iCanal++)
	{
		canaux[iCanal] = 0;
		
		trameBuffer[iCanal].message = messageBuffer[iCanal];
		
		for(iBuffer = 0; iBuffer < TAILLE_BUFFER; iBuffer++)
			trameBloquant[iCanal][iBuffer].message = messages[iCanal][iBuffer];	
			
		indiceBloquantAlt[iCanal] = BLOQUANT_A;		
		previousIndiceBloquantAltRecu[iCanal] = BLOQUANT_A;
		
		flagEnvoiBuffer[iCanal] = DOWN;
		flagEnvoiBloquant[iCanal] = DOWN;
		
		flagAckBloquantRecu[iCanal] = DOWN;
	}
}
	
void InitAckBuffer(void){

	//Trame Ack bloquant
	trameAckBuffer.nbChar = 1;
	trameAckBuffer.message = messageAckBuffer;
	trameAckBuffer.message[0] = ACK_BUFFER;
}

void InitAckBloquant(void){
	
	//Trame Ack bloquant 
	trameAckBloquant.nbChar = 1;
	trameAckBloquant.message = messageAckBloquant;
	trameAckBloquant.message[0]= ACK_BLOQUANT;
}
//Init timer 5, ticks every 1ms
void InitTimer5(void)
{
	//--Timer5
	T5CONbits.TON 	= 1;	//Start the timer
	T5CONbits.TSIDL = 0;
	T5CONbits.TGATE = 0;
	T5CONbits.TCS	= 0;
	T5CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR5 = 0; 				//Clear timer register
	PR5  = TIMER_PR_VALUE;  //Load the period value (Pas) 1/(4e6/8/500) = 1ms

	IPC7bits.T5IP = 7; 		//Set Timer5 Interrupt Priority Level
	IFS1bits.T5IF = 0; 		//Clear Timer5 Interrupt Flag
	IEC1bits.T5IE = 1; 		//Enable Timer5 interrupt
}

//Function to call in Main
void EnvoiMiwi(char destinataire, char bloquant, Trame trame)
{
	int i = 0;
	unsigned char indiceCanal;
	
	indiceCanal = CanalAttribution(destinataire);
	
	if(!bloquant)
	{
		//Alternance des indices messages bloquant
		if(indiceBufferAlt[indiceCanal] == BUFFER_A)
			indiceBufferAlt[indiceCanal] = BUFFER_B;
		else indiceBufferAlt[indiceCanal] = BUFFER_A;
		
		//Copie de la trame dans trameBuffer
		for(i=0;i<trame.nbChar;i++)
		{
			trameBuffer[indiceCanal].message[i+2] = trame.message[i];
		}
		trameBuffer[indiceCanal].message[0] = indiceBufferAlt[indiceCanal];
		trameBuffer[indiceCanal].message[1] = MY_SHORT_ADDRESS;
		trameBuffer[indiceCanal].nbChar = trame.nbChar + 2;
		
		destinataireBuffer[indiceCanal] = destinataire;
		dateEnvoiBuffer[indiceCanal] = now;
		Envoi(destinataireBuffer[indiceCanal], trameBuffer[indiceCanal]);
	}
	else
	{
		//Alternance des indices messages bloquant
		if(indiceBloquantAlt[indiceCanal] == BLOQUANT_A)
			indiceBloquantAlt[indiceCanal] = BLOQUANT_B;
		else indiceBloquantAlt[indiceCanal] = BLOQUANT_A;
		
		
		//Copie de la trame dans trameBloquant
		for(i = 0; i < trame.nbChar; i++)
		{
			trameBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]].message[i+2] = trame.message[i];
		}
		trameBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]].message[0] = indiceBloquantAlt[indiceCanal];
		trameBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]].message[1] = MY_SHORT_ADDRESS;
		trameBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]].nbChar = trame.nbChar + 2;
		
		destinataireBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]] = destinataire;
		
		if(indiceLibreBloquant[indiceCanal] == indiceEnvoiBloquant[indiceCanal])
		{
			dateEnvoiBloquant[indiceCanal] = now;
			Envoi(destinataireBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]], trameBloquant[indiceCanal][indiceLibreBloquant[indiceCanal]]);
		}	
		
		indiceLibreBloquant[indiceCanal]++;
		if(indiceLibreBloquant[indiceCanal] == TAILLE_BUFFER - 1)
			indiceLibreBloquant[indiceCanal] = 0;
	}
}
unsigned char CanalAttribution(unsigned char identifiant)
{
	int iCanal = 0;
	
	for(iCanal = 0 ; iCanal < NB_CANAUX ; iCanal++)
	{
		if(canaux[iCanal] == identifiant)
		{
			return iCanal;
		}
		else if(canaux[iCanal] == CANAL_VIDE)
		{
			canaux[iCanal] = identifiant;
			return iCanal;
		}
	}
	return 0;
}

char MiwiIsAck(void)
{
	int iCanal = 0;
	
	for(iCanal = 0 ; iCanal < NB_CANAUX ; iCanal++)
	{
		if(flagAckBloquantRecu[iCanal] != 0)
		{
			flagAckBloquantRecu[iCanal] = DOWN;
			return canaux[iCanal];
		}
	}
	return 0;
}

void ReceptionAck(char expediteur,char bloquant)
{
	unsigned char indiceCanal = CanalAttribution(expediteur);	
	
	if(bloquant)
	{	
		indiceEnvoiBloquant[indiceCanal]++;
		if(indiceEnvoiBloquant[indiceCanal] == TAILLE_BUFFER - 1)
			indiceEnvoiBloquant[indiceCanal] = 0;
			
		if(indiceEnvoiBloquant[indiceCanal] != indiceLibreBloquant[indiceCanal])
		{
			Envoi(expediteur,trameBloquant[indiceCanal][indiceEnvoiBloquant[indiceCanal]]);
			dateEnvoiBloquant[indiceCanal] = now;
		}
		else
		{
			dateEnvoiBloquant[indiceCanal] = 0;
		}
		
		flagAckBloquantRecu[indiceCanal] = expediteur;
	}
	else
	{
		dateEnvoiBuffer[indiceCanal] = 0;
	}
}

void ReceptionMiwi(char expediteur, Trame trame)
{
	int i;
	unsigned char indiceCanal = 0;
	unsigned char indiceBloquantAltRecu;
	unsigned char indiceBufferAltRecu;
		
	indiceCanal = CanalAttribution(expediteur);	
	
	if(trame.nbChar > LONGEUR_TRAME_ACK && !VerifChecksum(trame))
		return;
		
	if(trame.message[0] == BLOQUANT_A || trame.message[0] == BLOQUANT_B)
	{	
		LED = !ETAT_LED;

		EnvoiAck(expediteur, trameAckBloquant);
		indiceBloquantAltRecu = trame.message[0];
		
		if(indiceBloquantAltRecu != previousIndiceBloquantAltRecu[indiceCanal])	
		{				
			trame.message = &trame.message[2];
			trame.nbChar -= 3;
			
			trameATraiter.nbChar = trame.nbChar;			
			for(i = 0; i < trame.nbChar; i++)
			{ 
				trameATraiter.message[i] = trame.message[i];
			}
			
			flagDataIsReady = UP;	
		}
		previousIndiceBloquantAltRecu[indiceCanal] = indiceBloquantAltRecu;
	}
	else if (trame.message[0] == BUFFER_A || trame.message[0] == BUFFER_B)
	{
		LED = !ETAT_LED;

		EnvoiAck(expediteur, trameAckBuffer);
		indiceBufferAltRecu = trame.message[0];
		
		if(indiceBufferAltRecu != previousIndiceBufferAltRecu[indiceCanal])	
		{	
			trame.message = &trame.message[2];
			trame.nbChar -= 3;
			
			trameATraiter.nbChar = trame.nbChar;		
			for(i = 0; i < trame.nbChar; i++)
			{ 
				trameATraiter.message[i] = trame.message[i];
			}
			
			flagDataIsReady = UP;
		}
		previousIndiceBufferAltRecu[indiceCanal] = indiceBufferAltRecu;
	}
	else if (trame.message[0] == ACK_BLOQUANT)
		ReceptionAck(expediteur,TRUE);
		
	else if (trame.message[0] == ACK_BUFFER)
		ReceptionAck(expediteur,FALSE);
}

char VerifChecksum(Trame trame)
{
	return (CalculChecksum(trame, TRUE) == trame.message[trame.nbChar - 1]);
}

Trame MiwiGetData(void)
{
	flagDataIsReady = DOWN;
	return trameATraiter;
}


char MiwiIsDataReady(void)
{
	return 	flagDataIsReady;
}

//Ticks every 1ms
void __attribute__((__interrupt__,__auto_psv__)) _T5Interrupt(void){
	
	int iCanal = 0;
	//Clear flag
	IFS1bits.T5IF  = 0;
	
	//Date d'envoi Miwi	
	now++;

	if(now%500 == 0)
		LED = !ETAT_LED;
	
	for(iCanal = 0; iCanal < NB_CANAUX; iCanal++)
	{
		if(canaux[iCanal] == 0)
			return;
		
		if(dateEnvoiBuffer[iCanal] != 0 && dateEnvoiBuffer[iCanal] < now - TIMEOUT_VALUE)
			flagEnvoiBuffer[iCanal] = UP;
			
		if(dateEnvoiBloquant[iCanal] != 0 && dateEnvoiBloquant[iCanal] < now - TIMEOUT_VALUE)
			flagEnvoiBloquant[iCanal] = UP;
	}
}

void MiwiTasks(void)
{
	unsigned char RxExpediteur = 0;
	unsigned char i = 0;
	int iCanal = 0;
	
	Trame trameRecu;
	static BYTE messRecu[50];
	trameRecu.message = messRecu;

	//Check if any msg have been received
	if(RadioRXPacket())
	{			
		trameRecu.nbChar = Rx.payloadLength;		
		//Copy received miwi message	
		for(i = 0; i< Rx.payloadLength ; i++)
		{
			trameRecu.message[i] = Rx.payload[i];
		}
		RxExpediteur = Rx.payload[1];
		ReceptionMiwi(RxExpediteur,trameRecu);
		RadioDiscardPacket();
	}
	
	for(iCanal = 0; iCanal < NB_CANAUX; iCanal++)
	{
		if(canaux[iCanal] == 0)
			return;
			
		//Send Ack for remaining msg
		if(flagEnvoiBuffer[iCanal])
		{
			Envoi(destinataireBuffer[iCanal], trameBuffer[iCanal]);
			dateEnvoiBuffer[iCanal] = now;
			flagEnvoiBuffer[iCanal] = DOWN;
		}
		if(flagEnvoiBloquant[iCanal])
		{
			Envoi(destinataireBloquant[iCanal][indiceEnvoiBloquant[iCanal]], trameBloquant[iCanal][indiceEnvoiBloquant[iCanal]]);
			dateEnvoiBloquant[iCanal] = now;
			flagEnvoiBloquant[iCanal] = DOWN;
		}		
	}
}
