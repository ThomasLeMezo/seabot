/***********************************************************************//**
 * \file  TIS_at.h
 * \brief Encapsulation des commandes AT des modems Iridium.
 *
 * \author Clément Bonnet
 * \date 2011-2012
 * \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
 ***********************************************************************/

#ifndef TIS_AT_H
#define	TIS_AT_H

#include "TIS_platforms.h"
#include "tis.h"

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_0
 * \brief Force du signal Iridium : pas de signal.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_0 0

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_1
 * \brief Force du signal Iridium : réception médiocre.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_1 1

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_2
 * \brief Force du signal Iridium : réception faible.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_2 2

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_3
 * \brief Force du signal Iridium : réception moyenne.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_3 3

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_4
 * \brief Force du signal Iridium : bonne réception.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_4 4

/***********************************************************************//**
 * \def TIS_SIGNAL_STRENGTH_5
 * \brief Force du signal Iridium : réception parfaite.
 ***********************************************************************/
#define TIS_SIGNAL_STRENGTH_5 5

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_AUTOBAUDING
 * \brief Vitesse de communication entre le modem et le satellite : automatique.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_AUTOBAUDING		"00"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_300_BPS_V21
 * \brief Vitesse de communication entre le modem et le satellite : 300 bps V.21.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_300_BPS_V21		"01"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_1200_BPS_V22
 * \brief Vitesse de communication entre le modem et le satellite : 1200 bps V.22.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_1200_BPS_V22		"02"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_2400_BPS_V22BIS
 * \brief Vitesse de communication entre le modem et le satellite : 2400 bps V.22bis.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_2400_BPS_V22BIS	"04"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_4800_BPS_V32
 * \brief Vitesse de communication entre le modem et le satellite : 4800 bps V.32.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_4800_BPS_V32		"06"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_9600_BPS_V32
 * \brief Vitesse de communication entre le modem et le satellite : 9600 bps V.32 (valeur par défaut).
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_9600_BPS_V32		"07"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_300_BPS_V110
 * \brief Vitesse de communication entre le modem et le satellite : 300 bps V.110.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_300_BPS_V110		"65"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_1200_BPS_V110
 * \brief Vitesse de communication entre le modem et le satellite : 1200 bps V.110.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_1200_BPS_V110		"66"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_2400_BPS_V110
 * \brief Vitesse de communication entre le modem et le satellite : 2400 bps V.110.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_2400_BPS_V110		"68"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_4800_BPS_V110
 * \brief Vitesse de communication entre le modem et le satellite : 4800 bps V.110.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_4800_BPS_V110		"70"

/***********************************************************************//**
 * \def TIS_AT_CBST_SPEED_9600_BPS_V110
 * \brief Vitesse de communication entre le modem et le satellite : 9600 bps V.110.
 *
 * Vous trouverez plus de détails dans la documentation des modems.
 ***********************************************************************/
#define TIS_AT_CBST_SPEED_9600_BPS_V110		"71"


/***********************************************************************//**
 * \def TIS_AT_SBDI_MT_QUEUED_EMPTY
 * \brief Pas de message SBD sortant.
 ***********************************************************************/
#define TIS_AT_SBDI_MT_QUEUED_EMPTY		0

/***********************************************************************//**
 * \def TIS_AT_SBDI_MT_STATUS_SUCCESS
 * \brief Message SBD reçue avec succès.
 ***********************************************************************/
#define TIS_AT_SBDI_MT_STATUS_SUCCESS	1

/***********************************************************************//**
 * \def TIS_AT_SBDI_MT_STATUS_ERROR
 * \brief Une erreur s'est produite lors de l'envoi du message SBD.
 ***********************************************************************/
#define TIS_AT_SBDI_MT_STATUS_ERROR		2

/***********************************************************************//**
 * \def TIS_AT_SBDI_MO_STATUS_EMPTY
 * \brief Pas de message SBD entrant.
 ***********************************************************************/
#define TIS_AT_SBDI_MO_STATUS_EMPTY		0

/***********************************************************************//**
 * \def TIS_AT_SBDI_MO_STATUS_SUCCESS
 * \brief Message SBD envoyé avec succès.
 ***********************************************************************/
#define TIS_AT_SBDI_MO_STATUS_SUCCESS	1

/***********************************************************************//**
 * \def TIS_AT_SBDI_MO_STATUS_ERROR
 * \brief Une erreur s'est produite lors de la réception du message SBD.
 ***********************************************************************/
#define TIS_AT_SBDI_MO_STATUS_ERROR		2

/***********************************************************************//**
 * \brief Envoie la commande AT+SBDD au modem.
 *
 * La commande AT+SBDD (Short Burst Data: Clear SBD Message Buffer(s)) permet de vider les tampons SBD du modem.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[in] MO_buffer Si TRUE, vide le tampon des messages sortants. Si false, ne vide pas le tampon de messages sortant 
 * \param[in] MT_buffer Si TRUE, vide le tampon des messages entrants. Si false, ne vide pas le tampon de messages entrant.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_SBDD(TIS_properties * properties, bool MO_buffer, bool MT_buffer);

/***********************************************************************//**
 * \brief Envoie la commande AT+SBDI au modem.
 *
 * La commande AT+SBDD (Short Burst Data: Initiate an SBD Session) permet de lancer une session SBD.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[out] MO_status Pointeur vers la variable où sera écrite le statut de l'envoie du message sortant.
 * \param[out] MT_status Pointeur vers la variable où sera écrite le statut de réception du message entrant.
 * \param[out] MT_queued  Pointeur vers la variable où sera écrite le nombre de message entrant en attente dans le satellite.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_SBDI(TIS_properties * properties, int32_t * MO_status, int32_t * MT_status, int32_t * MT_queued);

/***********************************************************************//**
 * \brief Envoie la commande AT+SBDRB au modem.
 *
 * La commande AT+SBDRB (Short Burst Data: Read Binary Data from ISU) permet de récupérer le message entrant contenue dans le tampon du modem.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[out] message Un pointeur vers le tampon où sera écrit le message. Ce tampon doit être de taille TIS_SBD_SIZE_MAX octets.
 * \param[out] MT_length Pointeur vers la variable où sera écrite la taille du message entrant.
 *
 * \return le nombre d'octet écrits dans message.
 *
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_SBDRB(TIS_properties * properties, uint8_t * message, int32_t * MT_length);

/***********************************************************************//**
 * \brief Envoie la commande AT+SBDWB au modem.
 *
 * La commande AT+SBDWB (Short Burst Data: Write Binary Data to the ISU) permet d'envoyer un message sortant vers le modem.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[in] message Un pointeur vers le tampon où est stocké le message. ce tampon doit être de taille TIS_MODEM_SBD_SIZE_MAX octets ou inférieur.
 * \param[in] count Le nombre d'octets à envoyer.
 *
 * \return le nombre d'octets envoyés avec succès.
 *
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_SBDWB(TIS_properties * properties, uint8_t * message, int32_t count);

/***********************************************************************//**
 * \brief Envoie la commande ATD au modem.
 *
 * La commande ATD permet d'établir une communication téléphonique (RUDICS).
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[in] number Le numéro à appeller.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_D(TIS_properties * properties, uint64_t number);

/***********************************************************************//**
 * \brief Envoie la commande AT+CPIN au modem.
 *
 * La commande AT+CPIN permet d'envoyer le code PIN de la carte SIM. Cette fonction ne retourne pas une erreur si le code PIN est incorrect.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[in] pin Le pointeur vers la chaine de caractère contenant le code PIN.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_CPIN(TIS_properties * properties, uint8_t * pin);

/***********************************************************************//**
 * \brief Envoie la commande AT+CREG? au modem.
 *
 * La commande AT+CREG? permet de connaitre l'état d'enregistrement du modem auprès de la constellation et sa configuration.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_CREG(TIS_properties * properties);

/***********************************************************************//**
 * \brief Envoie la commande AT+CSQ au modem.
 *
 * La commande +CSQ permet de connaitre la puissance du signal du satellite.
 *
 * \param[in] properties Pointeur vers la structure de type TIS_properties contenant les propriétés de la librairie.
 * \param[out] strength Pointeur vers la variable où sera écrite la puissance du signal.
 *
 * \return Un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_AT_CSQ(TIS_properties * properties, int32_t * strength);

#endif //TIS_AT_H
