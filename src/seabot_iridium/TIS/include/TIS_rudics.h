/***********************************************************************//**
 * \file TIS_rudics.h
 * \brief Eléments permettant de réaliser une communication via RUDICS.
 *
 * La communication RUDICS utilise le protocole Zmodem, il faut donc que la librairie Zmodem soit présente pour que cette partie soit opérationnelle.
 * Si vous n'utilisez pas le mode RUDICS et que vous voulez diminuer la taille du code, vous pouvez utiliser l'option DISABLE_RUDICS (voir tis.h).
 *
 * Ces fonctions communiquent avec le modem par l'intermédiaire des commandes AT définis dans TIS_at.h.
 * 
 * \author Clément Bonnet
 * \date 2011-2012
 * \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
 ***********************************************************************/

#ifndef TIS_RUDICS_H
#define	TIS_RUDICS_H

#include "TIS_platforms.h"
#include "tis.h"

/***********************************************************************//**
 * \brief Effectue une session d'envoie et de réception RUDICS.
 *
 * Les fichiers reconstitués sont stockés dans le dossier "receive_folder".
 *
 * Si le nombre d'erreurs de connexions (et non de déconnexions) consécutives "maximum_consecutive_errors" est atteint, la fonction se termine et renvoie l'erreur TIS_ERROR_TOO_MANY_CONSECUTIVE_ERRORS.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] maximum_consecutive_errors Nombre d'erreurs consécutives maximum avant de quitter.
 *
 * \return Un code d'erreur.
 * \sa TIS_errors.h
 *
 * \todo Déplacer les fichiers reçues complétement du dossier temporaire au dossier de réception.
 ***********************************************************************/
 int32_t TIS_RUDICS_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors);
					  	    
/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour récupérer l'avancement du fichier courant.
 *
 * \param[in] not_used Pour compatibilité de prototype uniquement, ne pas utiliser.
 * \param[in] progress Position dans le fichier courant.
 ***********************************************************************/
void TIS_progress(void * not_used, int64_t progress);

/***********************************************************************//**
 * \brief Récupère la valeur de l'avancement du fichier courant fourni par Zmodem.
 *
 * \return not_used Position dans le fichier courant.
 ***********************************************************************/
int64_t TIS_RUDICS_get_current_file_progress();

/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour lui permettre d'envoyer un octet sur la liaison série.
 *
 * \param[in] properties Pointeur vers les propriétés de la liaison série.
 * \param[in] data La donnée à envoyer.
 * \param[in] timeout La durée du timeout.
 *
 * \return 0  en cas de succès.
 * \return -1 en cas d'erreur ou de timeout.
 ***********************************************************************/
int32_t TIS_send_byte(void * properties, uint8_t data, uint32_t timeout);

/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour lui permettre de recevoir un octet sur la liaison série.
 *
 * \param[in] properties Pointeur vers les propriétés de la liaison série.
 * \param[in] timeout La durée du timeout.
 *
 * \return La donnée en cas de succès.
 * \return -1 en cas d'erreur ou de timeout.
 ***********************************************************************/
int32_t TIS_receive_byte(void * properties, uint32_t timeout);

/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour lui permettre de vider le tampon d'émission de la liaison série.
 *
 * \param[in] properties Pointeur vers les propriétés de la liaison série.
 ***********************************************************************/
void TIS_flush_TX(void * properties);

/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour lui permettre d'attendre des données sur la liaison série.
 *
 * \param[in] properties Pointeur vers les propriétés de la liaison série.
 * \param[in] timeout La durée du timeout.
 *
 * \return TRUE  dès que des données sont disponibles.
 * \return FALSE si le timeout a été atteint et qu'aucune donnée n'est disponible.
 ***********************************************************************/
bool TIS_wait_data(void * properties, uint32_t timeout);

/***********************************************************************//**
 * \brief Callback fournie à la librairie Zmodem pour lui permettre de signaler la réception d'un fichier.
 * 
 * Cette fonction déplace le fichier reçue du dossier temporaire au dossier de réception.
 ***********************************************************************/
void TIS_file_received(void * properties, int8_t* name);					  	 

#endif //TIS_RUDICS_H