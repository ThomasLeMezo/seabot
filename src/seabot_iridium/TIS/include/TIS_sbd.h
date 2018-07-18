/***********************************************************************//**
 * \file TIS_sbd.h
 * \brief Eléments permettant de réaliser une communication via SBD.
 *
 * Ces fonctions communiquent avec le modem par l'intermédiaire des commandes AT définis dans TIS_at.h.
 * 
 * \author Clément Bonnet
 * \date 2011-2012
 * \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
 ***********************************************************************/

#ifndef TIS_SBD_H
#define	TIS_SBD_H

#include "TIS_platforms.h"
#include "tis.h"

/***********************************************************************//**
 * \def TIS_SBD_MESSAGE_HEADER_SIZE
 * \brief Taille de l'en tête d'un message SBD.
 ***********************************************************************/
#define TIS_SBD_MESSAGE_HEADER_SIZE 		6

/***********************************************************************//**
 * \def TIS_SBD_FILE_TYPE_SCIENTIFIC_DATA
 * \brief Bits indiquant que le fichier est un fichier de données scientifiques.
 ***********************************************************************/
#define TIS_SBD_FILE_TYPE_SCIENTIFIC_DATA	0x00 //0b00000000

/***********************************************************************//**
 * \def TIS_SBD_FILE_TYPE_TECHNICAL_DATA
 * \brief Bits indiquant que le fichier est un fichier de données techniques.
 ***********************************************************************/
#define TIS_SBD_FILE_TYPE_TECHNICAL_DATA	0x40 //0b01000000

/***********************************************************************//**
 * \def TIS_SBD_FILE_TYPE_COMMAND
 * \brief Bits indiquant que le fichier est un fichier de commandes.
 ***********************************************************************/
#define TIS_SBD_FILE_TYPE_COMMAND			0x80 //0b10000000

/***********************************************************************//**
 * \def TIS_SBD_FILE_TYPE_MASK
 * \brief Masque permettant de sélectionner les bits indiquant le type de fichiers.
 ***********************************************************************/
#define TIS_SBD_FILE_TYPE_MASK				0xc0 //0b11000000

/***********************************************************************//**
 * \def TIS_SBD_HEADER_END_MESSAGE
 * \brief Bit indiquant que le message est la fin du fichier.
 ***********************************************************************/
#define TIS_SBD_HEADER_END_MESSAGE			0x20 //0b00100000

/***********************************************************************//**
 * \def TIS_SBD_HEADER_NOT_END_MESSAGE
 * \brief Bit indiquant que le message n'est pas la fin du fichier.
 ***********************************************************************/
#define TIS_SBD_HEADER_NOT_END_MESSAGE		0x00 //0b00000000

/***********************************************************************//**
 * \def TIS_SBD_MESSAGE_NUMBER_MAX
 * \brief Nombre maximum de parties dans un fichier échangé par SBD.
 ***********************************************************************/
#define TIS_SBD_MESSAGE_NUMBER_MAX 8191

/***********************************************************************//**
 * \def TIS_SBD_SIZE_MAX
 * \brief Taille maximal d'un SBD.
 *
 * Cette ne correspond pas à un modem en particulier mais à ce qui se fait de mieux, car elle est utilisée pour allouer de la mémoire.
 ***********************************************************************/
#define TIS_SBD_SIZE_MAX 1960 //Cette valeur correspond à ce que l'on fait de mieux au début 2012, si de meilleur modems sortent, il faut l'augmenter
					  	   
/***********************************************************************//**
 * \brief Effectue une session d'envoie et de réception SBD.
 *
 * Les fichiers reconstitués sont stockés dans le dossier "receive_folder".
 *
 * Si le nombre d'erreurs consécutives "maximum_consecutive_errors" est atteint, la fonction se termine et renvoie l'erreur TIS_ERROR_TOO_MANY_CONSECUTIVE_ERRORS.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] maximum_consecutive_errors Nombre d'erreurs consécutives maximum avant de quitter.
 *
 * \return Un code d'erreur.
 * \sa TIS_errors.h
 ***********************************************************************/
int32_t TIS_SBD_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors);
					  	   
/***********************************************************************//**
 * \brief Stocke un message SBD entrant dans un fichier.
 *
 * Si le message ne correspond pas au début d'un fichier, il est stocké dans un fichier temporaire en attendant le reste du fichier.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] message Pointeur vers les données du message.
 * \param[in] message_count Taille du message.
 *
 * \return Un code d'erreur.
 * \sa TIS_errors.h
 ***********************************************************************/
int32_t TIS_SBD_store_received_message(TIS_properties * properties, uint8_t * message, int32_t message_count);

/***********************************************************************//**
 * \brief Concatène deux fichiers.
 *
 * Après la concaténation, la position courante du fichier est placé à la fin du fichier destination.
 *
 * \param[in, out] destination Le descripteur du fichier qui verra l'autre ajouter à sa fin.
 * \param[in, out] source Le descripteur du fichier qui sera ajouté à la fin de l'autre.
 *
 * \return Un code d'erreur.
 * \sa TIS_errors.h
 ***********************************************************************/
int32_t TIS_file_append(FILE * destination, FILE * source);

/***********************************************************************//**
 * \brief Retourne la taille d'un fichier.
 *
 * La position courante du fichier n'est pas modifié par cette fonction.
 *
 * \param[in] fp Le descripteur du fichier.
 *
 * \return La taille du fichier.
 * \return 0, si le pointeur est égal à NULL.
 ***********************************************************************/
int32_t TIS_filelength(FILE * fp);

#endif //TIS_SBD_H