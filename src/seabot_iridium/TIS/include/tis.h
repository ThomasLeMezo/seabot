/***********************************************************************//**
* \mainpage Documentation de Transmission Iridium Simplifiée - Partie Instrument
* Cette documentation présente l'usage de la librairie TIS.
* Ci-dessous vous trouverez un exemple d'utilisation de la librairie pour réaliser des échanges en SBD. Certaines fonctionnalités dépendent
* de la plateforme utilisée et non de la librairie, des fonctions factices sont donc utilisées dans cet exemple.
* \code
* #include "tis.h"
*
* //Cete fonction prend en paramètre un tableau contenant des pointeurs vers le nom des fichiers à envoyer ainsi que le nombre de fichier à renvoyer.
* bool Iridium_send_and_receive_data(char ** files, int files_count) {
*	TIS_properties tis; //Déclaration de la structure de la librairie
*	UART_struct serial; //Structure contenant la configuration de la liaison série utilisée par le modem (dépend de votre plateforme et non de la librairie)
*   int i = 0;
*
*	//Initialise la liaison série (dépend de votre plateforme et non de la librairie)
*	UART_init(&serial);
*
*	//Initialise la structure de la librairie
*	if (TIS_init(&tis,
*				 "CMD",				//"CMD" est le chemin du dossier qui contiendra les fichier reçus.
*	 	 		 TIS_MODEM_9602,	//Modèle du modem
*	 	 		 310036010122330,	//Numéro IMEI du modem
*	 	 		 TIS_SERVICE_SBD,	//Service SBD
*	 	 		 NULL,				//Inutile avec un modem sans carte SIM
*	 	 		 0,					//Inutile en modem SBD
*	 	 		 NULL,				//Inutile en modem SBD
*	 	 		 files_count,		//Nombre de fichier envoyés
*	 	 		 &serial,			//Un pointeur vers la structure décrivant la liaison série
*	 	 		 UART_send_data,	//Fonction utilisant les appelles système de la plateforme pour envoyer des données sur la liaison série
*			 	 UART_receive_data, //Fonction utilisant les appelles système de la plateforme pour recevoir des données sur la liaison série
*			 	 UART_wait_data,	//Fonction utilisant les appelles système de la plateforme pour attendre des données sur la liaison série
*			 	 UART_flush_TX,		//Fonction utilisant les appelles système de la plateforme pour vider le tampon de sortie de la liaison série
*			 	 UART_flush_RX		//Fonction utilisant les appelles système de la plateforme pour vider le tampon d'entrée de la liaison série
*		) != TIS_ERROR_SUCCESS) {
*		return FALSE;
*	}
*
*	//Ajoute les fichiers à envoyer
* 	for (i = 0; i < files_count; i++) {
*		TIS_add_file_to_send(&tis, files[i]);
*	}
*
*	//Boucle d'envoie et de réception, s'arrête quand tous les fichier sont envoyés ou reçus
*	do {
*		int32_t result;
*
*		//Met le modem sous tension
*		Iridium_ON(); //(dépend de votre plateforme et non de la librairie)
*
*		//Lance la transmission
*		result = TIS_transmission(&tis, 10);
*
*		//Met le modem hors tension
*		Iridium_OFF(); //(dépend de votre plateforme et non de la librairie)
*
*		//Affiche des informations de diagnostiques
*		if (result != TIS_ERROR_SUCCESS) {
*			printf("Iridium : erreur %ld s'est produite pendant la transmission", result);
*		}
*		printf("Iridium : informations de diagnostiques : \n");
*		printf("\tNombre de messages SBD envoyés avec succès : %ld\n", tis.SBD_sent_without_error);
*		printf("\tNombre de messages SBD dont l'envoie a échoué : %ld\n", tis.SBD_sent_with_error);
*		printf("\tNombre de messages SBD reçues avec succès : %ld\n", tis.SBD_received_without_error);
*		printf("\tNombre de messages SBD dont la réception a échoué : %ld\n", tis.SBD_received_with_error);
*		printf("\tNombre de messages SBD en attente dans la constellation : %ld\n", tis.SBD_waiting_messages);
*		for (i = 0; i < file_count; i++) {
*			printf("Iridium : fichier %s, envoyé à %ld%\n", TIS_get_file_path(&tis, i), TIS_get_file_progress(&tis, i));
*		}
*		
*		//Si la communication n'est pas terminée, met en veille le système en attendant un nouveau crénaux de réception
*		if ((TIS_remaining_file_to_send(&tis) != 0) || (TIS_waiting_incoming_data() == TRUE)) {
*			sleep(300); //La durée dépend de votre application, la fonction depend de votre plateforme et non de la librairie)
*		}
*
*	//Si il reste des fichier en cours d'envoie ou en attente dans le satellite, reboucle
*	} while ((TIS_remaining_file_to_send(&tis) != 0) || (TIS_waiting_incoming_data() == TRUE));
*	
*	//Libère la mémoire occupée par la structure de la librairie
*	TIS_clean(&tis);
*	return TRUE;
* }
* \endcode
*
* \author Clément Bonnet
* \date 2011-2012
* \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
***********************************************************************/

/***********************************************************************//**
 * \file tis.h
 * \brief Eléments publiques de la librairie. Dans un programme, vous pouvez utiliser toutes ses fonctions et structures.
 *
 * Vous trouverez des exemples dans la page principale de cette documentation.
 *
 * \author Clément Bonnet
 * \date 2011-2012
 * \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
 ***********************************************************************/
 
#ifndef TIS_H
#define	TIS_H

/***********************************************************************//**
 * \def DISABLE_RUDICS
 * \brief Permet de désactiver le mode RUDICS.
 *
 * Pour réduire la taille de code lorsque le mode RUDICS n'est pas utilisé, on peut ne pas fournir la librairie Zmodem.
 * Dans ce cas, il faut mettre DISABLE_RUDICS à la valeur TRUE afin de supprimer les appels à cette librairie.
 ***********************************************************************/
#define DISABLE_RUDICS TRUE

#include "TIS_platforms.h"
#include "TIS_modems.h"
#include "TIS_errors.h"

/***********************************************************************//**
 * \def TIS_SERVICE_SBD
 * \brief Service Iridium SBD.
 ***********************************************************************/
#define TIS_SERVICE_SBD 0

/***********************************************************************//**
 * \def TIS_SERVICE_RUDICS
 * \brief Service Iridium RUDICS.
 ***********************************************************************/
#define TIS_SERVICE_RUDICS 1

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_EMPTY
 * \brief Indique que le fichier n'existe pas (valeur par défaut).
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_EMPTY		0

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_BEGINNING
 * \brief Indique que le fichier contient le début d'un fichier en cours de transfert.
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_BEGINNING	1

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_PART
 * \brief Indique que le fichier contient une partie n'étant ni le début, ni la fin d'un fichier en cours de transfert.
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_PART			2

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_END
 * \brief Indique que le fichier contient la fin d'un fichier en cours de transfert.
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_END			3

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_COMPLET
 * \brief Indique que le fichier contient un fichier complet en cours de transfert.
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_COMPLET		4

/***********************************************************************//**
 * \def TIS_SBD_INCOMMING_FILE_QUEUE_SIZE
 * \brief Longueur de la liste des fichier contenant des parties des fichiers en cours de réceptions.
 ***********************************************************************/
#define TIS_SBD_INCOMMING_FILE_QUEUE_SIZE 10

/***********************************************************************//**
 * \struct TIS_properties
 * \brief Contient toutes les données nécessaires au fonctionnement de la librairie.
 ***********************************************************************/
typedef struct {
	//Propriétés du modem
	TIS_modem	modem;			/**< Caractéristiques du modem utilisé. */
	uint64_t	IMEI_number;	/**< Numéro IMEI du modem utilisé. */
	uint8_t		pin[5];			/**< Code PIN de la carte SIM utilisée. */
	uint32_t	service;		/**< Service a utiliser lors des transferts. */
	
	//Gestion de la liaison série
	void *		serial_struct;															/**< Pointeur vers la stucture utilisé par les fonctions de communications. */
	int32_t		(*send_data)(void * serial_struct, uint8_t * data, int32_t count);		/**< Pointeur vers la fonction permettant l'envoie des données. */
	int32_t 	(*receive_data)(void * serial_struct, uint8_t * data, int32_t count);	/**< Pointeur vers la fonction permettant la réception des données. */
	int32_t 	(*wait_data)(void * serial_struct, uint32_t timeout);					/**< Pointeur vers la fonction permettant l'attente de données. */
	int32_t		(*flush_TX)(void * serial_struct);										/**< Pointeur vers la fonction permettant de vider de tampon d'émission. */
	int32_t		(*flush_RX)(void * serial_struct);										/**< Pointeur vers la fonction permettant de vider de tampon de reception. */
		
	//Fichiers en cours d'envoie
	uint8_t * sent_files_path;			/**< Pointeur vers le tableau contenant les chemins des fichiers à envoyer. */
	int64_t * sent_files_progress;		/**< Pointeur vers le tableau contenant le nombre d'octets envoyés pour chaque fichier. */
	uint16_t * sent_files_part_number;	/**< Pointeur vers le tableau contenant le numéro de la partie en cours d'envoie lorsque le fichier est envoyé via SBD. */
	uint8_t sent_files_count;			/**< Nombre de fichier actuellement stockés dans la mémoire. */
	uint8_t sent_files_allocated;		/**< Nombre de fichier pouvant être stockés compte tenu de la mémoire allouée. */
	
	//Dossier de réception
	uint8_t	receive_folder[MAXPATHLEN];	/**<  chemin du dossier où doivent être stockés les fichiers entrants. */
		
	//Propriétés spécifiques au mode SBD
	FILE * 		SBD_files[TIS_SBD_INCOMMING_FILE_QUEUE_SIZE];			/**< Tableau des descripteurs de fichiers entrant. */
	int8_t		SBD_file_name[TIS_SBD_INCOMMING_FILE_QUEUE_SIZE][13];	/**< Nom du fichier à reconstituer. */
	uint16_t	SBD_file_number[TIS_SBD_INCOMMING_FILE_QUEUE_SIZE]; 	/**< Contient le numéro du dernier message écrit dans le fichier entrant associé. */
	uint16_t	SBD_file_status[TIS_SBD_INCOMMING_FILE_QUEUE_SIZE];		/**< Statu du fichier associé. */
	
	//Propriétés spécifiques au mode RUDICS
	uint64_t	RUDICS_dial_number;						/**< Numéro appelé par le modem pour initier les communications RUDICS. */
	uint8_t		rudics_temporary_folder[MAXPATHLEN];	/**< Dossier dans lequel serons stockés les fichiers en cours de réception lors des échanges RUDICS. */
		
	//Données de diagnostique, elle sont remise à zéro à chaque transmission
	uint32_t	SBD_sent_without_error;		/**< Nombre de SBD envoyés sans erreurs durant la dernière session SBD. */
	uint32_t	SBD_sent_with_error;		/**< Nombre de SBD envoyés avec erreurs durant la dernière session SBD. */
	uint32_t	SBD_received_without_error;	/**< Nombre de SBD reçues sans erreurs durant la dernière session SBD. */
	uint32_t	SBD_received_with_error;	/**< Nombre de SBD reçues avec erreurs durant la dernière session SBD. */
	uint32_t	SBD_waiting_messages;		/**< Nombre de SBD en attente dans la constellation. */
	uint32_t	RUDICS_disconnections;		/**< Nombre de déconnexions durant la dernière session RUDICS. */
	uint32_t	RUDICS_incomplete_files;	/**< Nombre de fichier en cours de réception via RUDICS. Ces fichiers sont stockés dans le dossier temporaire et ne sont pas complets. */

} TIS_properties;

/***********************************************************************//**
 * \brief Initialise la structure de la librairie.
 * \public
 *
 * Les fonctions fournies à la librairie au travers de cette fonction ne doivent pas être bloquante.
 * Elles doivent posséder en interne un timeout afin de ne pas bloquer le système.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] receive_folder Pointeur vers la chaine de caractères contenant le chemin du dossier où doivent être stockés les fichiers entrants.
 * \param[in] modem Nom du modem, voir TIS_modems.h pour plus de détails.
 * \param[in] IMEI_number Numéro IMEI du modem utilisé par le système.
 * \param[in] service Service utilisé lors des transferts (RUDICS ou SBD).
 * \param[in] pin Code PIN de votre carte SIM, si vous n'avez pas de carte SIM utiliser la valeur NULL.
 * \param[in] rudics_dial_number Numéro appelé lors de la communication RUDICS.
 * \param[in] rudics_temporary_folder Dossier utiliser pour stocker les fichiers temporaires lors des transmissions RUDICS. Vous ne devez en aucun cas utiliser ce dossier pour un autre usage !
 * \param[in] sent_files_count Nombre de fichiers qui seront envoyés au cours de la même session.
 * \param[in] serial_struct Pointeur vers la structure définissant la liaison série.
 * \param[in] send_data Pointeur vers la fonction permettant l'envoie de données.
 * \param[in] receive_data Pointeur vers la fonction permettant la réception de données.
 * \param[in] wait_data Pointeur vers la fonction permettant l'attente de données.
 * \param[in] flush_TX Pointeur vers la fonction permettant de vider le tampon d'émission.
 * \param[in] flush_RX Pointeur vers la fonction permettant de vider le tampon de réception.
 *
 * \return Retourne un code d'erreur de la librairie.
 *
 * \sa TIS_modems.h TIS_errors.h
 ***********************************************************************/
int32_t TIS_init(TIS_properties *	properties,
				 uint8_t *			receive_folder,
				 int32_t			modem,
				 uint64_t			IMEI_number,
				 uint32_t			service,
				 uint8_t *			pin,
				 uint64_t			rudics_dial_number,
				 uint8_t * 			rudics_temporary_folder,
				 uint8_t			sent_files_count,
				 void *				serial_struct,
				 int32_t			(*send_data)(void * serial_struct, uint8_t * data, int32_t count),
				 int32_t 			(*receive_data)(void * serial_struct, uint8_t * data, int32_t count),
				 int32_t			(*wait_data)(void * serial_struct, uint32_t timeout),
				 int32_t			(*flush_TX)(void * serial_struct),
				 int32_t			(*flush_RX)(void * serial_struct));


/***********************************************************************//**
 * \brief Libère la mémoire occupée par la structure de la librairie
 * \public
 *
 * Cette fonction libère la mémoire alloué dynamiquement par la libraire et supprime les fichiers temporaires.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 ***********************************************************************/
void TIS_clean(TIS_properties * properties);

/***********************************************************************//**
 * \brief Cette fonction vous permet d'ajouter un fichier dans la liste des fichiers à envoyer
 * \public
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] path Pointeur vers la chaine de caractère contenant le chemin du fichier, cette chaîne est copiée par la fonction.
 *
 * \return En cas de succès, retourne l'index du fichier dans la liste des fichier à envoyer. En cas d'erreur, un code d'erreur est retourné sous la forme de l'opposé d'un code d'erreur.
 * \sa TIS_error.h
 ***********************************************************************/
int32_t TIS_add_file_to_send(TIS_properties * properties, uint8_t * path);


/***********************************************************************//**
 * \brief Cette fonction vous permet de vider la liste des fichier à envoyer.
 * \public
 *
 * Il n'est pas possible de supprimer un seul fichier de la liste. Il est à noter que les fichiers déjà envoyés ne sont jamais renvoyés.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 ***********************************************************************/
void TIS_remove_files_to_send(TIS_properties * properties);


/***********************************************************************//**
 * \brief Cette fonction vous permet de récupérer le chemin d'un fichier dans la liste des fichiers à envoyer grâce à son index.
 * \public
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] index Index du fichier dans la liste des fichiers à envoyer.
 *
 * \return Un pointeur vers le chemin du fichier. Un pointeur vers une chaine vide est renvoyé si le fichier n'existe pas.
 ***********************************************************************/
uint8_t * TIS_get_file_path(TIS_properties * properties, uint8_t index);

/***********************************************************************//**
 * \brief Cette fonction vous permet de récupérer l'avancement d'un fichier dans la liste des fichiers à envoyer grâce à son index.
 * \public
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] index Index du fichier dans la liste des fichiers à envoyer.
 *
 * \return La valeur en pourcentage d'avancement du fichier. Un fichier qui n'existe pas à toujours un avancement de 0.
 ***********************************************************************/
int32_t TIS_get_file_progress(TIS_properties * properties, uint8_t index);

/***********************************************************************//**
 * \brief Cette fonction vous permet de connaitre le nombre de fichier de la liste des fichiers à envoyer qui n'ont pas encore été envoyés.
 * \public
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 *
 * \return Le nombre de fichier restant à envoyer.
 ***********************************************************************/
uint8_t TIS_remaining_file_to_send(TIS_properties * properties);

/***********************************************************************//**
 * \brief Cette fonction vous permet de savoir si des fichiers sont en attente dans la constellation (SBD) ou sur le serveur (RUDICS).
 * \public
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 *
 * \return TRUE si des données doivent encore être reçues
 * \return FALSE si toutes les données ont été reçues
 ***********************************************************************/
bool TIS_waiting_incoming_data(TIS_properties * properties);

/***********************************************************************//**
 * \brief Effectue une session d'envoie et de réception de fichier via Iridium.
 *
 * Les fichiers reconstitués sont stockés dans le dossier "receive_folder".
 *
 * Il est à noter que la structure contenant les propriétés de la librairie ne doit pas être perdue entre deux appelles de "TIS_transmission" si tous les fichiers non sont pas reçues ou envoyés.
 * En effet, ceci aurais pour conséquence la perte des fichiers en cours de réception et le renvoie intégrale des fichiers en cours d'envoie.
 *
 * Si le nombre d'erreurs consécutives "maximum_consecutive_errors" est atteint, la fonction se termine et renvoie l'erreur TIS_ERROR_TOO_MANY_CONSECUTIVE_ERRORS.
 * Lors d'une communication RUDICS, les déconnexions ne sont pas considérées comme des erreurs.
 *
 * \param[in,out] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 * \param[in] maximum_consecutive_errors Nombre d'erreurs consécutives maximum avant de quitter.
 *
 * \return Un code d'erreur.
 * \sa TIS_errors.h
 ***********************************************************************/
int32_t TIS_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors);

/***********************************************************************//**
 * \brief Donne une idée de la puissance du signal satellite reçu.
 * \public
 *
 * Cette fonction repose sur des informations fournies par le modem qui ne sont pas fiables.
 * Cette fonction est fournie principalement pour être utilisé lors des phase de mise au point et de tests.
 * Cette fonction nécessite l'initialisation de la librairie et que le modem soit sous tension.
 *
 * \param[in] properties Pointeur vers la structure qui contient les propriétés de la librairie.
 *
 * \return Une valeur correspondant à la puissance du signal ou une valeur négative en cas d'échec.
 *
 * \sa TIS_SIGNAL_STRENGTH_0 TIS_SIGNAL_STRENGTH_1 TIS_SIGNAL_STRENGTH_2 TIS_SIGNAL_STRENGTH_3 TIS_SIGNAL_STRENGTH_4 TIS_SIGNAL_STRENGTH_5
 ***********************************************************************/
int32_t TIS_signal_strenght(TIS_properties * properties);

/***********************************************************************//**
 * \brief Retourne la position GPS du modem.
 * \public
 * \todo Fonction non implémentée
 ***********************************************************************/
int32_t TIS_get_gps_position();

/***********************************************************************//**
 * \brief Demande au modem de mettre hors tension sa partie Iridium
 * \public
 * \todo Fonction non implémentée
 ***********************************************************************/
int32_t TIS_power_off_iridium();

/***********************************************************************//**
 * \brief Demande au modem de mettre sous tension sa partie Iridium
 * \public
 * \todo Fonction non implémentée
 ***********************************************************************/
int32_t TIS_power_on_iridium();

/***********************************************************************//**
 * \brief Demande au modem de mettre hors tension sa partie GPS
 * \todo Fonction non implémentée
 ***********************************************************************/
int32_t TIS_power_off_gps();

/***********************************************************************//**
 * \brief Demande au modem de mettre sous tension sa partie GPS
 * \public
 * \todo Fonction non implémentée
 ***********************************************************************/
int32_t TIS_power_on_gps();

#endif //TIS_H
