#include "tis.h"
#include "TIS_sbd.h"
#include "TIS_rudics.h"
#include "TIS_at.h"

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

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
				 int32_t				(*flush_TX)(void * serial_struct),
				 int32_t				(*flush_RX)(void * serial_struct)) {
				 
	int32_t i;
		
	//Vérifie les paramètres d'entrée
	if ((properties == NULL) || (receive_folder == NULL) || (send_data == NULL) || (receive_data == NULL) || (wait_data == NULL) || (flush_TX == NULL) || (flush_RX == NULL)) {
		return TIS_ERROR_UNDEFINED_PARAMETER;
	}
	
	//Copie le chemin du dossier de réception
	strcpy(properties->receive_folder, receive_folder);
		
	//Copie de la configuration du modem
	properties->modem.sbd = TIS_modems[modem].sbd;
	properties->modem.rudics = TIS_modems[modem].rudics;
	properties->modem.sim_card = TIS_modems[modem].sim_card;
	properties->modem.gps = TIS_modems[modem].gps;
	properties->modem.sbd_size_max = TIS_modems[modem].sbd_size_max;
		
	properties->IMEI_number = IMEI_number;
			
	if (properties->modem.sim_card == TRUE) {
		if (pin == NULL) {
			return TIS_ERROR_UNDEFINED_PARAMETER;
		}
		properties->pin[0] = pin[0];
		properties->pin[1] = pin[1];
		properties->pin[2] = pin[2];
		properties->pin[3] = pin[3];
		properties->pin[4] = '\0';
	}
	
	//Désactive le mode rudics si aucun numéro d'appel n'est fournit
	if (rudics_dial_number == 0) {
		properties->modem.rudics = FALSE;
	}
		
	if ((service == TIS_SERVICE_SBD) && (properties->modem.sbd == TRUE)) {
		properties->service = TIS_SERVICE_SBD;
	} else if ((service == TIS_SERVICE_RUDICS) && (properties->modem.rudics == TRUE)){
		properties->service = TIS_SERVICE_RUDICS;
	} else {
		return TIS_ERROR_NOT_AVAILABLE;
	}
	
	if ((service == TIS_SERVICE_RUDICS) && (rudics_temporary_folder == NULL)) {
		return TIS_ERROR_UNDEFINED_PARAMETER;
	}
	
	//Copie le chemin du dossier temporaire utilisé en mode RUDICS
	if (rudics_temporary_folder != NULL) {
		strcpy(properties->rudics_temporary_folder, rudics_temporary_folder);
	}
	
	properties->sent_files_path = NULL;
	properties->sent_files_progress = NULL;
	properties->sent_files_part_number = NULL;
	
	//Alloue la mémoire pour stocker les informations sur les fichiers à envoyer si besoin
	if (sent_files_count != 0) {
		properties->sent_files_path = calloc(MAXPATHLEN * sent_files_count, sizeof(uint8_t));
		properties->sent_files_progress = calloc(sent_files_count, sizeof(int64_t));
		properties->sent_files_part_number = calloc(sent_files_count, sizeof(uint16_t));
		
		if ((properties->sent_files_path == NULL) || (properties->sent_files_progress == NULL) || (properties->sent_files_part_number == NULL)) {
			return TIS_ERROR_NOT_ENOUGH_MEMORY;
		}
	}
		
	properties->sent_files_count = 0;
	properties->sent_files_allocated = sent_files_count;
	
	//Initialisation des informations relatives au mode SBD
	for (i = 0; i < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE; i++) {
		properties->SBD_file_status[i] = TIS_SBD_INCOMMING_FILE_EMPTY;
	}
	
	//Initialisation des informations relatives au mode RUDICS
	properties->RUDICS_dial_number = rudics_dial_number;
	
	//Stockage les callbacks
	properties->serial_struct = serial_struct;
	properties->send_data = send_data;
	properties->receive_data = receive_data;
	properties->wait_data = wait_data;
	properties->flush_TX = flush_TX;
	properties->flush_RX = flush_RX;
	
	return TIS_ERROR_SUCCESS;
}

void TIS_clean(TIS_properties * properties) {
	int32_t i;
	
	//Ferme les fichiers ouverts,et supprime les fichiers temporaires
	for (i = 0; i < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE; i++) {
		if (properties->SBD_file_status[i] != TIS_SBD_INCOMMING_FILE_EMPTY) {
			if ((properties->SBD_file_status[i] == TIS_SBD_INCOMMING_FILE_BEGINNING) || (properties->SBD_file_status[i] == TIS_SBD_INCOMMING_FILE_COMPLET)) {
				fclose(properties->SBD_files[i]);
			} else {
				TIS_delete_temporary_file(properties->SBD_files[i]);
			}
		}
	}
	
	if (properties->sent_files_path != NULL) {
		free(properties->sent_files_path);
	}
	
	if (properties->sent_files_progress != NULL) {
		free(properties->sent_files_progress);
	}
	
	if (properties->sent_files_part_number != NULL) {
		free(properties->sent_files_part_number);
	}
	
}

int32_t TIS_add_file_to_send(TIS_properties * properties, uint8_t * path) {
	//Vérifie qu'il reste de la place dans la liste
	if (properties->sent_files_count + 1 > properties->sent_files_allocated) {
		return -TIS_ERROR_LIST_FULL;
	}
	
	if (strlen(path) > MAXPATHLEN) {
		return -TIS_ERROR_INVALID_FILE_NAME;
	}
		
	//Copie le chemin du fichier
	strcpy(properties->sent_files_path + (properties->sent_files_count * MAXPATHLEN), path);
	
	//Initialise la progression
	properties->sent_files_progress[properties->sent_files_count] = 0;
	properties->sent_files_part_number[properties->sent_files_count] = 0;
	
	//incrémente le compteur de fichiers
	properties->sent_files_count++;
	
	//Renvoie l'index du ficheir dans la liste
	return properties->sent_files_count - 1;
}

void TIS_remove_files_to_send(TIS_properties * properties) {
	properties->sent_files_count = 0;
	
	memset(properties->sent_files_path, 0,  MAXPATHLEN * properties->sent_files_allocated * sizeof(uint8_t));
	memset(properties->sent_files_progress, 0, properties->sent_files_allocated * sizeof(int64_t));
	memset(properties->sent_files_part_number, 0, properties->sent_files_allocated * sizeof(uint16_t));
}

uint8_t * TIS_get_file_path(TIS_properties * properties, uint8_t index) {
	return properties->sent_files_path + (index * MAXPATHLEN);
}

int32_t TIS_get_file_progress(TIS_properties * properties, uint8_t index) {
	int64_t size;
	
	size = TIS_get_file_size(properties->sent_files_path + (index * MAXPATHLEN));

	if (properties->sent_files_progress[index] == size) {
		return 100;
	}
	
	if ((properties->sent_files_progress[index] == 0) || (size == 0)) {
		return 0;
	}
	
	return (uint8_t)(((float)(properties->sent_files_progress[index]) / (float)(size)) * 100.0);
}

bool TIS_waiting_incoming_data(TIS_properties * properties) {
	if (properties->rudics_temporary_folder == NULL) {
		if (properties->SBD_waiting_messages == 0) {
			return FALSE;
		} else {
			return TRUE;
		}
	} else {
		if ((properties->SBD_waiting_messages == 0) && (TIS_folder_empty(properties->rudics_temporary_folder) == TRUE)) {
			return FALSE;
		} else {
			return TRUE;
		}
	}
	
}

uint8_t TIS_remaining_file_to_send(TIS_properties * properties) {
	uint8_t count = 0;
	uint8_t i;
	
	for (i = 0; i < properties->sent_files_count; i++) {
		if (TIS_get_file_progress(properties, i) != 100) {
			count++;
		}
	}
	
	return count;
}

int32_t TIS_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors) {
	int32_t result;
	int32_t errors;

	if (properties == NULL) {
		return TIS_ERROR_UNDEFINED_PARAMETER;
	}
	
	properties->SBD_sent_without_error = 0;
	properties->SBD_sent_with_error = 0;
	properties->SBD_received_without_error = 0;
	properties->SBD_received_with_error = 0;
	properties->SBD_waiting_messages = 0;
	properties->RUDICS_disconnections = 0;
	properties->RUDICS_incomplete_files = 0;
	
	//Si le modem à une carte SIM, envoie le code PIN
	if (properties->modem.sim_card == TRUE) {
		result = TIS_AT_CPIN(properties, properties->pin);
		if (result != TIS_ERROR_SUCCESS) {
			return result;
		}
		
		//Attend que le modem soit associé avec le réseau
		errors = 0;
		do {
			//Attend 10 seconde pour laisser le temps du modem pour s'enregistrer
			clock_t endwait;
			endwait = clock() + 10 * CLOCKS_PER_SEC;
			while (clock() < endwait) {}
			
			result = TIS_AT_CREG(properties);
		} while ((result != TIS_ERROR_SUCCESS) && (++errors < maximum_consecutive_errors));
		
		if (errors >= maximum_consecutive_errors) {
			return TIS_ERROR_NOT_REGISTERED;
		}
	}
		
	//Lance la session du protocole adapté				
	if (properties->service == TIS_SERVICE_SBD) {
		return TIS_SBD_transmission(properties, maximum_consecutive_errors);
	} else {
		return TIS_RUDICS_transmission(properties, maximum_consecutive_errors);
	}
	return TIS_ERROR_SUCCESS;
}

int32_t TIS_signal_strenght(TIS_properties * properties) {
	int32_t strength;
	int32_t result;
	
	//Si le modem à une carte SIM, envoie le code PIN
	if (properties->modem.sim_card == TRUE) {
		result = TIS_AT_CPIN(properties, properties->pin);
		if (result != TIS_ERROR_SUCCESS) {
			return result;
		}
	}
	
	
	result = TIS_AT_CSQ(properties, &strength);
	if (result != TIS_ERROR_SUCCESS) {
		return -1;
	} else {
		return strength;
	}
}

int32_t TIS_power_off_gps(TIS_properties * properties) {
	if (properties->modem.gps) {
		return TIS_ERROR_SUCCESS;
	} else {
		return TIS_ERROR_NOT_AVAILABLE;
	}
}

int32_t TIS_power_on_gps(TIS_properties * properties) {
	if (properties->modem.gps) {
		return TIS_ERROR_SUCCESS;
	} else {
		return TIS_ERROR_NOT_AVAILABLE;
	}
}

int32_t TIS_get_gps_position(TIS_properties * properties) {
	if (properties->modem.gps) {
		return TIS_ERROR_SUCCESS;
	} else {
		return TIS_ERROR_NOT_AVAILABLE;
	}
}

int32_t TIS_power_off_iridium() {
	return TIS_ERROR_SUCCESS;
}

int32_t TIS_power_on_iridium() {
	return TIS_ERROR_SUCCESS;
}
