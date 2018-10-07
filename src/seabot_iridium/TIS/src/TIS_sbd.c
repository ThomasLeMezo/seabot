#include "TIS_sbd.h"
#include "TIS_errors.h"
#include "TIS_at.h"

#include <string.h>
#include <time.h>
#include <errno.h>

int32_t TIS_SBD_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors) {
	
	int32_t sent_file_index;				//Index dans la liste des fichiers en cours d'envoi
	FILE * sent_file;						//Fichier en cours d'envoi
	int64_t sent_file_size;					//Taille du fichier en cours d'envoi
		
	int32_t MO_status;						//Resultat de l'envoi d'un message SBD
	int32_t MT_queued;						//Indique le nombre de message en attente dans le satellite
	int32_t MT_queued_previous;				//Indique le nombre de message en attente dans le satellite lors de la dernière session sans erreur
	int32_t MT_status;						//Resultat de la réception d'un message SBD
	
	int32_t result;
	int32_t consecutive_errors;
	
	MO_status = TIS_AT_SBDI_MO_STATUS_SUCCESS;
	MT_queued = TIS_AT_SBDI_MT_QUEUED_EMPTY;
	MT_status = TIS_AT_SBDI_MT_STATUS_ERROR;
	
	sent_file_index = 0; //Positionne l'index sur le premier fichier de la liste
	
	sent_file = NULL;
		
	consecutive_errors = 0; //Met à jour le compteur d'erreur consécutives
		
	//Vide les tampons SBD du modem
	result = TIS_AT_SBDD(properties, TRUE, TRUE); 
	if (result != TIS_SUCCESS) {
		return result;
	}
	
	do {
		/***********************************************************************
		* Gestion des messages sortants
		***********************************************************************/
		if ((sent_file_index < properties->sent_files_count) && (MO_status == TIS_AT_SBDI_MO_STATUS_SUCCESS)) {
			uint32_t sent_file_datation;
			uint8_t	sent_file_extension[5];
			uint8_t MO_message[TIS_SBD_SIZE_MAX];	//Message sortant
			int32_t MO_count;						//Taille du message sortant
			//Si le précédent est terminé
			if (sent_file == NULL) {
				//On cherche le prochain fichier à envoyer dans la liste
				while (TIS_get_file_progress(properties, sent_file_index) == 100) {
					sent_file_index++;
					//Si on atteint la fin de la liste, on quitte
					if (sent_file_index >= properties->sent_files_count) {
						break;
					}
				}
				
				//Ouvre le fichier
				if (sent_file_index < properties->sent_files_count) {
					sent_file = fopen(TIS_get_file_path(properties, sent_file_index), "rb" );
					if (sent_file == NULL) {
						return TIS_ERROR_FILE_ACCESS;
					}
					
					//Stocke la taille du fichier
					fseek(sent_file, 0, SEEK_END);
					sent_file_size = ftell(sent_file);
					
					//Si le fichier avais déjà été envoyé en partie, se déplace dans le fichier pour continuer l'envoi
					fseek(sent_file, properties->sent_files_progress[sent_file_index],SEEK_SET);
				}
			}
			
			//Si le fichier est ouvert
			if (sent_file != NULL) {
				//Génération de l'en-tête
				result = sscanf(TIS_get_file_path(properties, sent_file_index) + strlen(TIS_get_file_path(properties, sent_file_index)) - 12, "%"PRIX32".%s", &sent_file_datation, sent_file_extension);
				if (result != 2) {
					return TIS_ERROR_INVALID_FILE_NAME;
				}
				
				//Ajoute la datation
				MO_message[0] = sent_file_datation >> 24;
				MO_message[1] = sent_file_datation >> 16;
				MO_message[2] = sent_file_datation >> 8;
				MO_message[3] = sent_file_datation;
				
				//Ajoute le numéro de la partie
				MO_message[4] = properties->sent_files_part_number[sent_file_index] >> 8;
				MO_message[5] = properties->sent_files_part_number[sent_file_index] & 0xFF;
								
				//Ajoute le type de fichier
				if ((sent_file_extension[0] == 's') || (sent_file_extension[0] == 'S')) {
					MO_message[4] = MO_message[4] | TIS_SBD_FILE_TYPE_SCIENTIFIC_DATA;
				} else if ((sent_file_extension[0] == 't') || (sent_file_extension[0] == 'T')) {
					MO_message[4] = MO_message[4] | TIS_SBD_FILE_TYPE_TECHNICAL_DATA;
				} else {
					MO_message[4] = MO_message[4] | TIS_SBD_FILE_TYPE_COMMAND;
				}	
				
				//Lit le fichier et copie les données dans le message
				MO_count = fread(MO_message + TIS_SBD_MESSAGE_HEADER_SIZE, 1, properties->modem.sbd_size_max - TIS_SBD_MESSAGE_HEADER_SIZE, sent_file);	
				MO_count = MO_count + TIS_SBD_MESSAGE_HEADER_SIZE;
											
				//Détection de la fin de fichier
				if (feof(sent_file)) {
					//On met à jour l'en-tête du fichier
					MO_message[4] = MO_message[4] | TIS_SBD_HEADER_END_MESSAGE;
					//On ferme le fichier
					fclose(sent_file);
					
					sent_file = NULL;
					sent_file_index++;
				}
				
				//Envoie du message vers le modem
				do {
					result = TIS_AT_SBDWB(properties, MO_message, MO_count);
					if (result == TIS_SUCCESS) {
						break;				
					}
				} while (++consecutive_errors < maximum_consecutive_errors);
				if (consecutive_errors >= maximum_consecutive_errors) {
          return TIS_ERROR_SERIAL_ERROR_MAXIMUM_CONSECUTIVE_ERRORS;
				}
			}
			
		}
			
		/***********************************************************************
		* Session SBD
		***********************************************************************/
		MT_queued_previous = MT_queued;
		result = TIS_AT_SBDI(properties, &MO_status, &MT_status, &MT_queued);
			
		if (result != TIS_SUCCESS) {
			MO_status = TIS_AT_SBDI_MO_STATUS_ERROR;
			MT_status = TIS_AT_SBDI_MT_STATUS_ERROR;
			consecutive_errors++;
			continue;
		} else if ((MO_status == TIS_AT_SBDI_MO_STATUS_ERROR) || (MT_status == TIS_AT_SBDI_MT_STATUS_ERROR)) {	
			consecutive_errors++;
			if (MO_status == TIS_AT_SBDI_MO_STATUS_ERROR) {
				properties->SBD_sent_with_error++;
			}
			if  (MT_status == TIS_AT_SBDI_MT_STATUS_ERROR) {
				properties->SBD_received_with_error++;
				if (MT_queued_previous != TIS_AT_SBDI_MT_QUEUED_EMPTY) {
					MT_queued = MT_queued_previous;
				}
			}
		}	
				
		properties->SBD_waiting_messages = MT_queued;
		
		if (MO_status == TIS_AT_SBDI_MO_STATUS_SUCCESS) {
			consecutive_errors = 0; //Remet à jour le compteur d'erreur consécutives
			properties->SBD_sent_without_error++;
			
			if (properties->sent_files_progress != NULL) {
				//Met à jour l'avancement du fichier
				if (sent_file == NULL) {
					properties->sent_files_progress[sent_file_index-1] = sent_file_size;
				} else {
					properties->sent_files_progress[sent_file_index] = ftell(sent_file);
					properties->sent_files_part_number[sent_file_index]++;
					if (properties->sent_files_part_number[sent_file_index] > TIS_SBD_MESSAGE_NUMBER_MAX) {
						return TIS_ERROR_FILE_TOO_LARGE;
					}
				}
			}
		}		
		
		/***********************************************************************
		* Gestion des messages entrants
		***********************************************************************/
		if (MT_status == TIS_AT_SBDI_MT_STATUS_SUCCESS) {
			uint8_t MT_message[TIS_SBD_SIZE_MAX];	//Message entrant
			int32_t MT_count;						//Taille du message entrant
			consecutive_errors = 0; //Remet à jour le compteur d'erreur consécutives
	
			properties->SBD_received_without_error++;
			
			//On copie le message entrant depuis le modem
			do {
				result = TIS_AT_SBDRB(properties, MT_message, &MT_count); //Lie le message contenue dans le modem	
				if (result == TIS_SUCCESS) {
					break;				
				}
			} while (--consecutive_errors < maximum_consecutive_errors);
			if (consecutive_errors >= maximum_consecutive_errors) {
				return TIS_ERROR_SERIAL_ERROR;
			}
						
			//On stocke le message entrant
			result = TIS_SBD_store_received_message(properties, MT_message, MT_count);
			if (result != TIS_SUCCESS) {
				return result;
			}
		}
			
	} while (((sent_file_index < properties->sent_files_count) || (MO_status == TIS_AT_SBDI_MO_STATUS_ERROR) || (MT_queued != TIS_AT_SBDI_MT_QUEUED_EMPTY) || (MT_status == TIS_AT_SBDI_MT_STATUS_ERROR)) && (consecutive_errors < maximum_consecutive_errors));	

	if (consecutive_errors >= maximum_consecutive_errors) {
		return TIS_ERROR_TOO_MANY_CONSECUTIVE_ERRORS;
	}
	
	return TIS_SUCCESS;
}

int32_t TIS_SBD_store_received_message(TIS_properties * properties, uint8_t * message, int32_t message_count) {
	uint32_t current_index;
	uint16_t number;
	int8_t current_file_name[13];
	
	
	//Génère le nom du fichier
	sprintf(current_file_name, "%08"PRIX32".", (uint32_t)((uint32_t)message[0] << 24 | (uint32_t)message[1] << 16 | (uint32_t)message[2] << 8 | (uint32_t)message[3]));

	if ((message[4] & TIS_SBD_FILE_TYPE_MASK) == TIS_SBD_FILE_TYPE_SCIENTIFIC_DATA) {
		current_file_name[9]  = 'S';
		current_file_name[10] = 'D';
		current_file_name[11] = 'T';
		current_file_name[12] = '\0';
	} else if ((message[4] & TIS_SBD_FILE_TYPE_MASK) == TIS_SBD_FILE_TYPE_TECHNICAL_DATA) {
		current_file_name[9]  = 'T';
		current_file_name[10] = 'D';
		current_file_name[11] = 'T';
		current_file_name[12] = '\0';
	} else {
		current_file_name[9]  = 'C';
		current_file_name[10] = 'M';
		current_file_name[11] = 'D';
		current_file_name[12] = '\0';
	}
		
	//Extrait le numéro de message du message
	number = ((message[4] & 0x1F) << 8) | message[5];
	
	//Stocke le message sur le disque
	if (number == 0) {
		//Si c'est le premier message d'un fichier
		uint8_t received_file_path[MAXPATHLEN];
		
		//Recherche un emplacement non utilisé dans la liste
		current_index = 0;
		while ((properties->SBD_file_status[current_index] != TIS_SBD_INCOMMING_FILE_EMPTY) && (current_index < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE)) {
			current_index++;
		}
		if (current_index >= TIS_SBD_INCOMMING_FILE_QUEUE_SIZE) {
			return TIS_ERROR_TOO_MANY_FILES_PARTS;
		}
		
		//Stocke le nom du fichier
		strcpy(properties->SBD_file_name[current_index],current_file_name);
						
		//Génére le chemin du fichier
		sprintf(received_file_path, "%s"PATH_SEPARATOR"%s", properties->receive_folder, current_file_name);

		//Crée le fichier
		properties->SBD_files[current_index] = fopen(received_file_path,"w");
		if (properties->SBD_files[current_index] == NULL) {
			return TIS_ERROR_FILE_ACCESS;
		}
		
		//On écrit les données dans le fichier
		fwrite(message + TIS_SBD_MESSAGE_HEADER_SIZE, 1 , message_count - TIS_SBD_MESSAGE_HEADER_SIZE, properties->SBD_files[current_index]);
		
		//On met à jour le status
		if ((message[4] & TIS_SBD_HEADER_END_MESSAGE) == TIS_SBD_HEADER_END_MESSAGE) {
			properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_COMPLET;
		} else {
			properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_BEGINNING;
		}		
	} else {
		//Si ce n'est pas le premier message d'un fichier
		bool found = FALSE;
		uint32_t previous_index;
		
		//Recherche si ce message est la suite d'un message précédent
		for (previous_index = 0; previous_index < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE; previous_index++) {
			//Si c'est le début d'un fichier
			if (properties->SBD_file_status[previous_index] == TIS_SBD_INCOMMING_FILE_BEGINNING) {
				//Vérifie si c'est le bon fichier
				if (strcmp(properties->SBD_file_name[previous_index], current_file_name) == 0) {
					//test si c'est la partie précédente
					if (properties->SBD_file_number[previous_index] == number - 1) {
						found = TRUE;
						break;
					}
				}
			}
		}
		
		if (found == FALSE) {
			//Recherche un emplacement non utilisé dans la liste
			current_index = 0;
			while ((properties->SBD_file_status[current_index] != TIS_SBD_INCOMMING_FILE_EMPTY) && (current_index < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE)) {
				current_index++;
			}
			if (current_index >= TIS_SBD_INCOMMING_FILE_QUEUE_SIZE) {
				return TIS_ERROR_TOO_MANY_FILES_PARTS;
			}
			
			//Crée le fichier temporaire
			properties->SBD_files[current_index] = TIS_create_temporary_file();
			if (properties->SBD_files[current_index] == NULL) {
				return TIS_ERROR_FILE_ACCESS;
			}
			
			//On écrit le message dans le fichier
			fwrite(message + TIS_SBD_MESSAGE_HEADER_SIZE, 1 , message_count - TIS_SBD_MESSAGE_HEADER_SIZE, properties->SBD_files[current_index]);
			
			//Stocke le nom du fichier
			strcpy(properties->SBD_file_name[current_index],current_file_name);
			
			//On met à jour le status
			if ((message[4] & TIS_SBD_HEADER_END_MESSAGE) == TIS_SBD_HEADER_END_MESSAGE) {
				properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_END;
			} else {
				properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_PART;
			}
			
		} else {
			//Change la valeur de current_index pour écrire à la suite du fichier précédent
			current_index = previous_index;
			
			//On écrit le message dans le fichier
			fwrite(message + TIS_SBD_MESSAGE_HEADER_SIZE, 1 , message_count - TIS_SBD_MESSAGE_HEADER_SIZE, properties->SBD_files[current_index]);
			
			//On met à jour le status
			if ((message[4] & TIS_SBD_HEADER_END_MESSAGE) == TIS_SBD_HEADER_END_MESSAGE) {
				properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_COMPLET;
			} else {
				properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_BEGINNING;
			}
		}
	}
	//On met à jour le numéro du message
	properties->SBD_file_number[current_index] = number;
	
	//Vérifie que le fichier courrant contient le début du fichier, sinon ne fait rien
	if (properties->SBD_file_status[current_index] == TIS_SBD_INCOMMING_FILE_BEGINNING) {			
		bool found;
		
		do {
			//On recherche si l'on a déjà la suite du fichier
			uint32_t next_index;
			
			found = FALSE;
			for (next_index = 0; next_index < TIS_SBD_INCOMMING_FILE_QUEUE_SIZE; next_index++) {
				//test si le fichier est valide
				if (properties->SBD_file_status[next_index] != TIS_SBD_INCOMMING_FILE_EMPTY) {
					//Vérifier qu'il s'agit du même fichier
					if (strcmp(properties->SBD_file_name[next_index], current_file_name) == 0) {
						if (properties->SBD_file_number[next_index] == number + 1) {
							//Si l'on trouve la suite du fichier, on l'ajoute au fichier
							int32_t result;
							
							//On écrit la suite du fichier
							result = TIS_file_append(properties->SBD_files[current_index], properties->SBD_files[next_index]);
							if (result != TIS_SUCCESS) {
								return result;
							}
							
							//Ferme le fichier temporaire, celà le supprime également
							TIS_delete_temporary_file(properties->SBD_files[next_index]);
													
							//Met à jour le status du fichier courrant
							if (properties->SBD_file_status[next_index] == TIS_SBD_INCOMMING_FILE_END) {
								properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_COMPLET;
							}
							
							//Met à jour le status du fichier temporaire
							properties->SBD_file_status[next_index] = TIS_SBD_INCOMMING_FILE_EMPTY;
							
							//Met à jour le numéro du message
							properties->SBD_file_number[current_index] = properties->SBD_file_number[next_index];
							
							found = TRUE;
							break;
						}
					}
				}
			}
		} while ((found == TRUE) && (properties->SBD_file_status[current_index] != TIS_SBD_INCOMMING_FILE_COMPLET));
	}
	
	//Si le fichier est complet
	if (properties->SBD_file_status[current_index] == TIS_SBD_INCOMMING_FILE_COMPLET) {
		//On ferme le fichier et libère la liste
		fclose(properties->SBD_files[current_index]);
		properties->SBD_file_status[current_index] = TIS_SBD_INCOMMING_FILE_EMPTY;
	}
	
	return TIS_SUCCESS;
}

int32_t TIS_file_append(FILE * destination, FILE * source) {
	uint8_t ch;
	
	fseek(source, 0, SEEK_SET);
	fseek(destination, 0, SEEK_END);
	
	while(!feof(source)) {
		ch = fgetc(source);
		if(ferror(source)) {
			return TIS_ERROR_FILE_ACCESS;
		}
		
		if(!feof(source)) fputc(ch, destination);
		
		if(ferror(destination)) {
			return TIS_ERROR_FILE_ACCESS;
		}
	}
	
	return TIS_SUCCESS;
}

int32_t TIS_filelength(FILE * fp) {
	int32_t current_pos;
	int32_t length;

	if (fp == NULL) {
		return 0;
	}

	current_pos = ftell(fp);
	fseek(fp, 0, SEEK_END);
	length = ftell(fp);
	fseek(fp, current_pos, SEEK_SET);

	return length;
}
