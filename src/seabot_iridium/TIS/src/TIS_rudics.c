#include "TIS_rudics.h"
#include "TIS_errors.h"
#include "TIS_at.h"
#if DISABLE_RUDICS == FALSE
	#include "zmodem.h"
#endif

static int64_t TIS_RUDICS_current_file_progress;

int32_t TIS_RUDICS_transmission(TIS_properties * properties, int32_t maximum_consecutive_errors) {
	
	//Permet de couper la partie RUDICS afin de réduire la taille de code si ce mode n'est jamais utilisé
	#if DISABLE_RUDICS == TRUE
		return TIS_ERROR_NOT_AVAILABLE;
	#else
	
	int32_t result;
	bool sent;
	FILE * sent_file;
	int64_t bytes_received;
	int64_t bytes_sent;
	FILE * identification;
	int32_t i;
	zmodem_t zm;
	
	zmodem_init(&zm,
				properties,
				TIS_progress,
				TIS_file_received,
				TIS_send_byte,
				TIS_receive_byte,
				NULL,
				NULL,
				TIS_wait_data,
				TIS_flush_TX,
				NULL);
	
	//Génère le fichier d'identification
	identification = TIS_create_temporary_file();
	if (identification == NULL) {
		return TIS_ERROR_NO_SPACE_AVAILABLE;
	}
	fprintf(identification, "{\"identification\" : {\"type\" : \"instrument\", \"IMEI\" : %"PRIu64"}}\n", properties->IMEI_number);
	fseek(identification, 0, SEEK_SET);
	
	//Etablit la connection
	do {
		result = TIS_AT_D(properties, properties->RUDICS_dial_number);
		if ((result != TIS_ERROR_SUCCESS) && (result != TIS_ERROR_DIALUP)) {
			TIS_delete_temporary_file(identification);
			return result;
		}
	
	} while ((result == TIS_ERROR_DIALUP) &&  (--maximum_consecutive_errors > 0));
	if (maximum_consecutive_errors <= 0) {
		TIS_delete_temporary_file(identification);
		return TIS_ERROR_DIALUP;
	}

	//Envoie le fichier d'identification
	result = zmodem_send_file(&zm, "identification", identification, TRUE, &bytes_sent);
	zmodem_end_session(&zm);
			
	//Supprime le fichier d'identification
	TIS_delete_temporary_file(identification);
	
	if(result == FALSE) {
		return TIS_ERROR_RUDICS_IDENTIFICATION_FAILED;
	}
		
	//On récupère les fichiers de configurations via Zmodem
	printf("début réception zmodem\n");
	zmodem_recv_files(&zm, properties->rudics_temporary_folder, &bytes_received);
	printf("fin réception zmodem\n");
	
	//On envoi les fichiers de données via Zmodem
	printf("début envoi zmodem\n");
	
	for (i = 0; i < properties->sent_files_count; i++) {
		//Vérifie si le fichier a déjà été envoyé
		if (TIS_get_file_progress(properties, i) == 100) {
			continue;
		}
		
		//ouvre le fichier à envoyer
		sent_file = fopen(TIS_get_file_path(properties, i), "r");
		if (sent_file == NULL) {
			return TIS_ERROR_FILE_ACCESS;
		}
		
		//Envoie le fichier
		sent = zmodem_send_file(&zm, TIS_get_file_path(properties, i), sent_file, FALSE, &bytes_sent);
		if (sent == FALSE) {
			properties->sent_files_progress[i] = 0;
		} else {
			properties->sent_files_progress[i] = TIS_RUDICS_get_current_file_progress();
		}
		
		//ferme le fichier
		fclose(sent_file);
	}
	
	zmodem_end_session(&zm);
	
	printf("fin envoi zmodem\n");
	
	return TIS_ERROR_SUCCESS;
	
	#endif
}

int32_t TIS_send_byte(void * properties, uint8_t data, uint32_t timeout) {
	clock_t endwait;
	
  	endwait = clock() + timeout * CLOCKS_PER_SEC ;
  	while (clock() < endwait) {
		if (((TIS_properties*)properties)->send_data(((TIS_properties*)properties)->serial_struct, &data, 1) == TIS_ERROR_SUCCESS) {
			return 0;
		}
	}
  
	return -1;
}

int32_t TIS_receive_byte(void * properties, uint32_t timeout) {
	clock_t endwait;
	char data;
	
  	endwait = clock() + timeout * CLOCKS_PER_SEC ;
  	while (clock() < endwait) {
		if (((TIS_properties*)properties)->receive_data(((TIS_properties*)properties)->serial_struct, &data, 1) == TIS_ERROR_SUCCESS) {
			return ((int32_t)data) & 0xff;
		}
	}
	return -1;
}

void TIS_flush_TX(void * properties) {
	((TIS_properties*)properties)->flush_TX(((TIS_properties*)properties)->serial_struct);
}

bool TIS_wait_data(void * properties, uint32_t timeout) {
	if (((TIS_properties*)properties)->wait_data(((TIS_properties*)properties)->serial_struct, timeout) == TIS_ERROR_SUCCESS) {
		return TRUE;
	} else {
		return FALSE;
	}
}

int64_t TIS_RUDICS_get_current_file_progress() {
	return TIS_RUDICS_current_file_progress;
}

void TIS_progress(void * not_used, int64_t progress) {
	TIS_RUDICS_current_file_progress = progress;
}

void TIS_file_received(void * properties, int8_t* name) {
	FILE * copy;
	int8_t copy_path[MAXPATHLEN];
	FILE * received;
	int8_t received_path[MAXPATHLEN];
	char buffer[BUFSIZ] = {'\0'} ;
	size_t len = 0;
	
	//Génère les chemins
	sprintf(copy_path, "%s"PATH_SEPARATOR"%s", ((TIS_properties*)properties)->receive_folder,  name);
	sprintf(received_path, "%s"PATH_SEPARATOR"%s", ((TIS_properties*)properties)->rudics_temporary_folder,  name);
	
	//Ouvre les fichiers
    copy = fopen(copy_path, "wb");
    received = fopen(received_path, "rb");
    
    if ((copy != NULL) && (received != NULL)) {
       
       	//Copie le contenue du fichier
        while ((len = fread( buffer, BUFSIZ, 1, received)) > 0) {
            fwrite( buffer, BUFSIZ, 1, copy);
        }
        
    	//Ferme les fichiers
        fclose(copy);
        fclose(received);
    
    	//Supprimer le fichier d'origine
        remove(received_path);
    }
}

