#include "iridium.h"

#include "tis.h"
#include "ros/ros.h"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

Iridium::Iridium()
{

}

int32_t UART_init(int &fd){
    fd = open( "/dev/ttyAMA0", O_RDWR| O_NONBLOCK | O_NDELAY );
    /* Error Handling */
    if ( fd < 0 ){
        ROS_WARN("[Iridium] ERROR %i opening /dev/ttyAMA0 : %s", errno, strerror(errno));
        return TIS_ERROR_SERIAL_ERROR;
    }
    else
        return TIS_ERROR_SUCCESS;
}

//int32_t UART_send_data(void *serial_struct, uint8_t *data, int32_t count)
//{
//    int iResult = SendDataIridium((IRIDIUM*)serial_struct, data, count);

//    switch (iResult)
//    {
//    case EXIT_SUCCESS:
//        return TIS_ERROR_SUCCESS;
//    case EXIT_TIMEOUT:
//        return TIS_ERROR_TIMEOUT;
//    default:
//        return TIS_ERROR_SERIAL_ERROR;
//    }
//}

//int32_t UART_receive_data(void *serial_struct, uint8_t *data, int32_t count)
//{
//    int iResult = RecvDataIridium((IRIDIUM*)serial_struct, data, count);

//    switch (iResult)
//    {
//    case EXIT_SUCCESS:
//        return TIS_ERROR_SUCCESS;
//    case EXIT_TIMEOUT:
//        return TIS_ERROR_TIMEOUT;
//    default:
//        return TIS_ERROR_SERIAL_ERROR;
//    }
//}

//int32_t UART_wait_data(void *serial_struct, uint32_t timeout)
//{
//    int iResult = WaitDataIridium((IRIDIUM*)serial_struct, timeout);

//    switch (iResult)
//    {
//    case EXIT_SUCCESS:
//        return TIS_ERROR_SUCCESS;
//    case EXIT_TIMEOUT:
//        return TIS_ERROR_TIMEOUT;
//    default:
//        return TIS_ERROR_SERIAL_ERROR;
//    }
//}

//int32_t UART_flush_TX(void *serial_struct)
//{
//    UNREFERENCED_PARAMETER(serial_struct);

//    // This function is useless...

//    return TIS_ERROR_SUCCESS;
//}

//int32_t UART_flush_RX(void *serial_struct)
//{
//    if (PurgeDataIridium((IRIDIUM*)serial_struct) != EXIT_SUCCESS)
//    {
//        return TIS_ERROR_SERIAL_ERROR;
//    }

//    return TIS_ERROR_SUCCESS;
//}

//int32_t UART_release(void *serial_struct)
//{
//    if (ReleaseIridium((IRIDIUM*)serial_struct) != EXIT_SUCCESS)
//    {
//        return TIS_ERROR_SERIAL_ERROR;
//    }

//    return TIS_ERROR_SUCCESS;
//}

//int32_t Iridium_ON()
//{
//    if (SetDeviceCurrentState("CurIridiumState.txt", 1) != EXIT_SUCCESS)
//    {
//        return TIS_ERROR_FILE_ACCESS;
//    }

//    // Time to start.
//    mSleep(STARTUP_DELAY_IRIDIUM);

//    return TIS_ERROR_SUCCESS;
//}

//int32_t Iridium_OFF()
//{
//    if (SetDeviceCurrentState("CurIridiumState.txt", 0) != EXIT_SUCCESS)
//    {
//        return TIS_ERROR_FILE_ACCESS;
//    }

//    // Time to stop.
//    mSleep(SHUTDOWN_DELAY_IRIDIUM);

//    return TIS_ERROR_SUCCESS;
//}


//Cete fonction prend en paramètre un tableau contenant des pointeurs vers le nom des fichiers à envoyer ainsi que le nombre de fichier à renvoyer.
bool Iridium::send_and_receive_data(std::vector<std::string> &files){
    TIS_properties tis; //Déclaration de la structure de la librairie
//    int i = 0;
    //Initialise la liaison série (dépend de votre plateforme et non de la librairie)
    int fd;
    UART_init(fd);

    //Initialise la structure de la librairie
    if (TIS_init(&tis,
                 "CMD",				//"CMD" est le chemin du dossier qui contiendra les fichier reçus.
                 TIS_MODEM_9602,	//Modèle du modem
                 m_imei,	//Numéro IMEI du modem
                 TIS_SERVICE_SBD,	//Service SBD
                 NULL,				//Inutile avec un modem sans carte SIM
                 0,					//Inutile en modem SBD
                 NULL,				//Inutile en modem SBD
                 files.size(),		//Nombre de fichier envoyés
                 &fd,			//Un pointeur vers la structure décrivant la liaison série
                 UART_send_data,	//Fonction utilisant les appelles système de la plateforme pour envoyer des données sur la liaison série
                 UART_receive_data, //Fonction utilisant les appelles système de la plateforme pour recevoir des données sur la liaison série
                 UART_wait_data,	//Fonction utilisant les appelles système de la plateforme pour attendre des données sur la liaison série
                 UART_flush_TX,		//Fonction utilisant les appelles système de la plateforme pour vider le tampon de sortie de la liaison série
                 UART_flush_RX		//Fonction utilisant les appelles système de la plateforme pour vider le tampon d'entrée de la liaison série
                 ) != TIS_ERROR_SUCCESS) {
        return false;
    }
//    //Ajoute les fichiers à envoyer
//    for (i = 0; i < files_count; i++) {
//        TIS_add_file_to_send(&tis, files[i]);
//    }
//    //Boucle d'envoie et de réception, s'arrête quand tous les fichier sont envoyés ou reçus
//    do {
//        int32_t result;
//        //Met le modem sous tension
//        Iridium_ON(); //(dépend de votre plateforme et non de la librairie)
//        //Lance la transmission
//        result = TIS_transmission(&tis, 10);
//        //Met le modem hors tension
//        Iridium_OFF(); //(dépend de votre plateforme et non de la librairie)
//        //Affiche des informations de diagnostiques
//        if (result != TIS_ERROR_SUCCESS) {
//            printf("Iridium : erreur %ld s'est produite pendant la transmission", result);
//        }
//        printf("Iridium : informations de diagnostiques : \n");
//        printf("\tNombre de messages SBD envoyés avec succès : %ld\n", tis.SBD_sent_without_error);
//        printf("\tNombre de messages SBD dont l'envoie a échoué : %ld\n", tis.SBD_sent_with_error);
//        printf("\tNombre de messages SBD reçues avec succès : %ld\n", tis.SBD_received_without_error);
//        printf("\tNombre de messages SBD dont la réception a échoué : %ld\n", tis.SBD_received_with_error);
//        printf("\tNombre de messages SBD en attente dans la constellation : %ld\n", tis.SBD_waiting_messages);
//        for (i = 0; i < file_count; i++) {
//            printf("Iridium : fichier %s, envoyé à %ld%\n", TIS_get_file_path(&tis, i), TIS_get_file_progress(&tis, i));
//        }
//        //Si la communication n'est pas terminée, met en veille le système en attendant un nouveau crénaux de réception
//        if ((TIS_remaining_file_to_send(&tis) != 0) || (TIS_waiting_incoming_data() == TRUE)) {
//            sleep(300); //La durée dépend de votre application, la fonction depend de votre plateforme et non de la librairie)
//        }
//        //Si il reste des fichier en cours d'envoie ou en attente dans le satellite, reboucle
//    } while ((TIS_remaining_file_to_send(&tis) != 0) || (TIS_waiting_incoming_data() == TRUE));
//    //Libère la mémoire occupée par la structure de la librairie
//    TIS_clean(&tis);
//    return TRUE;
    return true;
}
