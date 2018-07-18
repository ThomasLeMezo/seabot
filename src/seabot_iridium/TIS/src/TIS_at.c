#include "TIS_at.h"
#include "TIS_errors.h"
#include <string.h>

int32_t TIS_AT_SBDD(TIS_properties * properties, bool MO_buffer, bool MT_buffer) {
    int32_t result;
    uint8_t data[18];

    //Si aucun buffer ne doit être vidé, termine la fonction
    if ((MO_buffer == FALSE) && (MT_buffer == FALSE)) {
        return TIS_ERROR_SUCCESS;
    }

    //Vide le tampon de réception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+SBDD", 7);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Envoi le paramètre
    if ((MO_buffer == TRUE) && (MT_buffer == FALSE)) {
        data[0] = '0';
    } else if ((MO_buffer == FALSE) && (MT_buffer == TRUE)) {
        data[0] = '1';
    } else	if ((MO_buffer == TRUE) && (MT_buffer == TRUE)) {
        data[0] = '2';
    }
    result = properties->send_data(properties->serial_struct, data, 1);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Termine la commande
    result = properties->send_data(properties->serial_struct, "\r", 1);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //R?cup?re la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 18);
    if ((result == TIS_ERROR_SUCCESS) && (data[11] == '1')) {
        return TIS_ERROR_UNKNOWN;
    } else if ((result != TIS_ERROR_SUCCESS) || (data[11] != '0') || (data[16] != 'O') || (data[17] != 'K')) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_SBDWB(TIS_properties * properties, uint8_t * message, int32_t count) {
    int32_t result;
    uint8_t data[18];
    int16_t check_sum;
    int32_t i;

    union uCheck
    {
        int16_t v;
        uint8_t c[2];
    } check;

    //Detect si le message est trop long
    if (count > properties->modem.sbd_size_max) {
        return TIS_ERROR_OVERFLOW;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+SBDWB=", 9);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Envoi la taille des donn?es
    sprintf(data,"%04" PRId32 "", count);
    result = properties->send_data(properties->serial_struct, data, 4);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Envoi la fin de la commande
    result = properties->send_data(properties->serial_struct, "\r", 1);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //R?cup?re la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 18);
    if ((result != TIS_ERROR_SUCCESS) || (data[16] != 'R') || (data[17] != 'E')) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Calcul la somme de contr?le
    check_sum = 0;
    for (i = 0; i < count; i++) {
        check_sum += message[i];
    }

    //Envoie les donn?es
    result = properties->send_data(properties->serial_struct, message, count);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Envoie la somme de contr?le
    #if BYTE_ORDER == LITTLE_ENDIAN
        check.v = check_sum;

        result = properties->send_data(properties->serial_struct, &(check.c[1]), 1);
        result = properties->send_data(properties->serial_struct, &(check.c[0]), 1);
    #else
        result = properties->send_data(properties->serial_struct, (uint8_t *)(&check_sum), 2);
    #endif

    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Re?oit le retour de la commande
    result = properties->receive_data(properties->serial_struct, data, 14);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Traite le retour de la commande
    if (data[7] == '1') {
        return TIS_ERROR_TIMEOUT;
    } else if  (data[7] == '2') {
        return TIS_ERROR_CHECK_SUM;
    } else if  (data[7] == '3') {
        return TIS_ERROR_OVERFLOW;
    } else if  (data[7] == 'E') {
        return TIS_ERROR_UNKNOWN;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_SBDRB(TIS_properties * properties, uint8_t * message, int32_t * MT_length) {
    int32_t result;
    uint8_t data[9];
    int16_t check_sum;
    int16_t check_sum_calc;
    int32_t i;

    union uCheck
    {
        int16_t v;
        uint8_t c[2];
    } check;

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+SBDRB\r", 9);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Vide le tampon de reception de la commande envoy?
    result = properties->receive_data(properties->serial_struct, data, 9);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //R?cup?re la taille du message
    result = properties->receive_data(properties->serial_struct, data, 2);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }
    *MT_length = (data[0] << 8) + data[1];

    //Il n'y a aucune donn?e dans le tampon
    if(*MT_length == 0) {
        //Vide le tampon de r?ception
        properties->flush_RX(properties->serial_struct);

        return TIS_ERROR_SUCCESS;
    }


    //R?cup?re le message
    result = properties->receive_data(properties->serial_struct, message, *MT_length);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //R?cup?re le check sum
    #if BYTE_ORDER == LITTLE_ENDIAN
        result = properties->receive_data(properties->serial_struct, &(check.c[1]), 1);
        if (result != TIS_ERROR_SUCCESS) {
            return TIS_ERROR_SERIAL_ERROR;
        }
        result = properties->receive_data(properties->serial_struct, &(check.c[0]), 1);
        if (result != TIS_ERROR_SUCCESS) {
            return TIS_ERROR_SERIAL_ERROR;
        }

        check_sum = check.v;
    #else
        result = properties->receive_data(properties->serial_struct, (uint8_t *)(&check_sum), 2);
        if (result != TIS_ERROR_SUCCESS) {
            return TIS_ERROR_SERIAL_ERROR;
        }
    #endif

    //V?rifie le CRC
    check_sum_calc = 0;
    for (i = 0; i < *MT_length; i++) {
        check_sum_calc += message[i];
    }
    if (check_sum_calc != check_sum) {
        return TIS_ERROR_CHECK_SUM;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_SBDI(TIS_properties * properties, int32_t * MO_status, int32_t * MT_status, int32_t * MT_queued) {
    uint8_t data[31];
    int32_t result;
    int32_t i;
    int32_t MO_msn; //Permet le traitement de donn?es, sa valeur est ignor? ensuite
    int32_t MT_length; //Permet le traitement de donn?es, sa valeur est ignor? ensuite
    int32_t MT_msn; //Permet le traitement de donn?es, sa valeur est ignor? ensuite

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+SBDI\r", 8);
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Vide le tampon de reception de la commande
    result = properties->receive_data(properties->serial_struct, data, 8);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Attend la r?ponse
    result = properties->wait_data(properties->serial_struct, 60);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de reception du d?but de la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 8);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }
    //R?cup?re la r?ponse
    i = 0;
    do {
        result = properties->receive_data(properties->serial_struct, data + i, 1);
        if (result != TIS_ERROR_SUCCESS) {
            return TIS_ERROR_SERIAL_ERROR;
        }
    } while ((data[i++] != '\r') && (i < 30));

    sscanf(data, "%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 "\r", MO_status, &MO_msn, MT_status, &MT_msn,& MT_length, MT_queued);

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_D(TIS_properties * properties, uint64_t number) {
    int32_t result;
    uint8_t data[26];

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //G?n?re la commande
    sprintf(data, "ATD00%" PRIu64 "\r", number);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, data, strlen(data));
    if (result != TIS_ERROR_SUCCESS) {
        return result;
    }

    //Vide le tampon de reception de la commande
    result = properties->receive_data(properties->serial_struct, data, strlen(data));
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Attend la r?ponse
    result = properties->wait_data(properties->serial_struct, 60);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }
    //R?cup?re la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 6);
    printf("data[0] = %c (%d)\n", data[0], data[0]);
    printf("data[1] = %c (%d)\n", data[1], data[1]);
    printf("data[2] = %c (%d)\n", data[2], data[2]);
    printf("data[3] = %c (%d)\n", data[3], data[3]);
    printf("data[4] = %c (%d)\n", data[4], data[4]);
    printf("data[5] = %c (%d)\n", data[5], data[5]);
    if ((result == TIS_ERROR_SUCCESS) && (data[2] == 'N') && (data[3] == 'O')) {
        return TIS_ERROR_DIALUP;
    } else if ((result == TIS_ERROR_SUCCESS) && (data[2] == 'B') && (data[3] == 'U') && (data[4] == 'S') && (data[5] == 'Y')) {
        return TIS_ERROR_DIALUP;
    } else if ((result != TIS_ERROR_SUCCESS) || (data[0] != 'O') || (data[1] != 'K')) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_CPIN(TIS_properties * properties, uint8_t * pin) {
    int32_t result;
    uint8_t data[15];

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+CPIN=\"", 9);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Envoi le code PIN
    result = properties->send_data(properties->serial_struct, pin, 4);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Envoi la fin de la commande
    result = properties->send_data(properties->serial_struct, "\"\r", 2);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de reception de la commande
    result = properties->receive_data(properties->serial_struct, data, 15);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Attend la r?ponse
    result = properties->wait_data(properties->serial_struct, 10);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_CREG(TIS_properties * properties) {
    int32_t result;
    uint8_t data[14];

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+CREG?\r", 9);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de reception de la commande
    result = properties->receive_data(properties->serial_struct, data, 10);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //R?cup?re la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 14);
    if ((result != TIS_ERROR_SUCCESS) || (data[13] != '1')) {
        return TIS_ERROR_NOT_REGISTERED;
    }

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}

int32_t TIS_AT_CSQ(TIS_properties * properties, int32_t * strength) {
    int32_t result;
    uint8_t data[8];

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    //Envoi la commande
    result = properties->send_data(properties->serial_struct, "AT+CSQ\r", 7);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Vide le tampon de reception de la commande
    result = properties->receive_data(properties->serial_struct, data, 7);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //Attend la r?ponse
    result = properties->wait_data(properties->serial_struct, 30);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_SERIAL_ERROR;
    }

    //R?cup?re la r?ponse
    result = properties->receive_data(properties->serial_struct, data, 8);
    if (result != TIS_ERROR_SUCCESS) {
        return TIS_ERROR_NOT_REGISTERED;
    }

    sscanf(data + 2, "+CSQ:%" PRId32 "", strength);

    //Vide le tampon de r?ception
    properties->flush_RX(properties->serial_struct);

    return TIS_ERROR_SUCCESS;
}
