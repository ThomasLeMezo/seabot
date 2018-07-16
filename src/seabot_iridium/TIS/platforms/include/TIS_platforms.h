/***********************************************************************//**
 * \file  TIS_platforms.h
 * \brief Ensemble des éléments spécifiques à une plateforme utilisés par TIS.
 *
 * Les éléments spécifiques à une plateforme sont principalement les types variables, la librairie utilise
 * les types POSIX, il faut donc les redéfinir sur les plateforme qui ne sont pas POSIX.
 *
 * Les implémentations des fonctions sont présente dans /platforms/src.
 *
 * \author Clément Bonnet
 * \date 2011-2012
 * \copyright Laboratoire de Physique des Océans. Ce code est couvert par la license CeCILL-B.
 ***********************************************************************/

#ifndef TIS_PLATFORMS_H
#define	TIS_PLATFORMS_H

#ifdef __MWERKS__
	//SI la plateforme est Persistor CF2
	//Les commentaire Doxygen sont valables pour toutes les plateformes et sont placés sur cette plateforme car elle demande le plus de redéfinitions.
		
	#include <stdio.h>
	#include <cfxbios.h>		// Persistor BIOS and I/O Definitions
	#include <cfxpico.h>		// Persistor PicoDOS Definitions
	#include <dirent.h>		// PicoDOS POSIX-like Directory Access Defines
		
	//Ajout des types POSIX
	#ifndef POSIX_TYPES
		/***********************************************************************//**
 		* \def POSIX_TYPES
		* \brief Indique que les types POSIX sont définis.
 		***********************************************************************/
		#define POSIX_TYPES
		
		
		/***********************************************************************//**
 		* \typedef int8_t
		* \brief Type POSIX standard : entier signé de 8 bits.
 		***********************************************************************/
		typedef char int8_t;
		
		/***********************************************************************//**
 		* \typedef uint8_t
		* \brief Type POSIX standard : entier non signé de 8 bits.
 		***********************************************************************/
		typedef unsigned char uint8_t;
		
		/***********************************************************************//**
 		* \typedef int16_t
		* \brief Type POSIX standard : entier signé de 16 bits.
 		***********************************************************************/
		typedef int int16_t;
		
		/***********************************************************************//**
 		* \typedef uint16_t
		* \brief Type POSIX standard : entier non signé de 16 bits.
 		***********************************************************************/
		typedef unsigned int uint16_t;
		
		/***********************************************************************//**
 		* \typedef int32_t
		* \brief Type POSIX standard : entier signé de 32 bits.
 		***********************************************************************/
		typedef long int32_t;
		
		/***********************************************************************//**
 		* \typedef uint32_t
		* \brief Type POSIX standard : entier non signé de 32 bits.
 		***********************************************************************/
		typedef unsigned long uint32_t;
		
		/***********************************************************************//**
 		* \typedef int64_t
		* \brief Type POSIX standard : entier signé de 64 bits.
 		***********************************************************************/
		typedef long long int64_t;
		
		/***********************************************************************//**
 		* \typedef uint64_t
		* \brief Type POSIX standard : entier non signé de 64 bits.
 		***********************************************************************/
		typedef unsigned long long uint64_t;

		/***********************************************************************//**
 		* \def PRId8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRId8	"hd"
		
		/***********************************************************************//**
 		* \def PRIi8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRIi8	"hi"
		
		/***********************************************************************//**
 		* \def PRIo8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 8.
 		***********************************************************************/
		#define PRIo8	"ho"
		
		/***********************************************************************//**
 		* \def PRIu8
		* \brief Type de printf POSIX : entier non signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRIu8	"hu"
		
		/***********************************************************************//**
 		* \def PRIx8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx8	"hx"
		
		/***********************************************************************//**
 		* \def PRIX8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX8	"hX"


		/***********************************************************************//**
 		* \def PRId16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRId16	"d"
		
		/***********************************************************************//**
 		* \def PRIi16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRIi16	"i"
		
		/***********************************************************************//**
 		* \def PRIo16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 8.
 		***********************************************************************/
		#define PRIo16	"o"
		
		/***********************************************************************//**
 		* \def PRIu16
		* \brief Type de printf POSIX : entier non signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRIu16	"u"
		
		/***********************************************************************//**
 		* \def PRIx16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx16	"x"
		
		/***********************************************************************//**
 		* \def PRIX16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX16	"X"


		/***********************************************************************//**
 		* \def PRId32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRId32	"ld"
		
		/***********************************************************************//**
 		* \def PRIi32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRIi32	"li"
		
		/***********************************************************************//**
 		* \def PRIo32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 8.
 		***********************************************************************/
		#define PRIo32	"lo"
		
		/***********************************************************************//**
 		* \def PRIu32
		* \brief Type de printf POSIX : entier non signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRIu32	"lu"
		
		/***********************************************************************//**
 		* \def PRIx32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx32	"lx"
		
		/***********************************************************************//**
 		* \def PRIX32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX32	"lX"


		/***********************************************************************//**
 		* \def PRId64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRId64	"lld"
		
		/***********************************************************************//**
 		* \def PRIi64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRIi64	"lli"
		
		/***********************************************************************//**
 		* \def PRIo64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 8.
 		***********************************************************************/
		#define PRIo64	"llo"
		
		/***********************************************************************//**
 		* \def PRIu64
		* \brief Type de printf POSIX : entier non signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRIu64	"llu"
		
		/***********************************************************************//**
 		* \def PRIx64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx64	"llx"
		
		/***********************************************************************//**
 		* \def PRIX64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX64	"llX"

	#endif
	
	#ifndef LITTLE_ENDIAN
		/***********************************************************************//**
 		* \def LITTLE_ENDIAN
		* \brief Indique que le système est en little endian.
 		***********************************************************************/
		#define LITTLE_ENDIAN 1234
	#endif
	#ifndef BIG_ENDIAN
		/***********************************************************************//**
 		* \def BIG_ENDIAN
		* \brief Indique que le système est en big endian.
 		***********************************************************************/
		#define BIG_ENDIAN    4321
	#endif
	#ifndef BYTE_ORDER
		/***********************************************************************//**
 		* \def BYTE_ORDER
		* \brief Contient l'endianness de la plateforme actuelle.
 		***********************************************************************/
		#define BYTE_ORDER BIG_ENDIAN
	#endif
	
	#ifndef MAXPATHLEN
		/***********************************************************************//**
 		* \def MAXPATHLEN
		* \brief Longueur maximal du chemin d'un fichier.
 		***********************************************************************/
		#define MAXPATHLEN 128
	#endif
	
	#ifndef PATH_SEPARATOR
		/***********************************************************************//**
 		* \def PATH_SEPARATOR
		* \brief Caractère de séparation de dossier dans les chemins.
 		***********************************************************************/
		#define PATH_SEPARATOR "\\"
	#endif
#elif defined(_MSC_VER)
	//Microsoft Visual Studio 2008

	// Disable some Visual Studio warnings.
	#ifndef CRT_SECURE_NO_DEPRECATE
		#define CRT_SECURE_NO_DEPRECATE
	#endif // CRT_SECURE_NO_DEPRECATE
	#ifndef _CRT_SECURE_NO_WARNINGS
		#define _CRT_SECURE_NO_WARNINGS
	#endif // _CRT_SECURE_NO_WARNINGS
	
	#include <stdio.h>
	#include <stdlib.h>
	#include <time.h>
	#include <sys/stat.h>
	#include <io.h>

	//Ajout des types POSIX
	#ifndef POSIX_TYPES
		/***********************************************************************//**
 		* \def POSIX_TYPES
		* \brief Indique que les types POSIX sont définis.
 		***********************************************************************/
		#define POSIX_TYPES
		
		
		/***********************************************************************//**
 		* \typedef int8_t
		* \brief Type POSIX standard : entier signé de 8 bits.
 		***********************************************************************/
		typedef char int8_t;
		
		/***********************************************************************//**
 		* \typedef uint8_t
		* \brief Type POSIX standard : entier non signé de 8 bits.
 		***********************************************************************/
		typedef unsigned char uint8_t;
		
		/***********************************************************************//**
 		* \typedef int16_t
		* \brief Type POSIX standard : entier signé de 16 bits.
 		***********************************************************************/
		typedef short int16_t;
		
		/***********************************************************************//**
 		* \typedef uint16_t
		* \brief Type POSIX standard : entier non signé de 16 bits.
 		***********************************************************************/
		typedef unsigned short uint16_t;
		
		/***********************************************************************//**
 		* \typedef int32_t
		* \brief Type POSIX standard : entier signé de 32 bits.
 		***********************************************************************/
		typedef int int32_t;
		
		/***********************************************************************//**
 		* \typedef uint32_t
		* \brief Type POSIX standard : entier non signé de 32 bits.
 		***********************************************************************/
		typedef unsigned int uint32_t;
		
		/***********************************************************************//**
 		* \typedef int64_t
		* \brief Type POSIX standard : entier signé de 64 bits.
 		***********************************************************************/
		typedef __int64 int64_t;
		
		/***********************************************************************//**
 		* \typedef uint64_t
		* \brief Type POSIX standard : entier non signé de 64 bits.
 		***********************************************************************/
		typedef unsigned __int64 uint64_t;

		/***********************************************************************//**
 		* \def PRId8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRId8	"hd"
		
		/***********************************************************************//**
 		* \def PRIi8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRIi8	"hi"
		
		/***********************************************************************//**
 		* \def PRIo8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 8.
 		***********************************************************************/
		#define PRIo8	"ho"
		
		/***********************************************************************//**
 		* \def PRIu8
		* \brief Type de printf POSIX : entier non signé de 8 bits en base 10.
 		***********************************************************************/
		#define PRIu8	"hu"
		
		/***********************************************************************//**
 		* \def PRIx8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx8	"hx"
		
		/***********************************************************************//**
 		* \def PRIX8
		* \brief Type de printf POSIX : entier signé de 8 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX8	"hX"


		/***********************************************************************//**
 		* \def PRId16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRId16	"d"
		
		/***********************************************************************//**
 		* \def PRIi16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRIi16	"i"
		
		/***********************************************************************//**
 		* \def PRIo16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 8.
 		***********************************************************************/
		#define PRIo16	"o"
		
		/***********************************************************************//**
 		* \def PRIu16
		* \brief Type de printf POSIX : entier non signé de 16 bits en base 10.
 		***********************************************************************/
		#define PRIu16	"u"
		
		/***********************************************************************//**
 		* \def PRIx16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx16	"x"
		
		/***********************************************************************//**
 		* \def PRIX16
		* \brief Type de printf POSIX : entier signé de 16 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX16	"X"


		/***********************************************************************//**
 		* \def PRId32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRId32	"ld"
		
		/***********************************************************************//**
 		* \def PRIi32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRIi32	"li"
		
		/***********************************************************************//**
 		* \def PRIo32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 8.
 		***********************************************************************/
		#define PRIo32	"lo"
		
		/***********************************************************************//**
 		* \def PRIu32
		* \brief Type de printf POSIX : entier non signé de 32 bits en base 10.
 		***********************************************************************/
		#define PRIu32	"lu"
		
		/***********************************************************************//**
 		* \def PRIx32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx32	"lx"
		
		/***********************************************************************//**
 		* \def PRIX32
		* \brief Type de printf POSIX : entier signé de 32 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX32	"lX"


		/***********************************************************************//**
 		* \def PRId64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRId64	"lld"
		
		/***********************************************************************//**
 		* \def PRIi64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRIi64	"lli"
		
		/***********************************************************************//**
 		* \def PRIo64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 8.
 		***********************************************************************/
		#define PRIo64	"llo"
		
		/***********************************************************************//**
 		* \def PRIu64
		* \brief Type de printf POSIX : entier non signé de 64 bits en base 10.
 		***********************************************************************/
		#define PRIu64	"llu"
		
		/***********************************************************************//**
 		* \def PRIx64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 16 (lettres minuscules).
 		***********************************************************************/
		#define PRIx64	"llx"
		
		/***********************************************************************//**
 		* \def PRIX64
		* \brief Type de printf POSIX : entier signé de 64 bits en base 16 (lettres majuscules).
 		***********************************************************************/
		#define PRIX64	"llX"

	#endif
	
	#ifndef LITTLE_ENDIAN
		/***********************************************************************//**
 		* \def LITTLE_ENDIAN
		* \brief Indique que le système est en little endian.
 		***********************************************************************/
		#define LITTLE_ENDIAN 1234
	#endif
	#ifndef BIG_ENDIAN
		/***********************************************************************//**
 		* \def BIG_ENDIAN
		* \brief Indique que le système est en big endian.
 		***********************************************************************/
		#define BIG_ENDIAN 4321
	#endif
	#ifndef BYTE_ORDER
		/***********************************************************************//**
 		* \def BYTE_ORDER
		* \brief Contient l'endianness de la plateforme actuelle.
 		***********************************************************************/
		#define BYTE_ORDER LITTLE_ENDIAN
	#endif
	
	#ifndef MAXPATHLEN
		/***********************************************************************//**
 		* \def MAXPATHLEN
		* \brief Longueur maximal du chemin d'un fichier.
 		***********************************************************************/
		#define MAXPATHLEN _MAX_PATH
	#endif

	#ifndef PATH_SEPARATOR
		/***********************************************************************//**
 		* \def PATH_SEPARATOR
		* \brief Caractère de séparation de dossier dans les chemins.
 		***********************************************************************/
		#define PATH_SEPARATOR "\\"
	#endif

    #ifndef __BOOL_DEFINED
		#define bool char
    #endif
#elif defined(_POSIX_C_SOURCE) || defined(__linux)
	//Plateforme compatible POSIX

	#include <stdio.h>
	#include <stdlib.h>
	#include <time.h>
	#include <stdint.h>
	#include <stdbool.h>
	#include <inttypes.h>
	#include <sys/param.h>
	#include <sys/stat.h>
	#include <sys/types.h>
	#include <dirent.h>

	#ifndef PATH_SEPARATOR
		/***********************************************************************//**
 		* \def PATH_SEPARATOR
		* \brief Caractère de séparation de dossier dans les chemins.
 		***********************************************************************/
		#define PATH_SEPARATOR "/"
	#endif
#else
	#pragma message ("TIS : Not supported platform.")
#endif

/***********************************************************************//**
* \def FALSE
* \brief Valeur booléenne correspondant à FAUX, compatible avec le type bool.
***********************************************************************/
#ifndef FALSE
#define FALSE 0
#endif // FALSE

/***********************************************************************//**
* \def TRUE
* \brief Valeur booléenne correspondant à VRAI, compatible avec le type bool.
***********************************************************************/
#ifndef TRUE
#define TRUE 1
#endif // TRUE

/***********************************************************************//**
 * \brief Crée un fichier temporaire.
 *
 * \return Un pointeur vers le descripteur de fichier du fichier temporaire crée.
 ***********************************************************************/
FILE * TIS_create_temporary_file();

/***********************************************************************//**
 * \brief Ferme un fichier temporaire, puis le supprimer
 *
 * \param[in] tfp Pointeur vers le descripteur de fichier du fichier temporaire.
 *
 * \return 0  en cas de succès.
 * \return -1 en cas d'erreur.
 ***********************************************************************/
int32_t TIS_delete_temporary_file(FILE * tfp);

/***********************************************************************//**
 * \brief Retourne la taille d'un fichier.
 *
 * \param[in] path Un pointeur vers le chemin du fichier.
 *
 * \return La taille du fichier, s'il existe.
 * \return 0, si le fichier n'existe pas.
 ***********************************************************************/
int64_t TIS_get_file_size(uint8_t * path);

/***********************************************************************//**
 * \brief Permet de savoir si un dossier est vide
 *
 * Un dossier qui n'existe pas est toujours vide.
 *
 * \param[in] path Un pointeur vers le chemin du dossier.
 *
 * \return TRUE s'il est vide.
 * \return FALSE s'il contient au moins un fichier ou un dossier.
 ***********************************************************************/
bool TIS_folder_empty(uint8_t * path);

#endif //TIS_PLATFORMS_H
