#ifdef __MWERKS__

#include "TIS_platforms.h"
#include "TIS_errors.h"

#include <sys/stat.h>

#include <stat.h>			// PicoDOS POSIX-like File Status Definitions
#include <errno.h>
#include <string.h>

static FILE *TmpFiles[PDX_OPEN_MAX];		// C auto zeros

//Fonction fournie par Persistor Instruments car la fonction tmpfile d'origine bug
FILE * TIS_create_temporary_file() {
	short		itf;	// iterator for TmpFiles
	char		*tmp_filename;


	for (itf = 0; itf < PDX_OPEN_MAX; itf++)
		if (TmpFiles[itf] == 0)	// find empty slot
			break;

	if (itf < PDX_OPEN_MAX)	// found empty slot
		{
		tmp_filename = tmpnam(0);
		errno = 0;	// reset errno's ENOENT which tmpnam sets during its search
		TmpFiles[itf] = fopen(tmp_filename, "wb+");
		return TmpFiles[itf];
		}

	errno = ENFILE;
	return 0;

}

//Fonction fournie par Persistor Instruments car la fonction tmpfile d'origine bug
int32_t TIS_delete_temporary_file(FILE * tfp) {
	char		path[FILENAME_MAX];
	short		itf;	// iterator for TmpFiles
	struct stat buf;
	short		result;

	for (itf = 0; itf < PDX_OPEN_MAX; itf++)
		{
		if (TmpFiles[itf] != 0)	// find filled slot
			{
			if (tfp == 0 || TmpFiles[itf] == tfp)	// all or match
				{
				result = fstat(TmpFiles[itf]->handle, &buf);
				if (result != 0)
					return -1;	// could benefit from better error handling
				strncpy(path, buf.st_path, FILENAME_MAX);
				result = fclose(TmpFiles[itf]);
				if (result == 0)
					{
					result = remove(path);	// okay to delete after close
					if (result == 0)
						{
						TmpFiles[itf] = 0;	// and clear table entry
						}
					}
				else
					return -1;					
				}
			}
		}

	return 0;
}

int64_t TIS_get_file_size(uint8_t * path) {
	struct stat st;
	if (stat(path, &st) == -1) {
		return 0;
	}
	
	return st.st_size;
}

bool TIS_folder_empty(uint8_t * path) {
	DIRENT de;
	short err;
	
	//Parcourt le dossier
	if ((err = DIRFindFirst(path, &de)) != dsdEndOfDir) {
		do	{
			if (err != 0) {
				break;
			}
			if (de.d_name[0] == '.') {
				continue;
			}
			return FALSE; //Un élément a été trouvé
		} while ((err = DIRFindNext(&de)) != dsdEndOfDir);
	}
	return TRUE;
}

#endif
