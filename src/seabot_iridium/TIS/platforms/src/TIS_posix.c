#if defined(_POSIX_C_SOURCE) || defined(__linux)

#include "TIS_platforms.h"

FILE * TIS_create_temporary_file() {
	return tmpfile();
}

int32_t TIS_delete_temporary_file(FILE * tfp) {
	return fclose(tfp);
}

int64_t TIS_get_file_size(uint8_t * path) {
	struct stat st;
	if (stat((char*)path, &st) == -1) {
		return 0;
	}
	
	return st.st_size;
}

bool TIS_folder_empty(uint8_t * path) {
	//TODO
	(void)(path);
	return TRUE;
/*
	DIR* dir = opendir((char*)path);
	if (dir == NULL) 
	{
		return TRUE;
	}
	// There are always "." and ".." in a directory.
	if (readdir(dir) == NULL)
	{
		closedir(dir);
		return TRUE;
	}
	if (readdir(dir) == NULL)
	{
		closedir(dir);
		return TRUE;
	}
	if (readdir(dir) == NULL)
	{
		closedir(dir);
		return TRUE;
	}
	closedir(dir);

	return FALSE;
*/
}

#endif
