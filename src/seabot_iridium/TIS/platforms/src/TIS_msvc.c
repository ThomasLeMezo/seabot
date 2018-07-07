#if defined(_MSC_VER)

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
	intptr_t handle = 0;
	struct _finddata_t file;
	char szPath[MAXPATHLEN+2];

	sprintf(szPath, "%s\\*", (char*)path);

	// There are always "." and ".." in a directory.
	handle = _findfirst(szPath, &file);
	if (handle == -1)
	{
		return TRUE;
	}
	if (_findnext(handle, &file) == -1)
	{
		_findclose(handle);
		return TRUE;
	}
	if (_findnext(handle, &file) == -1)
	{
		_findclose(handle);
		return TRUE;
	}
	_findclose(handle);

	return FALSE;
*/
}

#endif
