# Find PROJ4
# ~~~~~~~~~
# Copyright (c) 2013, Paul Ramsey <pramsey@cleverelephant.ca>
# (based on FindGEOS.cmake by Mateusz Loskot)
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
# CMake module to search for PROJ4 library
#
# If it's found it sets PROJ4_FOUND to TRUE
# and following variables are set:
#    PROJ4_INCLUDE_DIR
#    PROJ4_LIBRARY
#

SET(PROJ4_FOUND FALSE)

IF (NOT PROJ4_INCLUDE_DIR OR NOT PROJ4_LIBRARY)

FIND_PATH(PROJ4_INCLUDE_DIR proj_api.h
    PATHS
    /usr/local/include
    /usr/include
    /opt/include
    )

FIND_LIBRARY(PROJ4_LIBRARY
    NAMES proj
    PATHS $ENV{PROJ4_HOME}
    NO_DEFAULT_PATH
    PATH_SUFFIXES lib64 lib
    )

FIND_LIBRARY(PROJ4_LIBRARY
    NAMES proj
    PATHS
    /usr/local
    /usr
    /opt/local
    /opt/csw
    /opt
    PATH_SUFFIXES lib64 lib
    )

ENDIF ()


IF (PROJ4_INCLUDE_DIR AND PROJ4_LIBRARY)
SET(PROJ4_FOUND TRUE)
ENDIF (PROJ4_INCLUDE_DIR AND PROJ4_LIBRARY)

IF (PROJ4_FOUND)
IF (NOT PROJ4_FIND_QUIETLY)
MESSAGE(STATUS "Found PROJ4: ${PROJ4_LIBRARY}")
ENDIF (NOT PROJ4_FIND_QUIETLY)
ELSE (PROJ4_FOUND)
MESSAGE(STATUS "Could not find PROJ4")
ENDIF (PROJ4_FOUND)
