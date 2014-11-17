PROJECT(LandsatLook)

# Standard project setup.
SET(LANDSATLOOK_VERSION 0.0.1)
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_DEBUG_POSTFIX d)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(GDAL REQUIRED)

INCLUDE_DIRECTORIES(${GDAL_INCLUDE_DIR})
LINK_DIRECTORIES(${GDAL_LIBRARY_DIR})

ADD_LIBRARY(LandsatLook SHARED
	"src/LandsatLook.cpp"
	"src/LandsatLook.h"
)

TARGET_LINK_LIBRARIES(LandsatLook ${GDAL_LIBRARY})

# DOXYGEN Setup.
OPTION(BUILD_DOCUMENTATION "Enable to create build target for Doxygen documentation." OFF)
MARK_AS_ADVANCED(CLEAR BUILD_DOCUMENTATION)
IF(BUILD_DOCUMENTATION)
	FIND_PACKAGE(Doxygen REQUIRED)
	SET(HAVE_DOT "NO")
	IF(DOXYGEN_DOT_PATH)
		SET(HAVE_DOT "YES")
	ENDIF()

	CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
		${PROJECT_BINARY_DIR}/doc/doxyfile
		ADD_CUSTOM_TARGET(Documentation ${DOXYGEN_EXECUTABLE}
			${PROJECT_BINARY_DIR}/doc/doxyfile
		)
	)
ENDIF()
