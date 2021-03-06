# This policy was introduced in 3.0.0 and does not allow targets named "test" or "help" ...
# Since we do not rely on cmake testing we stick to the old policy for now.
if(POLICY CMP0037)
    cmake_policy(SET CMP0037 OLD)
endif()

PROJECT(LandsatLook)

# Standard project setup.
SET(LANDSATLOOK_VERSION 0.0.1)
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_DEBUG_POSTFIX d)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(GDAL REQUIRED)

INCLUDE_DIRECTORIES(src ${GDAL_INCLUDE_DIR})
LINK_DIRECTORIES(${GDAL_LIBRARY_DIR})

ADD_LIBRARY(LandsatLook SHARED
	"src/LandsatLook.cpp"
	"src/LandsatLook.h"
	"src/ConfigFile.cpp"
	"src/ConfigFile.h"
)

ADD_EXECUTABLE(testprogram "test/test.cpp")

TARGET_LINK_LIBRARIES(LandsatLook ${GDAL_LIBRARY})
TARGET_LINK_LIBRARIES(testprogram LandsatLook)

# DOXYGEN Setup.
OPTION(BUILD_DOCUMENTATION "Enable to create build target for Doxygen documentation." OFF)
MARK_AS_ADVANCED(CLEAR BUILD_DOCUMENTATION)
IF(BUILD_DOCUMENTATION)
	FIND_PACKAGE(Doxygen REQUIRED)

	CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/doc/doxyfile.cmake
		${PROJECT_BINARY_DIR}/doc/doxyfile
		ADD_CUSTOM_TARGET(Documentation ${DOXYGEN_EXECUTABLE}
			${PROJECT_BINARY_DIR}/doc/doxyfile
		)
	)
ENDIF()

