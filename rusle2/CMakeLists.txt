PROJECT(RUSLE2)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/")



IF(WIN32)
    # Use a debug postfix to distinguish build products. Mostly important on
    # Windows, because linking a debug library into a release executable (or
    # vice-versa, or just mixing C++ runtime versions) on Windows will lead
    # to crashes if the libraries use the C++ runtime.
    SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Add a postfix, usually d on windows")
else()
    # On GCC, we almost certainly want to these compiler flags.
    # If you set them here then they overwrite what the user sets in the cmake UI
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
ENDIF()

#
# Setup the the lib directory on a per platform basis
#
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    include(GNUInstallDirs)
endif(CMAKE_COMPILER_IS_GNUCXX)

if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR lib)
endif(NOT DEFINED CMAKE_INSTALL_LIBDIR)

#Boost
if( WIN32 )
    add_definitions( -DBOOST_ALL_DYN_LINK )
    add_definitions( -DHAVE_BOOST )
endif()

set( BOOST_USE_STATIC_LIBS OFF )
set( BOOST_USE_MULTITHREADED ON )
set( BOOST_USE_STATIC_RUNTIME OFF )
set( BOOST_ROOT "Please specify" CACHE PATH "Root directory where Boost is installed" )
find_package( Boost 1.49.0 COMPONENTS regex system thread date_time filesystem)
mark_as_advanced(CLEAR BOOST_ROOT)

INCLUDE(CMakeMacros)
INCLUDE(FindMapnik)

FIND_PACKAGE(GDAL REQUIRED)

if(NOT WIN32)
    find_package(Pixman)
    find_package(Freetype)
    find_package(Cairo)
endif()

find_package(ICU)

if(WIN32)
    string(REPLACE " " " -D" MAPNIK_DEFINES ${MAPNIK_DEFINES})
    set(MAPNIK_DEFINES "-D${MAPNIK_DEFINES}")
endif()
add_definitions( ${MAPNIK_DEFINES} )

SET(RUSLE2_SRC_FILES
    "src/rusle2/RUSLE2.cpp"
)

SET(FIELDGRIDGEN_SRC_FILES
    "src/FieldGridGen/FieldGridGen.cpp"
)

SET(RUSLE2_HEADER_FILES
    "src/rusle2/RUSLE2.h"
)

SET(FIELDGRIDGEN_HEADER_FILES
    "src/FieldGridGen/FieldGridGen.h"
)

SOURCE_GROUP("Header Files" FILES ${RUSLE2_HEADER_FILES} ${FIELDGRIDGEN_HEADER_FILES})
SOURCE_GROUP("Source Files" FILES ${RUSLE2_SRC_FILES} ${FIELDGRIDGEN_SRC_FILES})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/src
    ${Boost_INCLUDE_DIRS}
    ${GDAL_INCLUDE_DIR}
    ${MAPNIK_INCLUDE_DIRS}
)

if( ICU_FOUND )
   INCLUDE_DIRECTORIES( ${ICU_INCLUDE_DIRS} )
endif( ICU_FOUND )

if(NOT WIN32)
    INCLUDE_DIRECTORIES(
       ${CAIRO_INCLUDE_DIRS}
    )
endif()

LINK_DIRECTORIES(${GDAL_LIB_DIR})

ADD_LIBRARY(RUSLE2 SHARED ${RUSLE2_HEADER_FILES} ${RUSLE2_SRC_FILES})
ADD_LIBRARY(FieldGridGen SHARED ${FIELDGRIDGEN_HEADER_FILES} ${FIELDGRIDGEN_SRC_FILES})

ADD_EXECUTABLE(test "src/rusle2/test.cpp")
ADD_EXECUTABLE(fieldgridgentest "src/FieldGridGen/test.cpp")

TARGET_LINK_LIBRARIES(RUSLE2 ${GDAL_LIBRARY} )

TARGET_LINK_LIBRARIES(FieldGridGen ${GDAL_LIBRARY})
TARGET_LINK_LIBRARIES(FieldGridGen ${MAPNIK_LIBRARIES})
TARGET_LINK_LIBRARIES(FieldGridGen ${Boost_LIBRARIES})
TARGET_LINK_LIBRARIES(FieldGridGen RUSLE2)

TARGET_LINK_LIBRARIES(test RUSLE2)
# this might need RUSLE2 as well
TARGET_LINK_LIBRARIES(fieldgridgentest FieldGridGen RUSLE2)

# The archive/runtime part is so that the lib/dll pair goes into lib and bin
# respectively on Windows.
INSTALL(TARGETS RUSLE2
    EXPORT RUSLE2-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

INSTALL(TARGETS FieldGridGen
    EXPORT RUSLE2-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(FILES ${RUSLE2_HEADER_FILES}
        DESTINATION include/rusle2
        COMPONENT development)

install(FILES ${FIELDGRIDGEN_HEADER_FILES}
        DESTINATION include/FieldGridGen
        COMPONENT development)

CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/uninstall.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
    IMMEDIATE @ONLY
)

ADD_CUSTOM_TARGET(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
)

# Support for doxygen, if found; this will expose a "make doc" target.
FIND_PACKAGE(Doxygen)

IF(DOXYGEN_FOUND)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    ADD_CUSTOM_TARGET(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
ENDIF(DOXYGEN_FOUND)

set(VES_EXPORT_LIBRARY_TARGETS RUSLE2 FieldGridGen)
#
# Create and install all of the auto find tools
# =============================================

# Add all targets to the build-tree export set
if( EXISTS "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}LibraryDepends.cmake" )
    export(TARGETS ${VES_EXPORT_LIBRARY_TARGETS} APPEND
        FILE "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}LibraryDepends.cmake")
else()
    export(TARGETS ${VES_EXPORT_LIBRARY_TARGETS}
        FILE "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}LibraryDepends.cmake")
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create a storytellerBuildTreeSettings.cmake file for the use from the build tree
configure_file(${PROJECT_SOURCE_DIR}/CMakeModules/packageBuildTreeSettings.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}BuildTreeSettings.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/CMakeModules/packageConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/CMakeModules/Usepackage.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/Use${PROJECT_NAME}.cmake" @ONLY)

include(CMakePackageConfigHelpers)
# Create the packakgeConfig.cmake and packageConfigVersion files
configure_package_config_file( 
    ${PROJECT_SOURCE_DIR}/CMakeModules/packageConfig.cmake.in 
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PATH_VARS ${VES_CONFIGURE_VARS})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}Config.cmake 
              ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}ConfigVersion.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/Use${PROJECT_NAME}.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT development)

# Install the CMake find script for storyteller
configure_file(${PROJECT_SOURCE_DIR}/CMakeModules/Findpackage.cmake.in
    "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake" @ONLY)
install(FILES "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake"
    DESTINATION "share/${PROJECT_NAME}/extra"
    COMPONENT development)

# Install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}-targets DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)

