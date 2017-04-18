CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

project(Co_SiM)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Settings
set(BASE_DIR "${PROJECT_SOURCE_DIR}")
set(INI_PATH "${PROJECT_SOURCE_DIR}/data/setup.ini")
set(CMAKE_EXTRA_INCLUDE_FILES/src/Settings.h)
configure_file("${PROJECT_SOURCE_DIR}/src/Settings.h.in" "${PROJECT_SOURCE_DIR}/src/Settings.h")
set(CMAKE_EXTRA_INCLUDE_FILES)

#################################################################################################
# Find OpenSim

find_package(OpenSim REQUIRED)
if(OpenSim_FOUND)
    message(STATUS "OpenSim found at ${OPENSIM_ROOT_DIR}")
    include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Can't find OpenSim at ${OPENSIM_ROOT_DIR}")
endif()

#################################################################################################
# subdirectories


#################################################################################################
# Find FEbio

find_package(FEBio REQUIRED)
if(FEBio_FOUND)
    message(STATUS "FEBio found at ${FEBio_ROOT_DIR}")
	message(STATUS "FEBio includes are in found at ${FEBio_INCLUDE_DIRS}")
    include_directories(${FEBio_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Can't find FEbio at ${FEBio_ROOT_DIR}")
endif()

#################################################################################################
# subdirectories

add_subdirectory(src)

