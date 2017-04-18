#////////////////FEBIO///////////////////////////////////////////////////

# Search for an FEBio installation.
# -----------------------------------
set(FEBio_HOME "$ENV{FEBio_HOME}")
if(FEBio_INSTALL_DIR AND NOT "${FEBio_INSTALL_DIR}" STREQUAL "")
    set(FEBio_SEARCH_PATHS "${FEBio_INSTALL_DIR}")
elseif(FEBio_HOME)
    set(FEBio_SEARCH_PATHS "${FEBio_HOME}")
else()
    # Hunt for the installation.
    set(FEBio_SEARCH_PATHS)

    # Mac, Linux, Cygwin.
    if(UNIX)
        list(APPEND FEBio_SEARCH_PATHS /usr/local)
        # Unlikely (for when OpenSim is distributed through distro package
        # managers):
        list(APPEND FEBio_SEARCH_PATHS /usr)
    endif()

    if(APPLE)
        list(APPEND FEBio_SEARCH_PATHS /Developer)
    endif()

    # Windows 32 and 64 bit, Cygwin.
    if(WIN32)
        if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
            # 64 bit target on 64-bit Windows.
            set(PROGFILE_DIR "$ENV{ProgramW6432}")
        else()
            # Target is 32-bit on 64-bit Windows.
            set(PROGFILE_DIR "$ENV{ProgramFiles(x86)}")
            if(NOT PROGFILE_DIR)
                # On 32-bit Windows.
                set(PROGFILE_DIR "$ENV{ProgramFiles}")
            endif()
        endif()
        list(APPEND FEBio_SEARCH_PATHS ${PROGFILE_DIR})
    endif()
endif()


# FEBio_INCLUDE_DIR 
# ---------------------------------------------------
# We find OpenSim by finding sdk/include/OpenSim/OpenSim.h.
set(FEBio_INCLUDE_DIR_DOC
    "The location of FECore/FEModel.h and all FEbio headers.")


find_path(FEBio_INCLUDE_DIR
    NAMES "FECore/FEModel.h"
    PATHS ${FEBio_SEARCH_PATHS}
    PATH_SUFFIXES "sdk/include" "febio-2.5.0/sdk/include" "Febio-2.5.0/sdk/include"
    DOC ${FEBio_INCLUDE_DIR_DOC}
    )


# FEBio_ROOT_DIR
# ----------------
# Back out the root installation directory.
get_filename_component(FEBio_SDK_DIR "${FEBio_INCLUDE_DIR}" PATH)
get_filename_component(FEBio_ROOT_DIR "${FEBio_SDK_DIR}" PATH)

# FEBio_LIB_DIR and FEBio_BIN_DIR
# -----------------------------------
if(WIN32)
    set(FEBio_PLATFORM_LIB_RELPATH "sdk/lib/VS2013")
else()
    set(FEBio_PLATFORM_LIB_RELPATH "lib/VS2013")
endif()
set(FEBio_LIB_DIR ${FEBio_ROOT_DIR}/${FEBio_PLATFORM_LIB_RELPATH})
set(FEBio_BIN_DIR ${FEBio_ROOT_DIR}/bin)


# FEBio_LIBRARIES 
# ----------------------------------------------
set(FEBio_LIBRARIES_DOC "Suitable for target_link_libraries(). Contains only
    the libraries with 'feb' in their name.")

# This variables are for our purposes only; its name comes from convention:
set(FEBio_LIBRARY)

set(FEBio_LIBRARY_LIST
    FEBioHeat FEBioMech FEBioMix FEBioOpt FEBioPlot FEBioTest FEBioXML FECore NumCore)

foreach(LIB_NAME IN LISTS FEBio_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
        PATHS "${FEBio_LIB_DIR}"
        NO_DEFAULT_PATH)
   
    unset(FOUND_LIB CACHE)

    find_library(FOUND_LIB NAMES ${LIB_NAME}_d
        PATHS "${FEBio_LIB_DIR}"
        NO_DEFAULT_PATH)
    
    unset(FOUND_LIB CACHE)
endforeach()

# Wrap up
# -------
set(FEBio_INSTALL_DIR "${FEBio_ROOT_DIR}"
    CACHE PATH "The FEBio installation directory." FORCE)

# This CMake-supplied script provides standard error handling.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FEBio
    "
    Could NOT find FEBio. Try setting FEBio_INSTALL_DIR, or 
    create an environment variable FEBio_HOME."
    FEBio_INCLUDE_DIR)

# FEBio_FOUND is set automatically for us by find_package().
if(FEBio_FOUND)
    set(FEBio_INCLUDE_DIRS ${FEBio_INCLUDE_DIR})
    set(FEBio_LIBRARIES ${FEBio_LIBRARY})
 endif()

mark_as_advanced(
    FEBio_ROOT_DIR
    FEBio_INCLUDE_DIR
    FEBio_BIN_DIR
    FEBio_LIB_DIR
    FEBio_LIBRARY
)

# The following allows us to change the OpenSim installation we use.
unset(FEBio_INCLUDE_DIR CACHE)

