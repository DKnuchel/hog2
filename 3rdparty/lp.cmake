cmake_minimum_required(VERSION 2.8.3)
# For Windows we require CMake 3.12, but this is currently not
# available for Ubuntu 18.04.

#if(NOT FAST_DOWNWARD_MAIN_CMAKELISTS_READ)
#    message(
#        FATAL_ERROR
#        "Run cmake on the CMakeLists.txt in the 'src' directory, "
#        "not the one in 'src/search'. Please delete CMakeCache.txt "
#        "from the current directory and restart cmake.")
#endif()


## == Project ==
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/cmake_modules)
include(FastDownwardMacros)

project(downward)

# TODO: use multi-line strings to break up the long string when we switch to CMake 3.
# https://stackoverflow.com/questions/7637539/how-to-split-strings-across-multiple-lines-in-cmake
option(
  USE_GLIBCXX_DEBUG
  "Enable the libstdc++ debug mode that does additional safety checks. (On Linux systems, g++ and clang++ usually use libstdc++ for the C++ library.) The checks come at a significant performance cost and should only be enabled in debug mode. Enabling them makes the binary incompatible with libraries that are not compiled with this flag, which can lead to hard-to-debug errors."
  FALSE)

fast_downward_set_compiler_flags()
fast_downward_set_linker_flags()

# Collect source files needed for the active plugins.
include(3rdparty/DownwardFiles.cmake)
add_library(downward ${PLANNER_SOURCES})
#add_executable(downward ${PLANNER_SOURCES})

## == Includes ==

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext)

## == Libraries ==

# On Linux, find the rt library for clock_gettime().
if(UNIX AND NOT APPLE)
    target_link_libraries(downward rt)
endif()

# On Windows, find the psapi library for determining peak memory.
if(WIN32)
    cmake_policy(SET CMP0074 NEW)
    target_link_libraries(downward psapi)
endif()

# If any enabled plugin requires an LP solver, compile with all
# available LP solvers. If no solvers are installed, the planner will
# still compile, but using heuristics that depend on an LP solver will
# cause an error. This behavior can be overwritten by setting the
# option USE_LP to false.
option(
  USE_LP
  "Compile with support for all LP solvers installed on this system."
  TRUE)

set(PLUGIN_LP_SOLVER_ENABLED TRUE)
set(USE_LP TRUE)

if(PLUGIN_LP_SOLVER_ENABLED AND USE_LP)
    message("OSI PATH: ${DOWNWARD_COIN_ROOT}")
    find_package(OSI OPTIONAL_COMPONENTS Cpx Clp Grb Spx)
    if(OSI_FOUND)
        message("Found OSI Library")
    endif()
    if(OSI_Cpx_FOUND)
        message("Found CPX")
    endif()
    if(OSI_FOUND AND (OSI_Cpx_FOUND OR OSI_Clp_FOUND OR OSI_Grb_FOUND OR OSI_Spx_FOUND))
        if(USE_GLIBCXX_DEBUG)
            message(
                FATAL_ERROR
                "To prevent incompatibilities, the option USE_GLIBCXX_DEBUG is "
                "not supported when an LP solver is used. See issue982 for details.")
        endif()
        if(OSI_Cpx_FOUND)
            string(TOUPPER Cpx TMP_SOLVER_UPPER_CASE)
            mark_as_advanced(TMP_SOLVER_UPPER_CASE)
            add_definitions("-D COIN_HAS_${TMP_SOLVER_UPPER_CASE}")
            include_directories(${OSI_Cpx_INCLUDE_DIRS})
            target_link_libraries(downward ${OSI_Cpx_LIBRARIES})
            message("Checkpoint")
        endif()

        # Note that basic OSI libs must be added after (!) all OSI solver libs.
        add_definitions("-D USE_LP")
        include_directories(${OSI_INCLUDE_DIRS})
        target_link_libraries(downward ${OSI_LIBRARIES})

        find_package(ZLIB REQUIRED)
        if(ZLIB_FOUND)
            message("ZLIB found")
            include_directories(${ZLIB_INCLUDE_DIRS})
            target_link_libraries(downward ${ZLIB_LIBRARIES})
        endif()
    endif()
    if(CPLEX_RUNTIME_LIBRARY)
        message("CPLEX RUNTIME found")
    endif()
    if(OSI_Cpx_FOUND AND CPLEX_RUNTIME_LIBRARY)
        message("OSI FOUND!")
        add_custom_command(TARGET downward POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            ${CPLEX_RUNTIME_LIBRARY}
            $<TARGET_FILE_DIR:downward>
        )
    endif()
    message("OSI SHOULD HAVE BEEN FOUND BY NOW!")
endif()