cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 11)

include(cmake/ucm.cmake)
#include(cmake/FindSFML.cmake)

project(CMakeTest VERSION 0.1)

#include_directories(${CMAKE_SOURCE_DIR}/my_lib)

#set(SFML_STATIC_LIBRARIES TRUE)
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

#find_package(SFML COMPONENTS graphics window system REQUIRED)

#ucm_add_linker_flags("-framework GLUT" "-framework OpenGL" "-framework Cocoa")

ucm_add_dirs(abstraction abstractionalgorithms algorithms dynamic environments envutil generic graph graphalgorithms search utils TO HOG_FILES RECURSIVE)
ucm_remove_files(abstraction/AbstractionSearchEnvironment.cpp algorithms/SearchEnvironment.cpp environments/WeightedMap2DEnvironment.cpp environments/PancakePuzzle.cpp generic/RandomUnit.cpp generic/GraphCheck.cpp shared/HumanUnit.cpp shared/PatrolUnit.cpp shared/PRAStarUnit.cpp shared/RewardUnit.cpp absmapalgorithms/AbsMapUnit.cpp gui/SFML_HOG.cpp shared/SearchUnit.cpp shared/SharedAMapGroup.cpp FROM HOG_FILES)
ucm_remove_directories(gui/MAC gui/web FROM HOG_FILES)
add_library(HogCore STATIC ${HOG_FILES})

target_include_directories(HogCore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/absmapalgorithms ${CMAKE_CURRENT_SOURCE_DIR}/abstraction ${CMAKE_CURRENT_SOURCE_DIR}/abstractionalgorithms ${CMAKE_CURRENT_SOURCE_DIR}/algorithms ${CMAKE_CURRENT_SOURCE_DIR}/dynamic ${CMAKE_CURRENT_SOURCE_DIR}/environments ${CMAKE_CURRENT_SOURCE_DIR}/envutil ${CMAKE_CURRENT_SOURCE_DIR}/generic ${CMAKE_CURRENT_SOURCE_DIR}/graph ${CMAKE_CURRENT_SOURCE_DIR}/graphalgorithms ${CMAKE_CURRENT_SOURCE_DIR}/grids ${CMAKE_CURRENT_SOURCE_DIR}/gui ${CMAKE_CURRENT_SOURCE_DIR}/learning ${CMAKE_CURRENT_SOURCE_DIR}/mapalgorithms ${CMAKE_CURRENT_SOURCE_DIR}/maps ${CMAKE_CURRENT_SOURCE_DIR}/shared ${CMAKE_CURRENT_SOURCE_DIR}/search ${CMAKE_CURRENT_SOURCE_DIR}/simulation ${CMAKE_CURRENT_SOURCE_DIR}/utils)

ucm_add_dirs(apps/test TO HOG_MAIN RECURSIVE)

add_executable(CMakeTest ${HOG_MAIN})

target_link_libraries(CMakeTest HogCore)

# learning maps shared