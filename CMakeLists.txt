cmake_minimum_required(VERSION "3.19.2")
set (CMAKE_CXX_STANDARD 11)

option(PACKAGE_PATH "path to packages (optional)" OFF)
set (GLOBALINCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

if(NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  message (WARNING "It is recommended CMAKE_BUILD_TYPE is set to Release for the best performance \(-DCMAKE_BUILD_TYPE=Release\)")
endif()

if(PACKAGE_PATH)
  get_filename_component (PACKAGE_PATH ${PACKAGE_PATH} REALPATH)
  message ("PACKAGE_PATH SET: ${PACKAGE_PATH}")
endif()

set (CMAKE_CXX_FLAGS "-Wall -Wextra")
set (CMAKE_CXX_FLAGS_DEBUG "-g")
set (CMAKE_CXX_FLAGS_RELEASE "-O3")

project (main)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)


add_executable (main src/main.cpp)


target_include_directories (main PRIVATE "${GLOBALINCLUDE_DIR}/eigen-3.4.0")


cmake_path (SET JSON_SOURCE_DIR "${GLOBALINCLUDE_DIR}/json")
cmake_path (SET JSON_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/json")
add_subdirectory (${JSON_SOURCE_DIR} ${JSON_BINARY_DIR})


target_link_libraries(main filereader)
target_link_libraries(main contrastmax)


target_link_libraries (main nlohmann_json::nlohmann_json)
target_include_directories (main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/optim/)


add_subdirectory (lib)


SET (PACKAGE_PATH OFF CACHE INTERNAL "" FORCE)
