cmake_minimum_required(VERSION 3.16)
project(ventsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VENTSIM_WERROR "Treat warnings as errors" OFF)

add_library(ventsim INTERFACE)
target_include_directories(ventsim INTERFACE include)
target_compile_options(ventsim INTERFACE -Wall -Wextra -Wpedantic -Wshadow -Wconversion
                       $<$<BOOL:${VENTSIM_WERROR}>:-Werror>)

add_executable(ventsim_cli src/main.cpp)
set_target_properties(ventsim_cli PROPERTIES OUTPUT_NAME ventsim)
target_link_libraries(ventsim_cli ventsim)

enable_testing()
add_executable(test_ventsim tests/test_ventsim.cpp)
target_link_libraries(test_ventsim ventsim)
add_test(NAME test_ventsim COMMAND test_ventsim)
