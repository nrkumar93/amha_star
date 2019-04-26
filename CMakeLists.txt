cmake_minimum_required(VERSION 3.1...3.14)
project(amha_star)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

include_directories(include)

# Header-only ~TEMPLATED~ library. Wouldn't compile if not linked against it!!
add_library(amha_star INTERFACE)
target_include_directories(amha_star INTERFACE include/)
target_link_libraries(amha_star INTERFACE)


