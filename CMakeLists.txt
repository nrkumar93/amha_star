cmake_minimum_required(VERSION 3.1...3.14)
project(amha_star)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-all -Wno-long-long -pedantic")

include_directories(SYSTEM include)

# Header-only ~TEMPLATED~ library. Wouldn't compile if not linked against it!!
add_library(amha_star INTERFACE)
target_include_directories(amha_star INTERFACE include/)
target_link_libraries(amha_star INTERFACE)

# Test for the header-only library. Using the sliding test puzzle
add_library(sliding_puzzle
        src/sliding_puzzle.cpp
        )
add_executable(sliding_puzzle_test src/sliding_puzzle_test.cpp)
target_link_libraries(sliding_puzzle_test sliding_puzzle amha_star)


