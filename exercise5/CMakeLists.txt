cmake_minimum_required(VERSION 3.13)
project(ht)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-O3 -march=native")

add_executable(ht main.cpp dijkstra.h tests.h)