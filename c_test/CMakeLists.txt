cmake_minimum_required(VERSION 3.14)

PROJECT(c_test C)


find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthreads -ldl")

include_directories(include)
add_executable(main main.c)
target_link_libraries(main /home/ds3a/dev/state_estimation/target/debug/libstate_estimation.a)
