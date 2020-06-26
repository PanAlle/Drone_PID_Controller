cmake_minimum_required(VERSION 3.16)
project(Drone_PID_Controller C)

set(CMAKE_C_STANDARD 99)

add_executable(Drone_PID_Controller main.c drone_simulation.c)
