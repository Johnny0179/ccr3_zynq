cmake_minimum_required (VERSION 3.2)

# support c++11
# add_compile_options(-std=c++11)

# set project name
set(project_name "ccr3_zynq")

project(${project_name} C CXX)

# message(status "this is binary dir " ${zynq_ps_can_binary_dir})
# message(status "this is binary dir " ${zynq_ps_can_binary_dir})

# set the compiler
set(CMAKE_C_COMPILER "arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")

# compiler: language specific flags
set(CMAKE_C_FLAGS "-Wall -O0 ")

# support c++11
set(CMAKE_CXX_STANDARD 11)

#set additional flages
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -o0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# find additional toolchain executables
# find_program(ARM_SIZE_EXECUTABLE "arm-linux-gnueabihf-size")

# add libraries
add_subdirectory(lib)

# add main project
add_subdirectory(src)