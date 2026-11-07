set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER "/tools/Xilinx/Vitis/2022.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "/tools/Xilinx/Vitis/2022.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++")
set(CMAKE_SYSROOT "/tools/Xilinx/Vitis/2022.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/../cortexa9t2hf-neon-xilinx-linux-gnueabi")
set(CMAKE_FIND_ROOT_PATH
    "${CMAKE_SYSROOT}"
    "/home/user/zybo-sdk-deps/arm")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
