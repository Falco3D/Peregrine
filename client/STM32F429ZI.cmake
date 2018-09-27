# CMake doesn't do to well with arm-gcc
INCLUDE(CMakeForceCompiler)

# Make sure CMake doesn't make any bad assumptions about our system
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_SYSTEM_VERSION 1)

# Give CMake the tools
SET(CMAKE_C_COMPILER "/usr/local/bin/arm-none-eabi-gcc")
SET(CMAKE_C++_COMPILER "/usr/local/bin/arm-none-eabi-g++")
SET(CMAKE_ASM_COMPILER "/usr/local/bin/arm-none-eabi-gcc")
SET(CMAKE_UTIL_SIZE "/usr/local/bin/arm-none-eabi-size")
SET(CMAKE_UTIL_STRIP "/usr/local/bin/arm-none-eabi-strip")

# Point CMake to the linker script
SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STMCube/STM32F429ZITx_FLASH.ld)

# Set up our CPU
SET(CPU "-mcpu=cortex-m4")
SET(FPU "-mfpu=fpv4-sp-d16")
SET(FLOAT_ABI "-mfloat-abi=softfp")
SET(MCU "${CPU} -mthumb -mthumb-interwork ${FLOAT_ABI} ${FPU}")
SET(COMMON_FLAGS "${MCU} -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

# Set the basic flags
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17 -Wno-register")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT} -specs=nosys.specs ${MCU}")

# CMake thinks the compiler is broken due to no _exit symbol (but embedded systems don't exit!) so this is necessary
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs")
