INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR arm)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
#CMAKE_FORCE_C_COMPILER(/usr/local/bin/arm-none-eabi-gcc GNU)
#CMAKE_FORCE_CXX_COMPILER(/usr/local/bin/arm-none-eabi-g++ GNU)

SET(CMAKE_C_COMPILER "/usr/local/bin/arm-none-eabi-gcc")
SET(CMAKE_C++_COMPILER "/usr/local/bin/arm-none-eabi-g++")
#SET(CMAKE_LINKER "/usr/local/bin/arm-none-eabi-ld")
#SET(CMAKE_C_LINK_EXECUTABLE "<CMAKE_LINKER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
#SET(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_LINKER> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
#SET(CMAKE_ASM_LINK_EXECUTABLE "<CMAKE_LINKER> <CMAKE_ASM_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")



SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STMCube/STM32F429ZITx_FLASH.ld)

SET(CPU "-mcpu=cortex-m4")
SET(FPU "-mfpu=fpv4-sp-d16")
SET(FLOAT_ABI "-mfloat-abi=softfp")
SET(MCU "${CPU} -mthumb -mthumb-interwork ${FLOAT_ABI} ${FPU}")
SET(COMMON_FLAGS "${MCU} -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT} -specs=nosys.specs")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs")

