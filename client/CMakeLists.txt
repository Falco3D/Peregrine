include(CMakeForceCompiler)
project(Peregrine_Client)

cmake_minimum_required(VERSION 3.5)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/build")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

add_subdirectory(STMCube)


add_executable(Peregrine_Client_STM32F429ZI.elf
        "src/main.c"
)

#target_include_directories(Peregrine_Client_STM32F429ZI.elf PUBLIC
#    "src/"
#)


# Force CMAKE to do a static library test, otherwise it tries to use non-existant Special Purpose Registers
SET(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

target_link_libraries(Peregrine_Client_STM32F429ZI.elf
        STMCube_STM32F429ZI
)

SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.map")
SET(HEX_FILE "${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.hex")
SET(BIN_FILE "${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.bin")
add_custom_command(TARGET Peregrine_Client_STM32F429ZI.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:Peregrine_Client_STM32F429ZI.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:Peregrine_Client_STM32F429ZI.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}"
)
