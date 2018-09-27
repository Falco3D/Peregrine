cmake_minimum_required(VERSION 3.5)

# CMake needs to know this project has assembly, C, and C++
project(Peregrine_Client ASM C CXX)

# Tell CMake where to put important artifacts
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# Add the STMCube directory for the library
add_subdirectory(STMCube)

# Tell CMake about asm files
set(CMAKE_ASM_SOURCE_FILE_EXTENSIONS ".s")
set(CMAKE_ASM_OUTPUT_EXTENSION ".o")
set(CMAKE_ASM_FLAGS )

# Create our executable, with sources
add_executable(Peregrine_Client_STM32F429ZI.elf
        "src/main_task.cpp"
)

target_include_directories(Peregrine_Client_STM32F429ZI.elf PUBLIC
    "src/"
)


# Link the STMCube library
target_link_libraries(Peregrine_Client_STM32F429ZI.elf
        STMCube_STM32F429ZI
)


# Linker flags, make a map too
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.map")


# Generate the .hex and the .bin
SET(HEX_FILE "${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.hex")
SET(BIN_FILE "${CMAKE_SOURCE_DIR}/build/${CMAKE_PROJECT_NAME}.bin")
add_custom_command(TARGET Peregrine_Client_STM32F429ZI.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:Peregrine_Client_STM32F429ZI.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:Peregrine_Client_STM32F429ZI.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}"
)


# Print the size of the binary
add_custom_command(TARGET Peregrine_Client_STM32F429ZI.elf POST_BUILD
        COMMAND ${CMAKE_UTIL_SIZE} $<TARGET_FILE:Peregrine_Client_STM32F429ZI.elf>
        COMMENT "SIZE OF BINARY:"
        )
