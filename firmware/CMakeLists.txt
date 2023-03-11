set(PLATFORM src/stm32)

if (${PLATFORM} STREQUAL src/stm32)
    set(CMAKE_TOOLCHAIN_FILE stm32-f1xx.cmake)
endif()

project(open_pnp_head C CXX ASM)

cmake_minimum_required(VERSION 3.5.0)

if (${PLATFORM} STREQUAL src/stm32)
    add_definitions(-DSTM32F103xB)
    add_definitions(-DHSE_VALUE=8000000L)

    # Add project subdirectories
    add_subdirectory(Drivers)

    #############################
    # HAL and CMSIS dependencies
    #############################
    include_directories(drivers/STM32F1xx_HAL_Driver/Inc)
    include_directories(drivers/CMSIS/Include)
    include_directories(drivers/CMSIS/Device/ST/STM32F1xx/Include)
    include_directories(include)
endif()

# Add startup source file
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c" "src/*.s")
add_executable(${PROJECT_NAME} ${SOURCES})

if (${PLATFORM} STREQUAL src/stm32)
    # Add libraries
    target_link_libraries(${PROJECT_NAME} drivers)

    set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/lib/STM32F103C8Tx_FLASH.ld)
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${shared_options} \
        -T${LINKER_SCRIPT} \
        -specs=nosys.specs \
        -lnosys \
        -lc -lm \
        -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map,--cref \
        -Wl,--gc-sections")

    set(HEX_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
    set(BIN_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)

    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${HEX_FILE}
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${BIN_FILE}
            COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")

    find_program(OPENOCD openocd)
    message(STATUS "OpenOCD is in ${OPENOCD}")

    add_custom_target(Flash COMMAND ${OPENOCD} -f ${CMAKE_SOURCE_DIR}/openocd.cfg
            -c "init; halt; program ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex verify; reset run; exit")
            #-c "init; reset; halt; flash write_image erase ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex; sleep 200; reset run; exit")

    add_dependencies(Flash ${PROJECT_NAME})
endif()

