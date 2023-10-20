cmake_minimum_required(VERSION 3.15)

project(6dofRobotController)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

set(STM32F407_CONFIGURATION_SOURCE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/stm32f407_configuration/dependencies/system_stm32f4xx.c
    ${CMAKE_CURRENT_LIST_DIR}/stm32f407_configuration/startup/startup_stm32f40_41xxx.s
)

set(STM32F407_CONFIGURATION_INCLUDE_FILES
    ${CMAKE_CURRENT_LIST_DIR}/stm32f407_configuration
    ${CMAKE_CURRENT_LIST_DIR}/stm32f407_configuration/include
)

set(FREERTOS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/Source)

set(FREERTOS_SOURCE_FILES
    ${FREERTOS_SOURCE_DIR}/croutine.c
    ${FREERTOS_SOURCE_DIR}/event_groups.c
    ${FREERTOS_SOURCE_DIR}/list.c 
    ${FREERTOS_SOURCE_DIR}/queue.c 
    ${FREERTOS_SOURCE_DIR}/stream_buffer.c 
    ${FREERTOS_SOURCE_DIR}/tasks.c 
    ${FREERTOS_SOURCE_DIR}/timers.c 
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_SOURCE_DIR}/portable/MemMang/heap_4.c
)

set(FREERTOS_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/Source/include)

set(FREERTOS_INCLUDE_FILES
    ${FREERTOS_INCLUDE_DIR}
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM4F
)

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE})

target_sources(${EXECUTABLE} PRIVATE
    # CONFIGURATION
    ${STM32F407_CONFIGURATION_SOURCE_FILES}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp

    # DRIVERS
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/actuators/mg996r_servo/MG996R.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/sensors/lis3dh/LIS3DH.cpp

    # PERIPHERALS
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/adc/ADC.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/exti/EXTI.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/gpio/GPIO.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/pwm/PWM.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/spi_16bit/SPI_16bit.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/sys_clock/sys_clock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/timer/Timerconfiguration.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/peripherals/usart/USART.cpp

    # SERIAL COMMS
    ${CMAKE_CURRENT_LIST_DIR}/src/comms/serial/serial_comms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/comms/serial/serial_events/serial_events.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/comms/serial/serial_dump/serial_dump.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/comms/serial/events/accel_event/accel_event.cpp

    # FREERTOS
    ${FREERTOS_SOURCE_FILES}
)

target_include_directories(${EXECUTABLE} PRIVATE
    # CONFIGURATION
    ${STM32F407_CONFIGURATION_INCLUDE_FILES}
    ${CMAKE_CURRENT_LIST_DIR}/include/configs

    ${CMAKE_CURRENT_LIST_DIR}/include

    # COMMON
    ${CMAKE_CURRENT_LIST_DIR}/include/common

    # DRIVERS
    ${CMAKE_CURRENT_LIST_DIR}/include/drivers/actuators/mg996r_servo
    ${CMAKE_CURRENT_LIST_DIR}/include/drivers/sensors/lis3dh

    # PERIPHERALS
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/adc
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/exti
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/gpio
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/pwm
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/spi
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/spi_16bit
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/sys_clock
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/timer
    ${CMAKE_CURRENT_LIST_DIR}/include/peripherals/usart

    # SERIAL COMMS
    ${CMAKE_CURRENT_LIST_DIR}/include/comms/serial
    ${CMAKE_CURRENT_LIST_DIR}/include/comms/serial/serial_events
    ${CMAKE_CURRENT_LIST_DIR}/include/comms/serial/serial_dump
    ${CMAKE_CURRENT_LIST_DIR}/include/comms/serial/events/accel_event

    # FREERTOS
    ${FREERTOS_INCLUDE_FILES}
)

target_compile_definitions(${EXECUTABLE} PRIVATE
    -DSTM32F40_41xxx
)

target_compile_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -Wall
)

target_link_options(${EXECUTABLE} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=soft
    -specs=nosys.specs
    -specs=nano.specs
    -T${CMAKE_CURRENT_LIST_DIR}/stm32f407_configuration/STM32F407.ld
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
)

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE}
)

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
)
