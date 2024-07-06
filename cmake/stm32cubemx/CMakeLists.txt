cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE
	USE_FULL_LL_DRIVER
	USE_HAL_DRIVER
	STM32L432xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(CUBE_DRIVERS_ROOT_DIR ../../STM32CubeL4)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/CMSIS/Core/Core/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/stm32l4xx_it.c
    ../../Core/Src/stm32l4xx_hal_msp.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
    ${CUBE_DRIVERS_ROOT_DIR}/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
    ../../Core/Src/system_stm32l4xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32l432xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()

