project(STMCube C)

cmake_minimum_required(VERSION 3.0)

SET(CMAKE_VERBOSE_MAKEFILE ON)

# Create library with sources - Manually update with new STMCube generation
add_library(STMCube_STM32F429ZI
        "Src/adc.c"
        "Src/can.c"
        "Src/crc.c"
        "Src/dma.c"
        "Src/freertos.c"
        "Src/gpio.c"
        "Src/i2c.c"
        "Src/main.c"
        "Src/spi.c"
        "Src/stm32f4xx_hal_msp.c"
        "Src/stm32f4xx_hal_timebase_TIM.c"
        "Src/stm32f4xx_it.c"
        "Src/system_stm32f4xx.c"
        "Src/tim.c"
        "Src/usart.c"
        "Src/usb_device.c"
        "Src/usbd_cdc_if.c"
        "Src/usbd_conf.c"
        "Src/usbd_desc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c"
        "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
        "Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
        "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
        "Middlewares/Third_Party/FreeRTOS/Source/list.c"
        "Middlewares/Third_Party/FreeRTOS/Source/queue.c"
        "Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
        "Middlewares/Third_Party/FreeRTOS/Source/timers.c"
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
        "startup/startup_stm32f429xx.s"
)

# Define the right things for the library
target_compile_definitions(STMCube_STM32F429ZI PUBLIC
        "USE_HAL_DRIVER"
        "STM32F429xx"
)

# Include directories
target_include_directories( STMCube_STM32F429ZI PUBLIC
        "Inc"
        "Drivers/STM32F4xx_HAL_Driver/Inc"
        "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
        "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
        "Drivers/CMSIS/Device/ST/STM32F4xx/Include"
        "Middlewares/Third_Party/FreeRTOS/Source/include"
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"
        "Drivers/CMSIS/Include"
)
