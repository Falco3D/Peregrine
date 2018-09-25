project(STMCube ASM)

cmake_minimum_required(VERSION 3.0)


set(CMAKE_ASM_SOURCE_FILE_EXTENSIONS ".s")
set(CMAKE_ASM_OUTPUT_EXTENSION ".o")
set(CMAKE_ASM_FLAGS )


add_library(STMCube_STM32F429ZI
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
        "Src/system_stm32f4xx.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c"
        "Middlewares/Third_Party/FreeRTOS/Source/queue.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
        "Src/stm32f4xx_hal_msp.c"
        "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
        "Middlewares/Third_Party/FreeRTOS/Source/timers.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
        "Src/tim.c"
        "Src/gpio.c"
        "Src/dma.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
        "Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c"
        "Src/usbd_desc.c"
        "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_wwdg.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
        "Middlewares/Third_Party/FreeRTOS/Source/list.c"
        "Src/wwdg.c"
        "Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c"
        "Src/stm32f4xx_it.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c"
        "Src/adc.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
        "Src/freertos.c"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
        "Src/usbd_conf.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c"
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
        "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
        "Src/i2c.c"
        "Src/usb_device.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c"
        "Src/usart.c"
        "Src/usbd_cdc_if.c"
        "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c"
        "Src/iwdg.c"
        "Src/can.c"
        "Src/spi.c"
        "Src/crc.c"
        "Src/system.c"
        "startup/startup_stm32f429xx.s"
)

target_compile_definitions(STMCube_STM32F429ZI PUBLIC
        "USE_HAL_DRIVER"
        "STM32F429xx"
)

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
