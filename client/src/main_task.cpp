#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

osThreadId main_task_handle;

void main_task(void const * argument);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wwrite-strings"
extern "C" {

void start_main_task(void){
  osThreadDef(main_task, main_task, osPriorityNormal, 0, 128);
  main_task_handle = osThreadCreate(osThread(main_task), NULL);
}

};
#pragma GCC diagnostic pop

void main_task(void const * argument){
  for(;;)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    osDelay(200);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    osDelay(200);
  }
}





extern "C" {
static void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}
void NMI_Handler() { while (1) {}; }

//void SysTick_Handler() {while(1) {};}

void MemManage_Handler() { while (1) {}; }

void BusFault_Handler() { while (1) {}; }

void UsageFault_Handler() { while (1) {}; }

void DebugMon_Handler() { while (1) {}; }

void WWDG_IRQHandler() { while (1) {}; }

void PVD_IRQHandler() { while (1) {}; }

void TAMP_STAMP_IRQHandler() { while (1) {}; }

void RTC_WKUP_IRQHandler() { while (1) {}; }

void FLASH_IRQHandler() { while (1) {}; }

void RCC_IRQHandler() { while (1) {}; }

void EXTI0_IRQHandler() { while (1) {}; }

void EXTI1_IRQHandler() { while (1) {}; }

void EXTI2_IRQHandler() { while (1) {}; }

void EXTI3_IRQHandler() { while (1) {}; }

void EXTI4_IRQHandler() { while (1) {}; }

void DMA1_Stream1_IRQHandler() { while (1) {}; }

void DMA1_Stream2_IRQHandler() { while (1) {}; }

void DMA1_Stream3_IRQHandler() { while (1) {}; }

void DMA1_Stream6_IRQHandler() { while (1) {}; }

void ADC_IRQHandler() { while (1) {}; }

void CAN1_TX_IRQHandler() { while (1) {}; }

void CAN1_RX0_IRQHandler() { while (1) {}; }

void CAN1_RX1_IRQHandler() { while (1) {}; }

void CAN1_SCE_IRQHandler() { while (1) {}; }

void EXTI9_5_IRQHandler() { while (1) {}; }

void TIM1_BRK_TIM9_IRQHandler() { while (1) {}; }

void TIM1_UP_TIM10_IRQHandler() { while (1) {}; }

void TIM1_TRG_COM_TIM11_IRQHandler() { while (1) {}; }

void TIM1_CC_IRQHandler() { while (1) {}; }

void TIM2_IRQHandler() { while (1) {}; }

void TIM3_IRQHandler() { while (1) {}; }

void TIM4_IRQHandler() { while (1) {}; }

void I2C1_EV_IRQHandler() { while (1) {}; }

void I2C1_ER_IRQHandler() { while (1) {}; }

void I2C2_EV_IRQHandler() { while (1) {}; }

void I2C2_ER_IRQHandler() { while (1) {}; }

void SPI1_IRQHandler() { while (1) {}; }

void SPI2_IRQHandler() { while (1) {}; }

void USART1_IRQHandler() { while (1) {}; }

void USART2_IRQHandler() { while (1) {}; }

void USART3_IRQHandler() { while (1) {}; }

void EXTI15_10_IRQHandler() { while (1) {}; }

void RTC_Alarm_IRQHandler() { while (1) {}; }

void OTG_FS_WKUP_IRQHandler() { while (1) {}; }

void TIM8_BRK_TIM12_IRQHandler() { while (1) {}; }

void TIM8_TRG_COM_TIM14_IRQHandler() { while (1) {}; }

void TIM8_CC_IRQHandler() { while (1) {}; }

void DMA1_Stream7_IRQHandler() { while (1) {}; }

void FMC_IRQHandler() { while (1) {}; }

void SDIO_IRQHandler() { while (1) {}; }

void TIM5_IRQHandler() { while (1) {}; }

void SPI3_IRQHandler() { while (1) {}; }

void UART4_IRQHandler() { while (1) {}; }

void UART5_IRQHandler() { while (1) {}; }

void TIM6_DAC_IRQHandler() { while (1) {}; }

void TIM7_IRQHandler() { while (1) {}; }

void DMA2_Stream0_IRQHandler() { while (1) {}; }

void DMA2_Stream2_IRQHandler() { while (1) {}; }

void DMA2_Stream3_IRQHandler() { while (1) {}; }

void DMA2_Stream4_IRQHandler() { while (1) {}; }

void ETH_IRQHandler() { while (1) {}; }

void ETH_WKUP_IRQHandler() { while (1) {}; }

void CAN2_TX_IRQHandler() { while (1) {}; }

void CAN2_RX0_IRQHandler() { while (1) {}; }

void CAN2_RX1_IRQHandler() { while (1) {}; }

void CAN2_SCE_IRQHandler() { while (1) {}; }

void DMA2_Stream5_IRQHandler() { while (1) {}; }

void DMA2_Stream6_IRQHandler() { while (1) {}; }

void DMA2_Stream7_IRQHandler() { while (1) {}; }

void USART6_IRQHandler() { while (1) {}; }

void I2C3_EV_IRQHandler() { while (1) {}; }

void I2C3_ER_IRQHandler() { while (1) {}; }

void OTG_HS_EP1_OUT_IRQHandler() { while (1) {}; }

void OTG_HS_EP1_IN_IRQHandler() { while (1) {}; }

void OTG_HS_WKUP_IRQHandler() { while (1) {}; }

void OTG_HS_IRQHandler() { while (1) {}; }

void DCMI_IRQHandler() { while (1) {}; }

void HASH_RNG_IRQHandler() { while (1) {}; }

void FPU_IRQHandler() { while (1) {}; }

void UART7_IRQHandler() { while (1) {}; }

void UART8_IRQHandler() { while (1) {}; }

void SPI4_IRQHandler() { while (1) {}; }

void SPI5_IRQHandler() { while (1) {}; }

void SPI6_IRQHandler() { while (1) {}; }

void SAI1_IRQHandler() { while (1) {}; }

void LTDC_IRQHandler() { while (1) {}; }

void LTDC_ER_IRQHandler() { while (1) {}; }

void DMA2D_IRQHandler() { while (1) {}; }

//void TIM8_UP_TIM13_IRQHandler() {while (1) {};}
};
