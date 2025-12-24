/*

The MIT License (MIT)

Copyright (c) 2019 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <stdint.h>
#include "hal_include.h"

void NMI_Handler(void)
{
	__asm__ ("BKPT");
	while (1);
}

void HardFault_Handler(void)
{
	__asm__ ("BKPT");
	while (1);
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

extern PCD_HandleTypeDef hpcd_USB_FS;
void USB_Handler(void)
{
	HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void Default_Handler(void)
{
	__asm__ ("BKPT");
	while (1);
}

extern void Reset_Handler(void);

typedef void (*pFunc)(void);
extern uint32_t __StackTop;

#if defined(STM32F0)
__attribute__((used, section(".vectors")))
const pFunc InterruptVectorTable[48] = {
	(pFunc)(&__StackTop), // initial stack pointer
	Reset_Handler, // reset handler
	NMI_Handler, // -14: NMI
	HardFault_Handler, // -13: HardFault
	0,                    // -12: MemManage_Handler
	0,                    // -11: BusFault_Handler
	0,                    // -10: UsageFault_Handler
	0,                    //
	0,                    //
	0,                    //
	0,                    //
	0,                    // -5: SVC_Handler
	0,                    // -4: DebugMon_Handler
	0,                    //
	0, // -2: PendSV
	SysTick_Handler, // -1: SysTick
// External Interrupts
	0, // int 0: WWDG
	0, // int 1: PVD
	0, // int 2: RTC
	0, // int 3: FLASH
	0, // int 4: RCC_CRS
	0, // int 5: EXTI0_1
	0, // int 6: EXTI2_3
	0, // int 7: EXTI4_15
	0, // int 8: TSC
	0, // int 9: DMA_CH1
	0, // int 10: DMA_CH2_3
	0, // int 11: DMA_CH4_5_6_7
	0, // int 12: ADC_COMP
	0, // int 13: TIM1_BRK_UP_TRG_COM
	0, // int 14: TIM1_CC
	0, // int 15: TIM2
	0, // int 16: TIM3
	0, // int 17: TIM6_DAC
	0, // int 18: TIM7
	0, // int 19: TIM14
	0, // int 20: TIM15
	0, // int 21: TIM16
	0, // int 22: TIM17
	0, // int 23: I2C1
	0, // int 24: I2C2
	0, // int 25: SPI1
	0, // int 26: SPI2
	0, // int 27: USART1
	0, // int 28: USART2
	0, // int 29: USART3_4
	0, // int 30: CEC_CAN
	USB_Handler, // int 31: USB
};

#elif defined(STM32F4)
__attribute__((used, section(".vectors")))
const pFunc InterruptVectorTable[84] = {
	(pFunc)(&__StackTop), // initial stack pointer
	Reset_Handler,        // reset handler
	NMI_Handler,          // -14: NMI
	HardFault_Handler,    // -13: HardFault
	0,                    // -12: MemManage_Handler
	0,                    // -11: BusFault_Handler
	0,                    // -10: UsageFault_Handler
	0,                    //
	0,                    //
	0,                    //
	0,                    //
	0,                    // -5: SVC_Handler
	0,                    // -4: DebugMon_Handler
	0,                    //
	0,                    // -2: PendSV
	SysTick_Handler,      // -1: SysTick
// External Interrupts
	0,                    // int 0: WWDG
	0,                    // int 1: PVD
	0,                    // int 2: tamper and timestamp, EXTI line
	0,                    // int 3: RTC
	0,                    // int 4: FLASH
	0,                    // int 5: RCC
	0,                    // int 6: EXTI Line 0
	0,                    // int 7: EXTI Line 1
	0,                    // int 8: EXTI Line 2
	0,                    // int 9: EXTI Line 3
	0,                    // int 10: EXTI Line 4
	0,                    // int 11: DMA Stream 0
	0,                    // int 12: DMA Stream 1
	0,                    // int 13: DMA Stream 2
	0,                    // int 14: DMA Stream 3
	0,                    // int 15: DMA Stream 4
	0,                    // int 16: DMA Stream 5
	0,                    // int 17: DMA Stream 6
	0,                    // int 18: ADCs
	0,                    // int 19: CAN1 TX
	0,                    // int 20: CAN1 RX0
	0,                    // int 21: CAN1 RX1
	0,                    // int 22: CAN1 SCE
	0,                    // int 23: External Line [9:5]s
	0,                    // int 24: TIM1 Break and TIM9
	0,                    // int 25: TIM1 Update and TIM10
	0,                    // int 26: TIM1 Trigger and Commutation and TIM11
	0,                    // int 27: TIM1 Capture Compare
	0,                    // int 28: TIM2
	0,                    // int 29: TIM3
	0,                    // int 30: TIM4
	0,                    // int 31: I2C1 Event
	0,                    // int 32: I2C1 Error
	0,                    // int 33: I2C2 Event
	0,                    // int 34: I2C2 Error
	0,                    // int 35: SPI1
	0,                    // int 36: SPI2
	0,                    // int 36: USART1
	0,                    // int 37: USART2
	0,                    // int 38: USART3
	0,                    // int 39: External Line [15:10]s
	0,                    // int 40: RTC Alarm (A and B), EXTI Line
	0,                    // int 41: USB OTG FS Wakeup, EXTI Line
	0,                    // int 42: TIM8 Break and TIM12
	0,                    // int 43: TIM8 Update and TIM13
	0,                    // int 44: TIM8 Trigger and Commutation and TIM14
	0,                    // int 45: TIM8 Capture Compare
	0,                    // int 46: DMA1 Stream7
	0,                    // int 47: FSMC
	0,                    // int 48: SDIO
	0,                    // int 49: TIM5
	0,                    // int 50: SPI3
	0,                    // int 51: UART4
	0,                    // int 52: UART5
	0,                    // int 53: TIM6 and DAC1&2 underrun errors
	0,                    // int 54: TIM7
	0,                    // int 55: DMA2 Stream 0
	0,                    // int 56: DMA2 Stream 1
	0,                    // int 57: DMA2 Stream 2
	0,                    // int 58: DMA2 Stream 3
	0,                    // int 59: DMA2 Stream 4
	0,                    // int 60: Ethernet
	0,                    // int 61: Ethernet Wakeup, EXTI Line
	0,                    // int 62: CAN2 TX
	0,                    // int 63: CAN2 RX0
	0,                    // int 64: CAN2 RX1
	0,                    // int 65: CAN2 SCE
	USB_Handler,          // int 66: USB OTG FS
	// don't need to define any interrupts after this one
};
#elif defined(STM32G0)
__attribute__((used, section(".vectors")))
const pFunc InterruptVectorTable[48] = {
	(pFunc)(&__StackTop), // initial stack pointer
	Reset_Handler,        // reset handler
	NMI_Handler,          // -14: NMI
	HardFault_Handler,    // -13: HardFault
	0,                    // -12: MemManage_Handler
	0,                    // -11: BusFault_Handler
	0,                    // -10: UsageFault_Handler
	0,                    //
	0,                    //
	0,                    //
	0,                    //
	0,                    // -5: SVC_Handler
	0,                    // -4: DebugMon_Handler
	0,                    //
	0,                    // -2: PendSV
	SysTick_Handler,      // -1: SysTick
// External Interrupts
	0,                    /* Window WatchDog              */
	0,                    /* PVD through EXTI Line detect */
	0,                    /* RTC through the EXTI line    */
	0,                    /* FLASH                        */
	0,                    /* RCC & CRS                    */
	0,                    /* EXTI Line 0 and 1            */
	0,                    /* EXTI Line 2 and 3            */
	0,                    /* EXTI Line 4 to 15            */
	USB_Handler,          /* USB, UCPD1, UCPD2            */
	0,                    /* DMA1 Channel 1               */
	0,                    /* DMA1 Channel 2 and Channel 3 */
	0,                    /* DMA1 Ch4 to Ch7, DMA2 Ch1 to Ch5, DMAMUX1 overrun */
	0,                    /* ADC1, COMP1 and COMP2         */
	0,                    /* TIM1 Break, Update, Trigger and Commutation */
	0,                    /* TIM1 Capture Compare         */
	0,                    /* TIM2                         */
	0,                    /* TIM3, TIM4                   */
	0,                    /* TIM6, DAC and LPTIM1         */
	0,                    /* TIM7 and LPTIM2              */
	0,                    /* TIM14                        */
	0,                    /* TIM15                        */
	0,                    /* TIM16 & FDCAN1_IT0 & FDCAN2_IT0 */
	0,                    /* TIM17 & FDCAN1_IT1 & FDCAN2_IT1 */
	0,                    /* I2C1                         */
	0,                    /* I2C2, I2C3                   */
	0,                    /* SPI1                         */
	0,                    /* SPI2, SPI3                   */
	0,                    /* USART1                       */
	0,                    /* USART2 & LPUART2             */
	0,                    /* USART3, USART4, USART5, USART6, LPUART1   */
	0,                    /* CEC                          */
	// don't need to define any interrupts after this one
};
#elif defined(STM32G4)
__attribute__((used, section(".vectors")))
const pFunc InterruptVectorTable[102] = {
	(pFunc)(&__StackTop), // initial stack pointer
	Reset_Handler,        // reset handler
	NMI_Handler,          // -14: NMI
	HardFault_Handler,    // -13: HardFault
	0,                    // -12: MemManage_Handler
	0,                    // -11: BusFault_Handler
	0,                    // -10: UsageFault_Handler
	0,                    //
	0,                    //
	0,                    //
	0,                    //
	0,                    // -5: SVC_Handler
	0,                    // -4: DebugMon_Handler
	0,                    //
	0,                    // -2: PendSV
	SysTick_Handler,      // -1: SysTick
// External Interrupts
	0,                    /* 0: WWDG */
	0,                    /* 1: PVD_PVM */
	0,                    /* 2: RTC_TAMP_LSECSS */
	0,                    /* 3: RTC_WKUP */
	0,                    /* 4: FLASH */
	0,                    /* 5: RCC */
	0,                    /* 6: EXTI0 */
	0,                    /* 7: EXTI1 */
	0,                    /* 8: EXTI2 */
	0,                    /* 9: EXTI3 */
	0,                    /* 10: EXTI4 */
	0,                    /* 11: DMA1_Channel1 */
	0,                    /* 12: DMA1_Channel2 */
	0,                    /* 13: DMA1_Channel3 */
	0,                    /* 14: DMA1_Channel4 */
	0,                    /* 15: DMA1_Channel5 */
	0,                    /* 16: DMA1_Channel6 */
	0,                    /* 17: Reserved */
	0,                    /* 18: ADC1_2 */
	0,                    /* 19: USB_HP */
	USB_Handler,          /* 20: USB_LP */
	0,                    /* 21: FDCAN1_IT0 */
	0,                    /* 22: FDCAN1_IT1 */
	0,                    /* 23: EXTI9_5 */
	0,                    /* 24: TIM1_BRK_TIM15 */
	0,                    /* 25: TIM1_UP_TIM16 */
	0,                    /* 26: TIM1_TRG_COM_TIM17 */
	0,                    /* 27: TIM1_CC */
	0,                    /* 28: TIM2 */
	0,                    /* 29: TIM3 */
	0,                    /* 30: TIM4 */
	0,                    /* 31: I2C1_EV */
	0,                    /* 32: I2C1_ER */
	0,                    /* 33: I2C2_EV */
	0,                    /* 34: I2C2_ER */
	0,                    /* 35: SPI1 */
	0,                    /* 36: SPI2 */
	0,                    /* 37: USART1 */
	0,                    /* 38: USART2 */
	0,                    /* 39: USART3 */
	0,                    /* 40: EXTI15_10 */
	0,                    /* 41: RTC_Alarm */
	0,                    /* 42: USBWakeUp */
	0,                    /* 43: TIM8_BRK */
	0,                    /* 44: TIM8_UP */
	0,                    /* 45: TIM8_TRG_COM */
	0,                    /* 46: TIM8_CC */
	0,                    /* 47: Reserved */
	0,                    /* 48: Reserved */
	0,                    /* 49: LPTIM1 */
	0,                    /* 50: Reserved */
	0,                    /* 51: SPI3 */
	0,                    /* 52: UART4 */
	0,                    /* 53: Reserved */
	0,                    /* 54: TIM6_DAC */
	0,                    /* 55: TIM7 */
	0,                    /* 56: DMA2_Channel1 */
	0,                    /* 57: DMA2_Channel2 */
	0,                    /* 58: DMA2_Channel3 */
	0,                    /* 59: DMA2_Channel4 */
	0,                    /* 60: DMA2_Channel5 */
	0,                    /* 61: Reserved */
	0,                    /* 62: Reserved */
	0,                    /* 63: UCPD1 */
	0,                    /* 64: COMP1_2_3 */
	0,                    /* 65: COMP4 */
	0,                    /* 66-84: Reserved */
	// don't need to define any interrupts after this one
};
#endif
