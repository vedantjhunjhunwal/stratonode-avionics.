/* Minimal Cortex-M4F startup for STM32G474: vector table, .data/.bss init, then main(). */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void Reset_Handler(void);
void Default_Handler(void) { for (;;) { } }
void HardFault_Handler(void) { for (;;) { } } /* IWDG resets us; reset cause is logged at boot */
void SysTick_Handler(void);
void EXTI4_IRQHandler(void);

__attribute__((section(".isr_vector"), used)) static const void *const vectors[16 + 102] = {
    [0] = &_estack, [1] = Reset_Handler, [2] = Default_Handler, [3] = HardFault_Handler,
    [4 ... 14] = Default_Handler, [15] = SysTick_Handler,
    [16 ... 16 + 9] = Default_Handler,
    [16 + 10] = EXTI4_IRQHandler,
    [16 + 11 ... 16 + 101] = Default_Handler,
};

void Reset_Handler(void)
{
    uint32_t *s = &_sidata, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss;) *d++ = 0;
    main();
    for (;;) { }
}
