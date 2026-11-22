/* STM32G474CE target: register-level bring-up of the parts the flight logic needs first.
 *
 * Done here: SysTick 1 ms tick, load-switch GPIOs (PB13 radio, PB14 payload), TIM3 CH3/CH4 heater
 * PWM on PB0/PB1 (1 kHz), INA226 ALERT on PB4 -> EXTI4 power-fail interrupt, independent watchdog.
 * Not yet done (needs the board): the I2C1/I2C2, SPI1, USART1-3 and FDCAN1 peripheral drivers
 * behind hal_i2c_t / hal_spi_t; the stubs below return HAL_TIMEOUT so the app runs in its
 * sensor-fault fallback. Clock stays on the 16 MHz HSI until the HSE/PLL bring-up.
 */
#include <stdint.h>

#include "app.h"
#include "drivers.h"

#define REG(addr) (*(volatile uint32_t *)(addr))
/* RCC */
#define RCC_BASE 0x40021000u
#define RCC_AHB2ENR REG(RCC_BASE + 0x4C)
#define RCC_APB1ENR1 REG(RCC_BASE + 0x58)
#define RCC_APB2ENR REG(RCC_BASE + 0x60)
/* GPIO */
#define GPIOB_BASE 0x48000400u
#define GPIOB_MODER REG(GPIOB_BASE + 0x00)
#define GPIOB_PUPDR REG(GPIOB_BASE + 0x0C)
#define GPIOB_BSRR REG(GPIOB_BASE + 0x18)
#define GPIOB_AFRL REG(GPIOB_BASE + 0x20)
#define GPIOC_BASE 0x48000800u
#define GPIOC_MODER REG(GPIOC_BASE + 0x00)
#define GPIOC_BSRR REG(GPIOC_BASE + 0x18)
/* TIM3 */
#define TIM3_BASE 0x40000400u
#define TIM3_CR1 REG(TIM3_BASE + 0x00)
#define TIM3_EGR REG(TIM3_BASE + 0x14)
#define TIM3_CCMR2 REG(TIM3_BASE + 0x1C)
#define TIM3_CCER REG(TIM3_BASE + 0x20)
#define TIM3_PSC REG(TIM3_BASE + 0x28)
#define TIM3_ARR REG(TIM3_BASE + 0x2C)
#define TIM3_CCR3 REG(TIM3_BASE + 0x3C)
#define TIM3_CCR4 REG(TIM3_BASE + 0x40)
/* SYSCFG / EXTI / NVIC */
#define SYSCFG_EXTICR2 REG(0x40010000u + 0x0C)
#define EXTI_IMR1 REG(0x40010400u + 0x00)
#define EXTI_FTSR1 REG(0x40010400u + 0x0C)
#define EXTI_PR1 REG(0x40010400u + 0x14)
#define NVIC_ISER0 REG(0xE000E100u)
#define EXTI4_IRQn 10
/* SysTick, IWDG, FPU */
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define IWDG_KR REG(0x40003000u)
#define IWDG_PR REG(0x40003004u)
#define IWDG_RLR REG(0x40003008u)
#define SCB_CPACR REG(0xE000ED88u)

#define CORE_HZ 16000000u
#define PWM_ARR 999u /* 16 MHz / 16 / 1000 = 1 kHz */

static app_t g_app;
static volatile uint32_t g_ticks;

static hal_status_t i2c_stub(void *c, uint8_t a, const uint8_t *tx, size_t ntx, uint8_t *rx, size_t nrx)
{
    (void)c; (void)a; (void)tx; (void)ntx; (void)rx; (void)nrx;
    return HAL_TIMEOUT; /* I2C1 peripheral driver: board bring-up */
}

static void outputs_apply(const app_outputs_t *o)
{
    GPIOB_BSRR = o->radio_en ? (1u << 13) : (1u << (13 + 16));
    GPIOB_BSRR = o->payload_en ? (1u << 14) : (1u << (14 + 16));
    TIM3_CCR3 = (uint32_t)(o->duty_batt * (float)(PWM_ARR + 1));
    TIM3_CCR4 = (uint32_t)(o->duty_bay * (float)(PWM_ARR + 1));
}

void EXTI4_IRQHandler(void)
{
    EXTI_PR1 = 1u << 4; /* clear pending */
    app_power_fail_isr(&g_app);
    /* shed in hardware before returning: load switches off, heater PWM to 0 */
    GPIOB_BSRR = (1u << (13 + 16)) | (1u << (14 + 16));
    TIM3_CCR3 = 0;
    TIM3_CCR4 = 0;
}

void SysTick_Handler(void) { g_ticks++; }

static void board_init(void)
{
    SCB_CPACR |= 0xFu << 20; /* enable the FPU (CP10, CP11) */
    RCC_AHB2ENR |= (1u << 1) | (1u << 2); /* GPIOB, GPIOC */
    RCC_APB1ENR1 |= 1u << 1;              /* TIM3 */
    RCC_APB2ENR |= 1u << 0;               /* SYSCFG */

    /* PB13, PB14 outputs (load-switch ON, external pull-downs keep them off in reset) */
    GPIOB_BSRR = (1u << (13 + 16)) | (1u << (14 + 16));
    GPIOB_MODER = (GPIOB_MODER & ~((3u << 26) | (3u << 28))) | (1u << 26) | (1u << 28);
    /* PB0, PB1 alternate function 2 = TIM3_CH3, TIM3_CH4 */
    GPIOB_MODER = (GPIOB_MODER & ~(0xFu)) | (2u << 0) | (2u << 2);
    GPIOB_AFRL = (GPIOB_AFRL & ~0xFFu) | (2u << 0) | (2u << 4);
    /* PB4 input with pull-up (open-drain ALERT, external 10k too) */
    GPIOB_MODER &= ~(3u << 8);
    GPIOB_PUPDR = (GPIOB_PUPDR & ~(3u << 8)) | (1u << 8);
    /* PC13, PC14 LEDs (active low) */
    GPIOC_MODER = (GPIOC_MODER & ~((3u << 26) | (3u << 28))) | (1u << 26) | (1u << 28);

    /* TIM3: 1 kHz PWM mode 1 on CH3, CH4, preload enabled */
    TIM3_PSC = 15;
    TIM3_ARR = PWM_ARR;
    TIM3_CCMR2 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM3_CCR3 = 0;
    TIM3_CCR4 = 0;
    TIM3_CCER = (1u << 8) | (1u << 12);
    TIM3_EGR = 1;
    TIM3_CR1 = (1u << 7) | 1u;

    /* PB4 falling edge -> EXTI4 */
    SYSCFG_EXTICR2 = (SYSCFG_EXTICR2 & ~0xFu) | 1u; /* port B */
    EXTI_FTSR1 |= 1u << 4;
    EXTI_IMR1 |= 1u << 4;
    NVIC_ISER0 = 1u << EXTI4_IRQn;

    /* SysTick 1 ms */
    SYST_RVR = CORE_HZ / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7u;

    /* IWDG: LSI 32 kHz / 32 = 1 kHz, 100 ms timeout */
    IWDG_KR = 0xCCCC;
    IWDG_KR = 0x5555;
    IWDG_PR = 3;
    IWDG_RLR = 100;
    IWDG_KR = 0xAAAA;
}

int main(void)
{
    board_init();
    app_init(&g_app);
    hal_i2c_t i2c1 = {i2c_stub, 0};
    app_inputs_t in = {0};
    uint32_t last = 0;
    for (;;) {
        while (g_ticks == last) {
        }
        last++;
        in.v_in = 14.8f; /* ADC2 IN12 (VIN_SENSE): board bring-up */
        in.new_slow_sample = (last % 1000u) == 0;
        if (in.new_slow_sample) {
            in.t_bay_ok = tmp117_read_c(i2c1, 0x48, &in.t_bay_c) == HAL_OK;
            in.t_batt_ok = false;
        }
        outputs_apply(app_tick_1ms(&g_app, &in));
        GPIOC_BSRR = (g_app.h_batt.faults | g_app.h_bay.faults) ? (1u << (14 + 16)) : (1u << 14);
        IWDG_KR = 0xAAAA;
    }
}
