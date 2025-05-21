/*
 * Copyright (C) 2025 zhiharev-dev <zhiharev.dev@mail.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "led.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define VTOR_ADDRESS            0x08000000

#define HSI_CLOCK               16000000

#define RCC_HSERDY_TIMEOUT      100

#define RCC_LSERDY_TIMEOUT      5000

#define RCC_CPU_CLOCK           96000000

#define RCC_AHB_CLOCK           (RCC_CPU_CLOCK / 1)

#define RCC_APB1_CLOCK          (RCC_AHB_CLOCK / 2)

#define RCC_APB2_CLOCK          (RCC_AHB_CLOCK / 1)

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Системный таймер (1 мс) */
volatile uint32_t systick;

/* Состояние VDD */
volatile bool vdd_is_lower;

/* LEDs */
struct led led_blue = {
    .gpio = GPIOC,
    .pin = GPIO_ODR_OD13,
};

/* Private function prototypes --------------------------------------------- */

static void setup_hardware(void);

static void setup_vector_table(void);

static void setup_fpu(void);

static void app_main(void);

static void systick_init(const uint32_t frequency);

static void pwr_init(void);

static void flash_init(void);

static void rcc_init(void);

static void gpio_init(void);

static void gpio_led_init(void);

/* Private user code ------------------------------------------------------- */

int main(void)
{
    setup_hardware();
    app_main();
}
/* ------------------------------------------------------------------------- */

void error(void)
{
    __disable_irq();

    while (true) {
        /* Задержка */
        for (uint32_t i = 0; i < 9600; i++) {
            for (uint32_t j = 0; j < 1000; j++) {
                continue;
            }
        }
        /* Мигание светодиода - Ошибка */
        led_toggle(&led_blue);
    }
}
/* ------------------------------------------------------------------------- */

static void app_main(void)
{
    /* Включить светодиод - Рабочее состояние */
    led_on(&led_blue);

    while (true) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void setup_hardware(void)
{
    setup_vector_table();
    setup_fpu();

    systick_init(HSI_CLOCK);
    pwr_init();
    flash_init();
    rcc_init();
    systick_init(RCC_CPU_CLOCK);
    gpio_init();
}
/* ------------------------------------------------------------------------- */

static void setup_vector_table(void)
{
    __disable_irq();
    __set_PRIMASK(1);

    WRITE_REG(SCB->VTOR, VTOR_ADDRESS);

    __set_PRIMASK(0);
    __enable_irq();
}
/* ------------------------------------------------------------------------- */

static void setup_fpu(void)
{
    SET_BIT(SCB->CPACR, (0x03 << 20) | (0x03 << 22));
}
/* ------------------------------------------------------------------------- */

static void systick_init(const uint32_t frequency)
{
    /* Сбросить регистр управления */
    CLEAR_REG(SysTick->CTRL);

    /* Установить значение перезагрузки счетчика = 1 мс */
    WRITE_REG(SysTick->LOAD, (frequency / 1000) - 1);

    /* Установить текущее значение счетчика = 0 */
    CLEAR_REG(SysTick->VAL);

    /* Настроить тактирование от CPU и запустить таймер */
    WRITE_REG(SysTick->CTRL,
              SysTick_CTRL_CLKSOURCE_Msk
            | SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

static void pwr_init(void)
{
    /* Включить тактирование PWR */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk);

    /* Настроить VOS1 (100MHz) */
    MODIFY_REG(PWR->CR,
               PWR_CR_VOS_Msk,
               0x03 << PWR_CR_VOS_Pos);

    /* Отключить защиту от записи в домен резервного копирования */
    SET_BIT(PWR->CR, PWR_CR_DBP_Msk);

    /* Включить резервный регулятор */
    SET_BIT(PWR->CSR, PWR_CSR_BRE_Msk);
    while (!READ_BIT(PWR->CSR, PWR_CSR_BRR_Msk)) {
        continue;
    }

    /* Включить и настроить уровень PVD 2.9V */
    SET_BIT(PWR->CR,
            PWR_CR_PVDE_Msk
          | PWR_CR_PLS_Msk);

    /* Разрешить прерывание EXTI PVD output */
    SET_BIT(EXTI->IMR, EXTI_IMR_MR16_Msk);
    /* Включить Rising Trigger */
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR16_Msk);
    /* Включить Falling Trigger */
    SET_BIT(EXTI->FTSR, EXTI_FTSR_TR16_Msk);

    /* Настроить NVIC */
    NVIC_SetPriority(PVD_IRQn, 5);
    NVIC_EnableIRQ(PVD_IRQn);
}
/* ------------------------------------------------------------------------- */

static void flash_init(void)
{
    /* Настроить задержку чтения флэш-памяти = 3WS */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               0x03 << FLASH_ACR_LATENCY_Pos);

    /* Включить предварительную выборку данных */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN_Msk);

    /* Настроить кэш-инструкций */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_ICEN_Msk,
               FLASH_ACR_ICRST_Msk);

    for (uint32_t i = 0; i < 3; i++) {
        __NOP();
    }

    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_ICRST_Msk,
               FLASH_ACR_ICEN_Msk);

    /* Настроить кэш-данных */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_DCEN_Msk,
               FLASH_ACR_DCRST_Msk);

    for (uint32_t i = 0; i < 3; i++) {
        __NOP();
    }

    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_DCRST_Msk,
               FLASH_ACR_DCEN_Msk);
}
/* ------------------------------------------------------------------------- */

static void rcc_init(void)
{
    uint32_t tickstart;

    /* Включить HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    tickstart = systick;
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
        if (systick - tickstart > RCC_HSERDY_TIMEOUT) {
            error();
        }
    }

    /* Включить CSS HSE */
    SET_BIT(RCC->CR, RCC_CR_CSSON_Msk);

    /* Выключить PLL перед настройкой */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Msk);

    /* Настроить PLL */
    WRITE_REG(RCC->PLLCFGR,
              RCC_PLLCFGR_PLLSRC_Msk            /* Источник тактирования = HSE (25MHz) */
            | 0x19 << RCC_PLLCFGR_PLLM_Pos      /* PLLM = /25 (HSE = 25MHz / 25 = 1MHz) */
            | 0xC0 << RCC_PLLCFGR_PLLN_Pos      /* PLLN = x192 (1MHz * 192 = 192MHz) */
            | 0x00 << RCC_PLLCFGR_PLLP_Pos      /* PLLP = /2 (192MHz / 2 = 96MHz) */
            | 0x04 << RCC_PLLCFGR_PLLQ_Pos);    /* PLLQ = /4 (192MHz / 4 = 48MHz) */

    /* Включить PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON_Msk);
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY_Msk)) {
        continue;
    }

    /* Ожидание готовности PWR VOS после включения PLL */
    while (!READ_BIT(PWR->CSR, PWR_CSR_VOSRDY_Msk)) {
        continue;
    }

    /* Настроить BUS */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE_Msk
             | RCC_CFGR_PPRE1_Msk
             | RCC_CFGR_PPRE2_Msk,
               0x00 << RCC_CFGR_HPRE_Pos        /* AHB = /1 (96MHz / 1 = 96MHz) */
             | 0x04 << RCC_CFGR_PPRE1_Pos       /* APB1 = /2 (96MHz / 2 = 48MHz) */
             | 0x00 << RCC_CFGR_PPRE2_Pos);     /* APB2 = /1 (96MHz / 1 = 96MHz) */

    /* Настроить источник тактирования CPU */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               0x02 << RCC_CFGR_SW_Pos);        /* Источник тактирования = PLLP */

    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk)
            != 0x02 << RCC_CFGR_SWS_Pos) {
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void gpio_init(void)
{
    /* Включить тактирование */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Msk);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Msk);

    gpio_led_init();
}
/* ------------------------------------------------------------------------- */

static void gpio_led_init(void)
{
    /* LED_BLUE GPIOC13 */

    /* Установить начальное состояние = High */
    SET_BIT(GPIOC->ODR, GPIO_ODR_OD13_Msk);
    /* Настроить режим работы = GPO */
    MODIFY_REG(GPIOC->MODER,
               GPIO_MODER_MODE13_Msk,
               0x01 << GPIO_MODER_MODE13_Pos);
    /* Настроить тип вывода = Push-Pull */
    MODIFY_REG(GPIOC->OTYPER,
               GPIO_OTYPER_OT13_Msk,
               0x00 << GPIO_OTYPER_OT13_Pos);
    /* Настроить скорость работы вывода = Low Speed */
    MODIFY_REG(GPIOC->OSPEEDR,
               GPIO_OSPEEDR_OSPEED13_Msk,
               0x00 << GPIO_OSPEEDR_OSPEED13_Pos);
    /* Настроить подтяжку сигнала = Pull-Up */
    MODIFY_REG(GPIOC->PUPDR,
               GPIO_PUPDR_PUPD13_Msk,
               0x01 << GPIO_PUPDR_PUPD13_Pos);
}
/* ------------------------------------------------------------------------- */
