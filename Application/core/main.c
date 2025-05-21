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

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define VTOR_ADDRESS            0x08000000

#define HSI_CLOCK               16000000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Системный таймер (1 мс) */
volatile uint32_t systick;

/* Состояние VDD */
volatile bool vdd_is_lower;

/* Private function prototypes --------------------------------------------- */

static void setup_hardware(void);

static void setup_vector_table(void);

static void setup_fpu(void);

static void app_main(void);

static void systick_init(const uint32_t frequency);

static void pwr_init(void);

static void flash_init(void);

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
        continue;
    }
}
/* ------------------------------------------------------------------------- */

static void app_main(void)
{
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
