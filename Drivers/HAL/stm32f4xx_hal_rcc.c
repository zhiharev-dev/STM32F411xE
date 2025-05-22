/**
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

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_systick.h"
#include "stm32f4xx_hal_pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define RCC_HSERDY_TIMEOUT      100

#define RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_rcc_setup_hse(uint32_t state);

static void hal_rcc_setup_css_hse(uint32_t state);

static void hal_rcc_setup_lse(uint32_t state);

static void hal_rcc_setup_pll(struct rcc_pll *pll);

static void hal_rcc_setup_ahb(uint32_t div);

static void hal_rcc_setup_apb1(uint32_t div);

static void hal_rcc_setup_apb2(uint32_t div);

static void hal_rcc_setup_cpu_clock_source(uint32_t clksource);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Настроить RCC
 *
 * @param[in]       conf: Указатель на структуру данных для настройки RCC
 */
void hal_rcc_config(struct rcc_config *conf)
{
    assert(conf != NULL);

    hal_rcc_setup_hse(conf->hse_enable);
    hal_rcc_setup_css_hse(conf->css_hse_enable);
    hal_rcc_setup_lse(conf->lse_enable);
    hal_rcc_setup_pll(&conf->pll);
    hal_rcc_setup_ahb(conf->ahb_div);
    hal_rcc_setup_apb1(conf->apb1_div);
    hal_rcc_setup_apb2(conf->apb2_div);
    hal_rcc_setup_cpu_clock_source(conf->cpu_clksource);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить HSE
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_rcc_setup_hse(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_HSEON_Msk,
               state << RCC_CR_HSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_tick();

        while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
            if (hal_systick_tick() - tickstart > RCC_HSERDY_TIMEOUT) {
                hal_error();
            }
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить CSS HSE
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_rcc_setup_css_hse(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_CSSON_Msk,
               state << RCC_CR_CSSON_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить LSE
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_rcc_setup_lse(uint32_t state)
{
    MODIFY_REG(RCC->BDCR,
               RCC_BDCR_LSEON_Msk,
               state << RCC_BDCR_LSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_tick();

        while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY_Msk)) {
            if (hal_systick_tick() - tickstart > RCC_LSERDY_TIMEOUT) {
                hal_error();
            }
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PLL
 *
 * @param[in]       pll: Указатель на структуру данных PLL
 */
static void hal_rcc_setup_pll(struct rcc_pll *pll)
{
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Msk);

    /* Настроить параметры PLL */
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLSRC_Msk
             | RCC_PLLCFGR_PLLM_Msk
             | RCC_PLLCFGR_PLLN_Msk
             | RCC_PLLCFGR_PLLP_Msk
             | RCC_PLLCFGR_PLLQ_Msk,
                pll->clksource << RCC_PLLCFGR_PLLSRC_Pos
             |  pll->divm << RCC_PLLCFGR_PLLM_Pos
             |  pll->divn << RCC_PLLCFGR_PLLN_Pos
             | (pll->divp / 2 - 1) << RCC_PLLCFGR_PLLP_Pos
             |  pll->divq << RCC_PLLCFGR_PLLQ_Pos);

    if (pll->enable == HAL_ENABLE) {
        SET_BIT(RCC->CR, RCC_CR_PLLON_Msk);
        while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY_Msk))
            continue;
    }

    /* Дождаться готовности PWR VOS */
    if (pll->enable == HAL_ENABLE) {
        while (!hal_pwr_vos_is_ready())
            continue;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить AHB
 *
 * @param[in]       div: Делитель AHB RCC @ref enum rcc_ahb_div
 */
static void hal_rcc_setup_ahb(uint32_t div)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE_Msk,
               div << RCC_CFGR_HPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить APB1
 *
 * @param[in]       div: Делитель APB RCC @ref enum rcc_apb_div
 */
static void hal_rcc_setup_apb1(uint32_t div)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_PPRE1_Msk,
               div << RCC_CFGR_PPRE1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить APB2
 *
 * @param[in]       div: Делитель APB RCC @ref enum rcc_apb_div
 */
static void hal_rcc_setup_apb2(uint32_t div)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_PPRE2_Msk,
               div << RCC_CFGR_PPRE2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить источник тактирования CPU
 *
 * @param[in]       clksource: Источник тактирования CPU RCC @ref enum rcc_cpu_clock_source
 */
static void hal_rcc_setup_cpu_clock_source(uint32_t clksource)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               clksource << RCC_CFGR_SW_Pos);

    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            clksource << RCC_CFGR_SWS_Pos)
        continue;
}
/* ------------------------------------------------------------------------- */
