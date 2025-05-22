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

#ifndef STM32F4XX_HAL_RCC_H_
#define STM32F4XX_HAL_RCC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных RCC
 */
typedef RCC_TypeDef rcc_t;


/**
 * @brief           Определение перечисления источников тактирования PLL
 */
enum rcc_pll_clock_source {
    RCC_PLL_HSI_CLOCK,
    RCC_PLL_HSE_CLOCK,
};


/**
 * @brief           Определение перечисления делителей AHB
 */
enum rcc_ahb_div {
    RCC_AHB_NOT_DIV,
    RCC_AHB_DIV2 = 0b1000,
    RCC_AHB_DIV4,
    RCC_AHB_DIV8,
    RCC_AHB_DIV16,
    RCC_AHB_DIV64,
    RCC_AHB_DIV128,
    RCC_AHB_DIV256,
    RCC_AHB_DIV512,
};


/**
 * @brief           Определение перечисления делителей APB
 */
enum rcc_apb_div {
    RCC_APB_NOT_DIV,
    RCC_APB_DIV2 = 0b100,
    RCC_APB_DIV4,
    RCC_APB_DIV8,
    RCC_APB_DIV16,
};


/**
 * @brief           Определение перечисления источников тактирования CPU
 */
enum rcc_cpu_clock_source {
    RCC_CPU_HSI_CLOCK,
    RCC_CPU_HSE_CLOCK,
    RCC_CPU_PLL_CLOCK,
};


/**
 * @brief           Определение структуры данных PLL
 */
struct rcc_pll {
    uint32_t enable;                         /*!< Включить PLL @ref hal_state_t  */

    uint32_t clksource;                      /*!< Источник тактирования @ref enum rcc_pll_clock_source */

    uint32_t divm;                           /*!< Делитель DIVM = 2 <= PLLM <= 63 */

    uint32_t divn;                           /*!< Делитель DIVN = 192 <= PLLN <= 432 */

    uint32_t divp;                           /*!< Делитель DIVP = 2, 4, 6 или 8 */

    uint32_t divq;                           /*!< Делитель DIVQ = 2 <= PLLQ <= 15 */
};


/**
 * @brief           Определение структуры данных для настройки RCC
 */
struct rcc_config {
    uint32_t hse_enable;                        /*!< Включить HSE @ref hal_state_t */

    uint32_t css_hse_enable;                    /*!< Включить CSS HSE @ref hal_state_t */

    uint32_t lse_enable;                        /*!< Включить LSE @ref hal_state_t */

    struct rcc_pll pll;                         /*!< Настройки PLL */

    uint32_t ahb_div;                           /*!< Делитель AHB @ref enum rcc_ahb_div */

    uint32_t apb1_div;                          /*!< Делитель APB1 @ref enum rcc_apb_div */

    uint32_t apb2_div;                          /*!< Делитель APB2 @ref enum rcc_apb_div */

    uint32_t cpu_clksource;                     /*!< Источник тактирования @ref enum rcc_cpu_clock_source */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_rcc_config(struct rcc_config *conf);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_RCC_H_ */
