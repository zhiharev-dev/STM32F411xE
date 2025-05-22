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

#include "rcc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RCC
 */
void rcc_init(void)
{
    struct rcc_config conf = {
        .hse_enable = HAL_ENABLE,
        .css_hse_enable = HAL_ENABLE,
        .pll.enable = HAL_ENABLE,
        .pll.clksource = RCC_PLL_HSE_CLOCK,
        .pll.divm = 25,
        .pll.divn = 192,
        .pll.divp = 2,
        .pll.divq = 4,
        .ahb_div = RCC_AHB_NOT_DIV,
        .apb1_div = RCC_APB_DIV2,
        .apb2_div = RCC_APB_NOT_DIV,
        .cpu_clksource = RCC_CPU_PLL_CLOCK,
    };

    hal_rcc_config(&conf);
}
/* ------------------------------------------------------------------------- */
