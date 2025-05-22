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

#include "pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать PWR
 */
void pwr_init(void)
{
    struct pwr_config conf = {
        .vos = PWR_VOS1,
        .pvd_enable = HAL_ENABLE,
        .pvd_level = PWR_PVD_2V9,
        .bkp_regulator_enable = HAL_ENABLE,
    };

    HAL_PWR_ENABLE_CLOCK();

    hal_pwr_config(&conf);

    NVIC_SetPriority(PVD_IRQn, 5);
    NVIC_EnableIRQ(PVD_IRQn);
}
/* ------------------------------------------------------------------------- */
