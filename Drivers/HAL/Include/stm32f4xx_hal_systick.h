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

#ifndef STM32F4XX_HAL_SYSTICK_H_
#define STM32F4XX_HAL_SYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных SysTick
 */
typedef SysTick_Type systick_t;


/**
 * @brief           Определение перечисления источников тактирования SysTick
 */
enum systick_clock_source {
    SYSTICK_CPU_CLOCK_DIV8,
    SYSTICK_CPU_CLOCK,
};


/**
 * @brief           Определение структуры данных для настройки SysTick
 */
struct systick_config {
    uint32_t frequency;                         /*!< Частота тактирования (Гц) */

    uint32_t clksource;                         /*!< Источник тактирования @ref enum systick_clock_source */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_systick_config(struct systick_config *conf);

void hal_systick_it_handler(void);

void hal_systick_start_it(void);

void hal_systick_stop_it(void);

uint32_t hal_systick_tick(void);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_systick_period_elapsed_callback(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_SYSTICK_H_ */
