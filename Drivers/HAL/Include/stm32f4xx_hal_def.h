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

#ifndef STM32F4XX_HAL_DEF_H_
#define STM32F4XX_HAL_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "stm32f4xx.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Битовая маска
 *
 * @param[in]       x: Значение
 * @param[in]       n: Положение
 *
 * @return          Значение битовой маски
 */
#define HAL_BITMASK(x, n)       ((x) << (n))

/* Exported constants ------------------------------------------------------ */

/**
 * @brief           Максимальное значение задержки
 */
#define HAL_MAX_DELAY       0xFFFFFFFF

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение типа данных статуса
 */
typedef enum hal_status {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT,
} hal_status_t;


/**
 * @brief           Определение типа данных статуса флагов
 */
typedef enum hal_flag_status {
    HAL_RESET,
    HAL_SET,
} hal_flag_status_t;


/**
 * @brief           Определение типа данных состояния
 */
typedef enum hal_state {
    HAL_DISABLE,
    HAL_ENABLE,
} hal_state_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_error(void);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_error_callback(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_DEF_H_ */
