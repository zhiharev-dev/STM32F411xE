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

#ifndef STM32F4XX_HAL_FLASH_H_
#define STM32F4XX_HAL_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных FLASH
 */
typedef FLASH_TypeDef flash_t;


/**
 * @brief           Определение перечисления задержки чтения FLASH
 */
enum flash_latency {
    FLASH_LATENCY_0WS,
    FLASH_LATENCY_1WS,
    FLASH_LATENCY_2WS,
    FLASH_LATENCY_3WS,
    FLASH_LATENCY_4WS,
    FLASH_LATENCY_5WS,
    FLASH_LATENCY_6WS,
    FLASH_LATENCY_7WS,
    FLASH_LATENCY_8WS,
    FLASH_LATENCY_9WS,
    FLASH_LATENCY_10WS,
    FLASH_LATENCY_11WS,
    FLASH_LATENCY_12WS,
    FLASH_LATENCY_13WS,
    FLASH_LATENCY_14WS,
    FLASH_LATENCY_15WS,
};


/**
 * @brief           Определение структуры данных для настройки FLASH
 */
struct flash_config {
    uint32_t latency;                           /*!< Задержка чтения FLASH @ref enum flash_latency */

    uint32_t prefetch_enable;                   /*!< Включить предварительную выборку данных @ref hal_state_t */

    uint32_t icache_enable;                     /*!< Включить кэш-инструкций @ref hal_state_t */

    uint32_t dcache_enable;                     /*!< Включить кэш-данных @ref hal_state_t */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_flash_config(struct flash_config *conf);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_FLASH_H_ */
