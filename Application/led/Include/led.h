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

#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить светодиод
 *
 * @param[in]       handle: Указатель на структуру данных обработчика LED
 */
#define led_on(handle)          hal_gpio_set_state((struct gpio_handle *) handle, GPIO_RESET)

/**
 * @brief           Выключить светодиод
 *
 * @param[in]       handle: Указатель на структуру данных обработчика LED
 */
#define led_off(handle)         hal_gpio_set_state((struct gpio_handle *) handle, GPIO_SET)

/**
 * @brief           Переключить состояние светодиода
 *
 * @param[in]       handle: Указатель на структуру данных обработчика LED
 */
#define led_toggle(handle)      hal_gpio_toggle((struct gpio_handle *) handle)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/* Exported variables ------------------------------------------------------ */

#define led_blue gpio_led_blue

/* Exported function prototypes -------------------------------------------- */

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LED_H_ */
