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

#include "gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* LEDs */
struct gpio_handle gpio_led_blue = {
    .instance = GPIOC,
    .pin = GPIO_PIN13,
};

/* Private function prototypes --------------------------------------------- */

static void gpio_led_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO
 */
void gpio_init(void)
{
    HAL_GPIOA_ENABLE_CLOCK();
    HAL_GPIOC_ENABLE_CLOCK();

    gpio_led_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    struct gpio_config conf = {
        .mode = GPIO_OUTPUT,
        .output_type = GPIO_PUSH_PULL,
        .output_speed = GPIO_LOW_SPEED,
        .pupd = GPIO_PULL_UP,
        .af = 0,
    };

    hal_gpio_set_state(&gpio_led_blue, GPIO_SET);
    hal_gpio_config(&gpio_led_blue, &conf);
}
/* ------------------------------------------------------------------------- */
