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

/* W25Q */
struct gpio_handle gpio_w25q_cs = {
    .instance = GPIOA,
    .pin = GPIO_PIN4,
};

/* Private function prototypes --------------------------------------------- */

static void gpio_led_init(void);

static void gpio_w25q_init(void);

static void gpio_spi_init(void);

static void gpio_spi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO
 */
void gpio_init(void)
{
    HAL_GPIOA_ENABLE_CLOCK();
    HAL_GPIOC_ENABLE_CLOCK();

    gpio_led_init();
    gpio_w25q_init();
    gpio_spi_init();
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

/**
 * @brief           Инициализировать GPIO W25Q
 */
static void gpio_w25q_init(void)
{
    struct gpio_config conf = {
         .mode = GPIO_OUTPUT,
         .output_type = GPIO_PUSH_PULL,
         .output_speed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    hal_gpio_set_state(&gpio_w25q_cs, GPIO_SET);
    hal_gpio_config(&gpio_w25q_cs, &conf);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI
 */
static void gpio_spi_init(void)
{
    gpio_spi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI1
 */
static void gpio_spi1_init(void)
{
    struct gpio_config conf = {
        .mode = GPIO_AF,
        .output_type = GPIO_PUSH_PULL,
        .output_speed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 5,
    };

    struct gpio_handle gpio_spi_sck = {
        .instance = GPIOA,
        .pin = GPIO_PIN5,
    };
    struct gpio_handle gpio_spi_miso = {
        .instance = GPIOA,
        .pin = GPIO_PIN6,
    };
    struct gpio_handle gpio_spi_mosi = {
        .instance = GPIOA,
        .pin = GPIO_PIN7,
    };

    hal_gpio_config(&gpio_spi_sck, &conf);
    hal_gpio_config(&gpio_spi_miso, &conf);
    hal_gpio_config(&gpio_spi_mosi, &conf);
}
/* ------------------------------------------------------------------------- */
