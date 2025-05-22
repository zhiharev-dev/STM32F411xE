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

/* Includes ---------------------------------------------------------------- */

#include "spi.h"
#include "dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

struct spi_handle spi1 = {
    .instance = SPI1,
    .dma_rx = &dma2_stream2,
    .dma_tx = &dma2_stream3,
};
SemaphoreHandle_t spi1_mutex;
EventGroupHandle_t spi1_event_group;

/* Private function prototypes --------------------------------------------- */

static void spi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI
 */
void spi_init(void)
{
    HAL_SPI1_ENABLE_CLOCK();

    spi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI1
 */
static void spi1_init(void)
{
    struct spi_config conf = {
        .mode = SPI_MODE0,
        .div = SPI_DIV2,
        .frame_format = SPI_MSBFIRST,
        .frame_size = SPI_8BIT,
        .dma_enable = HAL_ENABLE,
    };

    hal_spi_config(&spi1, &conf);
    hal_spi_enable(&spi1);

    spi1_mutex = xSemaphoreCreateMutex();
    if (spi1_mutex == NULL) {
        hal_error();
    }

    spi1_event_group = xEventGroupCreate();
    if (spi1_event_group == NULL) {
        hal_error();
    }
}
/* ------------------------------------------------------------------------- */
