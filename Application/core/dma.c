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

#include "dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

struct dma_handle dma2_stream2 = {
    .instance = DMA2,
    .instance_stream = DMA2_Stream2,
    .stream_nb = 2,
};

struct dma_handle dma2_stream3 = {
    .instance = DMA2,
    .instance_stream = DMA2_Stream3,
    .stream_nb = 3,
};

/* Private function prototypes --------------------------------------------- */

static void dma_spi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA
 */
void dma_init(void)
{
    HAL_DMA2_ENABLE_CLOCK();

    dma_spi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA SPI1
 */
static void dma_spi1_init(void)
{
    struct dma_config conf = {
        .channel = DMA_CHANNEL3,
        .priority = DMA_VERY_HIGH_PRIORITY,
        .peripheral_data_size = DMA_8BIT,
        .memory_data_size = DMA_8BIT,
        .memory_inc_mode_enable = HAL_ENABLE,
        .it_enable = HAL_ENABLE,
    };

    /* SPI RX */
    conf.transfer_direction = DMA_PERIPHERAL_TO_MEMORY;

    hal_dma_config(&dma2_stream2, &conf);

    NVIC_SetPriority(DMA2_Stream2_IRQn, 10);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* SPI TX */
    conf.transfer_direction = DMA_MEMORY_TO_PERIPHERAL;

    hal_dma_config(&dma2_stream3, &conf);

    NVIC_SetPriority(DMA2_Stream3_IRQn, 10);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}
/* ------------------------------------------------------------------------- */
