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

#ifndef STM32F4XX_HAL_SPI_H_
#define STM32F4XX_HAL_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование SPI1
 */
#define HAL_SPI1_ENABLE_CLOCK() \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN_Msk)

/**
 * @brief           Включить тактирование SPI2
 */
#define HAL_SPI2_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN_Msk)

/**
 * @brief           Включить тактирование SPI3
 */
#define HAL_SPI3_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных SPI
 */
typedef SPI_TypeDef spi_t;


/**
 * @brief           Определение перечисления режимов работы SPI
 */
enum spi_mode {
    SPI_MODE0,                                  /*!< CPOL = 0, CPHA = 0 */
    SPI_MODE1,                                  /*!< CPOL = 1, CPHA = 0 */
    SPI_MODE2,                                  /*!< CPOL = 0, CPHA = 1 */
    SPI_MODE3,                                  /*!< CPOL = 1, CPHA = 1 */
};


/**
 * @brief           Определение перечисления делителя часов SPI
 */
enum spi_div {
    SPI_DIV2,
    SPI_DIV4,
    SPI_DIV8,
    SPI_DIV16,
    SPI_DIV32,
    SPI_DIV64,
    SPI_DIV128,
    SPI_DIV256,
};


/**
 * @brief           Определение перечисления форматов фрейма данных SPI
 */
enum spi_frame_format {
    SPI_MSBFIRST,
    SPI_LSBFIRST,
};


/**
 * @brief           Определение перечисления размеров фрейма данных SPI
 */
enum spi_frame_size {
    SPI_8BIT,
    SPI_16BIT,
};


/**
 * @brief           Определение структуры данных для настройки SPI
 */
struct spi_config {
    uint32_t mode;                              /*!< Режим работы @ref enum spi_mode */

    uint32_t div;                               /*!< Делитель часов @ref enum spi_div */

    uint32_t frame_format;                      /*!< Формат фрейма @ref enum spi_frame_format */

    uint32_t frame_size;                        /*!< Размер фрейма @ref enum spi_frame_size */

    uint32_t dma_enable;                        /*!< Включить DMA @ref hal_state_t */
};


/**
 * @brief           Определение структуры данных обработчика SPI
 */
struct spi_handle {
    spi_t *instance;                            /*!< Указатель на структуру данных SPI */

    void *dma_rx;                               /*!< Указатель на структуру данных обработчика DMA RX @ref struct dma_handle */

    void *dma_tx;                               /*!< Указатель на структуру данных обработчика DMA TX @ref struct dma_handle */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_spi_config(struct spi_handle *handle, struct spi_config *conf);

void hal_spi_transmit_receive_completed_dma_it_handler(struct spi_handle *handle);

void hal_spi_error_dma_it_handler(struct spi_handle *handle);

void hal_spi_enable(struct spi_handle *handle);

void hal_spi_disable(struct spi_handle *handle);

hal_status_t hal_spi_transmit_receive_dma(struct spi_handle *handle,
                                          const void *txdata,
                                          void *rxdata,
                                          size_t data_size);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_spi_transmit_receive_completed_callback(struct spi_handle *handle);

__WEAK void hal_spi_error_callback(struct spi_handle *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_SPI_H_ */
