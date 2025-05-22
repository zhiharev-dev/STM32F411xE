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

#ifndef STM32F4XX_HAL_DMA_H_
#define STM32F4XX_HAL_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование DMA1
 */
#define HAL_DMA1_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_Msk)

/**
 * @brief           Включить тактирование DMA2
 */
#define HAL_DMA2_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных DMA
 */
typedef DMA_TypeDef dma_t;


/**
 * @brief           Определение структуры данных потока DMA
 */
typedef DMA_Stream_TypeDef dma_stream_t;


/**
 * @brief           Определение перечисления каналов DMA
 */
enum dma_channel {
    DMA_CHANNEL0,
    DMA_CHANNEL1,
    DMA_CHANNEL2,
    DMA_CHANNEL3,
    DMA_CHANNEL4,
    DMA_CHANNEL5,
    DMA_CHANNEL6,
    DMA_CHANNEL7,
};


/**
 * @brief           Определение перечисления приоритетов DMA
 */
enum dma_priority {
    DMA_LOW_PRIORITY,
    DMA_MEDIUM_PRIORITY,
    DMA_HIGH_PRIORITY,
    DMA_VERY_HIGH_PRIORITY,
};


/**
 * @brief           Определение перечисления направления передачи данных DMA
 */
enum dma_transfer_direction {
    DMA_PERIPHERAL_TO_MEMORY,
    DMA_MEMORY_TO_PERIPHERAL,
    DMA_MEMORY_TO_MEMORY,
};


/**
 * @brief           Определение перечисления размера данных DMA
 */
enum dma_data_size {
    DMA_8BIT,
    DMA_16BIT,
    DMA_32BIT,
};


/**
 * @brief           Определение структуры данных для настройки DMA
 */
struct dma_config {
    uint32_t channel;                           /*!< Номер канала @ref enum dma_channel */

    uint32_t priority;                          /*!< Приоритет @ref enum dma_priority */

    uint32_t transfer_direction;                /*!< Направление передачи данных @ref enum dma_transfer_direction */

    uint32_t peripheral_data_size;              /*!< Размер данных периферии @ref enum dma_data_size */

    uint32_t peripheral_inc_mode_enable;        /*!< Режим инкремента периферии @ref hal_state_t */

    uint32_t memory_data_size;                  /*!< Размер данных памяти @ref enum dma_data_size */

    uint32_t memory_inc_mode_enable;            /*!< Режим инкремента памяти @ref hal_state_t */

    uint32_t it_enable;                         /*!< Включить прерывания @ref hal_state_t */
};


/**
 * @brief           Определение структуры данных обработчика DMA
 */
struct dma_handle {
    dma_t *instance;                            /*!< Указатель на структуру данных DMA */

    dma_stream_t *instance_stream;              /*!< Указатель на структуру данных потока DMA */

    uint32_t stream_nb;                         /*!< Номер потока */

    uint32_t transfer_data_nb;                  /*!< Количество передаваемых данных */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_dma_config(struct dma_handle *handle, struct dma_config *conf);

void hal_dma_it_handler(struct dma_handle *handle);

bool hal_dma_is_enabled(struct dma_handle *handle);

void hal_dma_enable(struct dma_handle *handle);

void hal_dma_disable(struct dma_handle *handle);

uint32_t hal_dma_data_nb_transferred(struct dma_handle *handle);

hal_status_t hal_dma_transfer_it(struct dma_handle *handle,
                                 void *peripheral_data,
                                 void *memory_data,
                                 uint32_t data_nb);

void hal_dma_abort_transfer_it(struct dma_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_dma_transfer_completed_callback(struct dma_handle *handle);

__WEAK void hal_dma_transfer_error_callback(struct dma_handle *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_DMA_H_ */
