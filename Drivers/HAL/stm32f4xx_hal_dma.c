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

#include "stm32f4xx_hal_dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

static const uint32_t DMA_TCIF_Msk[] = {
    DMA_LISR_TCIF0_Msk,
    DMA_LISR_TCIF1_Msk,
    DMA_LISR_TCIF2_Msk,
    DMA_LISR_TCIF3_Msk,
    DMA_HISR_TCIF4_Msk,
    DMA_HISR_TCIF5_Msk,
    DMA_HISR_TCIF6_Msk,
    DMA_HISR_TCIF7_Msk,
};

static const uint32_t DMA_HTIF_Msk[] = {
    DMA_LISR_HTIF0_Msk,
    DMA_LISR_HTIF1_Msk,
    DMA_LISR_HTIF2_Msk,
    DMA_LISR_HTIF3_Msk,
    DMA_HISR_HTIF4_Msk,
    DMA_HISR_HTIF5_Msk,
    DMA_HISR_HTIF6_Msk,
    DMA_HISR_HTIF7_Msk,
};

static const uint32_t DMA_TEIF_Msk[] = {
    DMA_LISR_TEIF0_Msk,
    DMA_LISR_TEIF1_Msk,
    DMA_LISR_TEIF2_Msk,
    DMA_LISR_TEIF3_Msk,
    DMA_HISR_TEIF4_Msk,
    DMA_HISR_TEIF5_Msk,
    DMA_HISR_TEIF6_Msk,
    DMA_HISR_TEIF7_Msk,
};

/* Private function prototypes --------------------------------------------- */

static void hal_dma_setup_channel(dma_stream_t *instance_stream, uint32_t channel);

static void hal_dma_setup_priority(dma_stream_t *instance_stream, uint32_t priority);

static void hal_dma_setup_transfer_direction(dma_stream_t *instance_stream, uint32_t direction);

static void hal_dma_setup_peripheral_data_size(dma_stream_t *instance_stream, uint32_t data_size);

static void hal_dma_setup_peripheral_inc_mode(dma_stream_t *instance_stream, uint32_t state);

static void hal_dma_setup_memory_data_size(dma_stream_t *instance_stream, uint32_t data_size);

static void hal_dma_setup_memory_inc_mode(dma_stream_t *instance_stream, uint32_t state);

static void hal_dma_setup_fifo(dma_stream_t *instance_stream);

static void hal_dma_setup_it(dma_stream_t *instance_stream, uint32_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Настроить DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 * @param[in]       conf: Указатель на структуру данных для настройки DMA
 */
void hal_dma_config(struct dma_handle *handle, struct dma_config *conf)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);
    assert(conf != NULL);

    hal_dma_disable(handle);

    hal_dma_setup_channel(handle->instance_stream, conf->channel);
    hal_dma_setup_priority(handle->instance_stream, conf->priority);
    hal_dma_setup_transfer_direction(handle->instance_stream, conf->transfer_direction);
    hal_dma_setup_peripheral_data_size(handle->instance_stream, conf->peripheral_data_size);
    hal_dma_setup_peripheral_inc_mode(handle->instance_stream, conf->peripheral_inc_mode_enable);
    hal_dma_setup_memory_data_size(handle->instance_stream, conf->memory_data_size);
    hal_dma_setup_memory_inc_mode(handle->instance_stream, conf->memory_inc_mode_enable);
    hal_dma_setup_fifo(handle->instance_stream);
    hal_dma_setup_it(handle->instance_stream, conf->it_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить номер канала потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       channel: Номер канала DMA @ref enum dma_channel
 */
static void hal_dma_setup_channel(dma_stream_t *instance_stream, uint32_t channel)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_CHSEL_Msk,
               channel << DMA_SxCR_CHSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить приоритет потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       priority: Приоритет DMA @ref enum dma_priority
 */
static void hal_dma_setup_priority(dma_stream_t *instance_stream, uint32_t priority)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PL_Msk,
               priority << DMA_SxCR_PL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить направление передачи данных потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       direction: Направление передачи данных DMA @ref enum dma_transfer_direction
 */
static void hal_dma_setup_transfer_direction(dma_stream_t *instance_stream, uint32_t direction)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_DIR_Msk,
               direction << DMA_SxCR_DIR_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить размер данных периферии потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       data_size: Размер данных DMA @ref enum dma_data_size
 */
static void hal_dma_setup_peripheral_data_size(dma_stream_t *instance_stream, uint32_t data_size)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PSIZE_Msk,
               data_size << DMA_SxCR_PSIZE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим инкремента периферии потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_dma_setup_peripheral_inc_mode(dma_stream_t *instance_stream, uint32_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PINC_Msk,
               state << DMA_SxCR_PINC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить размер данных памяти потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       data_size: Размер данных DMA @ref enum dma_data_size
 */
static void hal_dma_setup_memory_data_size(dma_stream_t *instance_stream, uint32_t data_size)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_MSIZE_Msk,
               data_size << DMA_SxCR_MSIZE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим инкремента памяти потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_dma_setup_memory_inc_mode(dma_stream_t *instance_stream, uint32_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_MINC_Msk,
               state << DMA_SxCR_MINC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить FIFO потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 */
static void hal_dma_setup_fifo(dma_stream_t *instance_stream)
{
    CLEAR_BIT(instance_stream->FCR, DMA_SxFCR_DMDIS_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить прерывания потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_dma_setup_it(dma_stream_t *instance_stream, uint32_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_TCIE_Msk
             | DMA_SxCR_TEIE_Msk,
               state << DMA_SxCR_TCIE_Pos
             | state << DMA_SxCR_TEIE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_it_handler(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);

    /* Обработать статус DMA */
    volatile uint32_t *ISR = NULL;
    volatile uint32_t *IFCR = NULL;

    if (handle->stream_nb < 4) {
        ISR = &handle->instance->LISR;
        IFCR = &handle->instance->LIFCR;
    } else {
        ISR = &handle->instance->HISR;
        IFCR = &handle->instance->HIFCR;
    }

    if (READ_BIT(*ISR, DMA_TEIF_Msk[handle->stream_nb])) {
        SET_BIT(*IFCR, DMA_TEIF_Msk[handle->stream_nb]);

        hal_dma_transfer_error_callback(handle);
    } else if (READ_BIT(*ISR, DMA_TCIF_Msk[handle->stream_nb])) {
        SET_BIT(*IFCR, DMA_TCIF_Msk[handle->stream_nb]);

        hal_dma_transfer_completed_callback(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить состояние DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
bool hal_dma_is_enabled(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_stream != NULL);

    return READ_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk) ? true : false;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_enable(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_stream != NULL);

    SET_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_disable(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_stream != NULL);

    CLEAR_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Количество переданных данных DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 *
 * @return          Количество данных
 */
uint32_t hal_dma_data_nb_transferred(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_stream != NULL);

    if (hal_dma_is_enabled(handle)) {
        return handle->transfer_data_nb - READ_REG(handle->instance_stream->NDTR);
    } else {
        return 0;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать данные DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 * @param[in]       peripheral_data: Указатель на данные периферии
 * @param[in]       memory_data: Указатель на данные памяти
 * @param[in]       data_nb: Количество данных
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_dma_transfer_it(struct dma_handle *handle,
                                 void *peripheral_data,
                                 void *memory_data,
                                 uint32_t data_nb)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);

    /* Проверить наличие данных */
    if (peripheral_data == NULL ||
        memory_data == NULL ||
        data_nb == 0)
        return HAL_ERROR;

    /* Выключить DMA */
    hal_dma_disable(handle);
    while (hal_dma_is_enabled(handle))
        continue;

    /* Сбросить статус DMA */
    volatile uint32_t *IFCR = NULL;

    if (handle->stream_nb < 4) {
        IFCR = &handle->instance->LIFCR;
    } else {
        IFCR = &handle->instance->HIFCR;
    }

    SET_BIT(*IFCR,
            DMA_TCIF_Msk[handle->stream_nb]
          | DMA_HTIF_Msk[handle->stream_nb]
          | DMA_TEIF_Msk[handle->stream_nb]);

    /* Установить адрес периферии */
    WRITE_REG(handle->instance_stream->PAR, (uint32_t) peripheral_data);
    /* Установить адрес памяти */
    WRITE_REG(handle->instance_stream->M0AR, (uint32_t) memory_data);
    /* Установить количество данных */
    WRITE_REG(handle->instance_stream->NDTR, (uint16_t) data_nb);

    /* Запомнить количество передаваемых данных */
    handle->transfer_data_nb = data_nb;

    /* Включить DMA */
    hal_dma_enable(handle);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать передачу данных DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_abort_transfer_it(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_stream != NULL);

    /* Сбросить количество передаваемых данных */
    handle->transfer_data_nb = 0;

    /* Выключить DMA */
    hal_dma_disable(handle);
}
/* ------------------------------------------------------------------------- */

void hal_dma_transfer_completed_callback(struct dma_handle *handle)
{

}
/* ------------------------------------------------------------------------- */

void hal_dma_transfer_error_callback(struct dma_handle *handle)
{

}
/* ------------------------------------------------------------------------- */
