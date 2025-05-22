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

#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_spi_setup_mode(spi_t *instance, uint32_t mode);

static void hal_spi_setup_div(spi_t *instance, uint32_t div);

static void hal_spi_setup_frame(spi_t *instance, uint32_t format, uint32_t size);

static void hal_spi_setup_nss(spi_t *instance);

static void hal_spi_setup_dma(spi_t *instance, uint32_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Настроить SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       conf: Указатель на структуру данных для настройки SPI
 */
void hal_spi_config(struct spi_handle *handle, struct spi_config *conf)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(conf != NULL);

    hal_spi_setup_mode(handle->instance, conf->mode);
    hal_spi_setup_div(handle->instance, conf->div);
    hal_spi_setup_frame(handle->instance, conf->frame_format, conf->frame_size);
    hal_spi_setup_nss(handle->instance);
    hal_spi_setup_dma(handle->instance, conf->dma_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим работы SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       mode: Режим работы SPI @ref spi_mode
 */
static void hal_spi_setup_mode(spi_t *instance, uint32_t mode)
{
    /* Установить MSRT */
    SET_BIT(instance->CR1, SPI_CR1_MSTR_Msk);

    /* Настроить CPOL и CPHA */
    switch (mode) {
        case SPI_MODE0:
            CLEAR_BIT(instance->CR1,
                      SPI_CR1_CPOL_Msk
                    | SPI_CR1_CPHA_Msk);
            break;
        case SPI_MODE1:
            MODIFY_REG(instance->CR1,
                       SPI_CR1_CPHA_Msk,
                       SPI_CR1_CPOL_Msk);
            break;
        case SPI_MODE2:
            MODIFY_REG(instance->CR1,
                       SPI_CR1_CPOL_Msk,
                       SPI_CR1_CPHA_Msk);
            break;
        case SPI_MODE3:
            SET_BIT(instance->CR1,
                    SPI_CR1_CPOL_Msk
                  | SPI_CR1_CPHA_Msk);
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить делитель часов SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       div: Делитель часов SPI @ref enum spi_div
 */
static void hal_spi_setup_div(spi_t *instance, uint32_t div)
{
    MODIFY_REG(instance->CR1,
               SPI_CR1_BR_Msk,
               div << SPI_CR1_BR_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить фрейм данных SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       format: Формат фрейма SPI @ref enun spi_frame_format
 * @param[in]       size: Размер фрейма SPI @ref enum spi_frame_size
 */
static void hal_spi_setup_frame(spi_t *instance, uint32_t format, uint32_t size)
{
    MODIFY_REG(instance->CR1,
               SPI_CR1_LSBFIRST_Msk
             | SPI_CR1_DFF_Msk,
               format << SPI_CR1_LSBFIRST_Pos
             | size << SPI_CR1_DFF_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить сигнал NSS SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 */
static void hal_spi_setup_nss(spi_t *instance)
{
    /* Настроить NSS = Software */
    SET_BIT(instance->CR1,
            SPI_CR1_SSM_Msk
          | SPI_CR1_SSI_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить DMA SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       state: Состояне @ref hal_state_t
 */
static void hal_spi_setup_dma(spi_t *instance, uint32_t state)
{
    MODIFY_REG(instance->CR2,
               SPI_CR2_RXDMAEN_Msk
             | SPI_CR2_TXDMAEN_Msk,
               state << SPI_CR2_RXDMAEN_Pos
             | state << SPI_CR2_TXDMAEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения передачи данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_transmit_receive_completed_dma_it_handler(struct spi_handle *handle)
{
    hal_spi_transmit_receive_completed_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ошибки передачи данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_error_dma_it_handler(struct spi_handle *handle)
{
    hal_spi_error_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_enable(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, SPI_CR1_SPE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_disable(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, SPI_CR1_SPE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       txdata: Указатель на данные
 * @param[out]      rxdata: Указатель на данные
 * @param[in]       data_size: Размер данных
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_spi_transmit_receive_dma(struct spi_handle *handle,
                                          const void *txdata,
                                          void * rxdata,
                                          size_t data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->dma_tx != NULL);
    assert(handle->dma_rx != NULL);

    /* Проверить наличие данных */
    if (txdata == NULL || rxdata == NULL || data_size == 0)
        return HAL_ERROR;

    /* Указатель на данные периферии */
    void *peripheral_data = (void *) &handle->instance->DR;

    /* Запустить прием данных DMA */
    if (hal_dma_transfer_it(handle->dma_rx,
                            peripheral_data,
                            (void *) rxdata,
                            data_size) == HAL_OK) {
        /* Запустить передачу данных DMA */
        if (hal_dma_transfer_it(handle->dma_tx,
                                peripheral_data,
                                (void *) txdata,
                                data_size) == HAL_OK) {
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

__WEAK void hal_spi_transmit_receive_completed_callback(struct spi_handle *handle)
{

}
/* ------------------------------------------------------------------------- */

__WEAK void hal_spi_error_callback(struct spi_handle *handle)
{

}
/* ------------------------------------------------------------------------- */
