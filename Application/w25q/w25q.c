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

#include "w25q.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define W25Q_MANUFACTURER_ID            0xEF

#define W25Q16                          0x14
#define W25Q32                          0x15
#define W25Q64                          0x16
#define W25Q128                         0x17
#define W25Q256                         0x18
#define W25Q512                         0x19

#define W25Q_SR1_BUSY_Pos               0
#define W25Q_SR1_BUSY_Msk               HAL_BITMASK(0x01, W25Q_SR1_BUSY_Pos)
#define W25Q_SR1_BUSY                   W25Q_SR1_BUSY_Msk

#define W25Q_SR1_WEL_Pos                1
#define W25Q_SR1_WEL_Msk                HAL_BITMASK(0x01, W25Q_SR1_WEL_Pos)
#define W25Q_SR1_WEL                    W25Q_SR1_WEL_Msk

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

struct w25q_handle w25q = {
    .hardware.spi = &spi1,
    .hardware.gpio_cs = &gpio_w25q_cs,
};

/* Private function prototypes --------------------------------------------- */

static hal_status_t w25q_read_device_id(void);

static hal_status_t w25q_read_unique_id(void);

static hal_status_t w25q_write_enable(void);

static hal_status_t w25q_read_sr1(void *data);

static hal_status_t w25q_transmit_receive(void *data, size_t data_size);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать W25Q
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_init(void)
{
    /* Назначить Mutex и Event Group */
    if (w25q.hardware.spi == &spi1) {
        w25q.hardware.spi_mutex = spi1_mutex;
        w25q.hardware.spi_event_group = spi1_event_group;
    } else {
        return HAL_ERROR;
    }

    /* Прочитать и проверить идентификатор устройства */
    if (w25q_read_device_id() != HAL_OK) {
        return HAL_ERROR;
    } else if (w25q.id.manufacturer_id != W25Q_MANUFACTURER_ID) {
        return HAL_ERROR;
    }

    /* Установить размер памяти */
    switch (w25q.id.device_id) {
        case W25Q16:
            w25q.size = 0x200000;
            break;
        case W25Q32:
            w25q.size = 0x400000;
            break;
        case W25Q64:
            w25q.size = 0x800000;
            break;
        case W25Q128:
            w25q.size = 0x1000000;
            break;
        case W25Q256:
            w25q.size = 0x2000000;
            break;
        case W25Q512:
            w25q.size = 0x4000000;
            break;
        default:
            return HAL_ERROR;
    }

    /* Прочитать уникальные данные устройства */
    if (w25q_read_unique_id() != HAL_OK) {
        return HAL_ERROR;
    } else {
        return HAL_OK;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения приема-передачи данных SPI
 */
void w25q_spi_transmit_receive_completed_it_handler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(w25q.hardware.spi_event_group,
                                  SPI_EV_TX_RX_CPLT,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ошибки SPI
 */
void w25q_spi_error_it_handler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(w25q.hardware.spi_event_group,
                                  SPI_EV_ERR,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать идентификатор W25Q
 *
 * @return          Статус @ref hal_status_t
 */
static hal_status_t w25q_read_device_id(void)
{
    hal_status_t status = HAL_ERROR;

    /* Подготовить данные */
    uint8_t buff[6] = {0x90, 0x00, 0x00, 0x00};

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
        return status;
    }

    /* Записать идентификаторы */
    taskENTER_CRITICAL();
    {
        w25q.id.manufacturer_id = buff[4];
        w25q.id.device_id = buff[5];
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать уникальный идентификатор W25Q
 *
 * @return          Статус @ref hal_status_t
 */
static hal_status_t w25q_read_unique_id(void)
{
    hal_status_t status = HAL_ERROR;

    /* Подготовить данные */
    uint8_t buff[13] = {0x4B, 0x00, 0x00, 0x00, 0x00};

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
        return status;
    }

    /* Записать уникальный идентификатор */
    taskENTER_CRITICAL();
    {
        memcpy(w25q.id.uid, &buff[5], 8);
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить разрешение на запись W25Q
 *
 * @return          Статус @ref hal_status_t
 */
static hal_status_t w25q_write_enable(void)
{
    hal_status_t status = HAL_ERROR;

    /* Подготовить данные */
    uint8_t buff[1] = {0x06};

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
        return status;
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать SR1 W25Q
 *
 * @param[out]      data: Указатель на данные (uint8_t)
 *
 * @return          Статус @ref hal_status_t
 */
static hal_status_t w25q_read_sr1(void *data)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить указатель на данные */
    if (data == NULL) {
        return status;
    }

    /* Подготовить данные */
    uint8_t buff[2] = {0x05};

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
        return status;
    }

    /* Записать ответ */
    taskENTER_CRITICAL();
    {
        *(uint8_t *) data = buff[1];
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные W25Q
 *
 * @param[in]       mem_addr: Адрес
 * @param[out]      data: Указатель на данные
 * @param[in]       data_size: Размер данных
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_fast_read(uint32_t mem_addr, void *data, size_t data_size)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить указатель и размер данных */
    if (data == NULL || data_size == 0) {
        return status;
    }

    /* Проверить адрес */
    if (mem_addr + data_size > w25q.size) {
        return status;
    }

    /* Подготовить данные */
    uint8_t *buff = pvPortMalloc(data_size + 5);
    if (buff == NULL) {
        w25q.state = W25Q_MALLOC_ERROR;
        return status;
    }

    buff[0] = 0x0B;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t)  mem_addr;
    buff[4] = 0;

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, data_size + 5);
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
    } else {
        /* Записать ответ */
        taskENTER_CRITICAL();
        {
            memcpy(data, &buff[5], data_size);
        }
        taskEXIT_CRITICAL();
    }

    vPortFree(buff);

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные W25Q
 *
 * @details         Примечание: Данные записываются в заранее стерные (0xFF) ячейки
 *                  Записываются данные, которые помещаются в страницы, остальные записаны не будут
 *
 * @param[in]       mem_addr: Адрес
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_page_program(uint32_t mem_addr, const void *data, size_t data_size)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить указатель и размер данных */
    if (data == NULL || data_size == 0) {
        return status;
    }

    /* Проверить адрес */
    if (mem_addr + data_size > w25q.size) {
        return status;
    }

    /* Разрешить запись */
    if (w25q_write_enable() != HAL_OK) {
        return status;
    }

    /* Проверить размер данных */
    uint32_t page_mem_addr_end = (mem_addr & 0xFFFFFF00) + W25Q_PAGE_SIZE;

    if (data_size > page_mem_addr_end - mem_addr) {
        data_size = page_mem_addr_end - mem_addr;
    }

    /* Подготовить данные */
    uint8_t *buff = pvPortMalloc(data_size + 4);
    if (buff == NULL) {
        w25q.state = W25Q_MALLOC_ERROR;
        return status;
    }

    buff[0] = 0x02;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t)  mem_addr;

    taskENTER_CRITICAL();
    {
        memcpy(&buff[4], data, data_size);
    }
    taskEXIT_CRITICAL();

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, data_size + 4);
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
    } else {
        uint8_t SR1 = W25Q_SR1_BUSY;
        while ((status = w25q_read_sr1(&SR1)) == HAL_OK) {
            if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk)) {
                break;
            }
        }
    }

    vPortFree(buff);

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить сектор флэш-памяти (4Кб) W25Q
 *
 * @param[in]       mem_addr: Адрес
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_sector_erase(uint32_t mem_addr)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить адрес */
    if (mem_addr > w25q.size) {
        return status;
    }

    /* Разрешить запись */
    if (w25q_write_enable() != HAL_OK) {
        return status;
    }

    /* Выделить адрес сектора */
    mem_addr = mem_addr & 0xFFFFF000;

    /* Подготовить данные */
    uint8_t buff[4] = {
        0x20,
        (uint8_t) (mem_addr >> 16),
        (uint8_t) (mem_addr >> 8),
        (uint8_t)  mem_addr,
    };

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
    } else {
        uint8_t SR1 = W25Q_SR1_BUSY;
        while ((status = w25q_read_sr1(&SR1)) == HAL_OK) {
            if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk)) {
                break;
            }
        }
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить блок флэш-памяти (64Кб) W25Q
 *
 * @param[in]       mem_addr: Адрес
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_block_erase(uint32_t mem_addr)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить адрес */
    if (mem_addr > w25q.size) {
        return status;
    }

    /* Разрешить запись */
    if (w25q_write_enable() != HAL_OK) {
        return status;
    }

    /* Выделить адрес блока */
    mem_addr = mem_addr & 0xFFFF0000;

    /* Подготовить данные */
    uint8_t buff[4] = {
        0xD8,
        (uint8_t) (mem_addr >> 16),
        (uint8_t) (mem_addr >> 8),
        (uint8_t)  mem_addr,
    };

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
    } else {
        uint8_t SR1 = W25Q_SR1_BUSY;
        while ((status = w25q_read_sr1(&SR1)) == HAL_OK) {
            if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk)) {
                break;
            }
        }
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить флэш-память W25Q
 *
 * @return          Статус @ref hal_status_t
 */
hal_status_t w25q_chip_erase(void)
{
    hal_status_t status = HAL_ERROR;

    /* Подготовить данные */
    uint8_t buff[1] = {0xC7};

    /* Прием-передача данных */
    status = w25q_transmit_receive(buff, sizeof(buff));
    if (status != HAL_OK) {
        w25q.state = W25Q_TRANSMIT_RECEIVE_ERROR;
    } else {
        uint8_t SR1 = W25Q_SR1_BUSY;
        while ((status = w25q_read_sr1(&SR1)) == HAL_OK) {
            if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk)) {
                break;
            }
        }
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных W25Q
 *
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 *
 * @return          Статус @ref hal_status_t
 */
static hal_status_t w25q_transmit_receive(void *data, size_t data_size)
{
    hal_status_t status = HAL_ERROR;

    /* Проверить указатель и размер данных */
    if (data == NULL || data_size == 0) {
        return status;
    }

    /* Захватить Mutex */
    if (xSemaphoreTake(w25q.hardware.spi_mutex, pdMS_TO_TICKS(50)) != pdPASS) return status;
    {
        /* Установить сигнал CS */
        hal_gpio_set_state(w25q.hardware.gpio_cs, GPIO_RESET);

        /* Передать/Принять данные */
        hal_spi_transmit_receive_dma(w25q.hardware.spi,
                                     data,
                                     data,
                                     data_size);

        EventBits_t bits = xEventGroupWaitBits(w25q.hardware.spi_event_group,
                                               SPI_EV_TX_RX_CPLT | SPI_EV_ERR,
                                               pdTRUE,
                                               pdFALSE,
                                               portMAX_DELAY);

        if (READ_BIT(bits, SPI_EV_TX_RX_CPLT_Msk)) {
            status = HAL_OK;
        }

        /* Сбросить сигнал CS */
        hal_gpio_set_state(w25q.hardware.gpio_cs, GPIO_SET);
    }
    xSemaphoreGive(w25q.hardware.spi_mutex);

    return status;
}
/* ------------------------------------------------------------------------- */
