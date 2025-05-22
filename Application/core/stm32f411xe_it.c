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

#include "stm32f411xe_it.h"
#include "systick.h"
#include "pwr.h"
#include "dma.h"
#include "spi.h"
#include "w25q.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

void NMI_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void HardFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void MemManage_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void BusFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void UsageFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void SysTick_Handler(void)
{
    hal_systick_it_handler();
}
/* ------------------------------------------------------------------------- */

void hal_systick_period_elapsed_callback(void)
{
    /* Обработать системный таймер FreeRTOS */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
/* ------------------------------------------------------------------------- */

void PVD_IRQHandler(void)
{
    hal_pwr_pvd_it_handler();
}
/* ------------------------------------------------------------------------- */

void hal_pwr_pvd_status_changed_callback(void)
{

}
/* ------------------------------------------------------------------------- */

void DMA2_Stream2_IRQHandler(void)
{
    hal_dma_it_handler(&dma2_stream2);
}
/* ------------------------------------------------------------------------- */

void DMA2_Stream3_IRQHandler(void)
{
    hal_dma_it_handler(&dma2_stream3);
}
/* ------------------------------------------------------------------------- */

void hal_dma_transfer_completed_callback(struct dma_handle *handle)
{
    if (handle == &dma2_stream2) {
        hal_spi_transmit_receive_completed_dma_it_handler(&spi1);
    }
}
/* ------------------------------------------------------------------------- */

void hal_dma_transfer_error_callback(struct dma_handle *handle)
{
    if (handle == &dma2_stream2 || handle == &dma2_stream3) {
        hal_spi_error_dma_it_handler(&spi1);
    }
}
/* ------------------------------------------------------------------------- */

void hal_spi_transmit_receive_completed_callback(struct spi_handle *handle)
{
    if (handle == &spi1) {
        w25q_spi_transmit_receive_completed_it_handler();
    }
}
/* ------------------------------------------------------------------------- */

void hal_spi_error_callback(struct spi_handle *handle)
{
    if (handle == &spi1) {
        w25q_spi_error_it_handler();
    }
}
/* ------------------------------------------------------------------------- */
