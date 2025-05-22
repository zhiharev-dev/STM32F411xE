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

#include "stm32f4xx_hal_flash.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_flash_setup_latency(uint32_t latency);

static void hal_flash_setup_prefetch(uint32_t state);

static void hal_flash_setup_icache(uint32_t state);

static void hal_flash_setup_dcache(uint32_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Настроить FLASH
 *
 * @param[in]       conf: Указатель на структуру данных для настройки FLASH
 */
void hal_flash_config(struct flash_config *conf)
{
    assert(conf != NULL);

    hal_flash_setup_latency(conf->latency);
    hal_flash_setup_prefetch(conf->prefetch_enable);
    hal_flash_setup_icache(conf->icache_enable);
    hal_flash_setup_dcache(conf->dcache_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить задержку чтения FLASH
 *
 * @param[in]       latency: Задержка чтения FLASH @ref enum flash_latency
 */
static void hal_flash_setup_latency(uint32_t latency)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               latency << FLASH_ACR_LATENCY_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить предварительную выборку данных
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_flash_setup_prefetch(uint32_t state)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_PRFTEN_Msk,
               state << FLASH_ACR_PRFTEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить кэш-инструкций
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_flash_setup_icache(uint32_t state)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_ICEN_Msk,
               FLASH_ACR_ICRST_Msk);

    for (uint32_t i = 0; i < 3; i++)
        __NOP();

    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_ICRST_Msk,
               state << FLASH_ACR_ICEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить кэш-данных
 *
 * @param[in]       state Состояние @ref hal_state_t
 */
static void hal_flash_setup_dcache(uint32_t state)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_DCEN_Msk,
               FLASH_ACR_DCRST_Msk);

    for (uint32_t i = 0; i < 3; i++)
        __NOP();

    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_DCRST_Msk,
               state << FLASH_ACR_DCEN_Pos);
}
/* ------------------------------------------------------------------------- */
