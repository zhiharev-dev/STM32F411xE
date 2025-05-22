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

#ifndef STM32F4XX_HAL_PWR_H_
#define STM32F4XX_HAL_PWR_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование PWR
 */
#define HAL_PWR_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных PWR
 */
typedef PWR_TypeDef pwr_t;


/**
 * @brief           Определение перечисления масштабирования напряжения PWR
 */
enum pwr_vos {
    PWR_VOS3 = 0b01,
    PWR_VOS2,
    PWR_VOS1,
};


/**
 * @brief           Определение перечисления уровней PVD PWR
 */
enum pwr_pvd_level {
    PWR_PVD_2V2,
    PWR_PVD_2V3,
    PWR_PVD_2V4,
    PWR_PVD_2V5,
    PWR_PVD_2V6,
    PWR_PVD_2V7,
    PWR_PVD_2V8,
    PWR_PVD_2V9,
};


/**
 * @brief           Определение перечисления статусов PVD PWR
 */
enum pwr_pvd_status {
    PWR_VDD_HIGHER_PVD,
    PWR_VDD_LOWER_PVD,
};


/**
 * @brief           Определение структуры данных для настройки PWR
 */
struct pwr_config {
    uint32_t vos;                               /*!< Значение масштабирования напряжения @ref enum pwr_vos */

    uint32_t pvd_enable;                        /*!< Включить PVD @ref hal_state_t  */

    uint32_t pvd_level;                         /*!< Уровень PVD @ref enum pwr_pvd_level */

    uint32_t bkp_regulator_enable;              /*!< Включить резервный регулятор @ref hal_state_t */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_pwr_config(struct pwr_config *conf);

bool hal_pwr_vos_is_ready(void);

void hal_pwr_pvd_it_handler(void);

uint32_t hal_pwr_pvd_status(void);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_pwr_pvd_status_changed_callback(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_PWR_H_ */
