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

#include "stm32f4xx_hal_pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_pwr_disable_bkp_protect(void);

static void hal_pwr_setup_vos(uint32_t vos);

static void hal_pwr_setup_pvd(uint32_t state, uint32_t level);

static void hal_pwr_setup_bkp_regulator(uint32_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Настроить PWR
 *
 * @param[in]       conf: Указатель на структуру данных для настройки PWR
 */
void hal_pwr_config(struct pwr_config *conf)
{
    assert(conf != NULL);

    hal_pwr_setup_vos(conf->vos);
    hal_pwr_setup_pvd(conf->pvd_enable, conf->pvd_level);
    hal_pwr_disable_bkp_protect();
    hal_pwr_setup_bkp_regulator(conf->bkp_regulator_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить защиту резервного домена
 */
static void hal_pwr_disable_bkp_protect(void)
{
    SET_BIT(PWR->CR, PWR_CR_DBP_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить VOS
 *
 * @param[in]       vos: Масштабирование напряжения PWR @ref enum pwr_vos
 */
static void hal_pwr_setup_vos(uint32_t vos)
{
    MODIFY_REG(PWR->CR,
               PWR_CR_VOS_Msk,
               vos << PWR_CR_VOS_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PVD
 *
 * @param[in]       state: Состояние PVD @ref hal_state_t
 * @param[in]       level: Уровень PVD @ref enum pwr_pvd_level
 */
static void hal_pwr_setup_pvd(uint32_t state, uint32_t level)
{
    /* Настроить PVD */
    MODIFY_REG(PWR->CR,
               PWR_CR_PVDE_Msk
             | PWR_CR_PLS_Msk,
               state << PWR_CR_PVDE_Pos
             | level << PWR_CR_PLS_Pos);

    /* EXTI PVD output */
    MODIFY_REG(EXTI->IMR,
               EXTI_IMR_MR16_Msk,
               state << EXTI_IMR_MR16_Pos);
    MODIFY_REG(EXTI->RTSR,
               EXTI_RTSR_TR16_Msk,
               state << EXTI_RTSR_TR16_Pos);
    MODIFY_REG(EXTI->FTSR,
               EXTI_FTSR_TR16_Msk,
               state << EXTI_FTSR_TR16_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить резервный регулятор
 *
 * @param[in]       state: Состояние @ref hal_state_t
 */
static void hal_pwr_setup_bkp_regulator(uint32_t state)
{
    MODIFY_REG(PWR->CSR,
               PWR_CSR_BRE_Msk,
               state << PWR_CSR_BRE_Pos);

    if (state == HAL_ENABLE) {
        while (!READ_BIT(PWR->CSR, PWR_CSR_BRR_Msk))
            continue;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить готовность VOS
 *
 * @return          Состояние @ref bool
 */
inline bool hal_pwr_vos_is_ready(void)
{
    return READ_BIT(PWR->CSR, PWR_CSR_VOSRDY_Msk) ? true : false;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание PWR PVD
 */
void hal_pwr_pvd_it_handler(void)
{
    /* Проверить прерывание EXTI16 */
    if (READ_BIT(EXTI->PR, EXTI_PR_PR16_Msk)) {
        /* Сбросить прерывание EXTI16 */
        SET_BIT(EXTI->PR, EXTI_PR_PR16_Msk);

        /* Вызвать Callback функцию */
        hal_pwr_pvd_status_changed_callback();
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Получить статус PVD PWR
 *
 * @return          Статус @ref enum pwr_pvd_status
 */
inline uint32_t hal_pwr_pvd_status(void)
{
    return READ_BIT(PWR->CSR, PWR_CSR_PVDO_Msk) ?
            PWR_VDD_LOWER_PVD : PWR_VDD_HIGHER_PVD;
}
/* ------------------------------------------------------------------------- */

void hal_pwr_pvd_status_changed_callback(void)
{

}
/* ------------------------------------------------------------------------- */
