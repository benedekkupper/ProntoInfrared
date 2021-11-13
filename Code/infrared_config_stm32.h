/**********************************************************************
 * @file    infrared_config_stm32.h
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   STM32 timer configuration for Infrared
 *
 * This file is part of ProntoInfrared (https://github.com/benedekkupper/ProntoInfrared).
 * Copyright (c) 2021 Benedek Kupper.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __INFRARED_CONFIG_STM32_H_
#define __INFRARED_CONFIG_STM32_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

namespace stm32
{
    class ir_pwm_timer
    {
    public:
        //*************************************************************
        //* the actual configuration section

        static ir_pwm_timer* tx()
        {
            // TIM15 channel 2 is PWM output
            static ir_pwm_timer tim(TIM15, TIM_CHANNEL_2, &MX_TIM15_Init, TIM1_BRK_TIM15_IRQn);
#define TX_IR_PWM_TIMER_ISR()       extern "C" void TIM1_BRK_TIM15_IRQHandler()

            // requirements for IR TX timer:
            assert(IS_TIM_REPETITION_COUNTER_INSTANCE(tim.regmap()));

            return &tim;
        }

        static ir_pwm_timer* rx()
        {
            // TIM2 channel 2 is direct capture channel, DMA is connected to pair channel 1
            static ir_pwm_timer tim(TIM2, TIM_CHANNEL_2, &MX_TIM2_Init, TIM2_IRQn,
                                    &htim2, TIM_DMA_ID_CC1);
#define RX_IR_PWM_TIMER_ISR()       extern "C" void TIM2_IRQHandler()

            // requirements for IR RX timer:
            assert(IS_TIM_32B_COUNTER_INSTANCE(tim.regmap())); // not a hard requirement actually
            assert(IS_TIM_DMA_CC_INSTANCE(tim.regmap()));
            assert(IS_TIM_DMABURST_INSTANCE(tim.regmap()));

            return &tim;
        }

        struct rx_pwm_pair_
        {
            // swap these counts depending on which is direct capture channel
            // CH1: total, on
            // CH2: on, total
            uint32_t on_count;
            uint32_t total_count;
        };

        uint32_t frequency() const
        {
            /* Things to consider to be able to definitively answer this:
             * 1. What's the core clock? -> SystemCoreClock
             * 2. Which APB bus is the TIMx connected to? -> (TIMx < APB2PERIPH_BASE) ? APB1 : APB2
             * 3. Is the TIMx connected to a special source clock? then use that value, otherwise continue to 4.
             * 4. Is the APB bus clock divided? -> divide SystemCoreClock by the divider, but multiply by 2 afterwards
             *
             * In this configuration the APB buses don't have any division, so the answer is straightforward: */
            return SystemCoreClock;
        }
        //*
        //*************************************************************

        // helper API
        TIM_TypeDef *regmap() const
        {
            return _timx;
        }

        constexpr uint32_t channel_index() const
        {
            return _channel / (TIM_CHANNEL_2 - TIM_CHANNEL_1);
        }

        constexpr uint32_t hal_channel() const
        {
            return _channel;
        }

        constexpr uint32_t ll_channel() const
        {
            return ll_channel(_channel);
        }

        constexpr uint32_t ll_channel_pair() const
        {
            // PWM input mode is only supported on channel 1+2 on all timers
            return ll_channel(TIM_CHANNEL_1) | ll_channel(TIM_CHANNEL_2);
        }

        struct __DMA_HandleTypeDef *cc_dma() const
        {
            assert(_htimx != nullptr);
            return _htimx->hdma[_dma_id];
        }

        constexpr IRQn_Type update_irq() const
        {
            return _update_irqn;
        }

    private:
        TIM_TypeDef *_timx;
        TIM_HandleTypeDef *_htimx;
        IRQn_Type _update_irqn;
        uint8_t _channel;
        uint8_t _dma_id;

        constexpr static uint32_t ll_channel(uint8_t channel)
        {
            return 1 << (channel * 4 / (TIM_CHANNEL_2 - TIM_CHANNEL_1));
        }

        ir_pwm_timer(TIM_TypeDef *TIMx, uint32_t TIM_CHANNEL_N, void (*MX_Init)(), IRQn_Type Update_IRQn,
                TIM_HandleTypeDef *htimx = nullptr, uint16_t TIM_DMA_ID_CCy = 0)
            : _timx(TIMx)
            , _htimx(htimx)
            , _update_irqn(Update_IRQn)
            , _channel(TIM_CHANNEL_N)
            , _dma_id(TIM_DMA_ID_CCy)
        {
            assert(TIM_DMA_ID_CCy < (sizeof(htimx->hdma) / sizeof(htimx->hdma[0])));
            MX_Init();
        }

        ~ir_pwm_timer()
        {
            if (_htimx != nullptr)
            {
                HAL_TIM_PWM_DeInit(_htimx);
            }
            else
            {
                LL_TIM_DeInit(_timx);
            }
        }
    };

    using rx_pwm_pair = ir_pwm_timer::rx_pwm_pair_;
}

#endif /* __INFRARED_CONFIG_STM32_H_ */