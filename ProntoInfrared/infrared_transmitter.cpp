/**********************************************************************
 * @file    infrared_transmitter.cpp
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   Infrared transmitter using STM32 timer
 *
 * This file is part of ProntoInfrared (https://github.com/benedekkupper/ProntoInfrared).
 * Copyright (c) 2024 Benedek Kupper.
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
#include "infrared_transmitter.hpp"
#include "infrared_config_stm32.hpp"

namespace infrared
{

transmitter::transmitter()
{
    auto tim = stm32::ir_pwm_timer::tx();

    // use active low drive
    LL_TIM_SetOffStates(tim->regmap(), LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_CC_EnableChannel(tim->regmap(), tim->ll_channel());

    NVIC_EnableIRQ(tim->update_irq());
}

bool transmitter::active() const
{
    return LL_TIM_IsEnabledCounter(stm32::ir_pwm_timer::tx()->regmap());
}

void transmitter::emit_begin()
{
    auto tim = stm32::ir_pwm_timer::tx();

    // clear timer bits from previous send
    LL_TIM_SetOnePulseMode(tim->regmap(), LL_TIM_ONEPULSEMODE_REPETITIVE);
    LL_TIM_DisableIT_UPDATE(tim->regmap());

    // set the timer clock to 2 * carrier frequency, reload to 2
    constexpr uint32_t period = 2;
    uint32_t prescaler = tim->frequency() / (period * current_code_->carrier_frequency());
    LL_TIM_SetPrescaler(tim->regmap(), prescaler - 1);
    LL_TIM_SetAutoReload(tim->regmap(), period - 1);

    // preload first symbol
    preload_next_symbol();

    // load configuration into registers
    LL_TIM_GenerateEvent_UPDATE(tim->regmap());
    // start the timer
    LL_TIM_EnableCounter(tim->regmap());

    // preload second symbol
    preload_next_symbol();

    // continue preloading following symbols in update ISR
    LL_TIM_EnableIT_UPDATE(tim->regmap());
}

TX_IR_PWM_TIMER_ISR()
{
    if (LL_TIM_IsActiveFlag_UPDATE(stm32::ir_pwm_timer::tx()->regmap()))
    {
        infrared::transmitter::instance().preload_next_symbol();
    }
}

void transmitter::preload_next_symbol()
{
    auto* const tim = stm32::ir_pwm_timer::tx();

    // we most likely got here due to an update event, clear it
    LL_TIM_ClearFlag_UPDATE(tim->regmap());

    // handle end of sequence
    if (remaining_symbols_ == 0)
    {
        if (repeat_ or active())
        {
            // restart the repeating part
            load_sequence(current_code_->repeat_pairs());
        }
        else
        {
            // when we don't have anything to send, the timer should already be stopped from OPM
            return emit_end();
        }
    }

    // the repetition counter is 8-bits wide only, and there are codes that use longer symbols
    // so add this logic to split longer symbols into multiple iterations
    auto next_length = residual_length_;
    if (next_length == 0)
    {
        next_length = *current_symbol_length_;
    }
    if (next_length <= stm32::ir_pwm_timer::max_repetitions())
    {
        residual_length_ = 0;
    }
    else
    {
        residual_length_ = next_length - stm32::ir_pwm_timer::max_repetitions();
        next_length = stm32::ir_pwm_timer::max_repetitions();
    }

    // set repetition counter to length of symbol
    LL_TIM_SetRepetitionCounter(tim->regmap(), next_length - 1);

    // ON will produce half / whole active depending on modulated / not
    // OFF will produce whole passive
    bool flash_on = (remaining_symbols_ & 1) == 0;
    auto& compare_reg = (&tim->regmap()->CCR1)[tim->channel_index()];
    if (flash_on)
    {
        auto arr = LL_TIM_GetAutoReload(tim->regmap());
        if (current_code_->is_oscillated())
        {
            // modulate the symbol with carrier frequency
            compare_reg = (arr + 1) / 2;
        }
        else
        {
            // unmodulated
            compare_reg = arr;
        }
    }
    else
    {
        // OFF
        compare_reg = 0;
    }

    // this symbol doesn't need to be extended any longer
    if (residual_length_ == 0)
    {
        // move on to the next symbol type
        current_symbol_length_++;
        remaining_symbols_--;

        if ((remaining_symbols_ == 0) and !repeat_)
        {
            // stop the timer after the last symbol
            LL_TIM_SetOnePulseMode(tim->regmap(), LL_TIM_ONEPULSEMODE_SINGLE);
        }
    }
}

bool transmitter::emit(pronto_hex::raw& code, callback cbk, bool continuous)
{
    if (active())
    {
        return false;
    }

    if (code.once_length() > 0)
    {
        // starting with once part of the code
        load_sequence(code.once_pairs());
    }
    else if (code.repeat_length() > 0)
    {
        // starting with repeating part of the code, send it at least once
        load_sequence(code.repeat_pairs());
    }
    else
    {
        return false;
    }

    // save context
    repeat_ = continuous;
    current_code_ = &code;
    complete_cbk_ = cbk;

    emit_begin();
    return true;
}

void transmitter::emit_end()
{
    // delete owned code
    if (complete_cbk_)
    {
        complete_cbk_(const_cast<pronto_hex::raw*>(current_code_));
    }
    current_code_ = nullptr;
    complete_cbk_ = {};
}

} // namespace infrared
