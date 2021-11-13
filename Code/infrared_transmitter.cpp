/**********************************************************************
 * @file    infrared_transmitter.cpp
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   Infrared transmitter using STM32 timer
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
#include "infrared_transmitter.h"
#include "infrared_config_stm32.h"

using namespace infrared;

transmitter::transmitter()
{
    auto tim = stm32::ir_pwm_timer::tx();

    // use active low drive
    LL_TIM_SetOffStates(tim->regmap(), LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_CC_EnableChannel(tim->regmap(), tim->ll_channel());

    NVIC_EnableIRQ(tim->update_irq());
}

bool transmitter::busy() const
{
    return LL_TIM_IsEnabledCounter(stm32::ir_pwm_timer::tx()->regmap());
}

bool transmitter::send(const pronto_hex::raw &code, complete_callback callback, size_t repeats)
{
    if (busy())
    {
        return false;
    }

    // prepare context
    _current_code = &code;
    _complete_cbk = callback;
    if (code.once_length() > 0)
    {
        // starting with once part of the code
        _remaining_symbols = _current_code->once_length() * 2;
        _current_symbol_length = &_current_code->once_pairs().data()->wLEDflash_on;
        _remaining_repeats = (_current_code->repeat_length() > 0) ? repeats : 0;
    }
    else if (code.repeat_length() > 0)
    {
        // starting with repeating part of the code, send it at least once
        _remaining_symbols = _current_code->once_length() * 2;
        _current_symbol_length = &_current_code->once_pairs().data()->wLEDflash_on;
        _remaining_repeats = repeats > 0 ? repeats - 1 : 0;
    }
    else
    {
        return false;
    }

    auto tim = stm32::ir_pwm_timer::tx();

    // clear timer bits from previous send
    LL_TIM_SetOnePulseMode(tim->regmap(), LL_TIM_ONEPULSEMODE_REPETITIVE);
    LL_TIM_DisableIT_UPDATE(tim->regmap());

    // set the timer clock to 2 * carrier frequency, reload to 2
    uint32_t prescaler = tim->frequency() / (2 * _current_code->carrier_frequency());
    constexpr uint32_t period = 2;
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

    return true;
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

    // when we don't have anything to send, the timer should already be stopped from OPM
    if ((_remaining_symbols + _remaining_repeats) == 0)
    {
        assert(!busy());
        return send_cleanup();
    }
    else if (_remaining_symbols == 0)
    {
        // restart the repeating part
        _remaining_symbols = _current_code->repeat_length() * 2;
        _current_symbol_length = &_current_code->repeat_pairs().data()->wLEDflash_on;
        _remaining_repeats--;
    }

    auto next_length = *_current_symbol_length;
    if (_remaining_symbols == 1)
    {
        // the last symbol (which is a space) is typically too long to fit into 8-bit repetition counter
        // which is only a problem if we're chaining codes
        next_length = std::min(next_length, pronto_hex::word(std::numeric_limits<uint8_t>::max() + 1));

        // TODO: we should instead multiply the prescaler or the reload register

        if (_remaining_repeats == 0)
        {
            // stop the timer after the last symbol
            LL_TIM_SetOnePulseMode(tim->regmap(), LL_TIM_ONEPULSEMODE_SINGLE);
        }
    }
    else
    {
        // this design expects the symbols to be up to 256 long, as this is the HW's limitation
        assert(next_length <= std::numeric_limits<uint8_t>::max());
    }

    // set repetition counter to length of symbol
    LL_TIM_SetRepetitionCounter(tim->regmap(), next_length - 1);

    // ON will produce half / whole active depending on modulated / not
    // OFF will produce whole passive
    bool flash_on = (_remaining_symbols & 1) == 0;
    auto &compare_reg = (&tim->regmap()->CCR1)[tim->channel_index()];
    if (flash_on)
    {
        auto arr = LL_TIM_GetAutoReload(tim->regmap());
        if (_current_code->is_oscillated())
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

    _current_symbol_length++;
    _remaining_symbols--;
}

void transmitter::send_cleanup()
{
    // TODO: delete owned code
    if (_complete_cbk)
    {
        _complete_cbk(*_current_code);
    }
    _current_code = nullptr;
    _complete_cbk = nullptr;
}
