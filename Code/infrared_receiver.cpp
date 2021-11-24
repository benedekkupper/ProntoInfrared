/**********************************************************************
 * @file    infrared_receiver.cpp
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   Infrared receiver using STM32 timer
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
#include "infrared_receiver.h"

using namespace infrared;

receiver::receiver()
{
    auto tim = stm32::ir_pwm_timer::rx();

    NVIC_EnableIRQ(tim->update_irq());

    // only counter overflow triggers Update IT
    // -> the resets triggered by CCn don't trigger it, only the timeout
    LL_TIM_SetUpdateSource(tim->regmap(), LL_TIM_UPDATESOURCE_COUNTER);

    // CC DMA event is actually triggered by update event
    // -> any of the CC pair can be used for DMA
    // -> both CC captures will be the latest values when the DMA transfers them
    LL_TIM_CC_SetDMAReqTrigger(tim->regmap(), LL_TIM_CCDMAREQUEST_UPDATE);

    // burst DMA mode is used to transfer both CC channels on a single CC event
    LL_TIM_ConfigDMABurst(tim->regmap(), offsetof(TIM_TypeDef, CCR1) / sizeof(uint32_t),
            LL_TIM_DMABURST_LENGTH_2TRANSFERS);

    tim->cc_dma_handle()->XferCpltCallback = &receiver::dma_event;
    tim->cc_dma_handle()->XferHalfCpltCallback = &receiver::dma_event;
    tim->cc_dma_handle()->Parent = reinterpret_cast<void*>(this);
}

bool receiver::busy() const
{
    return LL_TIM_IsEnabledCounter(stm32::ir_pwm_timer::rx()->regmap());
}

void receiver::start()
{
    auto tim = stm32::ir_pwm_timer::rx();

    abort_capture();

    // reset counter value
    LL_TIM_DisableIT_UPDATE(tim->regmap());
    LL_TIM_GenerateEvent_UPDATE(tim->regmap());
    LL_TIM_ClearFlag_UPDATE(tim->regmap());

    // configure DMA
    HAL_DMA_Start_IT(tim->cc_dma_handle(), reinterpret_cast<uintptr_t>(&tim->regmap()->DMAR),
            reinterpret_cast<uintptr_t>(_rx_pwm_buffer.data()),
            sizeof(_rx_pwm_buffer) / sizeof(uint32_t));
    __HAL_TIM_ENABLE_DMA(tim->hal_handle(), tim->cc_dma_request());

    // start the timer
    LL_TIM_EnableIT_UPDATE(tim->regmap());
    LL_TIM_CC_EnableChannel(tim->regmap(), tim->ll_channel_pair());
    LL_TIM_EnableCounter(tim->regmap());
}

void receiver::abort_capture()
{
    auto tim = stm32::ir_pwm_timer::rx();

    LL_TIM_CC_DisableChannel(tim->regmap(), tim->ll_channel_pair());
    __HAL_TIM_DISABLE_DMA(tim->hal_handle(), tim->cc_dma_request());
    HAL_DMA_Abort_IT(tim->cc_dma_handle());

    _period_count = 0;
    _rx_code.clear();
}

bool receiver::start(callback cbk, occurs mode)
{
    if (busy())
    {
        return false;
    }

    _success = false;
    _callback = cbk;
    _mode = mode;
    start();

    return true;
}

void receiver::stop()
{
    if (!busy())
    {
        return;
    }

    abort_capture();

    auto tim = stm32::ir_pwm_timer::rx();

    LL_TIM_DisableCounter(tim->regmap());
    LL_TIM_DisableIT_UPDATE(tim->regmap());
}

size_t receiver::remaining_pairs(__DMA_HandleTypeDef *hdma)
{
    return __HAL_DMA_GET_COUNTER(hdma) * sizeof(uint32_t) / sizeof(stm32::rx_pwm_pair);
}

void receiver::dma_event(__DMA_HandleTypeDef *hdma)
{
    auto this_ = reinterpret_cast<receiver*>(hdma->Parent);
    this_->process_captures(remaining_pairs(hdma));
}

void receiver::timeout()
{
    auto tim = stm32::ir_pwm_timer::rx();

    // flush the remaining DMA data
    if (LL_DMA_IsEnabledChannel(tim->cc_dma_regmap(), tim->cc_dma_channel()))
    {
        // all things being equal, the update interrupt is fired
        // before the DMA interrupt that is triggered by the same update event

        // process any further DMA data
        bool transfer_complete = __HAL_DMA_GET_FLAG(tim->cc_dma_handle(), tim->cc_dma_tc_flag());
        auto remaining = remaining_pairs(tim->cc_dma_handle());
        // the remaining count can be the buffer size if
        // a) there were no transfers
        // b) the whole buffer has been filled, and the register was reset to initial length due to circular mode
        if (transfer_complete || (remaining < _rx_pwm_buffer.size()))
        {
            process_captures(remaining);
        }

        // we have already processed the data for that DMA section,
        // so now just clear the flags to prevent getting callback from DMA ISR after completion
        __HAL_DMA_CLEAR_FLAG(tim->cc_dma_handle(), tim->cc_dma_tc_flag() | tim->cc_dma_ht_flag());

        // complete the detected IR code
        process_timeout();
    }

    // restart/stop capturing
    if ((_mode == occurs::ONCE) && _success)
    {
        stop();
    }
    else
    {
        start();
    }
}

RX_IR_PWM_TIMER_ISR()
{
    auto tim = stm32::ir_pwm_timer::rx();

    if (LL_TIM_IsActiveFlag_UPDATE(tim->regmap()))
    {
        LL_TIM_ClearFlag_UPDATE(tim->regmap());

        // the counter has expired without new falling edge,
        // consider this the end of the IR code
        infrared::receiver::instance().timeout();
    }
}

void receiver::process_captures(size_t remaining_pairs)
{
    size_t captured_pairs = _rx_pwm_buffer.size() - remaining_pairs;
    size_t index = 0;

    if (_period_count == 0)
    {
        // the very first capture pair is invalid, as the register values are from the previous code
        // so discard it
        index++;
        captured_pairs--;

        // run frequency identification algorithm
        _period_count = learn_period(index, captured_pairs);
        if (_period_count == 0)
        {
            abort_capture();
            return;
        }

        assert(!_rx_code.has_carrier_frequency());
        _rx_code.set_carrier_frequency(stm32::ir_pwm_timer::rx()->frequency() / _period_count);
    }
    // we handle half-complete interrupts as well,
    // so only process from the second half of the buffer
    else if (remaining_pairs < (_rx_pwm_buffer.size() / 2))
    {
        index = (_rx_pwm_buffer.size() / 2);
        captured_pairs -= (_rx_pwm_buffer.size() / 2);
    }
    // due to circular DMA, remaining count will reset to initial value when transfer is complete
    else if (remaining_pairs == _rx_pwm_buffer.size())
    {
        index = (_rx_pwm_buffer.size() / 2);
        captured_pairs = (_rx_pwm_buffer.size() / 2);
    }

    for (; captured_pairs > 0; captured_pairs--, index++)
    {
        const auto &pair = _rx_pwm_buffer[index];

        // check valid ON pulse length
        if ((pair.on_count < subtract_tolerance(_period_count, 2)) ||
            (pair.on_count > add_tolerance(_period_count, 2)))
        {
            break; // goto error
        }

        // each pair starts with an ON pulse
        _rx_code.increment_last_mark();

        if (pair.total_count < add_tolerance(_period_count))
        {
            //       _____
            // |____|     |
            // one flash followed by another
        }
        else
        {
            //       ________________
            // |____|    |           |
            //           v           v
            // last baud of flash + space
            _rx_code.append_spaces(get_off_length(pair.total_count));

            if (_rx_code.sequence_length() == _rx_code.max_size())
            {
                // we have reached the end of the code buffer,
                // but the code is longer
                // (the last capture will always be processed as a short flash)
                break; // goto error
            }
        }
    }

    // handle any error from the loop
    if (captured_pairs > 0)
    {
        abort_capture();
    }
}

void receiver::process_timeout()
{
    // complete the detected IR code
    if (_rx_code.has_carrier_frequency())
    {
        auto timeout_count = LL_TIM_GetAutoReload(stm32::ir_pwm_timer::rx()->regmap());
        _rx_code.append_spaces(get_off_length(timeout_count));

        // receive complete
        _success = true;
        if (_callback)
        {
            _callback(_rx_code);
        }
    }
}

uint32_t receiver::learn_period(size_t offset, size_t count)
{
    if (count == 0)
    {
        // return early to avoid divide by 0
        return 0;
    }

    time_accumulator acc;
    for (auto index = offset; count > 0; count--, index++)
    {
        const auto &pair = _rx_pwm_buffer[index];
        size_t off_halfperiods;

        acc.accumulate_on(pair);

        if (is_pair_single_period(pair))
        {
            //       _____
            // |____|     |
            // one flash followed by another
            off_halfperiods = 1;
        }
        else
        {
            //       ________________
            // |____|    |           |
            //           v           v
            // last baud of flash + space
            off_halfperiods = acc.get_off_halfperiods(pair);
        }

        acc.accumulate_periods(pair, (1 + off_halfperiods) / 2);

        // each capture has 1 halfperiod ON, so OFF halfperiods should be odd number
        if ((off_halfperiods % 2) != 1)
        {
            return 0;
        }
        // ON timing is beyond tolerable of average
        if ((acc.average_on() < subtract_tolerance(pair.on_count)) ||
            (acc.average_on() > add_tolerance(pair.on_count)))
        {
            return 0;
        }
        // total timing is beyond tolerable of average
        const auto avg_total = (off_halfperiods + 1) * acc.average_period() / 2;
        if ((avg_total < subtract_tolerance(pair.total_count)) ||
            (avg_total > add_tolerance(pair.total_count)))
        {
            return 0;
        }
    }

    return acc.average_period();
}
