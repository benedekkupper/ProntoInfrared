/**********************************************************************
 * @file    infrared_receiver.h
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   Infrared receiver using STM32 timer
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
#ifndef __INFRARED_RECEIVER_HPP_
#define __INFRARED_RECEIVER_HPP_

#include "etl/delegate.h"
#include "infrared_config_stm32.hpp"
#include "pronto_hex.hpp"

struct __DMA_HandleTypeDef;

namespace infrared
{
class receiver
{
    static constexpr size_t PRONTO_BURST_PAIR_BUFFER_SIZE = 256;
    static constexpr size_t RX_PWM_BUFFER_SIZE = 64;

  public:
    using callback = etl::delegate<void(const pronto_hex::raw&)>;

    static receiver& instance()
    {
        static receiver r;
        return r;
    }

    using occurs = pronto_hex::occurs;

    bool start(callback cbk, occurs mode = occurs::ONCE);
    void stop();

    bool busy() const;

    // only to be used by ISR
    void timeout();

  private:
    receiver();

    void start();
    void abort_capture();

    static void dma_event(__DMA_HandleTypeDef* hdma);
    static size_t remaining_pairs(__DMA_HandleTypeDef* hdma);

    void process_captures(size_t remaining_pairs);
    void process_timeout();

    uint32_t learn_period(size_t offset, size_t count);

    constexpr static unsigned TOLERANCE_BASE = 4;

    template <typename T>
    constexpr static T add_tolerance(T in, unsigned divider = 1)
    {
        return (in * (TOLERANCE_BASE / divider + 1) / TOLERANCE_BASE);
    }

    template <typename T>
    constexpr static T subtract_tolerance(T in, unsigned divider = 1)
    {
        return (in * (TOLERANCE_BASE / divider - 1) / TOLERANCE_BASE);
    }

    constexpr bool is_pair_single_period(const stm32::rx_pwm_pair& pair)
    {
        return (pair.on_count > subtract_tolerance(pair.total_count, 2)) and
               (pair.on_count < add_tolerance(pair.total_count, 2));
    }

    pronto_hex::word get_off_length(uint32_t count)
    {
        assert(period_count_);
        return (count - subtract_tolerance(period_count_)) / period_count_;
    }

    class time_accumulator
    {
        uint32_t on_count_acc_ = 0;
        uint32_t on_counts_ = 0;
        uint32_t period_count_acc_ = 0;
        uint32_t period_counts_ = 0;

      public:
        constexpr uint32_t average_period() const { return period_count_acc_ / period_counts_; }

        constexpr uint32_t average_on() const { return on_count_acc_ / on_counts_; }

        constexpr void accumulate_on(const stm32::rx_pwm_pair& pair)
        {
            on_count_acc_ += pair.on_count;
            on_counts_++;
        }

        constexpr void accumulate_periods(const stm32::rx_pwm_pair& pair, size_t periods)
        {
            period_count_acc_ += pair.total_count;
            period_counts_ += periods;
        }

        constexpr size_t get_off_halfperiods(const stm32::rx_pwm_pair& pair)
        {
            auto tolerated_off = pair.total_count - subtract_tolerance(pair.on_count);
            if (period_counts_ == 0)
            {
                // no prior info about period, so
                // we'd have to calculate the period based on the ON duration
                return tolerated_off / pair.on_count;
            }
            else
            {
                return 2 * tolerated_off / average_period();
            }
        }
    };

    callback callback_ = callback();
    pronto_hex::static_raw<PRONTO_BURST_PAIR_BUFFER_SIZE> rx_code_;

    std::array<stm32::rx_pwm_pair, RX_PWM_BUFFER_SIZE> rx_pwm_buffer_;
    uint32_t period_count_ = 0;
    occurs mode_ = occurs::ONCE;
    bool success_ = false;
};

} // namespace infrared

#endif // __INFRARED_RECEIVER_HPP_
