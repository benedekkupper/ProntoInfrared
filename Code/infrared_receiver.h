/**********************************************************************
 * @file    infrared_receiver.h
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
#ifndef __INFRARED_RECEIVER_H_
#define __INFRARED_RECEIVER_H_

#include "pronto_hex.h"
#include "infrared_config_stm32.h"
#include "etl/delegate.h"

struct __DMA_HandleTypeDef;

namespace infrared
{
    class receiver
    {
        static constexpr size_t PRONTO_BURST_PAIR_BUFFER_SIZE = 256;
        static constexpr size_t RX_PWM_BUFFER_SIZE = 64;

    public:
        using callback = etl::delegate<void(const pronto_hex::raw &)>;

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

        static void dma_event(__DMA_HandleTypeDef *hdma);
        static size_t remaining_pairs(__DMA_HandleTypeDef *hdma);

        void process_captures(size_t remaining_pairs);
        void process_timeout();

        uint32_t learn_period(size_t offset, size_t count);

        ETL_CONSTEXPR static unsigned TOLERANCE_BASE = 8;

        template<typename T>
        ETL_CONSTEXPR static T add_tolerance(T in, unsigned divider = 1)
        {
            return (in * (TOLERANCE_BASE / divider + 1) / TOLERANCE_BASE);
        }

        template<typename T>
        ETL_CONSTEXPR static T subtract_tolerance(T in, unsigned divider = 1)
        {
            return (in * (TOLERANCE_BASE / divider - 1) / TOLERANCE_BASE);
        }

        ETL_CONSTEXPR bool is_pair_single_period(const stm32::rx_pwm_pair& pair)
        {
            return (pair.on_count > subtract_tolerance(pair.total_count, 2)) &&
                   (pair.on_count < add_tolerance(pair.total_count, 2));
        }

        pronto_hex::word get_off_length(uint32_t count)
        {
            assert(_period_count);
            return (count - subtract_tolerance(_period_count)) / _period_count;
        }

        struct time_accumulator
        {
            uint32_t _on_count_acc = 0;
            uint32_t _on_counts = 0;
            uint32_t _period_count_acc = 0;
            uint32_t _period_counts = 0;

            ETL_CONSTEXPR uint32_t average_period() const
            {
                return _period_count_acc / _period_counts;
            }

            ETL_CONSTEXPR uint32_t average_on() const
            {
                return _on_count_acc / _on_counts;
            }

            ETL_CONSTEXPR void accumulate_on(const stm32::rx_pwm_pair& pair)
            {
                _on_count_acc += pair.on_count;
                _on_counts++;
            }

            ETL_CONSTEXPR void accumulate_periods(const stm32::rx_pwm_pair& pair, size_t periods)
            {
                _period_count_acc += pair.total_count;
                _period_counts += periods;
            }

            ETL_CONSTEXPR size_t get_off_halfperiods(const stm32::rx_pwm_pair& pair)
            {
                auto tolerated_off = pair.total_count - subtract_tolerance(pair.on_count);
                if (_period_counts == 0)
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


        callback _callback = callback();
        pronto_hex::static_raw<PRONTO_BURST_PAIR_BUFFER_SIZE> _rx_code;

        std::array<stm32::rx_pwm_pair, RX_PWM_BUFFER_SIZE> _rx_pwm_buffer;
        uint32_t _period_count = 0;
        occurs _mode = occurs::ONCE;
        bool _success = false;
    };
}

#endif /* __INFRARED_RECEIVER_H_ */
