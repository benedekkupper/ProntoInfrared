/**********************************************************************
 * @file    infrared_transmitter.h
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
#ifndef __INFRARED_TRANSMITTER_H_
#define __INFRARED_TRANSMITTER_H_

#include "pronto_hex.h"
#include "etl/delegate.h"

namespace infrared
{
    class transmitter
    {
    public:
        using callback = etl::delegate<void(pronto_hex::raw *)>;

        static transmitter& instance()
        {
            static transmitter t;
            return t;
        }

        template<typename T>
        bool emit(std::unique_ptr<pronto_hex::raw, T> code, bool continuous = false)
        {
            return emit(*code.release(), callback::create<T>(code.get_deleter()), continuous);
        }

        inline bool emit(const pronto_hex::raw &code, bool continuous = false)
        {
            return emit(*const_cast<pronto_hex::raw*>(&code), callback(), continuous);
        }

        /// @brief  Emit an infrared code in either single shot mode, or continuously, until terminated.
        bool emit(pronto_hex::raw &code, callback cbk, bool continuous = false);

        /// @brief  Indicates if an infrared code transmission is in progress.
        bool active() const;

        /// @brief  End the current continuously repeating code transmission. The termination is asynchronous.
        void terminate()
        {
            _repeat = false;
        }

        // only to be used by ISR
        void preload_next_symbol();

    private:
        void load_sequence(const etl::span<const pronto_hex::raw::burst_pair> &seq)
        {
            _remaining_symbols = seq.size() * 2;
            _current_symbol_length = &seq.data()->wLEDflash_on;
            _residual_length = 0;
        }

        void emit_begin();
        void emit_end();

        transmitter();

        callback _complete_cbk = callback();
        const pronto_hex::raw *_current_code = nullptr;
        const pronto_hex::word *_current_symbol_length = nullptr;
        size_t _remaining_symbols = 0;
        pronto_hex::word _residual_length = 0;
        bool _repeat = false;
    };
}

#endif /* __INFRARED_TRANSMITTER_H_ */
