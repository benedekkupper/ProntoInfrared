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
        bool send(std::unique_ptr<pronto_hex::raw, T> code, size_t repeats = 0)
        {
            return send(*code.release(), callback::create<T>(code.get_deleter()), repeats);
        }

        bool send(pronto_hex::raw &code, callback cbk, size_t repeats = 0);

        inline bool send(const pronto_hex::raw &code, size_t repeats = 0)
        {
            return send(*const_cast<pronto_hex::raw*>(&code), callback(), repeats);
        }

        bool busy() const;

        // only to be used by ISR
        void preload_next_symbol();

    private:
        void send_cleanup();

        transmitter();

        callback _complete_cbk = callback();
        const pronto_hex::raw *_current_code = nullptr;
        const pronto_hex::word *_current_symbol_length = nullptr;
        size_t _remaining_symbols = 0;
        size_t _remaining_repeats = 0;
    };
}

#endif /* __INFRARED_TRANSMITTER_H_ */
