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

using complete_callback = void (*)(const pronto_hex::raw &);

namespace infrared
{
    class transmitter
    {
    public:
        static transmitter& instance()
        {
            static transmitter t;
            return t;
        }

        bool send(const pronto_hex::raw &code, complete_callback callback = nullptr, size_t repeats = 0);

        bool busy() const;

        // only to be used by ISR
        void preload_next_symbol();

    private:
        void send_cleanup();

        transmitter();

        complete_callback _complete_cbk = nullptr;
        const pronto_hex::raw *_current_code = nullptr;
        const pronto_hex::word *_current_symbol_length = nullptr;
        size_t _remaining_symbols = 0;
        size_t _remaining_repeats = 0;
    };
}

#endif /* __INFRARED_TRANSMITTER_H_ */
