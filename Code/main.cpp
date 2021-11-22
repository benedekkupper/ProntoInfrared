/**********************************************************************
 * @file    main.cpp
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   The application's main()
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
#include "main.h"
#include "infrared_transmitter.h"
#include "infrared_receiver.h"

pronto_hex::static_raw<2> testsignal_2 ( 37000,
        { 0x0001, 0x0003, 0x0002, 0x0003 }
);
pronto_hex::static_raw<3 + 2> testsignal ( 37000,
        { 0x0006, 0x0002, 0x0004, 0x0002, 0x0004, 0x0006 },
        { 0x0006, 0x0003, 0x0003, 0x000C }
);

static void received_code(const pronto_hex::raw &code)
{
    // some way to debug the result
    volatile int x = code.carrier_frequency();
    x++;
}

extern "C" int app_main()
{
    // initialize both transmitter and receiver first
    infrared::transmitter::instance();
    infrared::receiver::instance();

    // start receiving the code
    infrared::receiver::instance().start(infrared::receiver::callback::create<&received_code>());

    // send test signal
    infrared::transmitter::instance().send(testsignal, 1);

    while (1)
    {
        __WFI(); // simplest power saving
    }
}
