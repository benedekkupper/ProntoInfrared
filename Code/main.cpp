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

pronto_hex::static_raw<2> testsignal_2 ( 37000,
        { 0x0001, 0x0003, 0x0002, 0x0003 }
);
pronto_hex::static_raw<3 + 2> testsignal ( 37000,
        { 0x0006, 0x0002, 0x0004, 0x0002, 0x0004, 0x0006 },
        { 0x0006, 0x0003, 0x0003, 0x000C }
);

extern "C" int app_main()
{
    infrared::transmitter::instance().send(testsignal, nullptr, 3);

    while (1)
    {
        __WFI(); // simplest power saving
    }
}
