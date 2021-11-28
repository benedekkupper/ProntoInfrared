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

static constexpr size_t PRONTO_STRING_BUFFER_SIZE = 512;

static std::array<char, PRONTO_STRING_BUFFER_SIZE> rx_buffer;
static std::array<char, PRONTO_STRING_BUFFER_SIZE> tx_buffer;

extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef *const uart = &huart2;

static void received_code(const pronto_hex::raw &code)
{
    auto slen = code.to_string(tx_buffer);
    if ((slen > 0) && (slen <= tx_buffer.size() - 2))
    {
        tx_buffer[slen++] = '\n';
        tx_buffer[slen++] = '\r';
        HAL_UART_Transmit_DMA(uart, reinterpret_cast<uint8_t*>(tx_buffer.data()), slen);
    }
}

static void process_uart_rx()
{
    static int start_index = -1;
    static size_t processed_index = 0;

    if ((HAL_UART_GetState(uart) & HAL_UART_STATE_BUSY_RX) != HAL_UART_STATE_BUSY_RX)
    {
        HAL_UART_Receive_DMA(uart, reinterpret_cast<uint8_t*>(rx_buffer.data()), rx_buffer.size());
        start_index = -1;
        processed_index = 0;
    }
    else
    {
        size_t remaining =  __HAL_DMA_GET_COUNTER(uart->hdmarx);
        size_t last_index = rx_buffer.size() - remaining;

        // first find the first 0 digit
        if ((start_index < 0) && (processed_index < last_index))
        {
            for (auto &i = processed_index; i < last_index; i++)
            {
                auto c = rx_buffer[i];
                if (c == '0')
                {
                    start_index = i;
                    break;
                }
            }
        }

        // then find the line termination
        if ((start_index >= 0) && (processed_index < last_index))
        {
            for (auto &i = processed_index; i < last_index; i++)
            {
                auto c = rx_buffer[i];
                if ((c == '\n') || (c == '\r'))
                {
                    auto code_str = etl::span<const char>(&rx_buffer[start_index], processed_index - start_index);
                    auto code = pronto_hex::raw::from_string(code_str);
                    if (code.get() != nullptr)
                    {
                        infrared::transmitter::instance().emit(std::move(code));
                    }

                    HAL_UART_AbortReceive(uart);
                    break;
                }
            }
        }

    }
}

extern "C" int app_main()
{
    using ir_tx = infrared::transmitter;
    using ir_rx = infrared::receiver;

    // initialize both transmitter and receiver first
    ir_tx::instance();
    ir_rx::instance();

    MX_USART2_UART_Init();

    // start receiving the code
    ir_rx::instance().start(ir_rx::callback::create<&received_code>(), ir_rx::occurs::REPEAT);

    // send test signal
    //ir_tx::instance().send(testsignal, 1);

    while (1)
    {
        process_uart_rx();

        __WFI(); // simplest power saving
    }
}
