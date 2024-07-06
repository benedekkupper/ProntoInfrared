/**********************************************************************
 * @file    pronto_hex.cpp
 * @author  Benedek Kupper
 * @version 1.0
 * @brief   Pronto HEX infrared code format
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
#include "pronto_hex.hpp"
#include <cctype>
#include <cstdlib>

namespace pronto_hex
{

size_t raw::to_string(const std::span<char>& buffer) const
{
    // check if we fit in the buffer, including termination
    if (buffer.size() < (word_to_string_size(word_size()) + 1))
    {
        buffer[0] = '\0';
        return 0;
    }

    // convert word-by-word
    size_t out_offset = 0;
    auto words = std::span<const word>(reinterpret_cast<const word*>(this), word_size());
    for (auto w : words)
    {
        size_t i = out_offset + WORD_STRING_SIZE;

        // word to chars, starting from lowest nibble
        while (w > 0)
        {
            char nibble = static_cast<char>(w & 0xF);
            i--;
            buffer[i] = (nibble < 0xA) ? ('0' + nibble) : ('A' + nibble - 0xA);
            w >>= 4;
        }
        // always fill with zeros to same width
        while (i > out_offset)
        {
            i--;
            buffer[i] = '0';
        }
        out_offset += WORD_STRING_SIZE;

        // add separator
        buffer[out_offset] = ' ';
        out_offset++;
    }

    // add termination, replacing the last ' '
    buffer[out_offset - 1] = '\0';
    return out_offset - 1;
}

bool raw::parse_into(const std::span<const char>& buffer, raw* target, size_t pairs_count)
{
    size_t len = string_to_word_size(buffer.size());

    // check for size error
    if (len < (pairs_count * 2))
    {
        return false;
    }

    // parse words
    auto words = reinterpret_cast<word*>(target);
    for (word *w = &words[0], offset = 0; w < &words[len]; w++, offset += (WORD_STRING_SIZE + 1))
    {
        char* wend;

        *w = strtol(&buffer[offset], &wend, 16);

        // check that strtol() processed exact number of characters, and that the next is a space
        if ((wend != &buffer[offset + WORD_STRING_SIZE]) or
            ((wend <= &buffer.back()) and !isspace(*wend)))
        {
            // invalidate header and abort
            target->header_.wFrqDiv = 0;
            break;
        }
    }

    // now check the header to see if valid RAW format
    if (!is_raw_header(target->header_) or (target->word_size() != len))
    {
        // TODO: maybe clear the header
        return false;
    }
    else
    {
        return true;
    }
}

std::unique_ptr<raw> raw::from_string(const std::span<const char>& buffer)
{
    // check for size error
    if ((buffer.size() % (WORD_STRING_SIZE + 1)) != WORD_STRING_SIZE)
    {
        return {};
    }

    // convert string length to word length
    size_t len = string_to_word_size(buffer.size());

    // allocate word buffer
    auto ptr = reinterpret_cast<raw*>(new word[len]);
    if ((ptr != nullptr) and parse_into(buffer, ptr, len / 2))
    {
        return std::unique_ptr<raw>(ptr);
    }
    else
    {
        return {};
    }
}

} // namespace pronto_hex
