/**********************************************************************
 * @file    pronto_hex.hpp
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
#ifndef __PRONTO_HEX_HPP_
#define __PRONTO_HEX_HPP_

#include <array>
#include <cassert>
#include <cstdint>
#include <memory>
#include <span>

using frequency = std::uint32_t;

namespace pronto_hex
{
using word = std::uint16_t;

enum class format : word
{
    // - Raw Formats
    RAW_OSCILLATED = 0x0000,
    RAW_UNMODULATED = 0x0100,
    // - Predefined Formats
    INDEX_TO_UDB = 0x8000,
    // '- Template Based Formats
    PREDEFINED_CODE_OF_VARIABLE_LENGTH = 0x7000,
    //  '- Template Based Formats of Fixed Size
    RC5 = 0x5000,
    RC5X = 0x5001,
    RC6_MODE_0 = 0x6000,
    NEC = 0x9000,
    YAMAHA_NEC = 0x9001,
    NEC_A = 0x900A,
    NEC_B = 0x900B,
    NEC_C = 0x900C,
    NEC_D = 0x900D,
    NEC_E = 0x900E,
};

enum class occurs : unsigned char
{
    ONCE = 0,
    REPEAT = 1
};

// this header is present in all pronto codes,
// but its contents beyond the format ID are only valid for raw formats,
// otherwise they are reserved
struct header
{
    format wFmtID;
    word wFrqDiv;    // Carrier frequency divider = 4,145146 MHz / <signal carrier>
    word nOnceSeq;   // Number of burst pairs at once sequence
    word nRepeatSeq; // Number of burst pairs at repeat sequence
};

class raw
{
    constexpr static size_t WORD_STRING_SIZE = sizeof(word) * 2;

  public:
    constexpr static frequency BASE_FREQ_Hz = 4145146;

    struct burst_pair
    {
        word wLEDflash_on; // Amount of periods when LED flashes with carrier / LED is on
        word wLEDoff;      // Amount of periods when LED is off
    };

    constexpr static bool is_raw_header(const header& h)
    {
        return ((h.wFmtID == format::RAW_OSCILLATED) or (h.wFmtID == format::RAW_UNMODULATED)) and
               (h.wFrqDiv > 1);
    }

    constexpr format get_format() const { return header_.wFmtID; }

    // there are only two types of raw formats
    constexpr bool is_oscillated() const { return get_format() == format::RAW_OSCILLATED; }

    constexpr frequency carrier_frequency() const { return BASE_FREQ_Hz / header_.wFrqDiv; }

    constexpr bool has_carrier_frequency() const { return header_.wFrqDiv > 1; }

    constexpr void set_carrier_frequency(frequency carrier_freq)
    {
        assert(carrier_freq > 0);
        header_.wFrqDiv = BASE_FREQ_Hz / carrier_freq;
    }

    constexpr size_t once_length() const { return header_.nOnceSeq; }

    constexpr size_t repeat_length() const { return header_.nRepeatSeq; }

    constexpr word sequence_count(occurs occur) const
    {
        return (&header_.nOnceSeq)[static_cast<size_t>(occur)];
    }

    constexpr size_t sequence_length(size_t repeats = 1) const
    {
        return once_length() + repeat_length() * repeats;
    }

    std::span<const burst_pair> once_pairs() const
    {
        return std::span<const burst_pair>(
            reinterpret_cast<const burst_pair*>(&header_.wFmtID + sizeof(header_) / sizeof(word)),
            once_length());
    }

    std::span<const burst_pair> repeat_pairs() const
    {
        return std::span<const burst_pair>(
            reinterpret_cast<const burst_pair*>(&header_.wFmtID + sizeof(header_) / sizeof(word) +
                                                once_length() * 2),
            repeat_length());
    }

    size_t to_string(const std::span<char>& buffer) const;

    static std::unique_ptr<raw> from_string(const std::span<const char>& buffer);

  protected:
    constexpr raw(frequency carrier_freq, word once, word repeat = 0,
                  format f = format::RAW_OSCILLATED)
        : header_{f, static_cast<word>(BASE_FREQ_Hz / carrier_freq), once, repeat}
    {
        assert((f == format::RAW_OSCILLATED) or (f == format::RAW_UNMODULATED));
        assert(carrier_freq > 0);
    }

    constexpr raw(format f = format::RAW_OSCILLATED)
        : header_{f, BASE_FREQ_Hz / BASE_FREQ_Hz, 0, 0} // avoid divide by 0
    {
        assert((f == format::RAW_OSCILLATED) or (f == format::RAW_UNMODULATED));
    }

    constexpr word& sequence_count(occurs occur)
    {
        return (&header_.nOnceSeq)[static_cast<size_t>(occur)];
    }

    static bool parse_into(const std::span<const char>& buffer, raw* target, size_t pairs_count);

    constexpr static size_t string_to_word_size(size_t str_size)
    {
        return (str_size + 1) / (WORD_STRING_SIZE + 1);
    }

    constexpr static size_t word_to_string_size(size_t word_size)
    {
        // including separating spaces
        return word_size * (WORD_STRING_SIZE + 1) - 1;
    }

  private:
    size_t word_size() const { return (sizeof(header_) / sizeof(word)) + sequence_length() * 2; }

    header header_;
};

template <const size_t PAIRS_COUNT>
class static_raw : public raw
{
    static_assert(PAIRS_COUNT > 0, "Burst pairs must be present in any raw code");

    using this_type = static_raw<PAIRS_COUNT>;

  public:
    constexpr static size_t max_size() { return PAIRS_COUNT; }

    // create a complete code
    constexpr static_raw(frequency carrier_freq, std::initializer_list<word> once_list,
                         std::initializer_list<word> repeat_list = {})
        : raw(carrier_freq, once_list.size() / 2, repeat_list.size() / 2)
    {
        assert((once_list.size() % 2 == 0) and (repeat_list.size() % 2 == 0));
        assert((once_list.size() + repeat_list.size()) <= (max_size() * 2));

        auto it = once_list.begin();
        for (auto& pair : pairs_)
        {
            pair.wLEDflash_on = *it++;
            pair.wLEDoff = *it++;
            if (it == once_list.end())
            {
                it = repeat_list.begin();
            }
            else if (it == repeat_list.end())
            {
                break;
            }
        }
    }

    // create a code buffer to be filled
    constexpr static_raw(format f = format::RAW_OSCILLATED) : raw(f), pairs_() {}

    constexpr void increment_last_mark(occurs occur = occurs::ONCE)
    {
        auto& nseq = sequence_count(occur);

        // first mark of the sequence
        if (nseq == 0)
        {
            assert((repeat_length() == 0) or (occur == occurs::REPEAT));

            nseq++;
        }

        auto pair = &last_pair();

        // beginning of new mark after space
        if (pair->wLEDoff > 0)
        {
            assert(max_size() > sequence_length());

            nseq++;
            pair++;
        }

        pair->wLEDflash_on++;
    }

    constexpr void append_spaces(word count)
    {
        auto pair = &last_pair();
        assert(pair->wLEDflash_on > 0);

        pair->wLEDoff = count;
    }

    constexpr void clear() { *this = this_type(); }

    bool load_string(const std::span<const char>& buffer)
    {
        return parse_into(buffer, this, max_size());
    }

  private:
    std::array<burst_pair, PAIRS_COUNT> pairs_;

    constexpr burst_pair& last_pair() { return pairs_[sequence_length() - 1]; }
};

} // namespace pronto_hex

#endif // __PRONTO_HEX_HPP_
