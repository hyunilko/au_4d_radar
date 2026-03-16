/* conversion.hpp
 * Copyright (C) 2026 AU Inc.
 *
 * Author   : AU
 * Desc     : Header file for helper functions
 */

#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <cstdint>
#include <cstring>
#include <cstddef>

class Conversion {
public:
    /* =========================
     * Helper functions for BE/LE encoding/decoding
     * ========================= */

    /* -------------------- 16/24/32/64-bit big-endian -------------------- */
    static inline void u16_to_be(uint16_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>((value >> 8) & 0xFFu);
        buffer[1] = static_cast<uint8_t>( value       & 0xFFu);
    }

    static inline uint16_t be_to_u16(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint16_t>(buffer[0]) << 8) |
                static_cast<uint16_t>(buffer[1]);
    }

    static inline void u24_to_be(uint32_t value, uint8_t* buffer) noexcept
    {
        value &= 0x00FFFFFFu;
        buffer[0] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[2] = static_cast<uint8_t>( value        & 0xFFu);
    }

    static inline uint32_t be_to_u24(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint32_t>(buffer[0]) << 16) |
               (static_cast<uint32_t>(buffer[1]) <<  8) |
                static_cast<uint32_t>(buffer[2]);
    }

    static inline void u32_to_be(uint32_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>((value >> 24) & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buffer[2] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[3] = static_cast<uint8_t>( value        & 0xFFu);
    }

    static inline uint32_t be_to_u32(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint32_t>(buffer[0]) << 24) |
               (static_cast<uint32_t>(buffer[1]) << 16) |
               (static_cast<uint32_t>(buffer[2]) <<  8) |
                static_cast<uint32_t>(buffer[3]);
    }

    static inline void u64_to_be(uint64_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>((value >> 56) & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >> 48) & 0xFFu);
        buffer[2] = static_cast<uint8_t>((value >> 40) & 0xFFu);
        buffer[3] = static_cast<uint8_t>((value >> 32) & 0xFFu);
        buffer[4] = static_cast<uint8_t>((value >> 24) & 0xFFu);
        buffer[5] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buffer[6] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[7] = static_cast<uint8_t>( value        & 0xFFu);
    }

    static inline uint64_t be_to_u64(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint64_t>(buffer[0]) << 56) |
               (static_cast<uint64_t>(buffer[1]) << 48) |
               (static_cast<uint64_t>(buffer[2]) << 40) |
               (static_cast<uint64_t>(buffer[3]) << 32) |
               (static_cast<uint64_t>(buffer[4]) << 24) |
               (static_cast<uint64_t>(buffer[5]) << 16) |
               (static_cast<uint64_t>(buffer[6]) <<  8) |
                static_cast<uint64_t>(buffer[7]);
    }

    /* -------------------- 16/24/32/64-bit little-endian -------------------- */
    static inline void u16_to_le(uint16_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>( value        & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >> 8)  & 0xFFu);
    }

    static inline uint16_t le_to_u16(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint16_t>(buffer[1]) << 8) |
                static_cast<uint16_t>(buffer[0]);
    }

    static inline void u24_to_le(uint32_t value, uint8_t* buffer) noexcept
    {
        value &= 0x00FFFFFFu;
        buffer[0] = static_cast<uint8_t>( value        & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    }

    static inline uint32_t le_to_u24(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint32_t>(buffer[2]) << 16) |
               (static_cast<uint32_t>(buffer[1]) <<  8) |
                static_cast<uint32_t>(buffer[0]);
    }

    static inline void u32_to_le(uint32_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>( value        & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
    }

    static inline uint32_t le_to_u32(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint32_t>(buffer[3]) << 24) |
               (static_cast<uint32_t>(buffer[2]) << 16) |
               (static_cast<uint32_t>(buffer[1]) <<  8) |
                static_cast<uint32_t>(buffer[0]);
    }

    static inline void u64_to_le(uint64_t value, uint8_t* buffer) noexcept
    {
        buffer[0] = static_cast<uint8_t>( value        & 0xFFu);
        buffer[1] = static_cast<uint8_t>((value >>  8) & 0xFFu);
        buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
        buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
        buffer[4] = static_cast<uint8_t>((value >> 32) & 0xFFu);
        buffer[5] = static_cast<uint8_t>((value >> 40) & 0xFFu);
        buffer[6] = static_cast<uint8_t>((value >> 48) & 0xFFu);
        buffer[7] = static_cast<uint8_t>((value >> 56) & 0xFFu);
    }

    static inline uint64_t le_to_u64(const uint8_t* buffer) noexcept
    {
        return (static_cast<uint64_t>(buffer[7]) << 56) |
               (static_cast<uint64_t>(buffer[6]) << 48) |
               (static_cast<uint64_t>(buffer[5]) << 40) |
               (static_cast<uint64_t>(buffer[4]) << 32) |
               (static_cast<uint64_t>(buffer[3]) << 24) |
               (static_cast<uint64_t>(buffer[2]) << 16) |
               (static_cast<uint64_t>(buffer[1]) <<  8) |
                static_cast<uint64_t>(buffer[0]);
    }

    /* -------------------- byte-swap helpers -------------------- */
    static inline uint16_t swap_endian16(uint16_t value) noexcept
    {
        return static_cast<uint16_t>(
            ((value >> 8) & 0x00FFu) |
            ((value << 8) & 0xFF00u));
    }

    static inline uint32_t swap_endian32(uint32_t value) noexcept
    {
        return ((value >> 24) & 0x000000FFu) |
               ((value >>  8) & 0x0000FF00u) |
               ((value <<  8) & 0x00FF0000u) |
               ((value << 24) & 0xFF000000u);
    }

    static inline uint64_t swap_endian64(uint64_t value) noexcept
    {
        return ((value >> 56) & 0x00000000000000FFULL) |
               ((value >> 40) & 0x000000000000FF00ULL) |
               ((value >> 24) & 0x0000000000FF0000ULL) |
               ((value >>  8) & 0x00000000FF000000ULL) |
               ((value <<  8) & 0x000000FF00000000ULL) |
               ((value << 24) & 0x0000FF0000000000ULL) |
               ((value << 40) & 0x00FF000000000000ULL) |
               ((value << 56) & 0xFF00000000000000ULL);
    }

    /* -------------------- float (IEEE-754, 32-bit) -------------------- */
    static inline void float_to_be(float value, uint8_t* buffer) noexcept
    {
        uint32_t bits = 0u;
        std::memcpy(&bits, &value, sizeof(bits));
        u32_to_be(bits, buffer);
    }

    static inline float be_to_float(const uint8_t* buffer) noexcept
    {
        const uint32_t bits = be_to_u32(buffer);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static inline void float_to_le(float value, uint8_t* buffer) noexcept
    {
        uint32_t bits = 0u;
        std::memcpy(&bits, &value, sizeof(bits));
        u32_to_le(bits, buffer);
    }

    static inline float le_to_float(const uint8_t* buffer) noexcept
    {
        const uint32_t bits = le_to_u32(buffer);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    /* native-endian raw float load */
    static inline float convertToFloat(const uint8_t* buffer) noexcept
    {
        float value = 0.0f;
        std::memcpy(&value, buffer, sizeof(value));
        return value;
    }

    /* -------------------- pointer/address helpers -------------------- */
    static inline uintptr_t pointer_to_address(const void* ptr) noexcept
    {
        return reinterpret_cast<uintptr_t>(ptr);
    }

    static inline void* address_to_pointer(uintptr_t addr) noexcept
    {
        return reinterpret_cast<void*>(addr);
    }
};

#endif // CONVERSION_HPP