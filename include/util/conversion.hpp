/* conversion.h
 * Copyright (C) 2026 AU Inc.
 *
 * Author   : AU
 * Desc     : Header file for Helper functions
 */

#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <cstdint>

class Conversion {
public:
    /* =========================
    *  Helper functions for BE encoding/decoding
    * ========================= */

    /* -------------------- 16/24/32-bit big-endian -------------------- */
    static inline void u16_to_be(uint16_t value, uint8_t *buffer) {
        buffer[0] = (uint8_t)((value >> 8) & 0xFFu);
        buffer[1] = (uint8_t)(value & 0xFFu);
    }

    static inline uint16_t be_to_u16(const uint8_t *buffer) {
        return ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }

    static inline void u24_to_be(uint32_t value, uint8_t *buffer) {
        value &= 0xFFFFFFu;
        buffer[0] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[1] = (uint8_t)((value >> 8)  & 0xFFu);
        buffer[2] = (uint8_t)((value >> 0)  & 0xFFu);
    }

    static inline uint32_t be_to_u24(const uint8_t *buffer)
    {
        return ((uint32_t)buffer[0] << 16) |
               ((uint32_t)buffer[1] << 8)  |
               ((uint32_t)buffer[2] << 0);
    }

    static inline void u32_to_be(uint32_t value, uint8_t *buffer) {
        buffer[0] = (uint8_t)((value >> 24) & 0xFFu);
        buffer[1] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[2] = (uint8_t)((value >> 8)  & 0xFFu);
        buffer[3] = (uint8_t)(value & 0xFFu);
    }

    static inline uint32_t be_to_u32(const uint8_t *buffer) {
        return ((uint32_t)buffer[0] << 24) |
               ((uint32_t)buffer[1] << 16) |
               ((uint32_t)buffer[2] << 8)  |
               ((uint32_t)buffer[3] << 0);
    }

    static inline void u64_to_be(uint64_t v, uint8_t* buffer)
    {
        buffer[0] = (uint8_t)((v >> 56) & 0xFF);
        buffer[1] = (uint8_t)((v >> 48) & 0xFF);
        buffer[2] = (uint8_t)((v >> 40) & 0xFF);
        buffer[3] = (uint8_t)((v >> 32) & 0xFF);
        buffer[4] = (uint8_t)((v >> 24) & 0xFF);
        buffer[5] = (uint8_t)((v >> 16) & 0xFF);
        buffer[6] = (uint8_t)((v >> 8) & 0xFF);
        buffer[7] = (uint8_t)(v & 0xFF);
    }

    static inline uint64_t be_to_u64(const uint8_t* buffer)
    {
        return ((uint64_t)buffer[0] << 56) |
               ((uint64_t)buffer[1] << 48) |
               ((uint64_t)buffer[2] << 40) |
               ((uint64_t)buffer[3] << 32) |
               ((uint64_t)buffer[4] << 24) |
               ((uint64_t)buffer[5] << 16) |
               ((uint64_t)buffer[6] <<  8) |
               ((uint64_t)buffer[7] <<  0);
    }
    /* -------------------- 16/24/32-bit little-endian -------------------- */
    static inline void u16_to_le(uint16_t value, uint8_t *buffer) {
        buffer[1] = (uint8_t)((value >> 8) & 0xFFu);
        buffer[0] = (uint8_t)(value & 0xFFu);
    }

    static inline uint16_t le_to_u16(const uint8_t *buffer) {
        return ((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0];
    }

    static inline void u24_to_le(uint32_t value, uint8_t *buffer)
    {
        value &= 0xFFFFFFu;
        buffer[2] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[1] = (uint8_t)((value >> 8)  & 0xFFu);
        buffer[0] = (uint8_t)((value >> 0)  & 0xFFu);
    }

    static inline uint32_t le_to_u24(const uint8_t *buffer) {
        return ((uint32_t)buffer[2] << 16) |
               ((uint32_t)buffer[1] << 8)  |
               (uint32_t)buffer[0];
    }

    static inline void u32_to_le(uint32_t value, uint8_t *buffer) {
        buffer[3] = (uint8_t)((value >> 24) & 0xFFu);
        buffer[2] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[1] = (uint8_t)((value >> 8)  & 0xFFu);
        buffer[0] = (uint8_t)(value & 0xFFu);
    }

    static inline uint32_t le_to_u32(const uint8_t *buffer) {
        return ((uint32_t)buffer[3] << 24) |
               ((uint32_t)buffer[2] << 16) |
               ((uint32_t)buffer[1] << 8)  |
               (uint32_t)buffer[0];
    }

    static inline void u64_to_le(uint64_t value, uint8_t *buffer)
    {
        buffer[7] = (uint8_t)((value >> 56) & 0xFFu);
        buffer[6] = (uint8_t)((value >> 48) & 0xFFu);
        buffer[5] = (uint8_t)((value >> 40) & 0xFFu);
        buffer[4] = (uint8_t)((value >> 32) & 0xFFu);
        buffer[3] = (uint8_t)((value >> 24) & 0xFFu);
        buffer[2] = (uint8_t)((value >> 16) & 0xFFu);
        buffer[1] = (uint8_t)((value >>  8) & 0xFFu);
        buffer[0] = (uint8_t)( value        & 0xFFu);
    }

    static inline uint64_t le_to_u64(const uint8_t *buffer)
    {
        return ((uint64_t)buffer[7] << 56) |
               ((uint64_t)buffer[6] << 48) |
               ((uint64_t)buffer[5] << 40) |
               ((uint64_t)buffer[4] << 32) |
               ((uint64_t)buffer[3] << 24) |
               ((uint64_t)buffer[2] << 16) |
               ((uint64_t)buffer[1] <<  8) |
               (uint64_t)buffer[0];
    }

    /* -------------------- byte-swap helpers -------------------- */
    static inline uint32_t swap_endian32(uint32_t value) {
        return ((value >> 24) & 0x000000FFu) |
               ((value >> 8)  & 0x0000FF00u) |
               ((value << 8)  & 0x00FF0000u) |
               ((value << 24) & 0xFF000000u);
    }

    static inline uint16_t swap_endian16(uint16_t value) {
        return (uint16_t)(((value >> 8) & 0x00FFu) |
                        ((value << 8) & 0xFF00u));
    }

    /* -------------------- float (IEEE-754, 32-bit) -------------------- */
    /* note:
    * - Float ↔ bytes conversion preserves bit patterns (including sign/NaN payloads).
    * - Encodes/decodes in the specified endianness, regardless of platform endianness.
    */

    static inline void float_to_be(float value, uint8_t *buffer) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        u32_to_be(bits, buffer);
    }

    static inline float be_to_float(const uint8_t *buffer) {
        uint32_t bits = be_to_u32(buffer);
        float value;
        memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static inline void float_to_le(float value, uint8_t *buffer) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        u32_to_le(bits, buffer);
    }

    static inline float le_to_float(const uint8_t *buffer) {
        uint32_t bits = le_to_u32(buffer);
        float value;
        memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static inline float convertToFloat(const uint8_t* buffer) {
        return *((float *)buffer);
    }    

    static inline uint32_t pointer_to_address(void *ptr)
    {
        const uintptr_t addr = (uintptr_t)ptr;
        return((uint32_t)addr);
    }

    static inline void* address_to_pointer(uint32_t addr)
    {
        const uintptr_t ptrAddr = (uintptr_t)addr;
        return (void*)ptrAddr;
    }

private:
    static std::mutex bufferMutex;
};

#endif // CONVERSION_HPP
