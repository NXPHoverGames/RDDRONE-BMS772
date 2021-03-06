/*
 *
 * UAVCAN common serialization support routines.                                                                +-+ +-+
 *
 * Not meant to be included directly by a user. Please include nunavut.h
 * or nunavut_le.h instead, depending on your platform architecture.
 *
 *                                                                                                            \  -  /
 *  This software is distributed under the terms of the MIT License.                                            ---
 *                                                                                                               o
 * +------------------------------------------------------------------------------------------------------------------+
 */
#ifndef NUNAVUT_SUPPORT_NUNAVUT_COMMON_H_INCLUDED
#define NUNAVUT_SUPPORT_NUNAVUT_COMMON_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <float.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

typedef uint32_t NunavutUnsignedBitLength;
typedef int32_t  NunavutSignedBitLength;
typedef float    NunavutFloat32;
typedef double   NunavutFloat64;

// appropriate name?
// do we typedef this, leave it anon, or enum NAME
#define NUNAVUT_ERR_INVALID_BUF 1
#define NUNAVUT_ERR_INVALID_LEN 2
#define NUNAVUT_ERR_INVALID_OFFSET 3
#define NUNAVUT_ERR_INVALID_TAG 4

/// By default, this macro resolves to the standard assert(). The user can redefine this if necessary.
/// To disable assertion checks completely, make it expand into `(void)(0)`.
#ifndef NUNAVUT_ASSERT
// Intentional violation of MISRA: assertion macro cannot be replaced with a function definition.
#    define NUNAVUT_ASSERT(x) assert(x)  // NOSONAR
#endif

#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#    error "Unsupported language: ISO C11 or a newer version is required."
#endif

/// Detect whether the target platform is compatible with IEEE 754.
#define NUNAVUT_PLATFORM_IEEE754_FLOAT \
    ((FLT_RADIX == 2) && (FLT_MANT_DIG == 24) && (FLT_MIN_EXP == -125) && (FLT_MAX_EXP == 128))
#define NUNAVUT_PLATFORM_IEEE754_DOUBLE \
    ((FLT_RADIX == 2) && (DBL_MANT_DIG == 53) && (DBL_MIN_EXP == -1021) && (DBL_MAX_EXP == 1024))

// --------------------------------------------- COMMON ITEMS ---------------------------------------------

/// Per the DSDL specification, 1 byte = 8 bits.
#define NUNAVUT_BYTE_WIDTH 8U
#define NUNAVUT_BYTE_MAX 0xFFU

#define NUNAVUT_WIDTH16 16U
#define NUNAVUT_WIDTH32 32U
#define NUNAVUT_WIDTH64 64U

// --------------------------------------------- PRIMITIVE SERIALIZATION ---------------------------------------------

static inline NunavutUnsignedBitLength nunavutInternalChooseMin(NunavutUnsignedBitLength a, NunavutUnsignedBitLength b)
{
    return (a < b) ? a : b;
}

static inline NunavutUnsignedBitLength nunavutInternalGetBitCopySize(
    const size_t                   buf_size_bytes,
    const NunavutUnsignedBitLength offset_bit,
    const NunavutUnsignedBitLength requested_length_bit,
    const uint8_t                  value_length_bit)
{
    const NunavutUnsignedBitLength buf_size_bit  = buf_size_bytes * NUNAVUT_BYTE_WIDTH;
    const NunavutUnsignedBitLength remaining_bit = buf_size_bit - nunavutInternalChooseMin(buf_size_bit, offset_bit);
    return nunavutInternalChooseMin(remaining_bit, nunavutInternalChooseMin(requested_length_bit, value_length_bit));
}

// --------------------------------------------- PUBLIC API - BIT ARRAY ---------------------------------------------

static inline void nunavutCopyBits(const NunavutUnsignedBitLength length_bit,
                                   const NunavutUnsignedBitLength src_offset_bit,
                                   const NunavutUnsignedBitLength dst_offset_bit,
                                   const uint8_t* const           src,
                                   uint8_t* const                 dst)
{
    // The algorithm was originally designed by Ben Dyer for Libuavcan v0:
    // https://github.com/UAVCAN/libuavcan/blob/ba6929f9625d7ea3eb00/libuavcan/src/marshal/uc_bit_array_copy.cpp#L12-L58
    // This version is modified for v1 where the bit order is the opposite.
    NUNAVUT_ASSERT((src != NULL) && (dst != NULL) && (src != dst));
    NUNAVUT_ASSERT((src < dst)
                       ? ((src + ((src_offset_bit + length_bit + NUNAVUT_BYTE_WIDTH) / NUNAVUT_BYTE_WIDTH)) <= dst)
                       : ((dst + ((dst_offset_bit + length_bit + NUNAVUT_BYTE_WIDTH) / NUNAVUT_BYTE_WIDTH)) <= src));
    if ((0U == (length_bit % NUNAVUT_BYTE_WIDTH)) &&      //
        (0U == (src_offset_bit % NUNAVUT_BYTE_WIDTH)) &&  //
        (0U == (dst_offset_bit % NUNAVUT_BYTE_WIDTH)))
    {
        // Intentional violation of MISRA: Pointer arithmetics.
        // This is done to remove the API constraint that offsets be under 8 bits.
        // Fewer constraints reduce the chance of API misuse.
        (void) memcpy(dst + (dst_offset_bit / NUNAVUT_BYTE_WIDTH),  // NOSONAR NOLINT
                      src + (src_offset_bit / NUNAVUT_BYTE_WIDTH),  // NOSONAR
                      length_bit / NUNAVUT_BYTE_WIDTH);
    }
    else
    {
        NunavutUnsignedBitLength       src_off  = src_offset_bit;
        NunavutUnsignedBitLength       dst_off  = dst_offset_bit;
        const NunavutUnsignedBitLength last_bit = src_off + length_bit;
        while (last_bit > src_off)
        {
            const uint8_t src_mod = (uint8_t)(src_off % NUNAVUT_BYTE_WIDTH);
            const uint8_t dst_mod = (uint8_t)(dst_off % NUNAVUT_BYTE_WIDTH);
            const uint8_t max_mod = (src_mod > dst_mod) ? src_mod : dst_mod;

            const uint8_t size = (uint8_t) nunavutInternalChooseMin(NUNAVUT_BYTE_WIDTH - max_mod, last_bit - src_off);
            NUNAVUT_ASSERT((size > 0U) && (size <= NUNAVUT_BYTE_WIDTH));

            // Suppress a false warning from Clang-Tidy & Sonar that size is being over-shifted. It's not.
            const uint8_t mask = (uint8_t)((((1U << size) - 1U) << dst_mod) & NUNAVUT_BYTE_MAX);  // NOLINT NOSONAR
            NUNAVUT_ASSERT(mask > 0U);

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            const uint8_t in = (uint8_t)((uint8_t)(src[src_off / NUNAVUT_BYTE_WIDTH] >> src_mod) << dst_mod) &
                               NUNAVUT_BYTE_MAX;  // NOSONAR

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            const uint8_t a = dst[dst_off / NUNAVUT_BYTE_WIDTH] & ((uint8_t) ~mask);  // NOSONAR
            const uint8_t b = in & mask;

            // Intentional violation of MISRA: indexing on a pointer.
            // This simplifies the implementation greatly and avoids pointer arithmetics.
            dst[dst_off / NUNAVUT_BYTE_WIDTH] = a | b;  // NOSONAR

            src_off += size;
            dst_off += size;
        }
        NUNAVUT_ASSERT(last_bit == src_off);
    }
}

// --------------------------------------------- PUBLIC API - INTEGER ---------------------------------------------

static inline void nunavutSetBit(uint8_t* const buf, const NunavutUnsignedBitLength off_bit, const bool value)
{
    NUNAVUT_ASSERT(buf != NULL);
    const uint8_t val = value ? 1U : 0U;
    nunavutCopyBits(1U, 0U, off_bit, &val, buf);
}

static inline void nunavutSetUxx(uint8_t* const                 buf,
                                 const NunavutUnsignedBitLength off_bit,
                                 const uint64_t                 value,
                                 const uint8_t                  len_bit)
{
    _Static_assert(NUNAVUT_WIDTH64 == (sizeof(uint64_t) * NUNAVUT_BYTE_WIDTH), "Unexpected size of uint64_t");
    NUNAVUT_ASSERT(buf != NULL);
    const NunavutUnsignedBitLength saturated_len_bit = nunavutInternalChooseMin(len_bit, NUNAVUT_WIDTH64);
    nunavutCopyBits(saturated_len_bit, 0U, off_bit, (const uint8_t*) &value, buf);
}

static inline void nunavutSetIxx(uint8_t* const                 buf,
                                 const NunavutUnsignedBitLength off_bit,
                                 const int64_t                  value,
                                 const uint8_t                  len_bit)
{
    // The naive sign conversion is safe and portable according to the C standard:
    // 6.3.1.3.3: if the new type is unsigned, the value is converted by repeatedly adding or subtracting one more
    // than the maximum value that can be represented in the new type until the value is in the range of the new type.
    nunavutSetUxx(buf, off_bit, (uint64_t) value, len_bit);
}

static inline uint8_t nunavutGetU8(const uint8_t* const           buf,
                                   const size_t                   buf_size,
                                   const NunavutUnsignedBitLength off_bit,
                                   const uint8_t                  len_bit);

static inline bool nunavutGetBit(const uint8_t* const           buf,
                                 const size_t                   buf_size,
                                 const NunavutUnsignedBitLength off_bit)
{
    return 1U == nunavutGetU8(buf, buf_size, off_bit, 1U);
}

static inline uint8_t nunavutGetU8(const uint8_t* const           buf,
                                   const size_t                   buf_size,
                                   const NunavutUnsignedBitLength off_bit,
                                   const uint8_t                  len_bit)
{
    NUNAVUT_ASSERT(buf != NULL);
    const NunavutUnsignedBitLength copy_size =
        nunavutInternalGetBitCopySize(buf_size, off_bit, len_bit, NUNAVUT_BYTE_WIDTH);
    NUNAVUT_ASSERT(copy_size <= (sizeof(uint8_t) * NUNAVUT_BYTE_WIDTH));
    uint8_t val = 0;
    nunavutCopyBits(copy_size, off_bit, 0U, buf, &val);
    return val;
}

static inline uint16_t nunavutGetU16(const uint8_t* const           buf,
                                     const size_t                   buf_size,
                                     const NunavutUnsignedBitLength off_bit,
                                     const uint8_t                  len_bit)
{
    NUNAVUT_ASSERT(buf != NULL);
    const NunavutUnsignedBitLength copy_size =
        nunavutInternalGetBitCopySize(buf_size, off_bit, len_bit, NUNAVUT_WIDTH16);
    NUNAVUT_ASSERT(copy_size <= (sizeof(uint16_t) * NUNAVUT_BYTE_WIDTH));
    uint16_t val = 0U;
    nunavutCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
}

static inline uint32_t nunavutGetU32(const uint8_t* const           buf,
                                     const size_t                   buf_size,
                                     const NunavutUnsignedBitLength off_bit,
                                     const uint8_t                  len_bit)
{
    NUNAVUT_ASSERT(buf != NULL);
    const NunavutUnsignedBitLength copy_size =
        nunavutInternalGetBitCopySize(buf_size, off_bit, len_bit, NUNAVUT_WIDTH32);
    NUNAVUT_ASSERT(copy_size <= (sizeof(uint32_t) * NUNAVUT_BYTE_WIDTH));
    uint32_t val = 0U;
    nunavutCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
}

static inline uint64_t nunavutGetU64(const uint8_t* const           buf,
                                     const size_t                   buf_size,
                                     const NunavutUnsignedBitLength off_bit,
                                     const uint8_t                  len_bit)
{
    NUNAVUT_ASSERT(buf != NULL);
    const NunavutUnsignedBitLength copy_size =
        nunavutInternalGetBitCopySize(buf_size, off_bit, len_bit, NUNAVUT_WIDTH64);
    NUNAVUT_ASSERT(copy_size <= (sizeof(uint64_t) * NUNAVUT_BYTE_WIDTH));
    uint64_t val = 0U;
    nunavutCopyBits(copy_size, off_bit, 0U, buf, (uint8_t*) &val);
    return val;
}

static inline int8_t nunavutGetI8(const uint8_t* const           buf,
                                  const size_t                   buf_size,
                                  const NunavutUnsignedBitLength off_bit,
                                  const uint8_t                  len_bit)
{
    const uint8_t sat = (uint8_t) nunavutInternalChooseMin(len_bit, NUNAVUT_BYTE_WIDTH);
    uint8_t       val = nunavutGetU8(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val = ((sat < NUNAVUT_BYTE_WIDTH) && neg) ? (uint8_t)(val | ~((1U << sat) - 1U)) : val;  // Sign extension
    return neg ? (int8_t)((-(int8_t)(uint8_t) ~val) - 1) : (int8_t) val;
}

static inline int16_t nunavutGetI16(const uint8_t* const           buf,
                                    const size_t                   buf_size,
                                    const NunavutUnsignedBitLength off_bit,
                                    const uint8_t                  len_bit)
{
    const uint8_t sat = (uint8_t) nunavutInternalChooseMin(len_bit, NUNAVUT_WIDTH16);
    uint16_t      val = nunavutGetU16(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val = ((sat < NUNAVUT_WIDTH16) && neg) ? (uint16_t)(val | ~((1U << sat) - 1U)) : val;  // Sign extension
    return neg ? (int16_t)((-(int16_t)(uint16_t) ~val) - 1) : (int16_t) val;
}

static inline int32_t nunavutGetI32(const uint8_t* const           buf,
                                    const size_t                   buf_size,
                                    const NunavutUnsignedBitLength off_bit,
                                    const uint8_t                  len_bit)
{
    const uint8_t sat = (uint8_t) nunavutInternalChooseMin(len_bit, NUNAVUT_WIDTH32);
    uint32_t      val = nunavutGetU32(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val = ((sat < NUNAVUT_WIDTH32) && neg) ? (uint32_t)(val | ~((1UL << sat) - 1U)) : val;  // Sign extension
    return neg ? (int32_t)((-(int32_t) ~val) - 1) : (int32_t) val;
}

static inline int64_t nunavutGetI64(const uint8_t* const           buf,
                                    const size_t                   buf_size,
                                    const NunavutUnsignedBitLength off_bit,
                                    const uint8_t                  len_bit)
{
    const uint8_t sat = (uint8_t) nunavutInternalChooseMin(len_bit, NUNAVUT_WIDTH64);
    uint64_t      val = nunavutGetU64(buf, buf_size, off_bit, sat);
    const bool    neg = (sat > 0U) && ((val & (1ULL << (sat - 1U))) != 0U);
    val = ((sat < NUNAVUT_WIDTH64) && neg) ? (uint64_t)(val | ~((1ULL << sat) - 1U)) : val;  // Sign extension
    return neg ? (int64_t)((-(int64_t) ~val) - 1) : (int64_t) val;
}

// --------------------------------------------- PUBLIC API - FLOAT16 ---------------------------------------------

#if NUNAVUT_PLATFORM_IEEE754_FLOAT

_Static_assert(NUNAVUT_WIDTH32 == (sizeof(NunavutFloat32) * NUNAVUT_BYTE_WIDTH), "Unsupported floating point model");

// Intentional violation of MISRA: we need this union because the alternative is far more error prone.
// We have to rely on low-level data representation details to do the conversion; unions are helpful.
typedef union  // NOSONAR
{
    uint32_t       bits;
    NunavutFloat32 real;
} NunavutFloat32Bits;

static inline uint16_t float16Pack(const NunavutFloat32 value)
{
    // The no-lint statements suppress the warnings about magic numbers.
    // The no-lint statements suppress the warning about the use of union. This is required for low-level bit access.
    const uint32_t           round_mask = ~(uint32_t) 0x0FFFU;                 // NOLINT NOSONAR
    const NunavutFloat32Bits f32inf     = {.bits = ((uint32_t) 255U) << 23U};  // NOLINT NOSONAR
    const NunavutFloat32Bits f16inf     = {.bits = ((uint32_t) 31U) << 23U};   // NOLINT NOSONAR
    const NunavutFloat32Bits magic      = {.bits = ((uint32_t) 15U) << 23U};   // NOLINT NOSONAR
    NunavutFloat32Bits       in         = {.real = value};                     // NOSONAR
    const uint32_t           sign       = in.bits & (((uint32_t) 1U) << 31U);  // NOLINT NOSONAR
    in.bits ^= sign;
    uint16_t out = 0;
    if (in.bits >= f32inf.bits)
    {
        out = (in.bits > f32inf.bits) ? (uint16_t) 0x7FFFU : (uint16_t) 0x7C00U;  // NOLINT NOSONAR
    }
    else
    {
        in.bits &= round_mask;
        in.real *= magic.real;
        in.bits -= round_mask;
        if (in.bits > f16inf.bits)
        {
            in.bits = f16inf.bits;
        }
        out = (uint16_t)(in.bits >> 13U);  // NOLINT NOSONAR
    }
    out |= (uint16_t)(sign >> 16U);  // NOLINT NOSONAR
    return out;
}

static inline NunavutFloat32 float16Unpack(const uint16_t value)
{
    // The no-lint statements suppress the warnings about magic numbers.
    // The no-lint statements suppress the warning about the use of union. This is required for low-level bit access.
    const NunavutFloat32Bits magic   = {.bits = ((uint32_t) 0xEFU) << 23U};             // NOLINT NOSONAR
    const NunavutFloat32Bits inf_nan = {.bits = ((uint32_t) 0x8FU) << 23U};             // NOLINT NOSONAR
    NunavutFloat32Bits       out     = {.bits = ((uint32_t)(value & 0x7FFFU)) << 13U};  // NOLINT NOSONAR
    out.real *= magic.real;
    if (out.real >= inf_nan.real)
    {
        out.bits |= ((uint32_t) 0xFFU) << 23U;  // NOLINT NOSONAR
    }
    out.bits |= ((uint32_t)(value & 0x8000U)) << 16U;  // NOLINT NOSONAR
    return out.real;
}

static inline void nunavutSetF16(uint8_t* const buf, const NunavutUnsignedBitLength off_bit, const NunavutFloat32 value)
{
    nunavutSetUxx(buf, off_bit, float16Pack(value), NUNAVUT_WIDTH16);
}

static inline NunavutFloat32 nunavutGetF16(const uint8_t* const           buf,
                                           const size_t                   buf_size,
                                           const NunavutUnsignedBitLength off_bit)
{
    return float16Unpack(nunavutGetU16(buf, buf_size, off_bit, NUNAVUT_WIDTH16));
}

#endif  // NUNAVUT_PLATFORM_IEEE754_FLOAT

// --------------------------------------------- PUBLIC API - FLOAT32 ---------------------------------------------

#if NUNAVUT_PLATFORM_IEEE754_FLOAT

_Static_assert(NUNAVUT_WIDTH32 == (sizeof(NunavutFloat32) * NUNAVUT_BYTE_WIDTH), "Unsupported floating point model");

static inline void nunavutSetF32(uint8_t* const buf, const NunavutUnsignedBitLength off_bit, const NunavutFloat32 value)
{
    // Intentional violation of MISRA: use union to perform fast conversion from an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        NunavutFloat32 fl;
        uint32_t       in;
    } const tmp = {value};  // NOSONAR
    nunavutSetUxx(buf, off_bit, tmp.in, sizeof(tmp) * NUNAVUT_BYTE_WIDTH);
}

static inline NunavutFloat32 nunavutGetF32(const uint8_t* const           buf,
                                           const size_t                   buf_size,
                                           const NunavutUnsignedBitLength off_bit)
{
    // Intentional violation of MISRA: use union to perform fast conversion to an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        uint32_t       in;
        NunavutFloat32 fl;
    } const tmp = {nunavutGetU32(buf, buf_size, off_bit, NUNAVUT_WIDTH32)};  // NOSONAR
    return tmp.fl;
}

#endif  // NUNAVUT_PLATFORM_IEEE754_FLOAT

// --------------------------------------------- PUBLIC API - FLOAT64 ---------------------------------------------

#if NUNAVUT_PLATFORM_IEEE754_DOUBLE

_Static_assert(NUNAVUT_WIDTH64 == (sizeof(NunavutFloat64) * NUNAVUT_BYTE_WIDTH), "Unsupported floating point model");

static inline void nunavutSetF64(uint8_t* const buf, const NunavutUnsignedBitLength off_bit, const NunavutFloat64 value)
{
    // Intentional violation of MISRA: use union to perform fast conversion from an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        NunavutFloat64 fl;
        uint64_t       in;
    } const tmp = {value};  // NOSONAR
    nunavutSetUxx(buf, off_bit, tmp.in, sizeof(tmp) * NUNAVUT_BYTE_WIDTH);
}

static inline NunavutFloat64 nunavutGetF64(const uint8_t* const           buf,
                                           const size_t                   buf_size,
                                           const NunavutUnsignedBitLength off_bit)
{
    // Intentional violation of MISRA: use union to perform fast conversion to an IEEE 754-compatible native
    // representation into a serializable integer. The assumptions about the target platform properties are made
    // clear. In the future we may add a more generic conversion that is platform-invariant.
    union  // NOSONAR
    {
        uint64_t       in;
        NunavutFloat64 fl;
    } const tmp = {nunavutGetU64(buf, buf_size, off_bit, NUNAVUT_WIDTH64)};  // NOSONAR
    return tmp.fl;
}

#endif  // NUNAVUT_PLATFORM_IEE754_DOUBLE

#ifdef __cplusplus
}
#endif

#endif /* NUNAVUT_SUPPORT_NUNAVUT_COMMON_H_INCLUDED */
