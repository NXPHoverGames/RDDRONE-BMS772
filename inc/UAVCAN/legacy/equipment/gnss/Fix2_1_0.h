/*
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2021 NXP 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.1.0 (serialization was enabled)
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/gnss/Fix2.1.0.uavcan
// Generated at:  2021-04-12 07:48:50.230044 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     legacy.equipment.gnss.Fix2
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef LEGACY_EQUIPMENT_GNSS_FIX2_1_0_INCLUDED_
#define LEGACY_EQUIPMENT_GNSS_FIX2_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/time/SynchronizedTimestamp_1_0.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/gnss/Fix2.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/gnss/Fix2.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/gnss/Fix2.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/equipment/gnss/Fix2.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define legacy_equipment_gnss_Fix2_1_0_HAS_FIXED_PORT_ID_ false

#define legacy_equipment_gnss_Fix2_1_0_FULL_NAME_             "legacy.equipment.gnss.Fix2"
#define legacy_equipment_gnss_Fix2_1_0_FULL_NAME_AND_VERSION_ "legacy.equipment.gnss.Fix2.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_equipment_gnss_Fix2_1_0_EXTENT_BYTES_                    246UL
#define legacy_equipment_gnss_Fix2_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 123UL
static_assert(legacy_equipment_gnss_Fix2_1_0_EXTENT_BYTES_ >= legacy_equipment_gnss_Fix2_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint3 GNSS_TIME_STANDARD_NONE = 0
#define legacy_equipment_gnss_Fix2_1_0_GNSS_TIME_STANDARD_NONE (0U)
/// saturated uint3 GNSS_TIME_STANDARD_TAI = 1
#define legacy_equipment_gnss_Fix2_1_0_GNSS_TIME_STANDARD_TAI (1U)
/// saturated uint3 GNSS_TIME_STANDARD_UTC = 2
#define legacy_equipment_gnss_Fix2_1_0_GNSS_TIME_STANDARD_UTC (2U)
/// saturated uint3 GNSS_TIME_STANDARD_GPS = 3
#define legacy_equipment_gnss_Fix2_1_0_GNSS_TIME_STANDARD_GPS (3U)
/// saturated uint8 NUM_LEAP_SECONDS_UNKNOWN = 0
#define legacy_equipment_gnss_Fix2_1_0_NUM_LEAP_SECONDS_UNKNOWN (0U)
/// saturated uint2 STATUS_NO_FIX = 0
#define legacy_equipment_gnss_Fix2_1_0_STATUS_NO_FIX (0U)
/// saturated uint2 STATUS_TIME_ONLY = 1
#define legacy_equipment_gnss_Fix2_1_0_STATUS_TIME_ONLY (1U)
/// saturated uint2 STATUS_2D_FIX = 2
#define legacy_equipment_gnss_Fix2_1_0_STATUS_2D_FIX (2U)
/// saturated uint2 STATUS_3D_FIX = 3
#define legacy_equipment_gnss_Fix2_1_0_STATUS_3D_FIX (3U)
/// saturated uint4 MODE_SINGLE = 0
#define legacy_equipment_gnss_Fix2_1_0_MODE_SINGLE (0U)
/// saturated uint4 MODE_DGPS = 1
#define legacy_equipment_gnss_Fix2_1_0_MODE_DGPS (1U)
/// saturated uint4 MODE_RTK = 2
#define legacy_equipment_gnss_Fix2_1_0_MODE_RTK (2U)
/// saturated uint4 MODE_PPP = 3
#define legacy_equipment_gnss_Fix2_1_0_MODE_PPP (3U)
/// saturated uint6 SUB_MODE_DGPS_OTHER = 0
#define legacy_equipment_gnss_Fix2_1_0_SUB_MODE_DGPS_OTHER (0U)
/// saturated uint6 SUB_MODE_DGPS_SBAS = 1
#define legacy_equipment_gnss_Fix2_1_0_SUB_MODE_DGPS_SBAS (1U)
/// saturated uint6 SUB_MODE_RTK_FLOAT = 0
#define legacy_equipment_gnss_Fix2_1_0_SUB_MODE_RTK_FLOAT (0U)
/// saturated uint6 SUB_MODE_RTK_FIXED = 1
#define legacy_equipment_gnss_Fix2_1_0_SUB_MODE_RTK_FIXED (1U)

/// Array metadata for: saturated float32[3] ned_velocity
#define legacy_equipment_gnss_Fix2_1_0_ned_velocity_ARRAY_CAPACITY_           3U
#define legacy_equipment_gnss_Fix2_1_0_ned_velocity_ARRAY_IS_VARIABLE_LENGTH_ false
/// Array metadata for: saturated float16[<=36] covariance
#ifndef legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_CAPACITY_
#define legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_CAPACITY_           36U
#elif !defined(legacy_equipment_gnss_Fix2_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_equipment_gnss_Fix2_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_CAPACITY_ > 36U
#  error legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_CAPACITY_ > 36U
#endif
#define legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// uavcan.time.SynchronizedTimestamp.1.0 timestamp
    uavcan_time_SynchronizedTimestamp_1_0 timestamp;

    /// uavcan.time.SynchronizedTimestamp.1.0 gnss_timestamp
    uavcan_time_SynchronizedTimestamp_1_0 gnss_timestamp;

    /// saturated uint3 gnss_time_standard
    uint8_t gnss_time_standard;

    /// saturated uint8 num_leap_seconds
    uint8_t num_leap_seconds;

    /// saturated int37 longitude_deg_1e8
    int64_t longitude_deg_1e8;

    /// saturated int37 latitude_deg_1e8
    int64_t latitude_deg_1e8;

    /// saturated int27 height_ellipsoid_mm
    int32_t height_ellipsoid_mm;

    /// saturated int27 height_msl_mm
    int32_t height_msl_mm;

    /// saturated float32[3] ned_velocity
    float ned_velocity[3];

    /// saturated uint6 sats_used
    uint8_t sats_used;

    /// saturated uint2 status
    uint8_t status;

    /// saturated uint4 mode
    uint8_t mode;

    /// saturated uint6 sub_mode
    uint8_t sub_mode;

    /// saturated float16[<=36] covariance
    struct  /// Array address equivalence guarantee: &elements[0] == &covariance
    {
        float elements[legacy_equipment_gnss_Fix2_1_0_covariance_ARRAY_CAPACITY_];
        size_t count;
    } covariance;

    /// saturated float16 pdop
    float pdop;
} legacy_equipment_gnss_Fix2_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_equipment_gnss_Fix2_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_equipment_gnss_Fix2_1_0_serialize_(
    const legacy_equipment_gnss_Fix2_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_equipment_gnss_Fix2_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 984UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // uavcan.time.SynchronizedTimestamp.1.0 timestamp
        size_t _size_bytes0_ = 7UL;  // Nested object (max) size, in bytes.
        int8_t _err0_ = uavcan_time_SynchronizedTimestamp_1_0_serialize_(
            &obj->timestamp, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += _pad0_;
    }

    {   // uavcan.time.SynchronizedTimestamp.1.0 gnss_timestamp
        size_t _size_bytes1_ = 7UL;  // Nested object (max) size, in bytes.
        int8_t _err2_ = uavcan_time_SynchronizedTimestamp_1_0_serialize_(
            &obj->gnss_timestamp, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err2_ < 0)
        {
            return _err2_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested object.
    }




    {   // saturated uint3 gnss_time_standard
        uint8_t _sat0_ = obj->gnss_time_standard;
        if (_sat0_ > 7U)
        {
            _sat0_ = 7U;
        }
        buffer[offset_bits / 8U] = (uint8_t)(_sat0_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 3U;
    }




    {   // void13
        const int8_t _err3_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 13U);  // Optimize?
        if (_err3_ < 0)
        {
            return _err3_;
        }
        offset_bits += 13UL;
    }




    {   // saturated uint8 num_leap_seconds
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->num_leap_seconds);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }




    {   // saturated int37 longitude_deg_1e8
        int64_t _sat1_ = obj->longitude_deg_1e8;
        if (_sat1_ < -68719476736LL)
        {
            _sat1_ = -68719476736LL;
        }
        if (_sat1_ > 68719476735LL)
        {
            _sat1_ = 68719476735LL;
        }
        const int8_t _err4_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat1_, 37U);
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += 37U;
    }




    {   // saturated int37 latitude_deg_1e8
        int64_t _sat2_ = obj->latitude_deg_1e8;
        if (_sat2_ < -68719476736LL)
        {
            _sat2_ = -68719476736LL;
        }
        if (_sat2_ > 68719476735LL)
        {
            _sat2_ = 68719476735LL;
        }
        const int8_t _err5_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat2_, 37U);
        if (_err5_ < 0)
        {
            return _err5_;
        }
        offset_bits += 37U;
    }




    {   // saturated int27 height_ellipsoid_mm
        int32_t _sat3_ = obj->height_ellipsoid_mm;
        if (_sat3_ < -67108864L)
        {
            _sat3_ = -67108864L;
        }
        if (_sat3_ > 67108863L)
        {
            _sat3_ = 67108863L;
        }
        const int8_t _err6_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat3_, 27U);
        if (_err6_ < 0)
        {
            return _err6_;
        }
        offset_bits += 27U;
    }




    {   // saturated int27 height_msl_mm
        int32_t _sat4_ = obj->height_msl_mm;
        if (_sat4_ < -67108864L)
        {
            _sat4_ = -67108864L;
        }
        if (_sat4_ > 67108863L)
        {
            _sat4_ = 67108863L;
        }
        const int8_t _err7_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, _sat4_, 27U);
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += 27U;
    }




    {   // saturated float32[3] ned_velocity
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            // Saturation code not emitted -- assume the native representation of float32 is conformant.
            static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
            const int8_t _err8_ = nunavutSetF32(&buffer[0], capacity_bytes, offset_bits, obj->ned_velocity[0]);
            if (_err8_ < 0)
            {
                return _err8_;
            }
            offset_bits += 32U;
        }
        {   // Array element #1
            // Saturation code not emitted -- assume the native representation of float32 is conformant.
            static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
            const int8_t _err9_ = nunavutSetF32(&buffer[0], capacity_bytes, offset_bits, obj->ned_velocity[1]);
            if (_err9_ < 0)
            {
                return _err9_;
            }
            offset_bits += 32U;
        }
        {   // Array element #2
            // Saturation code not emitted -- assume the native representation of float32 is conformant.
            static_assert(NUNAVUT_PLATFORM_IEEE754_FLOAT, "Native IEEE754 binary32 required. TODO: relax constraint");
            const int8_t _err10_ = nunavutSetF32(&buffer[0], capacity_bytes, offset_bits, obj->ned_velocity[2]);
            if (_err10_ < 0)
            {
                return _err10_;
            }
            offset_bits += 32U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }




    {   // saturated uint6 sats_used
        uint8_t _sat5_ = obj->sats_used;
        if (_sat5_ > 63U)
        {
            _sat5_ = 63U;
        }
        buffer[offset_bits / 8U] = (uint8_t)(_sat5_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 6U;
    }




    {   // saturated uint2 status
        uint8_t _sat6_ = obj->status;
        if (_sat6_ > 3U)
        {
            _sat6_ = 3U;
        }
        const int8_t _err11_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat6_, 2U);
        if (_err11_ < 0)
        {
            return _err11_;
        }
        offset_bits += 2U;
    }




    {   // saturated uint4 mode
        uint8_t _sat7_ = obj->mode;
        if (_sat7_ > 15U)
        {
            _sat7_ = 15U;
        }
        buffer[offset_bits / 8U] = (uint8_t)(_sat7_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 4U;
    }




    {   // saturated uint6 sub_mode
        uint8_t _sat8_ = obj->sub_mode;
        if (_sat8_ > 63U)
        {
            _sat8_ = 63U;
        }
        const int8_t _err12_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat8_, 6U);
        if (_err12_ < 0)
        {
            return _err12_;
        }
        offset_bits += 6U;
    }




    {   // saturated float16[<=36] covariance
        if (obj->covariance.count > 36)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        const int8_t _err13_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->covariance.count, 8U);
        if (_err13_ < 0)
        {
            return _err13_;
        }
        offset_bits += 8U;
        for (size_t _index0_ = 0U; _index0_ < obj->covariance.count; ++_index0_)
        {
            float _sat9_ = obj->covariance.elements[_index0_];
            if (isfinite(_sat9_))
            {
                if (_sat9_ < ((float) -65504.0))
                {
                    _sat9_ = ((float) -65504.0);
                }
                if (_sat9_ > ((float) 65504.0))
                {
                    _sat9_ = ((float) 65504.0);
                }
            }
            const int8_t _err14_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat9_);
            if (_err14_ < 0)
            {
                return _err14_;
            }
            offset_bits += 16U;
        }
    }




    {   // saturated float16 pdop
        float _sat10_ = obj->pdop;
        if (isfinite(_sat10_))
        {
            if (_sat10_ < ((float) -65504.0))
            {
                _sat10_ = ((float) -65504.0);
            }
            if (_sat10_ > ((float) 65504.0))
            {
                _sat10_ = ((float) 65504.0);
            }
        }
        const int8_t _err15_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat10_);
        if (_err15_ < 0)
        {
            return _err15_;
        }
        offset_bits += 16U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err16_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err16_ < 0)
        {
            return _err16_;
        }
        offset_bits += _pad1_;
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.





    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_equipment_gnss_Fix2_1_0_deserialize_(
    legacy_equipment_gnss_Fix2_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // uavcan.time.SynchronizedTimestamp.1.0 timestamp
    {
        size_t _size_bytes2_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err17_ = uavcan_time_SynchronizedTimestamp_1_0_deserialize_(
            &out_obj->timestamp, &buffer[offset_bits / 8U], &_size_bytes2_);
        if (_err17_ < 0)
        {
            return _err17_;
        }
        offset_bits += _size_bytes2_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.time.SynchronizedTimestamp.1.0 gnss_timestamp
    {
        size_t _size_bytes3_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err18_ = uavcan_time_SynchronizedTimestamp_1_0_deserialize_(
            &out_obj->gnss_timestamp, &buffer[offset_bits / 8U], &_size_bytes3_);
        if (_err18_ < 0)
        {
            return _err18_;
        }
        offset_bits += _size_bytes3_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // saturated uint3 gnss_time_standard
    if ((offset_bits + 3U) <= capacity_bits)
    {
        out_obj->gnss_time_standard = buffer[offset_bits / 8U] & 7U;
    }
    else
    {
        out_obj->gnss_time_standard = 0U;
    }
    offset_bits += 3U;




    // void13
    offset_bits += 13;




    // saturated uint8 num_leap_seconds
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->num_leap_seconds = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->num_leap_seconds = 0U;
    }
    offset_bits += 8U;




    // saturated int37 longitude_deg_1e8
    out_obj->longitude_deg_1e8 = nunavutGetI64(&buffer[0], capacity_bytes, offset_bits, 37);
    offset_bits += 37U;




    // saturated int37 latitude_deg_1e8
    out_obj->latitude_deg_1e8 = nunavutGetI64(&buffer[0], capacity_bytes, offset_bits, 37);
    offset_bits += 37U;




    // saturated int27 height_ellipsoid_mm
    out_obj->height_ellipsoid_mm = nunavutGetI32(&buffer[0], capacity_bytes, offset_bits, 27);
    offset_bits += 27U;




    // saturated int27 height_msl_mm
    out_obj->height_msl_mm = nunavutGetI32(&buffer[0], capacity_bytes, offset_bits, 27);
    offset_bits += 27U;




    // saturated float32[3] ned_velocity
    // Array element #0
    out_obj->ned_velocity[0] = nunavutGetF32(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 32U;
    // Array element #1
    out_obj->ned_velocity[1] = nunavutGetF32(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 32U;
    // Array element #2
    out_obj->ned_velocity[2] = nunavutGetF32(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 32U;




    // saturated uint6 sats_used
    if ((offset_bits + 6U) <= capacity_bits)
    {
        out_obj->sats_used = buffer[offset_bits / 8U] & 63U;
    }
    else
    {
        out_obj->sats_used = 0U;
    }
    offset_bits += 6U;




    // saturated uint2 status
    out_obj->status = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 2);
    offset_bits += 2U;




    // saturated uint4 mode
    if ((offset_bits + 4U) <= capacity_bits)
    {
        out_obj->mode = buffer[offset_bits / 8U] & 15U;
    }
    else
    {
        out_obj->mode = 0U;
    }
    offset_bits += 4U;




    // saturated uint6 sub_mode
    out_obj->sub_mode = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 6);
    offset_bits += 6U;




    // saturated float16[<=36] covariance
    // Array length prefix: truncated uint8
    out_obj->covariance.count = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;
    if (out_obj->covariance.count > 36U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index1_ = 0U; _index1_ < out_obj->covariance.count; ++_index1_)
    {
        out_obj->covariance.elements[_index1_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
        offset_bits += 16U;
    }




    // saturated float16 pdop
    out_obj->pdop = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void legacy_equipment_gnss_Fix2_1_0_initialize_(legacy_equipment_gnss_Fix2_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_equipment_gnss_Fix2_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // LEGACY_EQUIPMENT_GNSS_FIX2_1_0_INCLUDED_

