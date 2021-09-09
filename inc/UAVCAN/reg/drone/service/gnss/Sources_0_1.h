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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/gnss/Sources.0.1.uavcan
// Generated at:  2021-04-12 07:49:02.648557 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.service.gnss.Sources
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef REG_DRONE_SERVICE_GNSS_SOURCES_0_1_INCLUDED_
#define REG_DRONE_SERVICE_GNSS_SOURCES_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdbool.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/gnss/Sources.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/gnss/Sources.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/gnss/Sources.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/service/gnss/Sources.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_service_gnss_Sources_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_service_gnss_Sources_0_1_FULL_NAME_             "reg.drone.service.gnss.Sources"
#define reg_drone_service_gnss_Sources_0_1_FULL_NAME_AND_VERSION_ "reg.drone.service.gnss.Sources.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_service_gnss_Sources_0_1_EXTENT_BYTES_                    6UL
#define reg_drone_service_gnss_Sources_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 6UL
static_assert(reg_drone_service_gnss_Sources_0_1_EXTENT_BYTES_ >= reg_drone_service_gnss_Sources_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    /// saturated bool gps_l1
    bool gps_l1;

    /// saturated bool gps_l2
    bool gps_l2;

    /// saturated bool gps_l5
    bool gps_l5;

    /// saturated bool glonass_l1
    bool glonass_l1;

    /// saturated bool glonass_l2
    bool glonass_l2;

    /// saturated bool glonass_l3
    bool glonass_l3;

    /// saturated bool galileo_e1
    bool galileo_e1;

    /// saturated bool galileo_e5a
    bool galileo_e5a;

    /// saturated bool galileo_e5b
    bool galileo_e5b;

    /// saturated bool galileo_e6
    bool galileo_e6;

    /// saturated bool beidou_b1
    bool beidou_b1;

    /// saturated bool beidou_b2
    bool beidou_b2;

    /// saturated bool sbas
    bool sbas;

    /// saturated bool gbas
    bool gbas;

    /// saturated bool rtk_base
    bool rtk_base;

    /// saturated bool imu
    bool imu;

    /// saturated bool visual_odometry
    bool visual_odometry;

    /// saturated bool dead_reckoning
    bool dead_reckoning;

    /// saturated bool uwb
    bool uwb;

    /// saturated bool magnetic_compass
    bool magnetic_compass;

    /// saturated bool gyro_compass
    bool gyro_compass;

    /// saturated bool other_compass
    bool other_compass;
} reg_drone_service_gnss_Sources_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_service_gnss_Sources_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_service_gnss_Sources_0_1_serialize_(
    const reg_drone_service_gnss_Sources_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef reg_drone_service_gnss_Sources_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 48UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated bool gps_l1
        buffer[offset_bits / 8U] = obj->gps_l1 ? 1U : 0U;
        offset_bits += 1U;
    }




    {   // saturated bool gps_l2
        if (obj->gps_l2)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool gps_l5
        if (obj->gps_l5)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool glonass_l1
        if (obj->glonass_l1)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool glonass_l2
        if (obj->glonass_l2)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool glonass_l3
        if (obj->glonass_l3)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool galileo_e1
        if (obj->galileo_e1)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool galileo_e5a
        if (obj->galileo_e5a)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool galileo_e5b
        buffer[offset_bits / 8U] = obj->galileo_e5b ? 1U : 0U;
        offset_bits += 1U;
    }




    {   // saturated bool galileo_e6
        if (obj->galileo_e6)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool beidou_b1
        if (obj->beidou_b1)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool beidou_b2
        if (obj->beidou_b2)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // void5
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 5U);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 5UL;
    }




    {   // saturated bool sbas
        if (obj->sbas)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool gbas
        if (obj->gbas)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool rtk_base
        if (obj->rtk_base)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // void3
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 3U);  // Optimize?
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += 3UL;
    }




    {   // saturated bool imu
        if (obj->imu)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool visual_odometry
        buffer[offset_bits / 8U] = obj->visual_odometry ? 1U : 0U;
        offset_bits += 1U;
    }




    {   // saturated bool dead_reckoning
        if (obj->dead_reckoning)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool uwb
        if (obj->uwb)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // void4
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 4U);  // Optimize?
        if (_err2_ < 0)
        {
            return _err2_;
        }
        offset_bits += 4UL;
    }




    {   // saturated bool magnetic_compass
        if (obj->magnetic_compass)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // saturated bool gyro_compass
        buffer[offset_bits / 8U] = obj->gyro_compass ? 1U : 0U;
        offset_bits += 1U;
    }




    {   // saturated bool other_compass
        if (obj->other_compass)
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] | (1U << (offset_bits % 8U)));
        }
        else
        {
            buffer[offset_bits / 8U] = (uint8_t)(buffer[offset_bits / 8U] & ~(1U << (offset_bits % 8U)));
        }
        offset_bits += 1U;
    }




    {   // void14
        const int8_t _err3_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, 14U);  // Optimize?
        if (_err3_ < 0)
        {
            return _err3_;
        }
        offset_bits += 14UL;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += _pad0_;
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
static inline int8_t reg_drone_service_gnss_Sources_0_1_deserialize_(
    reg_drone_service_gnss_Sources_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated bool gps_l1
    if (offset_bits < capacity_bits)
    {
        out_obj->gps_l1 = (buffer[offset_bits / 8U] & 1U) != 0U;
    }
    else
    {
        out_obj->gps_l1 = false;
    }
    offset_bits += 1U;




    // saturated bool gps_l2
    if (offset_bits < capacity_bits)
    {
        out_obj->gps_l2 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->gps_l2 = false;
    }
    offset_bits += 1U;




    // saturated bool gps_l5
    if (offset_bits < capacity_bits)
    {
        out_obj->gps_l5 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->gps_l5 = false;
    }
    offset_bits += 1U;




    // saturated bool glonass_l1
    if (offset_bits < capacity_bits)
    {
        out_obj->glonass_l1 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->glonass_l1 = false;
    }
    offset_bits += 1U;




    // saturated bool glonass_l2
    if (offset_bits < capacity_bits)
    {
        out_obj->glonass_l2 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->glonass_l2 = false;
    }
    offset_bits += 1U;




    // saturated bool glonass_l3
    if (offset_bits < capacity_bits)
    {
        out_obj->glonass_l3 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->glonass_l3 = false;
    }
    offset_bits += 1U;




    // saturated bool galileo_e1
    if (offset_bits < capacity_bits)
    {
        out_obj->galileo_e1 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->galileo_e1 = false;
    }
    offset_bits += 1U;




    // saturated bool galileo_e5a
    if (offset_bits < capacity_bits)
    {
        out_obj->galileo_e5a = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->galileo_e5a = false;
    }
    offset_bits += 1U;




    // saturated bool galileo_e5b
    if (offset_bits < capacity_bits)
    {
        out_obj->galileo_e5b = (buffer[offset_bits / 8U] & 1U) != 0U;
    }
    else
    {
        out_obj->galileo_e5b = false;
    }
    offset_bits += 1U;




    // saturated bool galileo_e6
    if (offset_bits < capacity_bits)
    {
        out_obj->galileo_e6 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->galileo_e6 = false;
    }
    offset_bits += 1U;




    // saturated bool beidou_b1
    if (offset_bits < capacity_bits)
    {
        out_obj->beidou_b1 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->beidou_b1 = false;
    }
    offset_bits += 1U;




    // saturated bool beidou_b2
    if (offset_bits < capacity_bits)
    {
        out_obj->beidou_b2 = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->beidou_b2 = false;
    }
    offset_bits += 1U;




    // void5
    offset_bits += 5;




    // saturated bool sbas
    if (offset_bits < capacity_bits)
    {
        out_obj->sbas = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->sbas = false;
    }
    offset_bits += 1U;




    // saturated bool gbas
    if (offset_bits < capacity_bits)
    {
        out_obj->gbas = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->gbas = false;
    }
    offset_bits += 1U;




    // saturated bool rtk_base
    if (offset_bits < capacity_bits)
    {
        out_obj->rtk_base = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->rtk_base = false;
    }
    offset_bits += 1U;




    // void3
    offset_bits += 3;




    // saturated bool imu
    if (offset_bits < capacity_bits)
    {
        out_obj->imu = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->imu = false;
    }
    offset_bits += 1U;




    // saturated bool visual_odometry
    if (offset_bits < capacity_bits)
    {
        out_obj->visual_odometry = (buffer[offset_bits / 8U] & 1U) != 0U;
    }
    else
    {
        out_obj->visual_odometry = false;
    }
    offset_bits += 1U;




    // saturated bool dead_reckoning
    if (offset_bits < capacity_bits)
    {
        out_obj->dead_reckoning = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->dead_reckoning = false;
    }
    offset_bits += 1U;




    // saturated bool uwb
    if (offset_bits < capacity_bits)
    {
        out_obj->uwb = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->uwb = false;
    }
    offset_bits += 1U;




    // void4
    offset_bits += 4;




    // saturated bool magnetic_compass
    if (offset_bits < capacity_bits)
    {
        out_obj->magnetic_compass = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->magnetic_compass = false;
    }
    offset_bits += 1U;




    // saturated bool gyro_compass
    if (offset_bits < capacity_bits)
    {
        out_obj->gyro_compass = (buffer[offset_bits / 8U] & 1U) != 0U;
    }
    else
    {
        out_obj->gyro_compass = false;
    }
    offset_bits += 1U;




    // saturated bool other_compass
    if (offset_bits < capacity_bits)
    {
        out_obj->other_compass = (buffer[offset_bits / 8U] & (1U << (offset_bits % 8U))) != 0U;
    }
    else
    {
        out_obj->other_compass = false;
    }
    offset_bits += 1U;




    // void14
    offset_bits += 14;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_service_gnss_Sources_0_1_initialize_(reg_drone_service_gnss_Sources_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_service_gnss_Sources_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_SERVICE_GNSS_SOURCES_0_1_INCLUDED_
