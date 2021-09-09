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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/dynamic_node_id/server/AppendEntries.1.0.uavcan
// Generated at:  2021-04-12 07:48:28.839929 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     legacy.protocol.dynamic_node_id.server.AppendEntries
// Version:       1.0
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef LEGACY_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPEND_ENTRIES_1_0_INCLUDED_
#define LEGACY_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPEND_ENTRIES_1_0_INCLUDED_

#include <legacy/protocol/dynamic_node_id/server/Entry_1_0.h>
#include <nunavut/support/serialization.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/dynamic_node_id/server/AppendEntries.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/dynamic_node_id/server/AppendEntries.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/dynamic_node_id/server/AppendEntries.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/legacy/protocol/dynamic_node_id/server/AppendEntries.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define legacy_protocol_dynamic_node_id_server_AppendEntries_1_0_HAS_FIXED_PORT_ID_ false

#define legacy_protocol_dynamic_node_id_server_AppendEntries_1_0_FULL_NAME_             "legacy.protocol.dynamic_node_id.server.AppendEntries"
#define legacy_protocol_dynamic_node_id_server_AppendEntries_1_0_FULL_NAME_AND_VERSION_ "legacy.protocol.dynamic_node_id.server.AppendEntries.1.0"

#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_FULL_NAME_             "legacy.protocol.dynamic_node_id.server.AppendEntries.Request"
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_FULL_NAME_AND_VERSION_ "legacy.protocol.dynamic_node_id.server.AppendEntries.Request.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_EXTENT_BYTES_                    158UL
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 79UL
static_assert(legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_EXTENT_BYTES_ >= legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// saturated uint16 DEFAULT_MIN_ELECTION_TIMEOUT_MS = 2000
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_DEFAULT_MIN_ELECTION_TIMEOUT_MS (2000U)
/// saturated uint16 DEFAULT_MAX_ELECTION_TIMEOUT_MS = 4000
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_DEFAULT_MAX_ELECTION_TIMEOUT_MS (4000U)

/// Array metadata for: legacy.protocol.dynamic_node_id.server.Entry.1.0[<=1] entries
#ifndef legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_CAPACITY_
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_CAPACITY_           1U
#elif !defined(legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_CAPACITY_ > 1U
#  error legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_CAPACITY_ > 1U
#endif
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// saturated uint32 term
    uint32_t term;

    /// saturated uint32 prev_log_term
    uint32_t prev_log_term;

    /// saturated uint8 prev_log_index
    uint8_t prev_log_index;

    /// saturated uint8 leader_commit
    uint8_t leader_commit;

    /// legacy.protocol.dynamic_node_id.server.Entry.1.0[<=1] entries
    struct  /// Array address equivalence guarantee: &elements[0] == &entries
    {
        legacy_protocol_dynamic_node_id_server_Entry_1_0 elements[legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_entries_ARRAY_CAPACITY_];
        size_t count;
    } entries;
} legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_serialize_(
    const legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 632UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated uint32 term
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->term, 32U);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 32U;
    }




    {   // saturated uint32 prev_log_term
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->prev_log_term, 32U);
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += 32U;
    }




    {   // saturated uint8 prev_log_index
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->prev_log_index);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }




    {   // saturated uint8 leader_commit
        // Saturation code not emitted -- native representation matches the serialized representation.
        buffer[offset_bits / 8U] = (uint8_t)(obj->leader_commit);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err2_ < 0)
        {
            return _err2_;
        }
        offset_bits += _pad0_;
    }

    {   // legacy.protocol.dynamic_node_id.server.Entry.1.0[<=1] entries
        if (obj->entries.count > 1)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint8
        buffer[offset_bits / 8U] = (uint8_t)(obj->entries.count);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 8U;
        for (size_t _index0_ = 0U; _index0_ < obj->entries.count; ++_index0_)
        {
            size_t _size_bytes0_ = 21UL;  // Nested object (max) size, in bytes.
            // Constant delimiter header can be written ahead of the nested object.
            const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _size_bytes0_, 32U);
            if (_err4_ < 0)
            {
                return _err4_;
            }
            offset_bits += 32U;
            int8_t _err3_ = legacy_protocol_dynamic_node_id_server_Entry_1_0_serialize_(
                &obj->entries.elements[_index0_], &buffer[offset_bits / 8U], &_size_bytes0_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
            offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
        }
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err5_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err5_ < 0)
        {
            return _err5_;
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
static inline int8_t legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_deserialize_(
    legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated uint32 term
    out_obj->term = nunavutGetU32(&buffer[0], capacity_bytes, offset_bits, 32);
    offset_bits += 32U;




    // saturated uint32 prev_log_term
    out_obj->prev_log_term = nunavutGetU32(&buffer[0], capacity_bytes, offset_bits, 32);
    offset_bits += 32U;




    // saturated uint8 prev_log_index
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->prev_log_index = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->prev_log_index = 0U;
    }
    offset_bits += 8U;




    // saturated uint8 leader_commit
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->leader_commit = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->leader_commit = 0U;
    }
    offset_bits += 8U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // legacy.protocol.dynamic_node_id.server.Entry.1.0[<=1] entries
    // Array length prefix: truncated uint8
    if ((offset_bits + 8U) <= capacity_bits)
    {
        out_obj->entries.count = buffer[offset_bits / 8U] & 255U;
    }
    else
    {
        out_obj->entries.count = 0U;
    }
    offset_bits += 8U;
    if (out_obj->entries.count > 1U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    for (size_t _index1_ = 0U; _index1_ < out_obj->entries.count; ++_index1_)
    {
        {
            // Delimiter header: truncated uint32
            size_t _size_bytes1_ = 0U;
            _size_bytes1_ = nunavutGetU32(&buffer[0], capacity_bytes, offset_bits, 32);
            offset_bits += 32U;
            if (_size_bytes1_ > (capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes)))
            {
                return -NUNAVUT_ERROR_REPRESENTATION_BAD_DELIMITER_HEADER;
            }
            const size_t _dh0_ = _size_bytes1_;  // Store the original delimiter header value.
            const int8_t _err6_ = legacy_protocol_dynamic_node_id_server_Entry_1_0_deserialize_(
                &out_obj->entries.elements[_index1_], &buffer[offset_bits / 8U], &_size_bytes1_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            // Advance the offset by the size of the delimiter header, even if the nested deserialization routine
            // consumed fewer bytes of data. This behavior implements the implicit truncation rule for nested objects.
            offset_bits += _dh0_ * 8U;
        }
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_initialize_(legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_protocol_dynamic_node_id_server_AppendEntries_Request_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#define legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_FULL_NAME_             "legacy.protocol.dynamic_node_id.server.AppendEntries.Response"
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_FULL_NAME_AND_VERSION_ "legacy.protocol.dynamic_node_id.server.AppendEntries.Response.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_EXTENT_BYTES_                    64UL
#define legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 5UL
static_assert(legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_EXTENT_BYTES_ >= legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    /// saturated uint32 term
    uint32_t term;

    /// saturated bool success
    bool success;
} legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_serialize_(
    const legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 40UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated uint32 term
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err7_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->term, 32U);
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += 32U;
    }




    {   // saturated bool success
        buffer[offset_bits / 8U] = obj->success ? 1U : 0U;
        offset_bits += 1U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad2_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err8_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad2_);  // Optimize?
        if (_err8_ < 0)
        {
            return _err8_;
        }
        offset_bits += _pad2_;
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
static inline int8_t legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_deserialize_(
    legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated uint32 term
    out_obj->term = nunavutGetU32(&buffer[0], capacity_bytes, offset_bits, 32);
    offset_bits += 32U;




    // saturated bool success
    if (offset_bits < capacity_bits)
    {
        out_obj->success = (buffer[offset_bits / 8U] & 1U) != 0U;
    }
    else
    {
        out_obj->success = false;
    }
    offset_bits += 1U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_initialize_(legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = legacy_protocol_dynamic_node_id_server_AppendEntries_Response_1_0_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // LEGACY_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPEND_ENTRIES_1_0_INCLUDED_
