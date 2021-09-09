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
// Source file:   /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/physics/kinematics/cartesian/PoseVar.0.1.uavcan
// Generated at:  2021-04-12 07:49:29.753235 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.drone.physics.kinematics.cartesian.PoseVar
// Version:       0.1
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  True

#ifndef REG_DRONE_PHYSICS_KINEMATICS_CARTESIAN_POSE_VAR_0_1_INCLUDED_
#define REG_DRONE_PHYSICS_KINEMATICS_CARTESIAN_POSE_VAR_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <reg/drone/physics/kinematics/cartesian/Pose_0_1.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/physics/kinematics/cartesian/PoseVar.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/physics/kinematics/cartesian/PoseVar.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/physics/kinematics/cartesian/PoseVar.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/reg/drone/physics/kinematics/cartesian/PoseVar.0.1.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_HAS_FIXED_PORT_ID_ false

#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_FULL_NAME_             "reg.drone.physics.kinematics.cartesian.PoseVar"
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_FULL_NAME_AND_VERSION_ "reg.drone.physics.kinematics.cartesian.PoseVar.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_EXTENT_BYTES_                    82UL
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 82UL
static_assert(reg_drone_physics_kinematics_cartesian_PoseVar_0_1_EXTENT_BYTES_ >= reg_drone_physics_kinematics_cartesian_PoseVar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: saturated float16[21] covariance_urt
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_covariance_urt_ARRAY_CAPACITY_           21U
#define reg_drone_physics_kinematics_cartesian_PoseVar_0_1_covariance_urt_ARRAY_IS_VARIABLE_LENGTH_ false

typedef struct
{
    /// reg.drone.physics.kinematics.cartesian.Pose.0.1 value
    reg_drone_physics_kinematics_cartesian_Pose_0_1 value;

    /// saturated float16[21] covariance_urt
    float covariance_urt[21];
} reg_drone_physics_kinematics_cartesian_PoseVar_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_drone_physics_kinematics_cartesian_PoseVar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_drone_physics_kinematics_cartesian_PoseVar_0_1_serialize_(
    const reg_drone_physics_kinematics_cartesian_PoseVar_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef reg_drone_physics_kinematics_cartesian_PoseVar_0_1_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 656UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // reg.drone.physics.kinematics.cartesian.Pose.0.1 value
        size_t _size_bytes0_ = 40UL;  // Nested object (max) size, in bytes.
        int8_t _err0_ = reg_drone_physics_kinematics_cartesian_Pose_0_1_serialize_(
            &obj->value, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
    }




    {   // saturated float16[21] covariance_urt
        const size_t _origin0_ = offset_bits;
        {   // Array element #0
            float _sat0_ = obj->covariance_urt[0];
            if (isfinite(_sat0_))
            {
                if (_sat0_ < ((float) -65504.0))
                {
                    _sat0_ = ((float) -65504.0);
                }
                if (_sat0_ > ((float) 65504.0))
                {
                    _sat0_ = ((float) 65504.0);
                }
            }
            const int8_t _err1_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat0_);
            if (_err1_ < 0)
            {
                return _err1_;
            }
            offset_bits += 16U;
        }
        {   // Array element #1
            float _sat1_ = obj->covariance_urt[1];
            if (isfinite(_sat1_))
            {
                if (_sat1_ < ((float) -65504.0))
                {
                    _sat1_ = ((float) -65504.0);
                }
                if (_sat1_ > ((float) 65504.0))
                {
                    _sat1_ = ((float) 65504.0);
                }
            }
            const int8_t _err2_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat1_);
            if (_err2_ < 0)
            {
                return _err2_;
            }
            offset_bits += 16U;
        }
        {   // Array element #2
            float _sat2_ = obj->covariance_urt[2];
            if (isfinite(_sat2_))
            {
                if (_sat2_ < ((float) -65504.0))
                {
                    _sat2_ = ((float) -65504.0);
                }
                if (_sat2_ > ((float) 65504.0))
                {
                    _sat2_ = ((float) 65504.0);
                }
            }
            const int8_t _err3_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat2_);
            if (_err3_ < 0)
            {
                return _err3_;
            }
            offset_bits += 16U;
        }
        {   // Array element #3
            float _sat3_ = obj->covariance_urt[3];
            if (isfinite(_sat3_))
            {
                if (_sat3_ < ((float) -65504.0))
                {
                    _sat3_ = ((float) -65504.0);
                }
                if (_sat3_ > ((float) 65504.0))
                {
                    _sat3_ = ((float) 65504.0);
                }
            }
            const int8_t _err4_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat3_);
            if (_err4_ < 0)
            {
                return _err4_;
            }
            offset_bits += 16U;
        }
        {   // Array element #4
            float _sat4_ = obj->covariance_urt[4];
            if (isfinite(_sat4_))
            {
                if (_sat4_ < ((float) -65504.0))
                {
                    _sat4_ = ((float) -65504.0);
                }
                if (_sat4_ > ((float) 65504.0))
                {
                    _sat4_ = ((float) 65504.0);
                }
            }
            const int8_t _err5_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat4_);
            if (_err5_ < 0)
            {
                return _err5_;
            }
            offset_bits += 16U;
        }
        {   // Array element #5
            float _sat5_ = obj->covariance_urt[5];
            if (isfinite(_sat5_))
            {
                if (_sat5_ < ((float) -65504.0))
                {
                    _sat5_ = ((float) -65504.0);
                }
                if (_sat5_ > ((float) 65504.0))
                {
                    _sat5_ = ((float) 65504.0);
                }
            }
            const int8_t _err6_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat5_);
            if (_err6_ < 0)
            {
                return _err6_;
            }
            offset_bits += 16U;
        }
        {   // Array element #6
            float _sat6_ = obj->covariance_urt[6];
            if (isfinite(_sat6_))
            {
                if (_sat6_ < ((float) -65504.0))
                {
                    _sat6_ = ((float) -65504.0);
                }
                if (_sat6_ > ((float) 65504.0))
                {
                    _sat6_ = ((float) 65504.0);
                }
            }
            const int8_t _err7_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat6_);
            if (_err7_ < 0)
            {
                return _err7_;
            }
            offset_bits += 16U;
        }
        {   // Array element #7
            float _sat7_ = obj->covariance_urt[7];
            if (isfinite(_sat7_))
            {
                if (_sat7_ < ((float) -65504.0))
                {
                    _sat7_ = ((float) -65504.0);
                }
                if (_sat7_ > ((float) 65504.0))
                {
                    _sat7_ = ((float) 65504.0);
                }
            }
            const int8_t _err8_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat7_);
            if (_err8_ < 0)
            {
                return _err8_;
            }
            offset_bits += 16U;
        }
        {   // Array element #8
            float _sat8_ = obj->covariance_urt[8];
            if (isfinite(_sat8_))
            {
                if (_sat8_ < ((float) -65504.0))
                {
                    _sat8_ = ((float) -65504.0);
                }
                if (_sat8_ > ((float) 65504.0))
                {
                    _sat8_ = ((float) 65504.0);
                }
            }
            const int8_t _err9_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat8_);
            if (_err9_ < 0)
            {
                return _err9_;
            }
            offset_bits += 16U;
        }
        {   // Array element #9
            float _sat9_ = obj->covariance_urt[9];
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
            const int8_t _err10_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat9_);
            if (_err10_ < 0)
            {
                return _err10_;
            }
            offset_bits += 16U;
        }
        {   // Array element #10
            float _sat10_ = obj->covariance_urt[10];
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
            const int8_t _err11_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat10_);
            if (_err11_ < 0)
            {
                return _err11_;
            }
            offset_bits += 16U;
        }
        {   // Array element #11
            float _sat11_ = obj->covariance_urt[11];
            if (isfinite(_sat11_))
            {
                if (_sat11_ < ((float) -65504.0))
                {
                    _sat11_ = ((float) -65504.0);
                }
                if (_sat11_ > ((float) 65504.0))
                {
                    _sat11_ = ((float) 65504.0);
                }
            }
            const int8_t _err12_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat11_);
            if (_err12_ < 0)
            {
                return _err12_;
            }
            offset_bits += 16U;
        }
        {   // Array element #12
            float _sat12_ = obj->covariance_urt[12];
            if (isfinite(_sat12_))
            {
                if (_sat12_ < ((float) -65504.0))
                {
                    _sat12_ = ((float) -65504.0);
                }
                if (_sat12_ > ((float) 65504.0))
                {
                    _sat12_ = ((float) 65504.0);
                }
            }
            const int8_t _err13_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat12_);
            if (_err13_ < 0)
            {
                return _err13_;
            }
            offset_bits += 16U;
        }
        {   // Array element #13
            float _sat13_ = obj->covariance_urt[13];
            if (isfinite(_sat13_))
            {
                if (_sat13_ < ((float) -65504.0))
                {
                    _sat13_ = ((float) -65504.0);
                }
                if (_sat13_ > ((float) 65504.0))
                {
                    _sat13_ = ((float) 65504.0);
                }
            }
            const int8_t _err14_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat13_);
            if (_err14_ < 0)
            {
                return _err14_;
            }
            offset_bits += 16U;
        }
        {   // Array element #14
            float _sat14_ = obj->covariance_urt[14];
            if (isfinite(_sat14_))
            {
                if (_sat14_ < ((float) -65504.0))
                {
                    _sat14_ = ((float) -65504.0);
                }
                if (_sat14_ > ((float) 65504.0))
                {
                    _sat14_ = ((float) 65504.0);
                }
            }
            const int8_t _err15_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat14_);
            if (_err15_ < 0)
            {
                return _err15_;
            }
            offset_bits += 16U;
        }
        {   // Array element #15
            float _sat15_ = obj->covariance_urt[15];
            if (isfinite(_sat15_))
            {
                if (_sat15_ < ((float) -65504.0))
                {
                    _sat15_ = ((float) -65504.0);
                }
                if (_sat15_ > ((float) 65504.0))
                {
                    _sat15_ = ((float) 65504.0);
                }
            }
            const int8_t _err16_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat15_);
            if (_err16_ < 0)
            {
                return _err16_;
            }
            offset_bits += 16U;
        }
        {   // Array element #16
            float _sat16_ = obj->covariance_urt[16];
            if (isfinite(_sat16_))
            {
                if (_sat16_ < ((float) -65504.0))
                {
                    _sat16_ = ((float) -65504.0);
                }
                if (_sat16_ > ((float) 65504.0))
                {
                    _sat16_ = ((float) 65504.0);
                }
            }
            const int8_t _err17_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat16_);
            if (_err17_ < 0)
            {
                return _err17_;
            }
            offset_bits += 16U;
        }
        {   // Array element #17
            float _sat17_ = obj->covariance_urt[17];
            if (isfinite(_sat17_))
            {
                if (_sat17_ < ((float) -65504.0))
                {
                    _sat17_ = ((float) -65504.0);
                }
                if (_sat17_ > ((float) 65504.0))
                {
                    _sat17_ = ((float) 65504.0);
                }
            }
            const int8_t _err18_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat17_);
            if (_err18_ < 0)
            {
                return _err18_;
            }
            offset_bits += 16U;
        }
        {   // Array element #18
            float _sat18_ = obj->covariance_urt[18];
            if (isfinite(_sat18_))
            {
                if (_sat18_ < ((float) -65504.0))
                {
                    _sat18_ = ((float) -65504.0);
                }
                if (_sat18_ > ((float) 65504.0))
                {
                    _sat18_ = ((float) 65504.0);
                }
            }
            const int8_t _err19_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat18_);
            if (_err19_ < 0)
            {
                return _err19_;
            }
            offset_bits += 16U;
        }
        {   // Array element #19
            float _sat19_ = obj->covariance_urt[19];
            if (isfinite(_sat19_))
            {
                if (_sat19_ < ((float) -65504.0))
                {
                    _sat19_ = ((float) -65504.0);
                }
                if (_sat19_ > ((float) 65504.0))
                {
                    _sat19_ = ((float) 65504.0);
                }
            }
            const int8_t _err20_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat19_);
            if (_err20_ < 0)
            {
                return _err20_;
            }
            offset_bits += 16U;
        }
        {   // Array element #20
            float _sat20_ = obj->covariance_urt[20];
            if (isfinite(_sat20_))
            {
                if (_sat20_ < ((float) -65504.0))
                {
                    _sat20_ = ((float) -65504.0);
                }
                if (_sat20_ > ((float) 65504.0))
                {
                    _sat20_ = ((float) 65504.0);
                }
            }
            const int8_t _err21_ = nunavutSetF16(&buffer[0], capacity_bytes, offset_bits, _sat20_);
            if (_err21_ < 0)
            {
                return _err21_;
            }
            offset_bits += 16U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err22_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err22_ < 0)
        {
            return _err22_;
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
static inline int8_t reg_drone_physics_kinematics_cartesian_PoseVar_0_1_deserialize_(
    reg_drone_physics_kinematics_cartesian_PoseVar_0_1* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // reg.drone.physics.kinematics.cartesian.Pose.0.1 value
    {
        size_t _size_bytes1_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        const int8_t _err23_ = reg_drone_physics_kinematics_cartesian_Pose_0_1_deserialize_(
            &out_obj->value, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err23_ < 0)
        {
            return _err23_;
        }
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested serialized representation.
    }




    // saturated float16[21] covariance_urt
    // Array element #0
    out_obj->covariance_urt[0] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #1
    out_obj->covariance_urt[1] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #2
    out_obj->covariance_urt[2] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #3
    out_obj->covariance_urt[3] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #4
    out_obj->covariance_urt[4] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #5
    out_obj->covariance_urt[5] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #6
    out_obj->covariance_urt[6] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #7
    out_obj->covariance_urt[7] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #8
    out_obj->covariance_urt[8] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #9
    out_obj->covariance_urt[9] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #10
    out_obj->covariance_urt[10] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #11
    out_obj->covariance_urt[11] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #12
    out_obj->covariance_urt[12] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #13
    out_obj->covariance_urt[13] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #14
    out_obj->covariance_urt[14] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #15
    out_obj->covariance_urt[15] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #16
    out_obj->covariance_urt[16] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #17
    out_obj->covariance_urt[17] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #18
    out_obj->covariance_urt[18] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #19
    out_obj->covariance_urt[19] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;
    // Array element #20
    out_obj->covariance_urt[20] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 16U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_drone_physics_kinematics_cartesian_PoseVar_0_1_initialize_(reg_drone_physics_kinematics_cartesian_PoseVar_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_drone_physics_kinematics_cartesian_PoseVar_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_DRONE_PHYSICS_KINEMATICS_CARTESIAN_POSE_VAR_0_1_INCLUDED_
