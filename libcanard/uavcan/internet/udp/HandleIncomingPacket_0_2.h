// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.3.0 (serialization was enabled)
// Source file:   /tmp/pyuavcan-cli-dsdld4ueqbaq/public_regulated_data_types-master/uavcan/internet/udp/500.HandleIncomingPacket.0.2.uavcan
// Generated at:  2021-08-04 05:03:57.600849 UTC
// Is deprecated: no
// Fixed port-ID: 500
// Full name:     uavcan.internet.udp.HandleIncomingPacket
// Version:       0.2
//
// Platform
//     python_implementation:  CPython
//     python_version:  3.9.1
//     python_release_level:  final
//     python_build:  ('default', 'Feb  9 2021 07:55:26')
//     python_compiler:  GCC 8.3.0
//     python_revision:
//     python_xoptions:  {}
//     runtime_platform:  Linux-5.4.0-66-generic-x86_64-with-glibc2.28
//
// Language Options
//     target_endianness:  little
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  True
//     enable_override_variable_array_capacity:  True

#ifndef UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_INCLUDED_
#define UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 434322821,
              "/tmp/pyuavcan-cli-dsdld4ueqbaq/public_regulated_data_types-master/uavcan/internet/udp/500.HandleIncomingPacket.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/tmp/pyuavcan-cli-dsdld4ueqbaq/public_regulated_data_types-master/uavcan/internet/udp/500.HandleIncomingPacket.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 1,
              "/tmp/pyuavcan-cli-dsdld4ueqbaq/public_regulated_data_types-master/uavcan/internet/udp/500.HandleIncomingPacket.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 1,
              "/tmp/pyuavcan-cli-dsdld4ueqbaq/public_regulated_data_types-master/uavcan/internet/udp/500.HandleIncomingPacket.0.2.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

#define uavcan_internet_udp_HandleIncomingPacket_0_2_HAS_FIXED_PORT_ID_ true
#define uavcan_internet_udp_HandleIncomingPacket_0_2_FIXED_PORT_ID_     500U

#define uavcan_internet_udp_HandleIncomingPacket_0_2_FULL_NAME_             "uavcan.internet.udp.HandleIncomingPacket"
#define uavcan_internet_udp_HandleIncomingPacket_0_2_FULL_NAME_AND_VERSION_ "uavcan.internet.udp.HandleIncomingPacket.0.2"

#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_FULL_NAME_             "uavcan.internet.udp.HandleIncomingPacket.Request"
#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_FULL_NAME_AND_VERSION_ "uavcan.internet.udp.HandleIncomingPacket.Request.0.2"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_EXTENT_BYTES_                    600UL
#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_ 512UL
static_assert(uavcan_internet_udp_HandleIncomingPacket_Request_0_2_EXTENT_BYTES_ >= uavcan_internet_udp_HandleIncomingPacket_Request_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: saturated uint8[<=508] payload
#ifndef uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_CAPACITY_
#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_CAPACITY_           508U
#elif !defined(uavcan_internet_udp_HandleIncomingPacket_Request_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_)
#  define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_
#endif
#if uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_CAPACITY_ > 508U
#  error uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_CAPACITY_ > 508U
#endif
#define uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_IS_VARIABLE_LENGTH_ true

typedef struct
{
    /// saturated uint16 session_id
    uint16_t session_id;

    /// saturated uint8[<=508] payload
    struct  /// Array address equivalence guarantee: &elements[0] == &payload
    {
        uint8_t elements[uavcan_internet_udp_HandleIncomingPacket_Request_0_2_payload_ARRAY_CAPACITY_];
        size_t count;
    } payload;
} uavcan_internet_udp_HandleIncomingPacket_Request_0_2;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_internet_udp_HandleIncomingPacket_Request_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_internet_udp_HandleIncomingPacket_Request_0_2_serialize_(
    const uavcan_internet_udp_HandleIncomingPacket_Request_0_2* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
#ifndef uavcan_internet_udp_HandleIncomingPacket_Request_0_2_DISABLE_SERIALIZATION_BUFFER_CHECK_

    if ((8U * (size_t) capacity_bytes) < 4096UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
#endif

    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated uint16 session_id
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        (void) memmove(&buffer[offset_bits / 8U], &obj->session_id, 2U);
        offset_bits += 16U;
    }




    {   // saturated uint8[<=508] payload
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 4080ULL) <= (capacity_bytes * 8U));
        if (obj->payload.count > 508)
        {
            return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
        }
        // Array length prefix: truncated uint16
        (void) memmove(&buffer[offset_bits / 8U], &obj->payload.count, 2U);
        offset_bits += 16U;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        // Optimization prospect: this item is aligned at the byte boundary, so it is possible to use memmove().
        nunavutCopyBits(&buffer[0], offset_bits, obj->payload.count * 8U, &obj->payload.elements[0], 0U);
        offset_bits += obj->payload.count * 8U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits >= 32ULL);
    NUNAVUT_ASSERT(offset_bits <= 4096ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
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
static inline int8_t uavcan_internet_udp_HandleIncomingPacket_Request_0_2_deserialize_(
    uavcan_internet_udp_HandleIncomingPacket_Request_0_2* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated uint16 session_id
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    out_obj->session_id = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;




    // saturated uint8[<=508] payload
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    // Array length prefix: truncated uint16
    out_obj->payload.count = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;
    if (out_obj->payload.count > 508U)
    {
        return -NUNAVUT_ERROR_REPRESENTATION_BAD_ARRAY_LENGTH;
    }
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    nunavutGetBits(&out_obj->payload.elements[0], &buffer[0], capacity_bytes, offset_bits, out_obj->payload.count * 8U);
    offset_bits += out_obj->payload.count * 8U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_internet_udp_HandleIncomingPacket_Request_0_2_initialize_(uavcan_internet_udp_HandleIncomingPacket_Request_0_2* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_internet_udp_HandleIncomingPacket_Request_0_2_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}



#define uavcan_internet_udp_HandleIncomingPacket_Response_0_2_FULL_NAME_             "uavcan.internet.udp.HandleIncomingPacket.Response"
#define uavcan_internet_udp_HandleIncomingPacket_Response_0_2_FULL_NAME_AND_VERSION_ "uavcan.internet.udp.HandleIncomingPacket.Response.0.2"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define uavcan_internet_udp_HandleIncomingPacket_Response_0_2_EXTENT_BYTES_                    63UL
#define uavcan_internet_udp_HandleIncomingPacket_Response_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_ 0UL
static_assert(uavcan_internet_udp_HandleIncomingPacket_Response_0_2_EXTENT_BYTES_ >= uavcan_internet_udp_HandleIncomingPacket_Response_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    uint8_t _dummy_;
} uavcan_internet_udp_HandleIncomingPacket_Response_0_2;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see uavcan_internet_udp_HandleIncomingPacket_Response_0_2_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t uavcan_internet_udp_HandleIncomingPacket_Response_0_2_serialize_(
    const uavcan_internet_udp_HandleIncomingPacket_Response_0_2* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    *inout_buffer_size_bytes = 0U;

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
static inline int8_t uavcan_internet_udp_HandleIncomingPacket_Response_0_2_deserialize_(
    uavcan_internet_udp_HandleIncomingPacket_Response_0_2* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    *inout_buffer_size_bytes = 0U;

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void uavcan_internet_udp_HandleIncomingPacket_Response_0_2_initialize_(uavcan_internet_udp_HandleIncomingPacket_Response_0_2* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = uavcan_internet_udp_HandleIncomingPacket_Response_0_2_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // UAVCAN_INTERNET_UDP_HANDLE_INCOMING_PACKET_0_2_INCLUDED_