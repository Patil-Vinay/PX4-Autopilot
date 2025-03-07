/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/debug/16370.KeyValue.uavcan
 */

#ifndef UAVCAN_PROTOCOL_DEBUG_KEYVALUE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DEBUG_KEYVALUE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Generic named parameter (key/value pair).
#

#
# Integers are exactly representable in the range (-2^24, 2^24) which is (-16'777'216, 16'777'216).
#
float32 value

#
# Tail array optimization is enabled, so if key length does not exceed 3 characters, the whole
# message can fit into one CAN frame. The message always fits into one CAN FD frame.
#
uint8[<=58] key
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.debug.KeyValue
saturated float32 value
saturated uint8[<=58] key
******************************************************************************/

#undef value
#undef key

namespace uavcan
{
namespace protocol
{
namespace debug
{

template <int _tmpl>
struct UAVCAN_EXPORT KeyValue_
{
    typedef const KeyValue_<_tmpl>& ParameterType;
    typedef KeyValue_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > value;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 58 > key;
    };

    enum
    {
        MinBitLen
            = FieldTypes::value::MinBitLen
            + FieldTypes::key::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::value::MaxBitLen
            + FieldTypes::key::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::value >::Type value;
    typename ::uavcan::StorageType< typename FieldTypes::key >::Type key;

    KeyValue_()
        : value()
        , key()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<502 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 16370 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.debug.KeyValue";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool KeyValue_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        value == rhs.value &&
        key == rhs.key;
}

template <int _tmpl>
bool KeyValue_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(value, rhs.value) &&
        ::uavcan::areClose(key, rhs.key);
}

template <int _tmpl>
int KeyValue_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::value::encode(self.value, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::key::encode(self.key, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int KeyValue_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::value::decode(self.value, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::key::decode(self.key, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature KeyValue_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xE02F25D6E0C98AE0ULL);

    FieldTypes::value::extendDataTypeSignature(signature);
    FieldTypes::key::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef KeyValue_<0> KeyValue;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::debug::KeyValue > _uavcan_gdtr_registrator_KeyValue;

}

} // Namespace debug
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::debug::KeyValue >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::debug::KeyValue::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::debug::KeyValue >::stream(Stream& s, ::uavcan::protocol::debug::KeyValue::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "value: ";
    YamlStreamer< ::uavcan::protocol::debug::KeyValue::FieldTypes::value >::stream(s, obj.value, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "key: ";
    YamlStreamer< ::uavcan::protocol::debug::KeyValue::FieldTypes::key >::stream(s, obj.key, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace debug
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::debug::KeyValue::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::debug::KeyValue >::stream(s, obj, 0);
    return s;
}

} // Namespace debug
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_DEBUG_KEYVALUE_HPP_INCLUDED