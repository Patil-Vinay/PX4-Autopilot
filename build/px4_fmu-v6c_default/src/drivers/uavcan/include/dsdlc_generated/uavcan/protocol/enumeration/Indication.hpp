/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/enumeration/380.Indication.uavcan
 */

#ifndef UAVCAN_PROTOCOL_ENUMERATION_INDICATION_HPP_INCLUDED
#define UAVCAN_PROTOCOL_ENUMERATION_INDICATION_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/param/NumericValue.hpp>

/******************************* Source text **********************************
#
# This message will be broadcasted when the node receives user input in the process of enumeration.
#

void6

#
# This field is unused; keep it empty
#
uavcan.protocol.param.NumericValue value

#
# Name of the enumerated parameter.
# This field must always be populated by the enumeratee.
# If multiple parameters were enumerated at once (e.g. ESC index and the direction of rotation),
# the field should contain the name of the most important parameter.
#
uint8[<=92] parameter_name
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.enumeration.Indication
void6
uavcan.protocol.param.NumericValue value
saturated uint8[<=92] parameter_name
******************************************************************************/

#undef _void_0
#undef value
#undef parameter_name

namespace uavcan
{
namespace protocol
{
namespace enumeration
{

template <int _tmpl>
struct UAVCAN_EXPORT Indication_
{
    typedef const Indication_<_tmpl>& ParameterType;
    typedef Indication_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 6, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > _void_0;
        typedef ::uavcan::protocol::param::NumericValue value;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 92 > parameter_name;
    };

    enum
    {
        MinBitLen
            = FieldTypes::_void_0::MinBitLen
            + FieldTypes::value::MinBitLen
            + FieldTypes::parameter_name::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::_void_0::MaxBitLen
            + FieldTypes::value::MaxBitLen
            + FieldTypes::parameter_name::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::value >::Type value;
    typename ::uavcan::StorageType< typename FieldTypes::parameter_name >::Type parameter_name;

    Indication_()
        : value()
        , parameter_name()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<815 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 380 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.enumeration.Indication";
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
bool Indication_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        value == rhs.value &&
        parameter_name == rhs.parameter_name;
}

template <int _tmpl>
bool Indication_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(value, rhs.value) &&
        ::uavcan::areClose(parameter_name, rhs.parameter_name);
}

template <int _tmpl>
int Indication_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::_void_0::encode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::value::encode(self.value, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::parameter_name::encode(self.parameter_name, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Indication_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::_void_0::decode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::value::decode(self.value, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::parameter_name::decode(self.parameter_name, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Indication_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xF4C6258908FD263BULL);

    FieldTypes::_void_0::extendDataTypeSignature(signature);
    FieldTypes::value::extendDataTypeSignature(signature);
    FieldTypes::parameter_name::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Indication_<0> Indication;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::enumeration::Indication > _uavcan_gdtr_registrator_Indication;

}

} // Namespace enumeration
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::enumeration::Indication >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::enumeration::Indication::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::enumeration::Indication >::stream(Stream& s, ::uavcan::protocol::enumeration::Indication::ParameterType obj, const int level)
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
    YamlStreamer< ::uavcan::protocol::enumeration::Indication::FieldTypes::value >::stream(s, obj.value, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "parameter_name: ";
    YamlStreamer< ::uavcan::protocol::enumeration::Indication::FieldTypes::parameter_name >::stream(s, obj.parameter_name, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace enumeration
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::enumeration::Indication::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::enumeration::Indication >::stream(s, obj, 0);
    return s;
}

} // Namespace enumeration
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_ENUMERATION_INDICATION_HPP_INCLUDED