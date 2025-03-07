/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/ardupilot/gnss/20002.Heading.uavcan
 */

#ifndef ARDUPILOT_GNSS_HEADING_HPP_INCLUDED
#define ARDUPILOT_GNSS_HEADING_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
bool heading_valid
bool heading_accuracy_valid

float16 heading_rad
float16 heading_accuracy_rad
******************************************************************************/

/********************* DSDL signature source definition ***********************
ardupilot.gnss.Heading
saturated bool heading_valid
saturated bool heading_accuracy_valid
saturated float16 heading_rad
saturated float16 heading_accuracy_rad
******************************************************************************/

#undef heading_valid
#undef heading_accuracy_valid
#undef heading_rad
#undef heading_accuracy_rad

namespace ardupilot
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT Heading_
{
    typedef const Heading_<_tmpl>& ParameterType;
    typedef Heading_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > heading_valid;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > heading_accuracy_valid;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > heading_rad;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > heading_accuracy_rad;
    };

    enum
    {
        MinBitLen
            = FieldTypes::heading_valid::MinBitLen
            + FieldTypes::heading_accuracy_valid::MinBitLen
            + FieldTypes::heading_rad::MinBitLen
            + FieldTypes::heading_accuracy_rad::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::heading_valid::MaxBitLen
            + FieldTypes::heading_accuracy_valid::MaxBitLen
            + FieldTypes::heading_rad::MaxBitLen
            + FieldTypes::heading_accuracy_rad::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::heading_valid >::Type heading_valid;
    typename ::uavcan::StorageType< typename FieldTypes::heading_accuracy_valid >::Type heading_accuracy_valid;
    typename ::uavcan::StorageType< typename FieldTypes::heading_rad >::Type heading_rad;
    typename ::uavcan::StorageType< typename FieldTypes::heading_accuracy_rad >::Type heading_accuracy_rad;

    Heading_()
        : heading_valid()
        , heading_accuracy_valid()
        , heading_rad()
        , heading_accuracy_rad()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<34 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 20002 };

    static const char* getDataTypeFullName()
    {
        return "ardupilot.gnss.Heading";
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
bool Heading_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        heading_valid == rhs.heading_valid &&
        heading_accuracy_valid == rhs.heading_accuracy_valid &&
        heading_rad == rhs.heading_rad &&
        heading_accuracy_rad == rhs.heading_accuracy_rad;
}

template <int _tmpl>
bool Heading_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(heading_valid, rhs.heading_valid) &&
        ::uavcan::areClose(heading_accuracy_valid, rhs.heading_accuracy_valid) &&
        ::uavcan::areClose(heading_rad, rhs.heading_rad) &&
        ::uavcan::areClose(heading_accuracy_rad, rhs.heading_accuracy_rad);
}

template <int _tmpl>
int Heading_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::heading_valid::encode(self.heading_valid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_accuracy_valid::encode(self.heading_accuracy_valid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_rad::encode(self.heading_rad, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_accuracy_rad::encode(self.heading_accuracy_rad, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Heading_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::heading_valid::decode(self.heading_valid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_accuracy_valid::decode(self.heading_accuracy_valid, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_rad::decode(self.heading_rad, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::heading_accuracy_rad::decode(self.heading_accuracy_rad, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Heading_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x315CAE39ECED3412ULL);

    FieldTypes::heading_valid::extendDataTypeSignature(signature);
    FieldTypes::heading_accuracy_valid::extendDataTypeSignature(signature);
    FieldTypes::heading_rad::extendDataTypeSignature(signature);
    FieldTypes::heading_accuracy_rad::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Heading_<0> Heading;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::ardupilot::gnss::Heading > _uavcan_gdtr_registrator_Heading;

}

} // Namespace gnss
} // Namespace ardupilot

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::ardupilot::gnss::Heading >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::ardupilot::gnss::Heading::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::ardupilot::gnss::Heading >::stream(Stream& s, ::ardupilot::gnss::Heading::ParameterType obj, const int level)
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
    s << "heading_valid: ";
    YamlStreamer< ::ardupilot::gnss::Heading::FieldTypes::heading_valid >::stream(s, obj.heading_valid, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "heading_accuracy_valid: ";
    YamlStreamer< ::ardupilot::gnss::Heading::FieldTypes::heading_accuracy_valid >::stream(s, obj.heading_accuracy_valid, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "heading_rad: ";
    YamlStreamer< ::ardupilot::gnss::Heading::FieldTypes::heading_rad >::stream(s, obj.heading_rad, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "heading_accuracy_rad: ";
    YamlStreamer< ::ardupilot::gnss::Heading::FieldTypes::heading_accuracy_rad >::stream(s, obj.heading_accuracy_rad, level + 1);
}

}

namespace ardupilot
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::ardupilot::gnss::Heading::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::ardupilot::gnss::Heading >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace ardupilot

#endif // ARDUPILOT_GNSS_HEADING_HPP_INCLUDED