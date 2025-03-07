/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/param/Empty.uavcan
 */

#ifndef UAVCAN_PROTOCOL_PARAM_EMPTY_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PARAM_EMPTY_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Ex nihilo nihil fit.
#
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.param.Empty
******************************************************************************/

namespace uavcan
{
namespace protocol
{
namespace param
{

template <int _tmpl>
struct UAVCAN_EXPORT Empty_
{
    typedef const Empty_<_tmpl>& ParameterType;
    typedef Empty_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
    };

    enum
    {
        MinBitLen
    };

    enum
    {
        MaxBitLen
    };

    // Constants

    // Fields

    Empty_()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<0 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.param.Empty";
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
bool Empty_<_tmpl>::operator==(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
bool Empty_<_tmpl>::isClose(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
int Empty_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
int Empty_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Empty_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x6C4D0E8EF37361DFULL);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Empty_<0> Empty;

// No default registration

} // Namespace param
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::param::Empty >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::param::Empty::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::param::Empty >::stream(Stream& s, ::uavcan::protocol::param::Empty::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
}

}

namespace uavcan
{
namespace protocol
{
namespace param
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::param::Empty::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::param::Empty >::stream(s, obj, 0);
    return s;
}

} // Namespace param
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_PARAM_EMPTY_HPP_INCLUDED