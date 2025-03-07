/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/camera_gimbal/Mode.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_CAMERA_GIMBAL_MODE_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_MODE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Gimbal operating mode
#

uint8 COMMAND_MODE_ANGULAR_VELOCITY        = 0
uint8 COMMAND_MODE_ORIENTATION_FIXED_FRAME = 1
uint8 COMMAND_MODE_ORIENTATION_BODY_FRAME  = 2
uint8 COMMAND_MODE_GEO_POI                 = 3
uint8 command_mode
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.camera_gimbal.Mode
saturated uint8 command_mode
******************************************************************************/

#undef command_mode
#undef COMMAND_MODE_ANGULAR_VELOCITY
#undef COMMAND_MODE_ORIENTATION_FIXED_FRAME
#undef COMMAND_MODE_ORIENTATION_BODY_FRAME
#undef COMMAND_MODE_GEO_POI

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <int _tmpl>
struct UAVCAN_EXPORT Mode_
{
    typedef const Mode_<_tmpl>& ParameterType;
    typedef Mode_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > COMMAND_MODE_ANGULAR_VELOCITY;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > COMMAND_MODE_ORIENTATION_FIXED_FRAME;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > COMMAND_MODE_ORIENTATION_BODY_FRAME;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > COMMAND_MODE_GEO_POI;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > command_mode;
    };

    enum
    {
        MinBitLen
            = FieldTypes::command_mode::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::command_mode::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::COMMAND_MODE_ANGULAR_VELOCITY >::Type COMMAND_MODE_ANGULAR_VELOCITY; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::COMMAND_MODE_ORIENTATION_FIXED_FRAME >::Type COMMAND_MODE_ORIENTATION_FIXED_FRAME; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::COMMAND_MODE_ORIENTATION_BODY_FRAME >::Type COMMAND_MODE_ORIENTATION_BODY_FRAME; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::COMMAND_MODE_GEO_POI >::Type COMMAND_MODE_GEO_POI; // 3

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::command_mode >::Type command_mode;

    Mode_()
        : command_mode()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<8 == MaxBitLen>::check();
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
        return "uavcan.equipment.camera_gimbal.Mode";
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
bool Mode_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        command_mode == rhs.command_mode;
}

template <int _tmpl>
bool Mode_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(command_mode, rhs.command_mode);
}

template <int _tmpl>
int Mode_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::command_mode::encode(self.command_mode, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Mode_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::command_mode::decode(self.command_mode, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Mode_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x9108C7785AEB69C4ULL);

    FieldTypes::command_mode::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename Mode_<_tmpl>::ConstantTypes::COMMAND_MODE_ANGULAR_VELOCITY >::Type
    Mode_<_tmpl>::COMMAND_MODE_ANGULAR_VELOCITY = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename Mode_<_tmpl>::ConstantTypes::COMMAND_MODE_ORIENTATION_FIXED_FRAME >::Type
    Mode_<_tmpl>::COMMAND_MODE_ORIENTATION_FIXED_FRAME = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename Mode_<_tmpl>::ConstantTypes::COMMAND_MODE_ORIENTATION_BODY_FRAME >::Type
    Mode_<_tmpl>::COMMAND_MODE_ORIENTATION_BODY_FRAME = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename Mode_<_tmpl>::ConstantTypes::COMMAND_MODE_GEO_POI >::Type
    Mode_<_tmpl>::COMMAND_MODE_GEO_POI = 3U; // 3

/*
 * Final typedef
 */
typedef Mode_<0> Mode;

// No default registration

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::camera_gimbal::Mode >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::camera_gimbal::Mode::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::camera_gimbal::Mode >::stream(Stream& s, ::uavcan::equipment::camera_gimbal::Mode::ParameterType obj, const int level)
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
    s << "command_mode: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::Mode::FieldTypes::command_mode >::stream(s, obj.command_mode, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::camera_gimbal::Mode::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::camera_gimbal::Mode >::stream(s, obj, 0);
    return s;
}

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_CAMERA_GIMBAL_MODE_HPP_INCLUDED