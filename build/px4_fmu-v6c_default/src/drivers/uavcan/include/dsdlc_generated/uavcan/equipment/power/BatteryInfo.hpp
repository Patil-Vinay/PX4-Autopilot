/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/power/1092.BatteryInfo.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_POWER_BATTERYINFO_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_POWER_BATTERYINFO_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Single battery info.
#
# Typical publishing rate should be around 0.2~1 Hz.
#
# Please refer to the Smart Battery data specification for some elaboration.
#

#
# Primary parameters.
# Some fields can be set to NAN if their values are unknown.
# Full charge capacity is expected to slowly reduce as the battery is aging. Normally its estimate is updated after
# every charging cycle.
#
float16 temperature             # [Kelvin]
float16 voltage                 # [Volt]
float16 current                 # [Ampere]
float16 average_power_10sec     # [Watt]        Average power consumption over the last 10 seconds
float16 remaining_capacity_wh   # [Watt hours]  Will be increasing during charging
float16 full_charge_capacity_wh # [Watt hours]  Predicted battery capacity when it is fully charged. Falls with aging
float16 hours_to_full_charge    # [Hours]       Charging is expected to complete in this time; zero if not charging

#
# Status flags.
# Notes:
#  - CHARGING must be always set as long as the battery is connected to a charger, even if the charging is complete.
#  - CHARGED must be cleared immediately when the charger is disconnected.
#
uint11 STATUS_FLAG_IN_USE       = 1     # The battery is currently used as a power supply
uint11 STATUS_FLAG_CHARGING     = 2     # Charger is active
uint11 STATUS_FLAG_CHARGED      = 4     # Charging complete, but the charger is still active
uint11 STATUS_FLAG_TEMP_HOT     = 8     # Battery temperature is above normal
uint11 STATUS_FLAG_TEMP_COLD    = 16    # Battery temperature is below normal
uint11 STATUS_FLAG_OVERLOAD     = 32    # Safe operating area violation
uint11 STATUS_FLAG_BAD_BATTERY  = 64    # This battery should not be used anymore (e.g. low SOH)
uint11 STATUS_FLAG_NEED_SERVICE = 128   # This battery requires maintenance (e.g. balancing, full recharge)
uint11 STATUS_FLAG_BMS_ERROR    = 256   # Battery management system/controller error, smart battery interface error
uint11 STATUS_FLAG_RESERVED_A   = 512   # Keep zero
uint11 STATUS_FLAG_RESERVED_B   = 1024  # Keep zero
uint11 status_flags

#
# State of Health (SOH) estimate, in percent.
# http://en.wikipedia.org/wiki/State_of_health
#
uint7 STATE_OF_HEALTH_UNKNOWN = 127     # Use this constant if SOH cannot be estimated
uint7 state_of_health_pct               # Health of the battery, in percent, optional

#
# Relative State of Charge (SOC) estimate, in percent.
# http://en.wikipedia.org/wiki/State_of_charge
#
uint7 state_of_charge_pct               # Percent of the full charge [0, 100]. This field is required
uint7 state_of_charge_pct_stdev         # SOC error standard deviation; use best guess if unknown

#
# Battery identification.
# Model instance ID must be unique within the same battery model name.
# Model name is a human-readable string that normally should include the vendor name, model name, and chemistry
# type of this battery. This field should be assumed case-insensitive. Example: "Zubax Smart Battery v1.1 LiPo".
#
uint8 battery_id                        # Identifies the battery within this vehicle, e.g. 0 - primary battery
uint32 model_instance_id                # Set to zero if not applicable
uint8[<32] model_name                   # Battery model name
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.power.BatteryInfo
saturated float16 temperature
saturated float16 voltage
saturated float16 current
saturated float16 average_power_10sec
saturated float16 remaining_capacity_wh
saturated float16 full_charge_capacity_wh
saturated float16 hours_to_full_charge
saturated uint11 status_flags
saturated uint7 state_of_health_pct
saturated uint7 state_of_charge_pct
saturated uint7 state_of_charge_pct_stdev
saturated uint8 battery_id
saturated uint32 model_instance_id
saturated uint8[<=31] model_name
******************************************************************************/

#undef temperature
#undef voltage
#undef current
#undef average_power_10sec
#undef remaining_capacity_wh
#undef full_charge_capacity_wh
#undef hours_to_full_charge
#undef status_flags
#undef state_of_health_pct
#undef state_of_charge_pct
#undef state_of_charge_pct_stdev
#undef battery_id
#undef model_instance_id
#undef model_name
#undef STATUS_FLAG_IN_USE
#undef STATUS_FLAG_CHARGING
#undef STATUS_FLAG_CHARGED
#undef STATUS_FLAG_TEMP_HOT
#undef STATUS_FLAG_TEMP_COLD
#undef STATUS_FLAG_OVERLOAD
#undef STATUS_FLAG_BAD_BATTERY
#undef STATUS_FLAG_NEED_SERVICE
#undef STATUS_FLAG_BMS_ERROR
#undef STATUS_FLAG_RESERVED_A
#undef STATUS_FLAG_RESERVED_B
#undef STATE_OF_HEALTH_UNKNOWN

namespace uavcan
{
namespace equipment
{
namespace power
{

template <int _tmpl>
struct UAVCAN_EXPORT BatteryInfo_
{
    typedef const BatteryInfo_<_tmpl>& ParameterType;
    typedef BatteryInfo_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_IN_USE;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_CHARGING;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_CHARGED;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_TEMP_HOT;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_TEMP_COLD;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_OVERLOAD;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_BAD_BATTERY;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_NEED_SERVICE;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_BMS_ERROR;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_RESERVED_A;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FLAG_RESERVED_B;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATE_OF_HEALTH_UNKNOWN;
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > voltage;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > current;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > average_power_10sec;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > remaining_capacity_wh;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > full_charge_capacity_wh;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > hours_to_full_charge;
        typedef ::uavcan::IntegerSpec< 11, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > status_flags;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > state_of_health_pct;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > state_of_charge_pct;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > state_of_charge_pct_stdev;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > battery_id;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > model_instance_id;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 31 > model_name;
    };

    enum
    {
        MinBitLen
            = FieldTypes::temperature::MinBitLen
            + FieldTypes::voltage::MinBitLen
            + FieldTypes::current::MinBitLen
            + FieldTypes::average_power_10sec::MinBitLen
            + FieldTypes::remaining_capacity_wh::MinBitLen
            + FieldTypes::full_charge_capacity_wh::MinBitLen
            + FieldTypes::hours_to_full_charge::MinBitLen
            + FieldTypes::status_flags::MinBitLen
            + FieldTypes::state_of_health_pct::MinBitLen
            + FieldTypes::state_of_charge_pct::MinBitLen
            + FieldTypes::state_of_charge_pct_stdev::MinBitLen
            + FieldTypes::battery_id::MinBitLen
            + FieldTypes::model_instance_id::MinBitLen
            + FieldTypes::model_name::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::temperature::MaxBitLen
            + FieldTypes::voltage::MaxBitLen
            + FieldTypes::current::MaxBitLen
            + FieldTypes::average_power_10sec::MaxBitLen
            + FieldTypes::remaining_capacity_wh::MaxBitLen
            + FieldTypes::full_charge_capacity_wh::MaxBitLen
            + FieldTypes::hours_to_full_charge::MaxBitLen
            + FieldTypes::status_flags::MaxBitLen
            + FieldTypes::state_of_health_pct::MaxBitLen
            + FieldTypes::state_of_charge_pct::MaxBitLen
            + FieldTypes::state_of_charge_pct_stdev::MaxBitLen
            + FieldTypes::battery_id::MaxBitLen
            + FieldTypes::model_instance_id::MaxBitLen
            + FieldTypes::model_name::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_IN_USE >::Type STATUS_FLAG_IN_USE; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_CHARGING >::Type STATUS_FLAG_CHARGING; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_CHARGED >::Type STATUS_FLAG_CHARGED; // 4
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_TEMP_HOT >::Type STATUS_FLAG_TEMP_HOT; // 8
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_TEMP_COLD >::Type STATUS_FLAG_TEMP_COLD; // 16
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_OVERLOAD >::Type STATUS_FLAG_OVERLOAD; // 32
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_BAD_BATTERY >::Type STATUS_FLAG_BAD_BATTERY; // 64
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_NEED_SERVICE >::Type STATUS_FLAG_NEED_SERVICE; // 128
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_BMS_ERROR >::Type STATUS_FLAG_BMS_ERROR; // 256
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_RESERVED_A >::Type STATUS_FLAG_RESERVED_A; // 512
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FLAG_RESERVED_B >::Type STATUS_FLAG_RESERVED_B; // 1024
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATE_OF_HEALTH_UNKNOWN >::Type STATE_OF_HEALTH_UNKNOWN; // 127

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::temperature >::Type temperature;
    typename ::uavcan::StorageType< typename FieldTypes::voltage >::Type voltage;
    typename ::uavcan::StorageType< typename FieldTypes::current >::Type current;
    typename ::uavcan::StorageType< typename FieldTypes::average_power_10sec >::Type average_power_10sec;
    typename ::uavcan::StorageType< typename FieldTypes::remaining_capacity_wh >::Type remaining_capacity_wh;
    typename ::uavcan::StorageType< typename FieldTypes::full_charge_capacity_wh >::Type full_charge_capacity_wh;
    typename ::uavcan::StorageType< typename FieldTypes::hours_to_full_charge >::Type hours_to_full_charge;
    typename ::uavcan::StorageType< typename FieldTypes::status_flags >::Type status_flags;
    typename ::uavcan::StorageType< typename FieldTypes::state_of_health_pct >::Type state_of_health_pct;
    typename ::uavcan::StorageType< typename FieldTypes::state_of_charge_pct >::Type state_of_charge_pct;
    typename ::uavcan::StorageType< typename FieldTypes::state_of_charge_pct_stdev >::Type state_of_charge_pct_stdev;
    typename ::uavcan::StorageType< typename FieldTypes::battery_id >::Type battery_id;
    typename ::uavcan::StorageType< typename FieldTypes::model_instance_id >::Type model_instance_id;
    typename ::uavcan::StorageType< typename FieldTypes::model_name >::Type model_name;

    BatteryInfo_()
        : temperature()
        , voltage()
        , current()
        , average_power_10sec()
        , remaining_capacity_wh()
        , full_charge_capacity_wh()
        , hours_to_full_charge()
        , status_flags()
        , state_of_health_pct()
        , state_of_charge_pct()
        , state_of_charge_pct_stdev()
        , battery_id()
        , model_instance_id()
        , model_name()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<437 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1092 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.power.BatteryInfo";
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
bool BatteryInfo_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        temperature == rhs.temperature &&
        voltage == rhs.voltage &&
        current == rhs.current &&
        average_power_10sec == rhs.average_power_10sec &&
        remaining_capacity_wh == rhs.remaining_capacity_wh &&
        full_charge_capacity_wh == rhs.full_charge_capacity_wh &&
        hours_to_full_charge == rhs.hours_to_full_charge &&
        status_flags == rhs.status_flags &&
        state_of_health_pct == rhs.state_of_health_pct &&
        state_of_charge_pct == rhs.state_of_charge_pct &&
        state_of_charge_pct_stdev == rhs.state_of_charge_pct_stdev &&
        battery_id == rhs.battery_id &&
        model_instance_id == rhs.model_instance_id &&
        model_name == rhs.model_name;
}

template <int _tmpl>
bool BatteryInfo_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(temperature, rhs.temperature) &&
        ::uavcan::areClose(voltage, rhs.voltage) &&
        ::uavcan::areClose(current, rhs.current) &&
        ::uavcan::areClose(average_power_10sec, rhs.average_power_10sec) &&
        ::uavcan::areClose(remaining_capacity_wh, rhs.remaining_capacity_wh) &&
        ::uavcan::areClose(full_charge_capacity_wh, rhs.full_charge_capacity_wh) &&
        ::uavcan::areClose(hours_to_full_charge, rhs.hours_to_full_charge) &&
        ::uavcan::areClose(status_flags, rhs.status_flags) &&
        ::uavcan::areClose(state_of_health_pct, rhs.state_of_health_pct) &&
        ::uavcan::areClose(state_of_charge_pct, rhs.state_of_charge_pct) &&
        ::uavcan::areClose(state_of_charge_pct_stdev, rhs.state_of_charge_pct_stdev) &&
        ::uavcan::areClose(battery_id, rhs.battery_id) &&
        ::uavcan::areClose(model_instance_id, rhs.model_instance_id) &&
        ::uavcan::areClose(model_name, rhs.model_name);
}

template <int _tmpl>
int BatteryInfo_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::temperature::encode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::encode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::encode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::average_power_10sec::encode(self.average_power_10sec, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_capacity_wh::encode(self.remaining_capacity_wh, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::full_charge_capacity_wh::encode(self.full_charge_capacity_wh, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hours_to_full_charge::encode(self.hours_to_full_charge, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status_flags::encode(self.status_flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_health_pct::encode(self.state_of_health_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_charge_pct::encode(self.state_of_charge_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_charge_pct_stdev::encode(self.state_of_charge_pct_stdev, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::battery_id::encode(self.battery_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::model_instance_id::encode(self.model_instance_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::model_name::encode(self.model_name, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int BatteryInfo_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::temperature::decode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::voltage::decode(self.voltage, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::current::decode(self.current, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::average_power_10sec::decode(self.average_power_10sec, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::remaining_capacity_wh::decode(self.remaining_capacity_wh, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::full_charge_capacity_wh::decode(self.full_charge_capacity_wh, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::hours_to_full_charge::decode(self.hours_to_full_charge, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status_flags::decode(self.status_flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_health_pct::decode(self.state_of_health_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_charge_pct::decode(self.state_of_charge_pct, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::state_of_charge_pct_stdev::decode(self.state_of_charge_pct_stdev, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::battery_id::decode(self.battery_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::model_instance_id::decode(self.model_instance_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::model_name::decode(self.model_name, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature BatteryInfo_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x249C26548A711966ULL);

    FieldTypes::temperature::extendDataTypeSignature(signature);
    FieldTypes::voltage::extendDataTypeSignature(signature);
    FieldTypes::current::extendDataTypeSignature(signature);
    FieldTypes::average_power_10sec::extendDataTypeSignature(signature);
    FieldTypes::remaining_capacity_wh::extendDataTypeSignature(signature);
    FieldTypes::full_charge_capacity_wh::extendDataTypeSignature(signature);
    FieldTypes::hours_to_full_charge::extendDataTypeSignature(signature);
    FieldTypes::status_flags::extendDataTypeSignature(signature);
    FieldTypes::state_of_health_pct::extendDataTypeSignature(signature);
    FieldTypes::state_of_charge_pct::extendDataTypeSignature(signature);
    FieldTypes::state_of_charge_pct_stdev::extendDataTypeSignature(signature);
    FieldTypes::battery_id::extendDataTypeSignature(signature);
    FieldTypes::model_instance_id::extendDataTypeSignature(signature);
    FieldTypes::model_name::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_IN_USE >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_IN_USE = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_CHARGING >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_CHARGING = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_CHARGED >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_CHARGED = 4U; // 4

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_TEMP_HOT >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_TEMP_HOT = 8U; // 8

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_TEMP_COLD >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_TEMP_COLD = 16U; // 16

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_OVERLOAD >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_OVERLOAD = 32U; // 32

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_BAD_BATTERY >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_BAD_BATTERY = 64U; // 64

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_NEED_SERVICE >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_NEED_SERVICE = 128U; // 128

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_BMS_ERROR >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_BMS_ERROR = 256U; // 256

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_RESERVED_A >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_RESERVED_A = 512U; // 512

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATUS_FLAG_RESERVED_B >::Type
    BatteryInfo_<_tmpl>::STATUS_FLAG_RESERVED_B = 1024U; // 1024

template <int _tmpl>
const typename ::uavcan::StorageType< typename BatteryInfo_<_tmpl>::ConstantTypes::STATE_OF_HEALTH_UNKNOWN >::Type
    BatteryInfo_<_tmpl>::STATE_OF_HEALTH_UNKNOWN = 127U; // 127

/*
 * Final typedef
 */
typedef BatteryInfo_<0> BatteryInfo;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::power::BatteryInfo > _uavcan_gdtr_registrator_BatteryInfo;

}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::power::BatteryInfo >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::power::BatteryInfo::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::power::BatteryInfo >::stream(Stream& s, ::uavcan::equipment::power::BatteryInfo::ParameterType obj, const int level)
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
    s << "temperature: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::temperature >::stream(s, obj.temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "voltage: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::voltage >::stream(s, obj.voltage, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "current: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::current >::stream(s, obj.current, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "average_power_10sec: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::average_power_10sec >::stream(s, obj.average_power_10sec, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "remaining_capacity_wh: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::remaining_capacity_wh >::stream(s, obj.remaining_capacity_wh, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "full_charge_capacity_wh: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::full_charge_capacity_wh >::stream(s, obj.full_charge_capacity_wh, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "hours_to_full_charge: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::hours_to_full_charge >::stream(s, obj.hours_to_full_charge, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "status_flags: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::status_flags >::stream(s, obj.status_flags, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "state_of_health_pct: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::state_of_health_pct >::stream(s, obj.state_of_health_pct, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "state_of_charge_pct: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::state_of_charge_pct >::stream(s, obj.state_of_charge_pct, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "state_of_charge_pct_stdev: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::state_of_charge_pct_stdev >::stream(s, obj.state_of_charge_pct_stdev, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "battery_id: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::battery_id >::stream(s, obj.battery_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "model_instance_id: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::model_instance_id >::stream(s, obj.model_instance_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "model_name: ";
    YamlStreamer< ::uavcan::equipment::power::BatteryInfo::FieldTypes::model_name >::stream(s, obj.model_name, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace power
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::power::BatteryInfo::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::power::BatteryInfo >::stream(s, obj, 0);
    return s;
}

} // Namespace power
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_POWER_BATTERYINFO_HPP_INCLUDED