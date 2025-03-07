/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/vinay/Downloads/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/dynamic_node_id/server/30.AppendEntries.uavcan
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/protocol/dynamic_node_id/server/Entry.hpp>

/******************************* Source text **********************************
#
# THIS DEFINITION IS SUBJECT TO CHANGE.
#
# This type is a part of the Raft consensus algorithm.
# Please refer to the specification for details.
#

#
# Given min election timeout and cluster size, the maximum recommended request interval can be derived as follows:
#
#   max recommended request interval = (min election timeout) / 2 requests / (cluster size - 1)
#
# The equation assumes that the Leader requests one Follower at a time, so that there's at most one pending call
# at any moment. Such behavior is optimal as it creates uniform bus load, but it is actually implementation-specific.
# Obviously, request interval can be lower than that if needed, but higher values are not recommended as they may
# cause Followers to initiate premature elections in case of intensive frame losses or delays.
#
# Real timeout is randomized in the range (MIN, MAX], according to the Raft paper.
#
uint16 DEFAULT_MIN_ELECTION_TIMEOUT_MS = 2000
uint16 DEFAULT_MAX_ELECTION_TIMEOUT_MS = 4000

#
# Refer to the Raft paper for explanation.
#
uint32 term
uint32 prev_log_term
uint8 prev_log_index
uint8 leader_commit

#
# Worst-case replication time per Follower can be computed as:
#
#   worst replication time = (127 log entries) * (2 trips of next_index) * (request interval per Follower)
#
Entry[<=1] entries

---

#
# Refer to the Raft paper for explanation.
#
uint32 term
bool success
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.dynamic_node_id.server.AppendEntries
saturated uint32 term
saturated uint32 prev_log_term
saturated uint8 prev_log_index
saturated uint8 leader_commit
uavcan.protocol.dynamic_node_id.server.Entry[<=1] entries
---
saturated uint32 term
saturated bool success
******************************************************************************/

#undef term
#undef prev_log_term
#undef prev_log_index
#undef leader_commit
#undef entries
#undef DEFAULT_MIN_ELECTION_TIMEOUT_MS
#undef DEFAULT_MAX_ELECTION_TIMEOUT_MS
#undef term
#undef success

namespace uavcan
{
namespace protocol
{
namespace dynamic_node_id
{
namespace server
{

struct UAVCAN_EXPORT AppendEntries_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
            typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > DEFAULT_MIN_ELECTION_TIMEOUT_MS;
            typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > DEFAULT_MAX_ELECTION_TIMEOUT_MS;
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > term;
            typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > prev_log_term;
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > prev_log_index;
            typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > leader_commit;
            typedef ::uavcan::Array< ::uavcan::protocol::dynamic_node_id::server::Entry, ::uavcan::ArrayModeDynamic, 1 > entries;
        };

        enum
        {
            MinBitLen
                = FieldTypes::term::MinBitLen
                + FieldTypes::prev_log_term::MinBitLen
                + FieldTypes::prev_log_index::MinBitLen
                + FieldTypes::leader_commit::MinBitLen
                + FieldTypes::entries::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::term::MaxBitLen
                + FieldTypes::prev_log_term::MaxBitLen
                + FieldTypes::prev_log_index::MaxBitLen
                + FieldTypes::leader_commit::MaxBitLen
                + FieldTypes::entries::MaxBitLen
        };

        // Constants
        static const typename ::uavcan::StorageType< typename ConstantTypes::DEFAULT_MIN_ELECTION_TIMEOUT_MS >::Type DEFAULT_MIN_ELECTION_TIMEOUT_MS; // 2000
        static const typename ::uavcan::StorageType< typename ConstantTypes::DEFAULT_MAX_ELECTION_TIMEOUT_MS >::Type DEFAULT_MAX_ELECTION_TIMEOUT_MS; // 4000

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::term >::Type term;
        typename ::uavcan::StorageType< typename FieldTypes::prev_log_term >::Type prev_log_term;
        typename ::uavcan::StorageType< typename FieldTypes::prev_log_index >::Type prev_log_index;
        typename ::uavcan::StorageType< typename FieldTypes::leader_commit >::Type leader_commit;
        typename ::uavcan::StorageType< typename FieldTypes::entries >::Type entries;

        Request_()
            : term()
            , prev_log_term()
            , prev_log_index()
            , leader_commit()
            , entries()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<249 == MaxBitLen>::check();
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

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
            typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > term;
            typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > success;
        };

        enum
        {
            MinBitLen
                = FieldTypes::term::MinBitLen
                + FieldTypes::success::MinBitLen
        };

        enum
        {
            MaxBitLen
                = FieldTypes::term::MaxBitLen
                + FieldTypes::success::MaxBitLen
        };

        // Constants

        // Fields
        typename ::uavcan::StorageType< typename FieldTypes::term >::Type term;
        typename ::uavcan::StorageType< typename FieldTypes::success >::Type success;

        Response_()
            : term()
            , success()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<33 == MaxBitLen>::check();
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

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 30 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.dynamic_node_id.server.AppendEntries";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    AppendEntries_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool AppendEntries_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        term == rhs.term &&
        prev_log_term == rhs.prev_log_term &&
        prev_log_index == rhs.prev_log_index &&
        leader_commit == rhs.leader_commit &&
        entries == rhs.entries;
}

template <int _tmpl>
bool AppendEntries_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(term, rhs.term) &&
        ::uavcan::areClose(prev_log_term, rhs.prev_log_term) &&
        ::uavcan::areClose(prev_log_index, rhs.prev_log_index) &&
        ::uavcan::areClose(leader_commit, rhs.leader_commit) &&
        ::uavcan::areClose(entries, rhs.entries);
}

template <int _tmpl>
int AppendEntries_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::term::encode(self.term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::prev_log_term::encode(self.prev_log_term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::prev_log_index::encode(self.prev_log_index, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::leader_commit::encode(self.leader_commit, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::entries::encode(self.entries, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int AppendEntries_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::term::decode(self.term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::prev_log_term::decode(self.prev_log_term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::prev_log_index::decode(self.prev_log_index, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::leader_commit::decode(self.leader_commit, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::entries::decode(self.entries, codec,  tao_mode);
    return res;
}

template <int _tmpl>
bool AppendEntries_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        term == rhs.term &&
        success == rhs.success;
}

template <int _tmpl>
bool AppendEntries_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(term, rhs.term) &&
        ::uavcan::areClose(success, rhs.success);
}

template <int _tmpl>
int AppendEntries_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::term::encode(self.term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::success::encode(self.success, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int AppendEntries_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::term::decode(self.term, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::success::decode(self.success, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature AppendEntries_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x102B89200D0E54D2ULL);

    Request::FieldTypes::term::extendDataTypeSignature(signature);
    Request::FieldTypes::prev_log_term::extendDataTypeSignature(signature);
    Request::FieldTypes::prev_log_index::extendDataTypeSignature(signature);
    Request::FieldTypes::leader_commit::extendDataTypeSignature(signature);
    Request::FieldTypes::entries::extendDataTypeSignature(signature);

    Response::FieldTypes::term::extendDataTypeSignature(signature);
    Response::FieldTypes::success::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename AppendEntries_::Request_<_tmpl>::ConstantTypes::DEFAULT_MIN_ELECTION_TIMEOUT_MS >::Type
    AppendEntries_::Request_<_tmpl>::DEFAULT_MIN_ELECTION_TIMEOUT_MS = 2000U; // 2000

template <int _tmpl>
const typename ::uavcan::StorageType< typename AppendEntries_::Request_<_tmpl>::ConstantTypes::DEFAULT_MAX_ELECTION_TIMEOUT_MS >::Type
    AppendEntries_::Request_<_tmpl>::DEFAULT_MAX_ELECTION_TIMEOUT_MS = 4000U; // 4000

/*
 * Final typedef
 */
typedef AppendEntries_ AppendEntries;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::protocol::dynamic_node_id::server::AppendEntries > _uavcan_gdtr_registrator_AppendEntries;

}

} // Namespace server
} // Namespace dynamic_node_id
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request >::stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::ParameterType obj, const int level)
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
    s << "term: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::FieldTypes::term >::stream(s, obj.term, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "prev_log_term: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::FieldTypes::prev_log_term >::stream(s, obj.prev_log_term, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "prev_log_index: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::FieldTypes::prev_log_index >::stream(s, obj.prev_log_index, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "leader_commit: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::FieldTypes::leader_commit >::stream(s, obj.leader_commit, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "entries: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::FieldTypes::entries >::stream(s, obj.entries, level + 1);
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response >::stream(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response::ParameterType obj, const int level)
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
    s << "term: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response::FieldTypes::term >::stream(s, obj.term, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "success: ";
    YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response::FieldTypes::success >::stream(s, obj.success, level + 1);
}

}

namespace uavcan
{
namespace protocol
{
namespace dynamic_node_id
{
namespace server
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::dynamic_node_id::server::AppendEntries::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace server
} // Namespace dynamic_node_id
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_APPENDENTRIES_HPP_INCLUDED