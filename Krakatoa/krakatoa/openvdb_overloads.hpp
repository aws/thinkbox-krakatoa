// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#if defined( OPENVDB_AVAILABLE )

#include <frantic/graphics/color3f.hpp>

#include <boost/tuple/tuple.hpp>

#include <openvdb/version.h>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace math {

inline boost::tuple<float, frantic::graphics::color3f> Abs( const boost::tuple<float, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f>( std::abs( t.get<0>() ), t.get<1>() );
}

inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
Abs( const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>( std::abs( t.get<0>() ),
                                                                                        t.get<1>(), t.get<2>() );
}

inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f> Abs(
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>(
        std::abs( t.get<0>() ), t.get<1>(), t.get<2>(), t.get<3>() );
}

} // namespace math
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

// OpenVDB uses these operators in their prune operation, where they try to determine if a value is close enough to the
// zeroValue.

namespace boost {
namespace tuples {
// Argument-dependent name lookup (I think?) makes it necessary to define these in the boost::tuples namespace

inline boost::tuple<float, frantic::graphics::color3f>
operator+( const boost::tuple<float, frantic::graphics::color3f>& s,
           const boost::tuple<float, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f>( s.get<0>() + t.get<0>(), s.get<1>() + t.get<1>() );
}
inline boost::tuple<float, frantic::graphics::color3f>
operator-( const boost::tuple<float, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f>( -t.get<0>(), t.get<1>() );
}
inline boost::tuple<float, frantic::graphics::color3f>
operator-( const boost::tuple<float, frantic::graphics::color3f>& s,
           const boost::tuple<float, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f>( s.get<0>() - t.get<0>(), s.get<1>() - t.get<1>() );
}
inline boost::tuple<float, frantic::graphics::color3f>
operator*( float scale, const boost::tuple<float, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f>( scale * t.get<0>(), scale * t.get<1>() );
}
inline boost::tuple<float, frantic::graphics::color3f>
operator*( const boost::tuple<float, frantic::graphics::color3f>& t, float scale ) {
    return scale * t;
}

inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
operator+( const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& s,
           const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>(
        s.get<0>() + t.get<0>(), s.get<1>() + t.get<1>(), s.get<2>() + t.get<2>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
operator-( const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>( -t.get<0>(), t.get<1>(),
                                                                                        t.get<2>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
operator-( const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& s,
           const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>(
        s.get<0>() - t.get<0>(), s.get<1>() - t.get<1>(), s.get<2>() - t.get<2>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
operator*( float scale, const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>(
        scale * t.get<0>(), scale * t.get<1>(), scale * t.get<2>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>
operator*( const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f>& t, float scale ) {
    return scale * t;
}

inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>
operator+(
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& s,
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>(
        s.get<0>() + t.get<0>(), s.get<1>() + t.get<1>(), s.get<2>() + t.get<2>(), s.get<3>() + t.get<3>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>
operator-(
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>(
        -t.get<0>(), t.get<1>(), t.get<2>(), t.get<3>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>
operator-(
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& s,
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>(
        s.get<0>() - t.get<0>(), s.get<1>() - t.get<1>(), s.get<2>() - t.get<2>(), s.get<3>() - t.get<3>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>
operator*(
    float scale,
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t ) {
    return boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>(
        scale * t.get<0>(), scale * t.get<1>(), scale * t.get<2>(), scale * t.get<3>() );
}
inline boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>
operator*(
    const boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f, frantic::graphics::color3f>& t,
    float scale ) {
    return scale * t;
}

} // namespace tuples
} // namespace boost

namespace frantic {
namespace graphics {

inline bool operator<( const frantic::graphics::color3f& lhs, const frantic::graphics::color3f& rhs ) {
    if( lhs.r != rhs.r )
        return lhs.r < rhs.r;
    else if( lhs.g != rhs.g )
        return lhs.g < rhs.g;
    else
        return lhs.b < rhs.b;
}

inline bool operator>( const frantic::graphics::color3f& lhs, const frantic::graphics::color3f& rhs ) {
    return operator<( rhs, lhs );
}

} // namespace graphics
} // namespace frantic

#endif