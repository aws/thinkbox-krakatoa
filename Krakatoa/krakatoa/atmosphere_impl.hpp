// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/atmosphere_interface.hpp>

#include <frantic/geometry/volume_collection.hpp>

namespace krakatoa {

class atmosphere_impl : public atmosphere_interface {
    typedef frantic::geometry::volume_collection volume_type;
    typedef boost::shared_ptr<volume_type> volume_ptr;

    frantic::graphics::color3f m_extinction;
    volume_ptr m_volume;

  public:
    inline atmosphere_impl();

    inline virtual ~atmosphere_impl();

    inline void set_volume( volume_ptr pVol );

    inline void set_extinction( const frantic::graphics::color3f& extinction );

    inline virtual void apply_atomsphere( frantic::graphics::color3f& inoutLight,
                                          const frantic::graphics::vector3f& from,
                                          const frantic::graphics::vector3f& to ) const;
};

atmosphere_impl::atmosphere_impl() {}

atmosphere_impl::~atmosphere_impl() {}

void atmosphere_impl::set_volume( volume_ptr pVol ) { m_volume = pVol; }

void atmosphere_impl::set_extinction( const frantic::graphics::color3f& extinction ) { m_extinction = extinction; }

void atmosphere_impl::apply_atomsphere( frantic::graphics::color3f& inoutLight, const frantic::graphics::vector3f& from,
                                        const frantic::graphics::vector3f& to ) const {
    float distance;
    if( !m_volume ) {
        distance = frantic::graphics::vector3f::distance( from, to );
    } else {
        distance = m_volume->get_segment_distance_in_volume( from, to );
    }

    inoutLight.r *= std::exp( -( distance * m_extinction.r ) );
    inoutLight.g *= std::exp( -( distance * m_extinction.g ) );
    inoutLight.b *= std::exp( -( distance * m_extinction.b ) );
}

} // namespace krakatoa
