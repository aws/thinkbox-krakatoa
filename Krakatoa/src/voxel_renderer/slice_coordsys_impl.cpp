// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/voxel_renderer/slice_coordsys_impl.hpp>

using frantic::graphics::transform4f;
using frantic::graphics::vector3f;

namespace krakatoa {
namespace voxel_renderer {

slice_coordsys_impl::slice_coordsys_impl() {}

slice_coordsys_impl::~slice_coordsys_impl() {}

int slice_coordsys_impl::reset( const frantic::graphics::transform4f& cameraToWorldTM, float voxelSize ) {
    m_isForward = true;
    m_currentSlice = 0;
    m_voxelSize = voxelSize;

    // We need to clobber any skew on the camera.
    vector3f t;
    transform4f p, r, s;
    cameraToWorldTM.decompose( p, t, r, s );

    m_sliceToWorldAxes[0] = r.get_column( 0 );
    m_sliceToWorldAxes[1] = r.get_column( 1 );
    m_sliceToWorldAxes[2] = r.get_column( 2 );

    /*m_sliceToWorldAxes[0] = cameraToWorldTM.get_column(0);
    m_sliceToWorldAxes[1] = cameraToWorldTM.get_column(1);
    m_sliceToWorldAxes[2] = cameraToWorldTM.get_column(2);*/

    return (int)std::ceil( vector3f::dot( m_sliceToWorldAxes[2], cameraToWorldTM.translation() ) / m_voxelSize );
}

std::pair<int, int> slice_coordsys_impl::reset( const frantic::graphics::transform4f& cameraToWorldTM,
                                                const frantic::graphics::transform4f& lightToWorldTM,
                                                float voxelSize ) {
    m_currentSlice = 0;
    m_voxelSize = voxelSize;

    // vector3f forward = lightToWorldTM.get_column(2) + cameraToWorldTM.get_column(2);
    // vector3f backward = lightToWorldTM.get_column(2) - cameraToWorldTM.get_column(2);

    int camSlice, lightSlice;
    if( vector3f::dot( cameraToWorldTM.get_column( 2 ), lightToWorldTM.get_column( 2 ) ) >= 0 ) {
        // if( vector3f::dot( cameraToWorldTM.get_column(2), forward ) * vector3f::dot( lightToWorldTM.get_column(2),
        // forward ) > vector3f::dot( cameraToWorldTM.get_column(2), backward ) * vector3f::dot(
        // lightToWorldTM.get_column(2), backward ) ){
        m_isForward = true;

        m_sliceToWorldAxes[2] = vector3f::normalize( lightToWorldTM.get_column( 2 ) + cameraToWorldTM.get_column( 2 ) );

        // int i = m_sliceToWorldAxes[2].get_largest_axis();
        // m_sliceToWorldAxes[2] = m_sliceToWorldAxes[2][i] >= 0 ? vector3f::from_axis( i ) : -vector3f::from_axis( i );

        m_sliceToWorldAxes[0] =
            vector3f::normalize( vector3f::cross( cameraToWorldTM.get_column( 1 ), m_sliceToWorldAxes[2] ) );
        m_sliceToWorldAxes[1] = vector3f::normalize( vector3f::cross( m_sliceToWorldAxes[2], m_sliceToWorldAxes[0] ) );

        camSlice =
            (int)std::ceil( vector3f::dot( m_sliceToWorldAxes[2], cameraToWorldTM.translation() ) / m_voxelSize );
        lightSlice =
            (int)std::ceil( vector3f::dot( m_sliceToWorldAxes[2], lightToWorldTM.translation() ) / m_voxelSize );
    } else {
        m_isForward = false;

        m_sliceToWorldAxes[2] = vector3f::normalize( lightToWorldTM.get_column( 2 ) - cameraToWorldTM.get_column( 2 ) );

        // int i = m_sliceToWorldAxes[2].get_largest_axis();
        // m_sliceToWorldAxes[2] = m_sliceToWorldAxes[2][i] >= 0 ? vector3f::from_axis( i ) : -vector3f::from_axis( i );

        m_sliceToWorldAxes[0] =
            vector3f::normalize( vector3f::cross( cameraToWorldTM.get_column( 1 ), m_sliceToWorldAxes[2] ) );
        m_sliceToWorldAxes[1] = vector3f::normalize( vector3f::cross( m_sliceToWorldAxes[2], m_sliceToWorldAxes[0] ) );

        camSlice =
            (int)std::floor( vector3f::dot( m_sliceToWorldAxes[2], cameraToWorldTM.translation() ) / m_voxelSize );
        lightSlice =
            (int)std::ceil( vector3f::dot( m_sliceToWorldAxes[2], lightToWorldTM.translation() ) / m_voxelSize );
    }

    return std::make_pair( camSlice, lightSlice );
}

bool slice_coordsys_impl::is_forward() const { return m_isForward; }

void slice_coordsys_impl::set_slice_index( int index ) { m_currentSlice = index; }

int slice_coordsys_impl::get_slice_index() const { return m_currentSlice; }

std::pair<float, float> slice_coordsys_impl::intersect_ray_with_slice( const frantic::graphics::ray3f& ray ) const {
    // This function was part of the critical path, so I have replaced the general concept with an optimized version.

    /*frantic::graphics::ray3f sliceRay( transform_from_world( ray.origin() ), transform_vector_from_world(
    ray.direction() ) );

    float tEnd = -sliceRay.origin().z / sliceRay.direction().z;
    float tStart = (1.f - sliceRay.origin().z) / sliceRay.direction().z;

    return std::make_pair( tStart, tEnd );*/

    float dotOrigin = vector3f::dot( m_sliceToWorldAxes[2], ray.origin() );
    float dotDirection = vector3f::dot( m_sliceToWorldAxes[2], ray.direction() );

    float tEnd = ( (float)m_currentSlice * m_voxelSize - dotOrigin ) / dotDirection;
    float tStart = tEnd + 1.f / dotDirection;

    return std::make_pair( tStart, tEnd );
}

vector3f slice_coordsys_impl::transform_from_world( frantic::graphics::vector3f p ) const {
    // return m_worldToSliceTM * p - vector3f(0, 0, m_currentSlice);
    return vector3f( vector3f::dot( m_sliceToWorldAxes[0], p ) / m_voxelSize,
                     vector3f::dot( m_sliceToWorldAxes[1], p ) / m_voxelSize,
                     vector3f::dot( m_sliceToWorldAxes[2], p ) / m_voxelSize - (float)m_currentSlice );
}

vector3f slice_coordsys_impl::transform_to_world( frantic::graphics::vector3f p ) const {
    // return m_sliceToWorldTM * (p + vector3f(0, 0, m_currentSlice));
    return ( p.x * m_voxelSize ) * m_sliceToWorldAxes[0] + ( p.y * m_voxelSize ) * m_sliceToWorldAxes[1] +
           ( ( (float)m_currentSlice + p.z ) * m_voxelSize ) * m_sliceToWorldAxes[2];
}

vector3f slice_coordsys_impl::transform_vector_from_world( frantic::graphics::vector3f p ) const {
    // return m_worldToSliceTM.transform_no_translation(p);
    return vector3f( vector3f::dot( m_sliceToWorldAxes[0], p ) / m_voxelSize,
                     vector3f::dot( m_sliceToWorldAxes[1], p ) / m_voxelSize,
                     vector3f::dot( m_sliceToWorldAxes[2], p ) / m_voxelSize );
}

vector3f slice_coordsys_impl::transform_vector_to_world( frantic::graphics::vector3f p ) const {
    // return m_sliceToWorldTM.transform_no_translation(p);
    return ( p.x * m_voxelSize ) * m_sliceToWorldAxes[0] + ( p.y * m_voxelSize ) * m_sliceToWorldAxes[1] +
           ( p.z * m_voxelSize ) * m_sliceToWorldAxes[2];
}

} // namespace voxel_renderer
} // namespace krakatoa
