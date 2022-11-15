// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/voxel_renderer/default_radial_filter3f.hpp>

namespace krakatoa {
namespace voxel_renderer {

default_radial_filter3f::default_radial_filter3f( float radius ) {
    m_radius = radius;
    m_intRadius = (int)ceilf( m_radius );
}

default_radial_filter3f::~default_radial_filter3f() {}

int default_radial_filter3f::get_radius() const { return m_intRadius; }

void default_radial_filter3f::get_weights( frantic::graphics::vector3f offset, float outWeights[] ) {
    float* weights = outWeights;

    float distZ2 = offset.z * offset.z;
    float denom = m_radius * m_radius;

    for( int y = -m_intRadius + 1; y <= m_intRadius; ++y ) {

        float distY = (float)y - offset.y;
        float distY2 = distY * distY;

        for( int x = -m_intRadius + 1; x <= m_intRadius; ++x, ++weights ) {

            float distX = (float)x - offset.x;
            float distX2 = distX * distX;

            float d = m_radius - sqrtf( distX2 + distY2 + distZ2 );
            if( d > 0 )
                *weights = d / denom;
            else
                *weights = 0;
        }
    }
}

} // namespace voxel_renderer
} // namespace krakatoa
