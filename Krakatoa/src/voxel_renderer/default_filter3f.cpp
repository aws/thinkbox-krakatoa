// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/voxel_renderer/default_filter3f.hpp>

namespace krakatoa {
namespace voxel_renderer {

default_filter3f::default_filter3f( float radius ) {
    m_radius = radius;
    m_intRadius = (int)ceilf( m_radius );
}

default_filter3f::~default_filter3f() {}

int default_filter3f::get_radius() const { return m_intRadius; }

void default_filter3f::get_weights( frantic::graphics::vector3f offset, float outWeights[] ) {
    float* weights = outWeights;
    float weightZ = m_radius - fabsf( offset.z );

    // Compensate to ensure total weight == 1.0
    // int(-r,r){r - |x-a|}dx = int(-r,0){r+x-a}dx  +  int(0,r){r-x+a}dx
    //                        = (rx+xx/2-ax)|-r,0   +  (rx-xx/2+ax)|0,r
    //                        = rr-rr/2-ar          +  rr-rr/2+ar
    //                        = rr
    float denom = m_radius * m_radius;
    denom *= denom * denom;

    for( int y = -m_intRadius + 1; y <= m_intRadius; ++y ) {
        float weightYZ = ( m_radius - fabsf( (float)y - offset.y ) ) * weightZ;

        for( int x = -m_intRadius + 1; x <= m_intRadius; ++x, ++weights ) {
            float weightXYZ = ( m_radius - fabsf( (float)x - offset.x ) ) * weightYZ;

            *weights = weightXYZ / denom;
        }
    }
}

} // namespace voxel_renderer
} // namespace krakatoa
