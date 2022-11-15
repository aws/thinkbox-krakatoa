// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/voxel_renderer/filter3f.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * A default three dimensional filter that applies a tensor product of three tent filters of the specified size.
 */
class default_filter3f : public filter3f {
    int m_intRadius;
    float m_radius;

  public:
    default_filter3f( float radius );

    virtual ~default_filter3f();

    virtual int get_radius() const;

    virtual void get_weights( frantic::graphics::vector3f offset, float outWeights[] );
};

} // namespace voxel_renderer
} // namespace krakatoa
