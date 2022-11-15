// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/vector3f.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * Abstract interface for a three dimensional filter.
 */
class filter3f {
  public:
    virtual ~filter3f() {}

    virtual int get_radius() const = 0;

    virtual void get_weights( frantic::graphics::vector3f offset, float outWeights[] ) = 0;
};

} // namespace voxel_renderer
} // namespace krakatoa
