// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics2d/vector2f.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * Abstract interface for a two dimensional filter.
 */
class filter2f {
  public:
    virtual ~filter2f() {}

    virtual int get_radius() const = 0;

    virtual void do_filter( frantic::graphics2d::vector2f offset, float outWeights[] ) = 0;
};

} // namespace voxel_renderer
} // namespace krakatoa
