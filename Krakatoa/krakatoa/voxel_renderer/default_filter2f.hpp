// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/voxel_renderer/filter2f.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * A default two dimensional filter that applies a tensor product of two tent filters of the specified size.
 */
class default_filter2f : public filter2f {
  public:
    default_filter2f();

    virtual ~default_filter2f();

    virtual int get_radius() const;

    virtual void do_filter( frantic::graphics2d::vector2f offset, float outWeights[] );
};

} // namespace voxel_renderer
} // namespace krakatoa
