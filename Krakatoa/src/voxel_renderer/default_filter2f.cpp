// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/voxel_renderer/default_filter2f.hpp>

namespace krakatoa {
namespace voxel_renderer {

default_filter2f::default_filter2f() {}

default_filter2f::~default_filter2f() {}

int default_filter2f::get_radius() const { return 1; }

void default_filter2f::do_filter( frantic::graphics2d::vector2f offset, float outWeights[] ) {
    float prod = offset.x * offset.y;

    outWeights[0] = 1.f - offset.x - offset.y + prod;
    outWeights[1] = offset.x - prod;
    outWeights[2] = offset.y - prod;
    outWeights[3] = prod;

    /*
    outWeights[0] = (1.f - offset.x) * (1.f - offset.y);
    outWeights[1] = offset.x * (1.f - offset.y);
    outWeights[2] = (1.f - offset.x) * offset.y;
    outWeights[3] = offset.x * offset.y;
    */
}

} // namespace voxel_renderer
} // namespace krakatoa
