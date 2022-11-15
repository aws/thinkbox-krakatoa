// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/occluded_layer_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

occluded_layer_render_element::occluded_layer_render_element() {}

occluded_layer_render_element::~occluded_layer_render_element() {}

render_element_interface* occluded_layer_render_element::clone() {
    std::unique_ptr<occluded_layer_render_element> pResult( new occluded_layer_render_element );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );

    return pResult.release();
}

} // namespace krakatoa
