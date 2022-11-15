// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/rendering/depthbuffer_singleface.hpp>

#include <krakatoa/render_element.hpp>
#include <krakatoa/render_element_interface.hpp>

namespace krakatoa {

class occluded_layer_render_element : public render_element<render_element_interface> {
  public:
    occluded_layer_render_element();
    virtual ~occluded_layer_render_element();

    /**
     * see render_element_interface::clone()
     */
    virtual render_element_interface* clone();
};

} // namespace krakatoa
