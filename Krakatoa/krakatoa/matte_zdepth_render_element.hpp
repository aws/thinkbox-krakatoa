// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/rendering/depthbuffer_singleface.hpp>

#include <krakatoa/render_element.hpp>
#include <krakatoa/render_element_interface.hpp>

namespace krakatoa {

class matte_zdepth_render_element : public render_element<render_element_interface> {
    frantic::rendering::depthbuffer_singleface m_depthBuffer;

    bool m_applyDepthRange;
    float m_minDepth;
    float m_maxDepth;

  public:
    matte_zdepth_render_element( bool applyDepthRange, float minDepth, float maxDepth );
    virtual ~matte_zdepth_render_element();

    /**
     * The depth buffer is better suited to matte_zdepth_render_element, so we provide this buffer for writing to
     * instead of the normal get_framebuffer() from render_element_interface.
     */
    virtual frantic::rendering::depthbuffer_singleface& get_depthbuffer();

    /**
     * Do not use!
     */
    virtual frantic::graphics2d::framebuffer<frantic::graphics::color6f>& get_framebuffer();

    /**
     * see render_element_interface::clone()
     */
    virtual render_element_interface* clone();

    /**
     * see render_element_interface::initialize()
     */
    virtual void initialize();

    /**
     * see render_element_interface::commit()
     */
    virtual void commit();
};

} // namespace krakatoa
