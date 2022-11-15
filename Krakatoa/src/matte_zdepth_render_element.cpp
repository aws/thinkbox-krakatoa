// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/matte_zdepth_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

matte_zdepth_render_element::matte_zdepth_render_element( bool applyDepthRange, float minDepth, float maxDepth )
    : m_applyDepthRange( applyDepthRange )
    , m_minDepth( minDepth )
    , m_maxDepth( maxDepth ) {}

matte_zdepth_render_element::~matte_zdepth_render_element() {}

frantic::rendering::depthbuffer_singleface& matte_zdepth_render_element::get_depthbuffer() { return m_depthBuffer; }

frantic::graphics2d::framebuffer<frantic::graphics::color6f>& matte_zdepth_render_element::get_framebuffer() {
    throw std::runtime_error( "MatteZDepth render elements do not have a color6f framebuffer" );
}

render_element_interface* matte_zdepth_render_element::clone() {
    std::unique_ptr<matte_zdepth_render_element> pResult(
        new matte_zdepth_render_element( m_applyDepthRange, m_minDepth, m_maxDepth ) );
    pResult->get_depthbuffer().set_size( get_depthbuffer().size() );

    return pResult.release();
}

void matte_zdepth_render_element::initialize() { get_depthbuffer().clear(); }

void matte_zdepth_render_element::commit() {
    if( m_applyDepthRange )
        get_depthbuffer().normalize_values( m_minDepth, m_maxDepth );

    render_element<render_element_interface>::commit();
}

} // namespace krakatoa
