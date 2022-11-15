// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/diffuse_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

diffuse_render_element::diffuse_render_element() {}

diffuse_render_element::~diffuse_render_element() {}

render_element_interface* diffuse_render_element::clone() {
    std::unique_ptr<diffuse_render_element> pResult( new diffuse_render_element );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

// From particle_render_element_interface

diffuse_render_element::draw_type diffuse_render_element::get_drawing_type() const { return draw_type_shader; }

void diffuse_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Diffuse") );
}

void diffuse_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("__Element_Diffuse") ) )
        pcm.define_channel<frantic::graphics::color3f>( _T("__Element_Diffuse") );
}

frantic::graphics::color3f diffuse_render_element::evaluate( const char* particle ) {
    return m_accessor.get( particle );
}

} // namespace krakatoa
