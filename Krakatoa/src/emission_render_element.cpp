// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/emission_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

emission_render_element::emission_render_element() {}

emission_render_element::~emission_render_element() {}

render_element_interface* emission_render_element::clone() {
    std::unique_ptr<emission_render_element> pResult( new emission_render_element );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

// From particle_render_element_interface

emission_render_element::draw_type emission_render_element::get_drawing_type() const { return draw_type_normal; }

void emission_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::color3f>( _T("Emission") );
}

void emission_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("Emission") ) )
        pcm.define_channel<frantic::graphics::color3f>( _T("Emission") );
}

frantic::graphics::color3f emission_render_element::evaluate( const char* particle ) {
    return m_accessor.get( particle );
}

} // namespace krakatoa
