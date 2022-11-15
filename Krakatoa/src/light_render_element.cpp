// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/light_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

light_render_element::light_render_element( const frantic::tstring& lightName, const frantic::tstring& internalName )
    : m_lightName( lightName )
    , m_internalName( _T("__Element_") + internalName ) {}

light_render_element::~light_render_element() {}

const frantic::tstring& light_render_element::get_light_name() const { return m_lightName; }

const frantic::tstring& light_render_element::get_channel_name() const { return m_internalName; }

render_element_interface* light_render_element::clone() {
    std::unique_ptr<light_render_element> pResult( new light_render_element( m_lightName, m_internalName ) );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

void light_render_element::commit() { render_element<particle_render_element_interface>::commit(); }

// From particle_render_element_interface

light_render_element::draw_type light_render_element::get_drawing_type() const { return draw_type_shader; }

void light_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::color3f>( m_internalName );
}

void light_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( m_internalName ) )
        pcm.define_channel( m_internalName, 3, frantic::channels::data_type_float16 );
}

frantic::graphics::color3f light_render_element::evaluate( const char* particle ) { return m_accessor.get( particle ); }

} // namespace krakatoa
