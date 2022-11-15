// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/channel_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

using namespace frantic::graphics;
using namespace krakatoa;

channel_render_element::channel_render_element( bool antiAlias, const frantic::tstring& channelName,
                                                frantic::channels::data_type_t type, std::size_t arity,
                                                const frantic::graphics::color3f& def )
    : m_antiAlias( antiAlias )
    , m_channelName( channelName )
    , m_type( type )
    , m_arity( arity )
    , m_default( def ) {}

channel_render_element::~channel_render_element() {}

render_element_interface* channel_render_element::clone() {
    std::unique_ptr<channel_render_element> pResult(
        new channel_render_element( m_antiAlias, m_channelName, m_type, m_arity, m_default ) );

    pResult->m_accessor = m_accessor;
    pResult->m_outputMap = m_outputMap;

    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->get_framebuffer().fill( frantic::graphics::color6f( 0.f ) );

    return pResult.release();
}

krakatoa::particle_render_element_interface::draw_type channel_render_element::get_drawing_type() const {
    if( m_antiAlias ) {
        return draw_type_antialias;
    } else {
        return draw_type_solid;
    }
}

void channel_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_outputMap = pcm;
    if( m_outputMap.has_channel( m_channelName ) ) {
        m_accessor = pcm.get_const_cvt_accessor<color3f>( m_channelName );
    } else {
        m_accessor = frantic::channels::channel_const_cvt_accessor<color3f>( m_default );
    }
}

void channel_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( m_channelName ) ) {
        pcm.define_channel( m_channelName, m_arity, m_type );
    }
}

frantic::graphics::color3f channel_render_element::evaluate( const char* particle ) {
    return m_accessor.get( particle );
}