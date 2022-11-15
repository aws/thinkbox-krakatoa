// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/tangent_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

tangent_render_element::tangent_render_element( scene_context_ptr context, bool doAntialiasing )
    : m_doAntialiasing( doAntialiasing )
    , m_context( context ) {}

tangent_render_element::~tangent_render_element() {}

render_element_interface* tangent_render_element::clone() {
    std::unique_ptr<tangent_render_element> pResult( new tangent_render_element( m_context, m_doAntialiasing ) );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

// From particle_render_element_interface

tangent_render_element::draw_type tangent_render_element::get_drawing_type() const {
    if( m_doAntialiasing )
        return draw_type_antialias;
    else
        return draw_type_solid;
}

void tangent_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::vector3f>( _T("Tangent") );
}

void tangent_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("Tangent") ) )
        pcm.define_channel<frantic::graphics::color3f>( _T("Tangent") );
}

frantic::graphics::color3f tangent_render_element::evaluate( const char* particle ) {
    frantic::graphics::vector3f camSpaceTangent = m_accessor.get( particle );
    camSpaceTangent = frantic::graphics::vector3f::normalize( camSpaceTangent );
    camSpaceTangent = m_context->get_camera().world_transform().transpose_transform_no_translation( camSpaceTangent );

    // Scale the normal into viewable range [0,1] from the range [-1,1].
    camSpaceTangent.x = frantic::math::clamp( camSpaceTangent.x * 0.5f + 0.5f, 0.f, 1.f );
    camSpaceTangent.y = frantic::math::clamp( camSpaceTangent.y * 0.5f + 0.5f, 0.f, 1.f );
    camSpaceTangent.z = frantic::math::clamp( camSpaceTangent.z * 0.5f + 0.5f, 0.f, 1.f );

    return frantic::graphics::color3f( camSpaceTangent.x, camSpaceTangent.y, camSpaceTangent.z );
}

} // namespace krakatoa
