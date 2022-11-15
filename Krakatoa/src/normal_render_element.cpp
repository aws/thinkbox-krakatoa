// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/normal_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

normal_render_element::normal_render_element( scene_context_ptr context, bool doAntialiasing, bool doScaling )
    : m_doAntialiasing( doAntialiasing )
    , m_context( context )
    , m_doScaling( doScaling ) {}

normal_render_element::~normal_render_element() {}

render_element_interface* normal_render_element::clone() {
    std::unique_ptr<normal_render_element> pResult(
        new normal_render_element( m_context, m_doAntialiasing, m_doScaling ) );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

// From particle_render_element_interface

normal_render_element::draw_type normal_render_element::get_drawing_type() const {
    if( m_doAntialiasing )
        return draw_type_antialias;
    else
        return draw_type_solid;
}

void normal_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::vector3f>( _T("Normal") );
}

void normal_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("Normal") ) )
        pcm.define_channel( _T("Normal"), 3, frantic::channels::data_type_float16 );
}

frantic::graphics::color3f normal_render_element::evaluate( const char* particle ) {
    frantic::graphics::vector3f camSpaceNorm = m_accessor.get( particle );
    camSpaceNorm = frantic::graphics::vector3f::normalize( camSpaceNorm );
    // camSpaceNorm = m_context->get_camera().world_transform().transpose_transform_no_translation( camSpaceNorm );

    if( m_doScaling ) {
        // Scale the normal into viewable range [0,1] from the range [-1,1].
        camSpaceNorm.x = frantic::math::clamp( camSpaceNorm.x * 0.5f + 0.5f, 0.f, 1.f );
        camSpaceNorm.y = frantic::math::clamp( camSpaceNorm.y * 0.5f + 0.5f, 0.f, 1.f );
        camSpaceNorm.z = frantic::math::clamp( camSpaceNorm.z * 0.5f + 0.5f, 0.f, 1.f );
    }

    return frantic::graphics::color3f( camSpaceNorm.x, camSpaceNorm.y, camSpaceNorm.z );
}

} // namespace krakatoa
