// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/zdepth_render_element.hpp>

#include <krakatoa/min_color6f.hpp>
#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

zdepth_render_element::zdepth_render_element( scene_context_ptr context, bool doAntialiasing, bool applyDepthRange,
                                              float minDepth, float maxDepth )
    : m_doAntialiasing( doAntialiasing )
    , m_applyDepthRange( applyDepthRange )
    , m_minDepth( minDepth )
    , m_maxDepth( maxDepth )
    , m_context( context ) {}

zdepth_render_element::~zdepth_render_element() {}

render_element_interface* zdepth_render_element::clone() {
    std::unique_ptr<zdepth_render_element> pResult(
        new zdepth_render_element( m_context, m_doAntialiasing, m_applyDepthRange, m_minDepth, m_maxDepth ) );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

void zdepth_render_element::commit() {
    if( !m_applyDepthRange ) {
        const float epsilon = 0.000001f;
        const float oneMinusEpsilon = 1.0f - epsilon;

        // This is a work-around for a numeric instability issues when trying to apply float::max to alpha blending
        // It simply sets any non-opaque pixels' depth to float::max, rather than trying to blend the values together
        for( size_t i = 0; i < get_framebuffer().data().size(); ++i ) {
            frantic::graphics::color6f currentColor = get_framebuffer().data()[i];
            if( currentColor.alpha().ar < oneMinusEpsilon || currentColor.alpha().ag < oneMinusEpsilon ||
                currentColor.alpha().ab < oneMinusEpsilon ) {
                get_framebuffer().data()[i].color() = frantic::graphics::color3f( std::numeric_limits<float>::max() );
            }
        }
    } else {
        get_framebuffer().fill_under( frantic::graphics::color6f( 1.f ) );
    }

    render_element<particle_render_element_interface>::commit();
}

// From particle_render_element_interface

zdepth_render_element::draw_type zdepth_render_element::get_drawing_type() const {
    if( m_doAntialiasing )
        return draw_type_antialias;
    else
        return draw_type_solid;
}

void zdepth_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::vector3f>( _T("Position") );
}

void zdepth_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("Position") ) )
        pcm.define_channel<frantic::graphics::color3f>( _T("Position") );
}

frantic::graphics::color3f zdepth_render_element::evaluate( const char* particle ) {
    float zDepth = -( ( m_context->get_camera().world_transform_inverse() * m_accessor.get( particle ) ).z );

    if( m_applyDepthRange ) {
        if( zDepth < m_minDepth )
            zDepth = m_minDepth;
        else if( zDepth > m_maxDepth )
            zDepth = m_maxDepth;

        zDepth = ( zDepth - m_minDepth ) / ( m_maxDepth - m_minDepth );
    }

    return frantic::graphics::color3f( zDepth );
}

void zdepth_render_element::combine( render_element_interface* rhs ) {
    get_framebuffer().apply_function( rhs->get_framebuffer(), min_color6f() );
}

} // namespace krakatoa
