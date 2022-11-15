// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/velocity_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <memory>

namespace krakatoa {

velocity_render_element::velocity_render_element( scene_context_ptr context, bool doAntialiasing, float frameRate,
                                                  bool applyMaxVelocity, float maxVelocity )
    : m_doAntialiasing( doAntialiasing )
    , m_frameRate( frameRate )
    , m_applyMaxVelocity( applyMaxVelocity )
    , m_maxVelocity( maxVelocity )
    , m_context( context ) {
    m_worldToCameraDeriv.set_to_zero();
}

velocity_render_element::~velocity_render_element() {}

void velocity_render_element::set_world_to_camera_deriv( const frantic::graphics::transform4f& worldToCameraDeriv ) {
    m_worldToCameraDeriv = worldToCameraDeriv;
}

render_element_interface* velocity_render_element::clone() {
    std::unique_ptr<velocity_render_element> pResult(
        new velocity_render_element( m_context, m_doAntialiasing, m_frameRate, m_applyMaxVelocity, m_maxVelocity ) );
    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->m_accessor = m_accessor;

    return pResult.release();
}

// From particle_render_element_interface

velocity_render_element::draw_type velocity_render_element::get_drawing_type() const {
    if( m_doAntialiasing )
        return draw_type_antialias;
    else
        return draw_type_solid;
}

void velocity_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_accessor = pcm.get_const_cvt_accessor<frantic::graphics::vector3f>( _T("Velocity") );
}

void velocity_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    if( !pcm.has_channel( _T("Velocity") ) )
        pcm.define_channel<frantic::graphics::color3f>( _T("Velocity") );
}

frantic::graphics::color3f velocity_render_element::evaluate( const char* particle ) {
    frantic::graphics::vector3f camSpaceVel =
        m_context->get_camera().world_transform_inverse().transform_no_translation( m_accessor.get( particle ) );
    camSpaceVel += m_worldToCameraDeriv * camSpaceVel;

    // Convert from units/second to units/frame
    camSpaceVel /= m_frameRate;

    // If we have a max velocity assigned we can adjust the velocity into a visible color range ie. [0,1].
    if( m_applyMaxVelocity ) {
        float maxVelocityRange = 2.f * m_maxVelocity;
        camSpaceVel.x = frantic::math::clamp( camSpaceVel.x / maxVelocityRange + 0.5f, 0.f, 1.f );
        camSpaceVel.y = frantic::math::clamp( camSpaceVel.y / maxVelocityRange + 0.5f, 0.f, 1.f );
        camSpaceVel.z = frantic::math::clamp( camSpaceVel.z / maxVelocityRange + 0.5f, 0.f, 1.f );
    }

    return frantic::graphics::color3f( camSpaceVel.x, camSpaceVel.y, camSpaceVel.z );
}

} // namespace krakatoa
