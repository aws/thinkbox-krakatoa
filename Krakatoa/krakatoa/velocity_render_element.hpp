// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/channels/channel_map.hpp>

#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/render_element.hpp>
#include <krakatoa/scene_context.hpp>

namespace krakatoa {

class velocity_render_element : public render_element<particle_render_element_interface> {
    scene_context_ptr m_context;
    frantic::graphics::transform4f m_worldToCameraDeriv;

    frantic::channels::channel_const_cvt_accessor<frantic::graphics::vector3f> m_accessor;

    bool m_doAntialiasing;
    bool m_applyMaxVelocity;
    float m_maxVelocity;
    float m_frameRate;

  public:
    velocity_render_element( scene_context_ptr context, bool doAntialiasing, float frameRate, bool applyMaxVelocity,
                             float maxVelocity );
    virtual ~velocity_render_element();

    /**
     * Sets a constant derivative for world-to-camera matrix.
     * This is sort of a hack to add camera motion to the render element.
     * The reason this can't be derived from the camera in the scene context is because a) we don't know the motion blur
     * duration, b) in cases of no motion blur, there is a zero motion blur duration, and no animation.
     */
    void set_world_to_camera_deriv( const frantic::graphics::transform4f& worldToCameraDeriv );

    /**
     * see render_element_interface::clone()
     */
    virtual render_element_interface* clone();

    /**
     * see particle_render_element_interface::get_drawing_type()
     */
    virtual draw_type get_drawing_type() const;

    /**
     * see particle_render_element_interface::set_channel_map()
     */
    virtual void set_channel_map( const frantic::channels::channel_map& pcm );

    /**
     * see particle_render_element_interface::add_required_channels()
     */
    virtual void add_required_channels( frantic::channels::channel_map& pcm );

    /**
     * see particle_render_element_interface::evaluate()
     */
    virtual frantic::graphics::color3f evaluate( const char* particle );
};

} // namespace krakatoa
