// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/render_element_interface.hpp>

#include <frantic/channels/channel_map.hpp>
#include <frantic/graphics/camera.hpp>

namespace krakatoa {

class particle_render_element_interface : public render_element_interface {
  public:
    typedef boost::shared_ptr<particle_render_element_interface> ptr_type;

  public:
    enum draw_type {
        // Draw each particle as if the output of the render element was the Emission channel. The particle's Color and
        // Absorption
        // contribute to the alpha, as well as affecting the final drawing color.
        draw_type_normal,

        // Draw each particle exactly like draw_type_normal, except don't divide out the density. Use this for light
        // reflecting off a particle.k
        draw_type_shader,

        // Draw each particle as an antialiased splat. The particle's Color and Absorption are ignored and alpha is
        // [1,1,1] before filtering.
        draw_type_antialias,

        // Draw each particle as a solid rectangle covering the specified filter radius. Each pixel in the 1x2, 2x2 or
        // 3x3 box gets alpha [1,1,1].
        draw_type_solid
    };

  public:
    virtual ~particle_render_element_interface() {}

    virtual draw_type get_drawing_type() const = 0;

    /**
     * Called exactly after the layput of particles has been determined. All particles passed to evaluate()
     * will have the layout described by 'pcm'.
     * @note this MUST be called after set_camera() since it may require information from the camera.
     */
    virtual void set_channel_map( const frantic::channels::channel_map& pcm ) = 0;

    /**
     * Called to add channels required by this render element to the channel map. This will be called before
     * set_channel_map() is called with the completed channel map.
     */
    virtual void add_required_channels( frantic::channels::channel_map& pcm ) = 0;

    /**
     * Called on each particle to determine the color it should contribute to this render element.
     */
    virtual frantic::graphics::color3f evaluate( const char* particle ) = 0;
};

typedef particle_render_element_interface::ptr_type particle_render_element_interface_ptr;

} // namespace krakatoa
