// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/channels/channel_map.hpp>

#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/render_element.hpp>
#include <krakatoa/scene_context.hpp>

namespace krakatoa {

class zdepth_render_element : public render_element<particle_render_element_interface> {
    scene_context_ptr m_context;

    frantic::channels::channel_const_cvt_accessor<frantic::graphics::vector3f> m_accessor;

    bool m_doAntialiasing;
    bool m_applyDepthRange;
    float m_minDepth;
    float m_maxDepth;

  public:
    zdepth_render_element( scene_context_ptr context, bool doAntialiasing, bool applyDepthRange, float minDepth,
                           float maxDepth );
    virtual ~zdepth_render_element();

    /**
     * see render_element_interface::clone()
     */
    virtual render_element_interface* clone();

    /**
     * see render_element_interface::commit()
     */
    virtual void commit();

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

    /**
     * When compositing multiple images (eg. for motion blur), the minimum value should be used.
     * This is required for compositing z-depth.
     */
    virtual inline bool use_minimize_compositing() { return true; }

    virtual void combine( render_element_interface* rhs );
};

} // namespace krakatoa
