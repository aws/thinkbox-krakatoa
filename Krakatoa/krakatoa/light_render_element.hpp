// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/channels/channel_map.hpp>

#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/render_element.hpp>
#include <krakatoa/scene_context.hpp>

namespace krakatoa {

class light_render_element : public render_element<particle_render_element_interface> {
    frantic::channels::channel_const_cvt_accessor<frantic::graphics::color3f> m_accessor;
    frantic::tstring m_lightName;
    frantic::tstring m_internalName;

  public:
    typedef boost::shared_ptr<light_render_element> ptr_type;

  public:
    light_render_element( const frantic::tstring& lightName, const frantic::tstring& internalName );
    virtual ~light_render_element();

    /**
     * Returns the name of the light that this render element is associated with.
     */
    const frantic::tstring& get_light_name() const;

    /**
     * Returns the name of the particle channel that this render element is associated with.
     */
    const frantic::tstring& get_channel_name() const;

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
};

typedef light_render_element::ptr_type light_render_element_ptr;

} // namespace krakatoa
