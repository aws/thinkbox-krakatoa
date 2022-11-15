// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/render_element.hpp>

namespace krakatoa {

class channel_render_element : public render_element<particle_render_element_interface> {
    frantic::tstring m_channelName;
    frantic::channels::data_type_t m_type;
    std::size_t m_arity;
    bool m_antiAlias;
    frantic::channels::channel_const_cvt_accessor<frantic::graphics::color3f> m_accessor;
    frantic::graphics::color3f m_default;
    frantic::channels::channel_map m_outputMap;

  public:
    channel_render_element( bool antiAlias, const frantic::tstring& m_channelName, frantic::channels::data_type_t type,
                            std::size_t arity, const frantic::graphics::color3f& def );

    virtual ~channel_render_element();

    virtual render_element_interface* clone();

    virtual draw_type get_drawing_type() const;

    virtual void set_channel_map( const frantic::channels::channel_map& pcm );

    virtual void add_required_channels( frantic::channels::channel_map& pcm );

    virtual frantic::graphics::color3f evaluate( const char* particle );
};
} // namespace krakatoa