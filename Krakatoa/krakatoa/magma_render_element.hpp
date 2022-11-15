// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/**
 * This file is obsolete. Do not use anything from here. I realized later too that putting a magma element here doesn't
 * make a ton of sense due to the extra dependency, and that most host applications (ie. MaxKrakatoa) will have custom
 * Magma stuff that krakatoa base can't know about.
 */

#pragma once

#include <frantic/channels/channel_map.hpp>
#include <frantic/channels/channel_operation_compiler.hpp>

#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/render_element.hpp>

namespace krakatoa {

class magma_render_element : public render_element<particle_render_element_interface> {
    frantic::channels::channel_map m_outputMap;
    frantic::channels::channel_const_cvt_accessor<frantic::graphics::color3f> m_accessor;

    std::vector<frantic::channels::channel_op_node*> m_exprTree;
    boost::shared_ptr<frantic::channels::channel_operation_compiler> m_pCompiledExpr;

    bool m_doAntialias;

  private:
    magma_render_element( bool doAntialias );

  public:
    magma_render_element( bool doAntialias, const std::vector<frantic::channels::channel_op_node*>& exprTree );
    virtual ~magma_render_element();

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
