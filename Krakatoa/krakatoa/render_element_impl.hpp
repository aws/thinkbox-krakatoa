// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace krakatoa {

template <class BaseClass>
render_element<BaseClass>::~render_element() {}

template <class BaseClass>
frantic::graphics2d::framebuffer<frantic::graphics::color6f>& render_element<BaseClass>::get_framebuffer() {
    return m_framebuffer;
}

template <class BaseClass>
void render_element<BaseClass>::register_commit_callback(
    const boost::function<void( render_element_interface& )>& callbackFn ) {
    m_callbackFunctions.push_back( callbackFn );
}

template <class BaseClass>
void render_element<BaseClass>::initialize() {}

template <class BaseClass>
void render_element<BaseClass>::combine( render_element_interface* rhs ) {
    get_framebuffer().blend_over( rhs->get_framebuffer() );
}

template <class BaseClass>
void render_element<BaseClass>::commit() {
    // Eval them in reverse order so that it behaves like a LiFo queue.
    std::vector<boost::function<void( render_element_interface& )>>::reverse_iterator it = m_callbackFunctions.rbegin(),
                                                                                      itEnd =
                                                                                          m_callbackFunctions.rend();
    for( ; it != itEnd; ++it )
        ( *it )( *this );
}

} // namespace krakatoa
