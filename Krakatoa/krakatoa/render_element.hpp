// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/render_element_interface.hpp>

#include <frantic/graphics/color_with_alpha.hpp>
#include <frantic/graphics2d/framebuffer.hpp>

#include <boost/function.hpp>

/**
 * @note Make sure to #include <krakatoa/render_element_impl.hpp> after including this file. If you
 *       are getting linker errors in this file, it likely means you forgot to include render_element_impl.hpp in the
 * .cpp of client classes.
 */

#pragma warning( push )
#pragma warning( disable : 4505 )

namespace krakatoa {

template <class BaseClass>
class render_element : public BaseClass {
    frantic::graphics2d::framebuffer<frantic::graphics::color6f> m_framebuffer;
    std::vector<boost::function<void( render_element_interface& )>> m_callbackFunctions;

  public:
    virtual ~render_element();

    /**
     * Access to the render element's framebuffer
     */
    virtual frantic::graphics2d::framebuffer<frantic::graphics::color6f>& get_framebuffer();

    /**
     * Use this to register a callback method when this render element is commited
     */
    virtual void register_commit_callback( const boost::function<void( render_element_interface& )>& callbackFn );

    /**
     * Called exactly once on a render element after all properties have been set, but before the first drawing to
     * its framebuffer
     */
    virtual void initialize();

    /**
     * Called to combine the results of cloned elements backed together. To be called in the same order the elements
     * were cloned from the original.
     */
    virtual void combine( render_element_interface* rhs );

    /**
     * Called exactly once on a render element after all drawing to its framebuffer has completed.
     */
    virtual void commit();

    /**
     * When compositing multiple images (eg. for motion blur), the minimum value should be used.
     * This is required for compositing z-depth.
     */
    virtual inline bool use_minimize_compositing() { return false; }
};

} // namespace krakatoa

#pragma warning( pop )
