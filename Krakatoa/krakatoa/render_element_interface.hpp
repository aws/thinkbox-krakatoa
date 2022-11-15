// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>

#include <frantic/graphics/color_with_alpha.hpp>
#include <frantic/graphics2d/framebuffer.hpp>

namespace krakatoa {

class render_element_interface {
  public:
    typedef boost::shared_ptr<render_element_interface> ptr_type;

  public:
    virtual ~render_element_interface() {}

    /**
     * Access to the render element's framebuffer
     */
    virtual frantic::graphics2d::framebuffer<frantic::graphics::color6f>& get_framebuffer() = 0;

    /**
     * Use this to register a callback method when this render element is commited
     */
    virtual void register_commit_callback( const boost::function<void( render_element_interface& )>& callbackFn ) = 0;

    /**
     * Clones this render element. This can be used to have multiple render elements with the same function drawing
     * to different framebuffers. This is useful for achieving multipass effects or splitting processing of elements
     * into threads.
     */
    virtual render_element_interface* clone() = 0;

    /**
     * Called exactly once on a render element after all properties have been set, but before the first drawing to
     * its framebuffer
     */
    virtual void initialize() = 0;

    /**
     * Called to combine the results of cloned elements backed together. To be called in the same order the elements
     * were cloned from the original.
     */
    virtual void combine( render_element_interface* rhs ) = 0;

    /**
     * Called exactly once when the render element is finished rendering. Fires callbacks registered with
     * register_commit_callback()
     */
    virtual void commit() = 0;

    /**
     * When compositing multiple images (eg. for motion blur), the minimum value should be used.
     * This is required for compositing z-depth.
     */
    virtual bool use_minimize_compositing() = 0;
};

typedef render_element_interface::ptr_type render_element_interface_ptr;

} // namespace krakatoa
