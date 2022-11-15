// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/shared_ptr.hpp>

#include <frantic/graphics2d/vector2.hpp>
#include <frantic/graphics2d/vector2f.hpp>
#include <frantic/strings/tstring.hpp>

namespace krakatoa {
namespace splat_renderer {

/**
 * Abstract interface for a two dimensional filter.
 */
class filter2f {
  public:
    typedef boost::shared_ptr<filter2f> ptr_type;

  public:
    static ptr_type create_instance( const frantic::tstring& name );

  public:
    virtual ~filter2f() {}

    virtual int get_width() const = 0;

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) = 0;
};

typedef filter2f::ptr_type filter2f_ptr;

} // namespace splat_renderer
} // namespace krakatoa
