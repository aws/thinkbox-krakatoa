// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/color_with_alpha.hpp>

namespace krakatoa {

struct min_color6f {
    static inline float max_if_zero( float in ) { return in <= 0.f ? std::numeric_limits<float>::max() : in; }

    frantic::graphics::color6f operator()( const frantic::graphics::color6f& original,
                                           const frantic::graphics::color6f& other ) const {
        return frantic::graphics::color6f(
            frantic::graphics::color3f( std::min( max_if_zero( original.c.r ), max_if_zero( other.c.r ) ),
                                        std::min( max_if_zero( original.c.g ), max_if_zero( other.c.g ) ),
                                        std::min( max_if_zero( original.c.b ), max_if_zero( other.c.b ) ) ),
            frantic::graphics::alpha3f( std::min( max_if_zero( original.a.ar ), max_if_zero( other.a.ar ) ),
                                        std::min( max_if_zero( original.a.ag ), max_if_zero( other.a.ag ) ),
                                        std::min( max_if_zero( original.a.ab ), max_if_zero( other.a.ab ) ) ) );
    }
};

} // namespace krakatoa
