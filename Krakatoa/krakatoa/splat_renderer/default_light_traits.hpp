// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/splat_renderer/filter2f.hpp>

#include <frantic/rendering/deep_attenuation_savers.hpp>
#include <frantic/rendering/depthbuffer_singleface.hpp>
#include <frantic/rendering/lights/directedlight_interface.hpp>

#include <frantic/graphics2d/framebuffer.hpp>

#include <cmath>

extern int g_filterRadius;

namespace krakatoa {
namespace splat_renderer {

class default_light_traits {
  public:
    typedef frantic::graphics2d::vector2f screen_space_type;
    typedef frantic::graphics2d::framebuffer<frantic::graphics::alpha3f> attenuation_buffer_type;
    typedef frantic::rendering::depthbuffer_singleface shadow_map_type;
    typedef frantic::graphics::alpha3f pixel_type;

    inline static bool get_screen_pos( const frantic::rendering::lights::lightinterface& light,
                                       const frantic::graphics::vector3f& lightSpacePos,
                                       screen_space_type& outScreenSpacePos ) {
        bool isValid = true;
        outScreenSpacePos = static_cast<const frantic::rendering::lights::directedlight_interface&>( light )
                                .get_camera()
                                .from_cameraspace_position( lightSpacePos, isValid );

        return isValid;
    }

    inline static float get_density_scale( const frantic::rendering::lights::lightinterface& light,
                                           const frantic::graphics::vector3f& lightSpacePos ) {
        return light.shadow_density() * static_cast<const frantic::rendering::lights::directedlight_interface&>( light )
                                            .get_camera()
                                            .area_differential_from_cameraspace_position( lightSpacePos );
    }

    inline static void draw_attenuation( const frantic::graphics::alpha3f& attenSample,
                                         const screen_space_type& screenPos, attenuation_buffer_type& buffer ) {
        // TODO: Replace with a filter2f
        buffer.draw_point( screenPos, attenSample );
    }

    inline static void draw_attenuation( const frantic::graphics::alpha3f& attenSample,
                                         const screen_space_type& screenPos, attenuation_buffer_type& buffer,
                                         const filter2f_ptr& filter ) {
        // TODO: Replace with a filter2f
        // buffer.draw_point( screenPos, attenSample );

        int width = filter->get_width();
        int size = width * width;

        float* weights = (float*)alloca( sizeof( float ) * size );

        frantic::graphics2d::vector2 pixelPos;
        filter->do_filter( screenPos, pixelPos, weights );

        for( int y = 0; y < width; ++y, weights += width ) {
            int py = pixelPos.y + y;
            if( (unsigned)py < (unsigned)buffer.height() ) {
                for( int x = 0; x < width; ++x ) {
                    int px = pixelPos.x + x;
                    if( (unsigned)px < (unsigned)buffer.width() )
                        buffer.blend_under( px, py, weights[x] * attenSample );
                }
            }
        }
    }

    inline static void add_deep_attenuation_sample( const frantic::graphics::alpha3f& attenSample,
                                                    const screen_space_type& screenPos, float z,
                                                    boost::shared_ptr<frantic::rendering::atten_saver> attenSaver ) {
        // Replace with a filter2f?
        if( attenSaver && !attenSaver->is_cubeface() ) {
            frantic::rendering::singleface_atten_saver* singlefaceAttenSaver =
                static_cast<frantic::rendering::singleface_atten_saver*>( attenSaver.get() );
            singlefaceAttenSaver->add_sample_bilinear( screenPos.x, screenPos.y, z, attenSample );
        }
    }
};

} // namespace splat_renderer
} // namespace krakatoa
