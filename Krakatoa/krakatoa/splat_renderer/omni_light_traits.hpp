// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

//#include "LightingEngineBase.hpp"

#include <frantic/rendering/deep_attenuation_savers.hpp>
#include <frantic/rendering/depthbuffer_cubeface.hpp>
#include <frantic/rendering/framebuffer_cubeface.hpp>

namespace krakatoa {
namespace splat_renderer {

class omni_light_traits {
  public:
    typedef frantic::graphics::vector3f screen_space_type;
    typedef frantic::rendering::framebuffer_cubeface<frantic::graphics::alpha3f> attenuation_buffer_type;
    typedef frantic::rendering::depthbuffer_cubeface shadow_map_type;
    typedef frantic::graphics::alpha3f pixel_type[6];

    inline static bool get_screen_pos( const frantic::rendering::lights::lightinterface& /*light*/,
                                       const frantic::graphics::vector3f& lightSpacePos,
                                       screen_space_type& outScreenSpacePos ) {
        outScreenSpacePos = lightSpacePos;
        return true;
    }

    inline static float get_density_scale( const frantic::rendering::lights::lightinterface& light,
                                           const frantic::graphics::vector3f& lightSpacePos ) {
        // Stolen from famebuffer_cubeface:
        //  The area of a voxel on screen at a distance d from the camera is (w/(2*d*tan(fov/2))^2.  In the case of a 90
        //  degree field of view, the tangent term is 1.  This value is calculated at the center of the cube faces.
        //
        // float draw_point_scaling_constant() const {
        //	return m_size * m_size / 4.f;
        // }

        return light.shadow_density() * (float)frantic::math::square<int>( light.shadow_map_size().xsize ) /
               ( 4.f * lightSpacePos.get_magnitude_squared() );
    }

    inline static void draw_attenuation( const frantic::graphics::alpha3f& attenSample,
                                         const screen_space_type& screenPos, attenuation_buffer_type& buffer ) {
        // TODO: Replace with a filter2f
        buffer.draw_point( screenPos, attenSample );
    }

    inline static void draw_attenuation( const frantic::graphics::alpha3f& attenSample,
                                         const screen_space_type& screenPos, attenuation_buffer_type& buffer,
                                         const filter2f_ptr& /*filter*/
    ) {
        // TODO: Replace with a filter2f
        buffer.draw_point( screenPos, attenSample );
    }

    inline static void add_deep_attenuation_sample( const frantic::graphics::alpha3f& attenSample,
                                                    const screen_space_type& screenPos, float /*z*/,
                                                    boost::shared_ptr<frantic::rendering::atten_saver> attenSaver ) {
        // Replace with a filter2f?
        if( attenSaver && attenSaver->is_cubeface() ) {
            frantic::rendering::cubeface_atten_saver* cubefaceAttenSaver =
                static_cast<frantic::rendering::cubeface_atten_saver*>( attenSaver.get() );
            cubefaceAttenSaver->add_sample_bilinear( screenPos, attenSample );
        }
    }
};

} // namespace splat_renderer
} // namespace krakatoa
