// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/intrusive_ptr.hpp>

#include <krakatoa/atmosphere_interface.hpp>
#include <krakatoa/geometry_renderer.hpp>

#include <frantic/graphics2d/framebuffer.hpp>
#include <frantic/rendering/framebuffer_cubeface.hpp>
#include <frantic/rendering/lights/directedlight_interface.hpp>
#include <frantic/rendering/lights/lightinterface.hpp>

namespace krakatoa {

class shadow_map_generator;
typedef boost::shared_ptr<shadow_map_generator> shadow_map_generator_ptr;

class scene_context;
typedef boost::intrusive_ptr<scene_context> scene_context_ptr;

class light_object {
  public:
    typedef frantic::graphics::color3f light_type;
    typedef boost::shared_ptr<light_object> ptr_type;

  public:
    static ptr_type create( boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj );

    virtual ~light_object() {}

    virtual frantic::rendering::lights::lightinterface& get_light_impl() = 0;

    virtual const frantic::rendering::lights::lightinterface& get_light_impl() const = 0;

    virtual void set_atmosphere( atmosphere_interface_ptr atmosphere ) = 0;

    virtual void set_shadow_map_generator( shadow_map_generator_ptr shadowMapGen ) = 0;

    virtual void begin( const scene_context_ptr context ) = 0;

    virtual void update( float mblurTime /*, bool isCanonicalTime = false*/ ) = 0;

    virtual void end() = 0;

    virtual light_type eval_lighting( const frantic::graphics::vector3f& worldPos,
                                      frantic::graphics::vector3f* outLocalPos = NULL ) = 0;
};

typedef light_object::ptr_type light_object_ptr;
// typedef directional_light_interface::ptr_type directional_light_interface_ptr;

class shadow_map_generator {
    matte_interface_ptr m_matteInterface;

  public:
    shadow_map_generator( matte_interface_ptr matteInterface )
        : m_matteInterface( matteInterface ) {}
    virtual ~shadow_map_generator() {}

    virtual void generate_shadow_map( const light_object& light, const scene_context_ptr context, float mblurTime,
                                      frantic::graphics2d::framebuffer<float>& outDepthImg ) {
        m_matteInterface->generate_depth_map(
            static_cast<const frantic::rendering::lights::directedlight_interface&>( light.get_light_impl() )
                .get_camera(),
            mblurTime, outDepthImg, true );
    }

    virtual void generate_shadow_map( const light_object& light, const scene_context_ptr context, float mblurTime,
                                      frantic::rendering::framebuffer_cubeface<float>& outDepthImg ) {
        using namespace frantic::graphics;
        using namespace frantic::graphics2d;
        transform4f renderTM = light.get_light_impl().transform_matrix( mblurTime );
        for( int i = 0; i < 6; ++i ) {
            camera<float> cubeCamera =
                camera<float>::from_cube_face( renderTM, static_cast<cube_face::default_cube_face>( i ) );
            framebuffer<float>& cubeSideBuffer = outDepthImg.get_cubeface_framebuffer( i );
            cubeCamera.set_output_size( size2( cubeSideBuffer.size() ) );
            m_matteInterface->generate_depth_map( cubeCamera, mblurTime, cubeSideBuffer, true );
        }
    }
};

} // namespace krakatoa
