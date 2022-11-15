// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <boost/shared_ptr.hpp>

#include <krakatoa/light_object.hpp>
#include <krakatoa/scene_context.hpp>

#include <frantic/rendering/deep_attenuation_loaders.hpp>
#include <frantic/rendering/depthbuffer_cubeface.hpp>
#include <frantic/rendering/depthbuffer_singleface.hpp>

using namespace frantic::graphics;
using namespace frantic::graphics2d;

namespace krakatoa {

class light_interface_impl : public light_object {

    boost::shared_ptr<frantic::rendering::lights::lightinterface> m_lightObj;
    atmosphere_interface_ptr m_atmosphere;
    float m_mblurTime;
    scene_context_ptr m_context;

    // for shadow maps
    shadow_map_generator_ptr m_shadowMapGen;
    frantic::rendering::depthbuffer_singleface m_shadowMap;

    // for deep shadow maps
    frantic::rendering::singleface_atten_loader* m_attenLoader;

  public:
    light_interface_impl( boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj );

    virtual ~light_interface_impl();

    virtual frantic::rendering::lights::lightinterface& get_light_impl() { return *m_lightObj; }

    virtual const frantic::rendering::lights::lightinterface& get_light_impl() const { return *m_lightObj; }

    virtual void set_atmosphere( atmosphere_interface_ptr atmosphere );

    virtual void set_shadow_map_generator( shadow_map_generator_ptr shadowMapGen ) { m_shadowMapGen = shadowMapGen; }

    virtual void begin( const scene_context_ptr context );

    virtual void update( float mblurTime /*, bool isCanonicalTime*/ );

    virtual void end();

    virtual light_type eval_lighting( const frantic::graphics::vector3f& worldPos,
                                      frantic::graphics::vector3f* outLocalPos );
};

light_interface_impl::light_interface_impl( boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj )
    : m_lightObj( lightObj ) {
    m_mblurTime = 0.5f;
    m_attenLoader = NULL;
}

light_interface_impl::~light_interface_impl() {}

void light_interface_impl::set_atmosphere( atmosphere_interface_ptr atmosphere ) { m_atmosphere = atmosphere; }

void light_interface_impl::begin( const scene_context_ptr context ) {
    m_context = context;
    m_mblurTime = 0.5f;

    if( m_lightObj->is_shadows_enabled() ) {

        if( m_shadowMapGen ) {
            m_shadowMap.set_size( m_lightObj->shadow_map_size() );
            m_shadowMapGen->generate_shadow_map( *this, context, 0.5f, m_shadowMap.as_framebuffer() );
        }

        // get the single-faced attenuation loader
        if( m_lightObj->get_attenuation_loader() && !m_lightObj->get_attenuation_loader()->is_cubeface() )
            m_attenLoader =
                static_cast<frantic::rendering::singleface_atten_loader*>( m_lightObj->get_attenuation_loader().get() );
    }
}

void light_interface_impl::update( float mblurTime /*, bool isCanonicalTime*/ ) {
    m_mblurTime = mblurTime;

    // TODO: Cache the light's transform matrix at the mblur time.
}

void light_interface_impl::end() {}

light_object::light_type light_interface_impl::eval_lighting( const vector3f& worldPos,
                                                              frantic::graphics::vector3f* outLocalPos ) {
    bool isValid = true;

    light_type result = m_lightObj->irradiance( worldPos, isValid );

    if( isValid ) {
        const frantic::graphics::camera<float>& lightCam =
            static_cast<frantic::rendering::lights::directedlight_interface&>( *m_lightObj ).get_camera();

        vector3f lightSpacePos = lightCam.world_transform_inverse( m_mblurTime ) * worldPos;
        vector2f screenPos = lightCam.from_cameraspace_position( lightSpacePos, isValid );

        if( m_lightObj->is_shadows_enabled() ) {
            if( m_shadowMapGen )
                result *= m_shadowMap.get_pcf_visibility_z( screenPos, -lightSpacePos.z );

            // add deep attenuation alpha contribution
            if( m_attenLoader ) {
                // both the lightCam output size and deep shadow map will always be NxN.
                float sampleScale = (float)m_attenLoader->size().xsize / lightCam.get_output_size().xsize;
                alpha3f shadow = m_attenLoader->get_sample_bilinear( screenPos.x * sampleScale,
                                                                     screenPos.y * sampleScale, -lightSpacePos.z );
                result[0] *= 1.0f - shadow.ar;
                result[1] *= 1.0f - shadow.ag;
                result[2] *= 1.0f - shadow.ab;
            }
        }

        if( m_atmosphere )
            m_atmosphere->apply_atomsphere( result, lightCam.camera_position( m_mblurTime, worldPos ), worldPos );

        if( outLocalPos )
            *outLocalPos = vector3f( screenPos.x, screenPos.y, -lightSpacePos.z );
    }

    return result;
}

class omni_light_interface_impl : public light_object {

    boost::shared_ptr<frantic::rendering::lights::lightinterface> m_lightObj;
    atmosphere_interface_ptr m_atmosphere;
    float m_mblurTime;
    scene_context_ptr m_context;
    frantic::graphics::transform4f m_worldToLightTM;

    // for shadow maps
    shadow_map_generator_ptr m_shadowMapGen;
    frantic::rendering::depthbuffer_cubeface m_shadowMap;

    // for deep shadow maps
    frantic::rendering::cubeface_atten_loader* m_attenLoader;

  public:
    omni_light_interface_impl( boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj );

    virtual ~omni_light_interface_impl();

    virtual frantic::rendering::lights::lightinterface& get_light_impl() { return *m_lightObj; }

    virtual const frantic::rendering::lights::lightinterface& get_light_impl() const { return *m_lightObj; }

    virtual void set_atmosphere( atmosphere_interface_ptr atmosphere ) { m_atmosphere = atmosphere; }

    virtual void set_shadow_map_generator( shadow_map_generator_ptr shadowMapGen ) { m_shadowMapGen = shadowMapGen; }

    virtual void begin( const scene_context_ptr context );

    virtual void update( float mblurTime );

    virtual void end() {}

    virtual light_type eval_lighting( const frantic::graphics::vector3f& worldPos,
                                      frantic::graphics::vector3f* outLocalPos );
};

omni_light_interface_impl::omni_light_interface_impl(
    boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj ) {
    m_lightObj = lightObj;
    m_attenLoader = NULL;
}

omni_light_interface_impl::~omni_light_interface_impl() {}

void omni_light_interface_impl::begin( const scene_context_ptr context ) {
    m_context = context;
    m_mblurTime = 0.5f;

    if( m_lightObj->is_shadows_enabled() ) {

        if( m_shadowMapGen ) {
            m_shadowMap.set_size( m_lightObj->shadow_map_size().xsize );
            m_shadowMapGen->generate_shadow_map( *this, m_context, m_mblurTime, m_shadowMap.as_framebuffer() );
        }

        // get the cube-faced attenuation loader
        if( m_lightObj->get_attenuation_loader() && m_lightObj->get_attenuation_loader()->is_cubeface() )
            m_attenLoader =
                static_cast<frantic::rendering::cubeface_atten_loader*>( m_lightObj->get_attenuation_loader().get() );
    }
}

void omni_light_interface_impl::update( float mblurTime ) {
    m_mblurTime = mblurTime;
    m_worldToLightTM = m_lightObj->transform_matrix( mblurTime ).to_inverse();
}

omni_light_interface_impl::light_type
omni_light_interface_impl::eval_lighting( const frantic::graphics::vector3f& worldPos,
                                          frantic::graphics::vector3f* outLocalPos ) {
    bool isValid = true;

    light_type result = m_lightObj->irradiance( worldPos, isValid );

    if( isValid ) {
        vector3f lightSpacePos = m_worldToLightTM * worldPos;

        if( m_lightObj->is_shadows_enabled() ) {
            if( m_shadowMapGen )
                result *= m_shadowMap.get_pcf_visibility_cameraspace_z( lightSpacePos );

            // add deep attenuation alpha contribution
            if( m_attenLoader ) {
                alpha3f shadow = m_attenLoader->get_sample_bilinear( lightSpacePos );
                result[0] *= 1.0f - shadow.ar;
                result[1] *= 1.0f - shadow.ag;
                result[2] *= 1.0f - shadow.ab;
            }
        }

        if( m_atmosphere )
            m_atmosphere->apply_atomsphere( result, m_lightObj->position( m_mblurTime, worldPos ), worldPos );

        if( outLocalPos )
            *outLocalPos = lightSpacePos;
    }

    return result;
}

light_object::ptr_type light_object::create( boost::shared_ptr<frantic::rendering::lights::lightinterface> lightObj ) {
    if( lightObj->is_directional_light() )
        return light_object_ptr( new light_interface_impl( lightObj ) );
    else
        return light_object_ptr( new omni_light_interface_impl( lightObj ) );
}

} // namespace krakatoa
