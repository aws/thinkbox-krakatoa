// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_light.hpp>

#include <krakatoasr_renderer/params.hpp>

namespace krakatoasr {

light::light() { m_data = new light_params; }

light::~light() { delete m_data; }

light::light( const light& t ) {
    m_data = new light_params;
    *this = t;
}

light& light::operator=( const light& t ) {
    *m_data = *t.m_data;
    return *this;
}

const light_params* light::get_data() const { return m_data; }

light_params* light::get_data() { return m_data; }

void light::set_name( const char* name ) { m_data->name = name; }

void light::set_flux( float r, float g, float b ) { m_data->flux.set( r, g, b ); }

void light::set_decay_exponent( int decayExponent ) { m_data->decayExponent = decayExponent; }

void light::enable_shadow_map( bool enableShadowMap ) { m_data->enableShadows = enableShadowMap; }
void light::set_shadow_density( float density ) { m_data->shadowDensity = density; }

void light::set_shadow_map_width( int width ) { m_data->shadowMapWidth = width; }

void light::use_near_attenuation( bool useNearAttenuation ) { m_data->useNearAtten = useNearAttenuation; }

void light::set_near_attenuation( float start, float end ) {
    m_data->nearAttenuationStart = start;
    m_data->nearAttenuationEnd = end;
}

void light::use_far_attenuation( bool useFarAttenuation ) { m_data->useFarAtten = useFarAttenuation; }

void light::set_far_attenuation( float start, float end ) {
    m_data->farAttenuationStart = start;
    m_data->farAttenuationEnd = end;
}

void light::set_decay_radius( float radius ) { m_data->decayRadius = radius; }

direct_light::direct_light() { set_defaults(); }

void direct_light::set_defaults() {
    reset_light_params( *m_data );
    m_data->lightType = "direct";
}

void direct_light::set_light_shape( light_shape_t shape ) { m_data->lightShape = shape; }

void direct_light::set_light_aspect( float aspect ) { m_data->lightAspect = aspect; }

void direct_light::set_rect_radius( float innerRadius, float outerRadius ) {
    m_data->innerRectRadius = innerRadius;
    m_data->outerRectRadius = outerRadius;
}

void direct_light::set_attenuation_map_input( const char* inFilename ) { m_data->attenMapLoadingPath = inFilename; }

void direct_light::set_attenuation_map_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                               bool spacingIsExponential ) {
    m_data->attenuationMapSavingPath = outFilename;
    m_data->attenuationMapDepthSamples = numDepthSamples;
    m_data->attenuationMapSampleSpacing = sampleSpacing;
    m_data->attenuationMapExponentialSampleSpacing = spacingIsExponential;
}

spot_light::spot_light() { set_defaults(); }

void spot_light::set_defaults() {
    reset_light_params( *m_data );
    m_data->lightType = "spot";
}

void spot_light::set_light_shape( light_shape_t shape ) { m_data->lightShape = shape; }

void spot_light::set_light_aspect( float aspect ) { m_data->lightAspect = aspect; }

void spot_light::set_cone_angle( float innerAngle, float outerAngle ) {
    m_data->innerConeAngle = innerAngle;
    m_data->outerConeAngle = outerAngle;
}

void spot_light::set_attenuation_map_input( const char* inFilename ) { m_data->attenMapLoadingPath = inFilename; }

void spot_light::set_attenuation_map_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                             bool spacingIsExponential ) {
    m_data->attenuationMapSavingPath = outFilename;
    m_data->attenuationMapDepthSamples = numDepthSamples;
    m_data->attenuationMapSampleSpacing = sampleSpacing;
    m_data->attenuationMapExponentialSampleSpacing = spacingIsExponential;
}

point_light::point_light() { set_defaults(); }

void point_light::set_defaults() {
    reset_light_params( *m_data );
    m_data->lightType = "point";
}

void point_light::set_attenuation_cubemap_exr_input( const char* inFilename ) {
    m_data->attenMapLoadingPath = inFilename;
}

void point_light::set_attenuation_cubemap_exr_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                                      bool spacingIsExponential ) {
    m_data->attenuationMapSavingPath = outFilename;
    m_data->attenuationMapDepthSamples = numDepthSamples;
    m_data->attenuationMapSampleSpacing = sampleSpacing;
    m_data->attenuationMapExponentialSampleSpacing = spacingIsExponential;
}

} // namespace krakatoasr
