// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <krakatoa/shader_functions.hpp>

#include <frantic/math/polynomial_roots.hpp>
#include <frantic/shading/highlights.hpp>

using namespace frantic::channels;
using namespace frantic::graphics;

namespace krakatoa {

shader_params::shader_params() {
    // defaults (as defined by max krakatoa)
    phaseFunction = _T("Isotropic");
    allocatePhaseEccentricty = false;
    phaseEccentricity = 0.0f;
    allocateSpecularLevel = false;
    specularLevel = 100.0f;
    allocateSpecularPower = false;
    specularPower = 10.0f;
    kingKongSpecularGlossinessVarying = false;
    kingKongSpecularGlossiness = 300.0f;
    kingKongSpecularLevelVarying = false;
    kingKongSpecularLevel = 25.0f;
    kingKongSpecularShiftVarying = false;
    kingKongSpecularShift = 0.1f;
    kingKongSpecular2GlossinessVarying = false;
    kingKongSpecular2Glossiness = 30.0f;
    kingKongSpecular2LevelVarying = false;
    kingKongSpecular2Level = 90.0f;
    kingKongSpecular2ShiftVarying = false;
    kingKongSpecular2Shift = -0.1f;
    kingKongGlintLevelVarying = false;
    kingKongGlintLevel = 400.0f;
    kingKongGlintSizeVarying = false;
    kingKongGlintSize = 0.5f;
    kingKongGlintGlossinessVarying = false;
    kingKongGlintGlossiness = 10.0f;
    kingKongDiffuseLevelVarying = false;
    kingKongDiffuseLevel = 0.0f;
}

shader_params::shader_params( const frantic::tstring& function ) {
    // defaults (as defined by max krakatoa)
    phaseFunction = function;
    allocatePhaseEccentricty = false;
    phaseEccentricity = 0.0f;
    allocateSpecularLevel = false;
    specularLevel = 100.0f;
    allocateSpecularPower = false;
    specularPower = 10.0f;
    kingKongSpecularGlossinessVarying = false;
    kingKongSpecularGlossiness = 300.0f;
    kingKongSpecularLevelVarying = false;
    kingKongSpecularLevel = 25.0f;
    kingKongSpecularShiftVarying = false;
    kingKongSpecularShift = 0.1f;
    kingKongSpecular2GlossinessVarying = false;
    kingKongSpecular2Glossiness = 30.0f;
    kingKongSpecular2LevelVarying = false;
    kingKongSpecular2Level = 90.0f;
    kingKongSpecular2ShiftVarying = false;
    kingKongSpecular2Shift = -0.1f;
    kingKongGlintLevelVarying = false;
    kingKongGlintLevel = 400.0f;
    kingKongGlintSizeVarying = false;
    kingKongGlintSize = 0.5f;
    kingKongGlintGlossinessVarying = false;
    kingKongGlintGlossiness = 10.0f;
    kingKongDiffuseLevelVarying = false;
    kingKongDiffuseLevel = 0.0f;
}

boost::shared_ptr<krakatoa_shader> create_shader( const shader_params& props ) {

    frantic::tstring fnName = props.phaseFunction;
    boost::shared_ptr<krakatoa_shader> pResult;

    if( fnName == _T("Isotropic") ) {
        pResult.reset( new krakatoa::isotropic_phase_function );
        // if( false /*Check for color override*/ )
        //	pResult->set_channel_default<color3f>( _T("Color"), color3f::white() /*Override color*/ );
    } else if( fnName == _T("Henyey-Greenstein") ) {
        pResult.reset( new krakatoa::HG_phase_function );
        // if( false /*Check for color override*/ )
        //	pResult->set_channel_default<color3f>( _T("Color"), color3f::white() /*Override color*/ );
        if( !props.allocatePhaseEccentricty ) {
            float eccentricity = props.phaseEccentricity;
            pResult->set_channel_default<float>( _T("Eccentricity"), eccentricity );
        }
    } else if( fnName == _T("Schlick") ) {
        pResult.reset( new krakatoa::schlick_phase_function );
        // if( false /*Check for color override*/ )
        //	pResult->set_channel_default<color3f>( _T("Color"), color3f::white() /*Override color*/ );
        if( !props.allocatePhaseEccentricty ) {
            float eccentricity = props.phaseEccentricity;
            pResult->set_channel_default<float>( _T("Eccentricity"), eccentricity );
        }
    } else if( fnName == _T("Phong Surface") ) {
        pResult.reset( new krakatoa::phong_phase_function );
        // if( false /*Check for color override*/ )
        //	pResult->set_channel_default<color3f>( _T("Color"), color3f::white() /*Override color*/ );
        if( !props.allocateSpecularLevel ) {
            float specularLevel = props.specularLevel / 100.f;
            pResult->set_channel_default<float>( _T("SpecularLevel"), specularLevel );
        }
        if( !props.allocateSpecularPower ) {
            float specularPower = props.specularPower;
            pResult->set_channel_default<float>( _T("SpecularPower"), specularPower );
        }
    } else if( fnName == _T("Kajiya-Kay Hair") ) {
        pResult.reset( new krakatoa::kajiyakay_phase_function );
        if( !props.allocateSpecularLevel ) {
            float specularLevel = props.specularLevel / 100.f;
            pResult->set_channel_default<float>( _T("SpecularLevel"), specularLevel );
        }
        if( !props.allocateSpecularPower ) {
            float specularPower = props.specularPower;
            pResult->set_channel_default<float>( _T("SpecularPower"), specularPower );
        }
        pResult->set_channel_default<float>( _T("DiffuseLevel"), 1.f );
    } else if( fnName == _T("Marschner") || fnName == _T("Marschner Hair") ||
               fnName == _T("King Kong Hair") ) { // TODO: remove "King Kong Hair" string
        pResult.reset( new krakatoa::marschner_phase_function );
        if( !props.kingKongSpecularGlossinessVarying ) {
            float specularPower = props.kingKongSpecularGlossiness;
            pResult->set_channel_default<float>( _T("SpecularGlossiness"), specularPower );
        }
        if( !props.kingKongSpecularLevelVarying ) {
            float specularLevel = props.kingKongSpecularLevel / 100.f;
            pResult->set_channel_default<float>( _T("SpecularLevel"), specularLevel );
        }
        if( !props.kingKongSpecularShiftVarying ) {
            float specularShift = props.kingKongSpecularShift;
            pResult->set_channel_default<float>( _T("SpecularShift"), specularShift );
        }

        if( !props.kingKongSpecular2GlossinessVarying ) {
            float specular2Power = props.kingKongSpecular2Glossiness;
            pResult->set_channel_default<float>( _T("Specular2Glossiness"), specular2Power );
        }
        if( !props.kingKongSpecular2LevelVarying ) {
            float specular2Level = props.kingKongSpecular2Level / 100.f;
            pResult->set_channel_default<float>( _T("Specular2Level"), specular2Level );
        }
        if( !props.kingKongSpecular2ShiftVarying ) {
            float specular2Shift = props.kingKongSpecular2Shift;
            pResult->set_channel_default<float>( _T("Specular2Shift"), specular2Shift );
        }
        if( !props.kingKongGlintLevelVarying ) {
            float glintLevel = props.kingKongGlintLevel / 100.f;
            pResult->set_channel_default<float>( _T("GlintLevel"), glintLevel );
        }
        if( !props.kingKongGlintSizeVarying ) {
            float glintSize = props.kingKongGlintSize;
            pResult->set_channel_default<float>( _T("GlintSize"), glintSize );
        }
        if( !props.kingKongGlintGlossinessVarying ) {
            float glintGlossiness = props.kingKongGlintGlossiness;
            pResult->set_channel_default<float>( _T("GlintGlossiness"), glintGlossiness );
        }

        if( !props.kingKongDiffuseLevelVarying ) {
            float diffuseLevel = props.kingKongDiffuseLevel / 100.f;
            pResult->set_channel_default<float>( _T("DiffuseLevel"), diffuseLevel );
        }
    }

    if( !pResult )
        throw std::runtime_error( "krakatoa::create_shader - The phase function \"" +
                                  frantic::strings::to_string( fnName ) + "\" is not supported" );
    return pResult;
}

isotropic_phase_function::isotropic_phase_function() { m_requiredChannels.end_channel_definition(); }

void isotropic_phase_function::set_channel_map( const channel_map& /*pcm*/ ) {}

color3f isotropic_phase_function::shade( const vector3f& /*toEye*/, const vector3f& /*toLight*/,
                                         const color3f& incidentLight, const color3f& scatterCoefficient,
                                         const char* /*renderData*/ ) const {
    return scatterCoefficient * incidentLight;
}

HG_phase_function::HG_phase_function() {
    m_requiredChannels.define_channel( _T("Eccentricity"), 1, data_type_float16 );
    m_requiredChannels.end_channel_definition();
}

void HG_phase_function::set_channel_map( const channel_map& pcm ) {
    if( pcm.has_channel( _T("Eccentricity") ) ) {
        m_eccentricityAccessor = pcm.get_cvt_accessor<float>( _T("Eccentricity") );
    } else {
        if( !has_default( _T("Eccentricity") ) )
            throw std::runtime_error( "HG_phase_function::set_channel_map() - The input \"Eccentricity\" is missing" );
        m_eccentricityAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("Eccentricity") ).get( m_defaultValues.get() ) );
    }
}

color3f HG_phase_function::shade( const vector3f& toEye, const vector3f& toLight, const color3f& incidentLight,
                                  const color3f& scatterCoefficient, const char* renderData ) const {
    float g = m_eccentricityAccessor.get( renderData );
    float cosTheta = -frantic::graphics::vector3f::dot( toEye, toLight );
    float phaseVal = ( 1.0f - g * g ) / powf( 1.0f + g * g - 2.0f * g * cosTheta, 1.5f );

    return phaseVal * scatterCoefficient * incidentLight;
}

schlick_phase_function::schlick_phase_function() {
    m_requiredChannels.define_channel( _T("Eccentricity"), 1, data_type_float16 );
    m_requiredChannels.end_channel_definition();
}

void schlick_phase_function::set_channel_map( const channel_map& pcm ) {
    if( pcm.has_channel( _T("Eccentricity") ) ) {
        m_eccentricityAccessor = pcm.get_cvt_accessor<float>( _T("Eccentricity") );
    } else {
        if( !has_default( _T("Eccentricity") ) )
            throw std::runtime_error( "HG_phase_function::set_channel_map() - The input \"Eccentricity\" is missing" );
        m_eccentricityAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("Eccentricity") ).get( m_defaultValues.get() ) );
    }
}

color3f schlick_phase_function::shade( const vector3f& toEye, const vector3f& toLight, const color3f& incidentLight,
                                       const color3f& scatterCoefficient, const char* renderData ) const {
    float k = m_eccentricityAccessor.get( renderData );
    float cosTheta = -frantic::graphics::vector3f::dot( toEye, toLight );
    float denomRoot = 1.f + k * cosTheta;
    float phaseVal = ( 1.0f - k * k ) / ( denomRoot * denomRoot );
    return phaseVal * scatterCoefficient * incidentLight;
}

phong_phase_function::phong_phase_function() {
    m_requiredChannels.define_channel( _T("Normal"), 3, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularPower"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularLevel"), 1, data_type_float16 );
    m_requiredChannels.end_channel_definition();
}

void phong_phase_function::set_channel_map( const channel_map& pcm ) {
    // The "Normal" channel is not optional.
    m_normalAccessor = pcm.get_cvt_accessor<vector3f>( _T("Normal") );

    if( pcm.has_channel( _T("SpecularLevel") ) ) {
        m_specLevelAccessor = pcm.get_cvt_accessor<float>( _T("SpecularLevel") );
    } else {
        if( !has_default( _T("SpecularLevel") ) )
            throw std::runtime_error(
                "phong_phase_function::set_channel_map() - The input \"SpecularLevel\" is missing" );
        m_specLevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("SpecularPower") ) ) {
        m_specPowerAccessor = pcm.get_cvt_accessor<float>( _T("SpecularPower") );
    } else {
        if( !has_default( _T("SpecularPower") ) )
            throw std::runtime_error(
                "phong_phase_function::set_channel_map() - The input \"SpecularPower\" is missing" );
        m_specPowerAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularPower") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("__Element_Specular") ) )
        m_specularElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Specular") );
    if( pcm.has_channel( _T("__Element_Diffuse") ) )
        m_diffuseElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Diffuse") );
}

void phong_phase_function::set_particle_defaults( char* particle ) const {
    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
}

color3f phong_phase_function::shade( const vector3f& toEye, const vector3f& toLight, const color3f& incidentLight,
                                     const color3f& scatterCoefficient, const char* renderData ) const {
    frantic::graphics::vector3f normal = frantic::graphics::vector3f::normalize( m_normalAccessor.get( renderData ) );
    float nDotL = vector3f::dot( toLight, normal );
    float nDotV = vector3f::dot( toEye, normal );

    if( nDotL <= 0 || nDotV <= 0 )
        return frantic::graphics::color3f::black();

    frantic::graphics::vector3f half = vector3f::normalize( toLight + toEye );

    float specLevel = m_specLevelAccessor.get( renderData );
    float specPower = m_specPowerAccessor.get( renderData );

    float Kd = nDotL;
    float Ks = specLevel * frantic::shading::phong_isotropic_highlight( normal, half, specPower );

    frantic::graphics::color3f scatteredLight = scatterCoefficient * incidentLight;

    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( (char*)renderData,
                                     m_specularElementWriter.get( renderData ) + Ks * scatteredLight );

    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( (char*)renderData, m_diffuseElementWriter.get( renderData ) + Kd * scatteredLight );

    return ( Kd + Ks ) * scatteredLight;
}

kajiyakay_phase_function::kajiyakay_phase_function() {
    m_requiredChannels.define_channel( _T("Tangent"), 3, data_type_float16 );
    m_requiredChannels.define_channel( _T("DiffuseLevel"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularLevel"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularPower"), 1, data_type_float16 );
    m_requiredChannels.end_channel_definition();
}

void kajiyakay_phase_function::set_channel_map( const channel_map& pcm ) {
    // The "Tangent" channel is not optional.
    m_tangentAccessor = pcm.get_cvt_accessor<vector3f>( _T("Tangent") );

    if( pcm.has_channel( _T("DiffuseLevel") ) ) {
        m_diffuseLevelAccessor = pcm.get_cvt_accessor<float>( _T("DiffuseLevel") );
    } else {
        if( !has_default( _T("DiffuseLevel") ) )
            throw std::runtime_error(
                "phong_phase_function::set_channel_map() - The input \"DiffuseLevel\" is missing" );
        m_diffuseLevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("DiffuseLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("SpecularLevel") ) ) {
        m_specLevelAccessor = pcm.get_cvt_accessor<float>( _T("SpecularLevel") );
    } else {
        if( !has_default( _T("SpecularLevel") ) )
            throw std::runtime_error(
                "phong_phase_function::set_channel_map() - The input \"SpecularLevel\" is missing" );
        m_specLevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("SpecularPower") ) ) {
        m_specPowerAccessor = pcm.get_cvt_accessor<float>( _T("SpecularPower") );
    } else {
        if( !has_default( _T("SpecularPower") ) )
            throw std::runtime_error(
                "phong_phase_function::set_channel_map() - The input \"SpecularPower\" is missing" );
        m_specPowerAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularPower") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("__Element_Specular") ) )
        m_specularElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Specular") );
    if( pcm.has_channel( _T("__Element_Diffuse") ) )
        m_diffuseElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Diffuse") );
}

void kajiyakay_phase_function::set_particle_defaults( char* particle ) const {
    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
}

color3f kajiyakay_phase_function::shade( const vector3f& toEye, const vector3f& toLight, const color3f& incidentLight,
                                         const color3f& scatterCoefficient, const char* renderData ) const {
    vector3f tangent = vector3f::normalize( m_tangentAccessor.get( renderData ) );
    float diffuseLevel = m_diffuseLevelAccessor.get( renderData );
    float specLevel = m_specLevelAccessor.get( renderData );
    float specPower = m_specPowerAccessor.get( renderData );

    float cos_t_l = frantic::graphics::vector3f::dot( tangent, toLight );
    float sin_t_l = sqrt( 1.f - cos_t_l * cos_t_l );
    float Kd = diffuseLevel * sin_t_l;

    // The Kajiya-Kay specular term is cos(theta - theta') where theta is the angle between
    // the hair tangent and the toLight vector, and theta' is the angle from the tangent
    // to the toEye vector.
    float cos_t_e = -frantic::graphics::vector3f::dot( tangent, toEye );
    float sin_t_e = sqrt( 1.f - cos_t_e * cos_t_e );
    float cos_angleDiff = cos_t_l * cos_t_e + sin_t_l * sin_t_e;
    float Ks = 0.f;
    if( cos_angleDiff > 0 )
        Ks = specLevel * powf( cos_angleDiff, specPower );

    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( (char*)renderData,
                                     m_specularElementWriter.get( renderData ) + Ks * incidentLight );
    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( (char*)renderData, m_diffuseElementWriter.get( renderData ) +
                                                           Kd * scatterCoefficient * incidentLight );

    return Kd * scatterCoefficient * incidentLight + Ks * incidentLight;
}

/// A zero-mean gaussian with the specified variance.
///@param variance The variance of the guassian distribution.
///@param x The value to evaluate the gaussian distribution at.
///@return a float in [0, +inf) that is the probability density function of the guassian distribution at x.
// static float gaussian( float variance, float x ){
//	float lhs = sqrt( 2 * (float)M_PI * variance );
//	float rhs = exp( -( x * x ) / ( 2 * variance ) );
//	return rhs / lhs;
// }

static float gaussian2( float variance, float x ) { return exp( -( x * x ) / ( 2 * variance ) ); }

/// Returns the fresnel reflection amount for a light ray striking a material at a specified angle and index of
/// refraction. The material is assumed to be embedded in a vacuum. The light is assumed to be un-polarized.
///@note See http://en.wikipedia.org/wiki/Fresnel_equations for more details
///@param ir The index of refraction of the surface being struck
///@param angle The angle the light ray is making wrt to the surface normal
///@return A float in [0,1] that indicates the portion of energy reflected from this surface.
// static float fresnel( float ir1, float ir2, float angle ){
//	float cosAngle = cos( angle );
//	float sinAngle = sin( angle );
//	float sinAngle2 = sinAngle * sinAngle;
//
//	float sqrtPart1 = sqrt(1 - sinAngle2 / (ir1 * ir1));
//	float sqrtPart2 = sqrt(1 - sinAngle2 / (ir2 * ir2));
//
//	float rS = ( cosAngle - ir1 * sqrtPart1 ) / ( cosAngle + ir1 * sqrtPart1 );
//	float rP = ( sqrtPart2 - ir2 * cosAngle ) / ( sqrtPart2 + ir2 * cosAngle );
//
//	return 0.5f * ( rS * rS + rP * rP );
// }

marschner_phase_function::marschner_phase_function() {
    m_requiredChannels.define_channel( _T("Tangent"), 3, data_type_float16 );
    m_requiredChannels.define_channel( _T("Normal"), 3, data_type_float16 );
    m_requiredChannels.define_channel( _T("DiffuseLevel"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularLevel"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularGlossiness"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("SpecularShift"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("Specular2Level"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("Specular2Glossiness"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("Specular2Shift"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("GlintLevel"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("GlintSize"), 1, data_type_float16 );
    m_requiredChannels.define_channel( _T("GlintGlossiness"), 1, data_type_float16 );
    m_requiredChannels.end_channel_definition();
}

void marschner_phase_function::set_channel_map( const channel_map& pcm ) {
    // The "Tangent" channel is not optional.
    m_tangentAccessor = pcm.get_cvt_accessor<vector3f>( _T("Tangent") );
    m_normalAccessor = pcm.get_cvt_accessor<vector3f>( _T("Normal") );

    if( pcm.has_channel( _T("SpecularGlossiness") ) ) {
        m_spec1PowerAccessor = pcm.get_cvt_accessor<float>( _T("SpecularGlossiness") );
    } else {
        if( !has_default( _T("SpecularGlossiness") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"SpecularGlossiness\" is missing" );
        m_spec1PowerAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularGlossiness") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("SpecularLevel") ) ) {
        m_spec1LevelAccessor = pcm.get_cvt_accessor<float>( _T("SpecularLevel") );
    } else {
        if( !has_default( _T("SpecularLevel") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"SpecularLevel\" is missing" );
        m_spec1LevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("SpecularShift") ) ) {
        m_spec1ShiftAccessor = pcm.get_cvt_accessor<float>( _T("SpecularShift") );
    } else {
        if( !has_default( _T("SpecularShift") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"SpecularShift\" is missing" );
        m_spec1ShiftAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("SpecularShift") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("Specular2Glossiness") ) ) {
        m_spec2PowerAccessor = pcm.get_cvt_accessor<float>( _T("Specular2Glossiness") );
    } else {
        if( !has_default( _T("Specular2Glossiness") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"Specular2Glossiness\" is missing" );
        m_spec2PowerAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("Specular2Glossiness") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("Specular2Level") ) ) {
        m_spec2LevelAccessor = pcm.get_cvt_accessor<float>( _T("Specular2Level") );
    } else {
        if( !has_default( _T("Specular2Level") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"Specular2Level\" is missing" );
        m_spec2LevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("Specular2Level") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("Specular2Shift") ) ) {
        m_spec2ShiftAccessor = pcm.get_cvt_accessor<float>( _T("Specular2Shift") );
    } else {
        if( !has_default( _T("Specular2Shift") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"Specular2Shift\" is missing" );
        m_spec2ShiftAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("Specular2Shift") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("DiffuseLevel") ) ) {
        m_diffuseLevelAccessor = pcm.get_cvt_accessor<float>( _T("DiffuseLevel") );
    } else {
        if( !has_default( _T("DiffuseLevel") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"DiffuseLevel\" is missing" );
        m_diffuseLevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("DiffuseLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("GlintLevel") ) ) {
        m_glintLevelAccessor = pcm.get_cvt_accessor<float>( _T("GlintLevel") );
    } else {
        if( !has_default( _T("GlintLevel") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"GlintLevel\" is missing" );
        m_glintLevelAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("GlintLevel") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("GlintSize") ) ) {
        m_glintSizeAccessor = pcm.get_cvt_accessor<float>( _T("GlintSize") );
    } else {
        if( !has_default( _T("GlintSize") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"GlintSize\" is missing" );
        m_glintSizeAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("GlintSize") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("GlintGlossiness") ) ) {
        m_glintGlossinessAccessor = pcm.get_cvt_accessor<float>( _T("GlintGlossiness") );
    } else {
        if( !has_default( _T("GlintGlossiness") ) )
            throw std::runtime_error(
                "marschner_phase_function::set_channel_map() - The input \"GlintGlossiness\" is missing" );
        m_glintGlossinessAccessor.reset(
            m_requiredChannels.get_cvt_accessor<float>( _T("GlintGlossiness") ).get( m_defaultValues.get() ) );
    }

    if( pcm.has_channel( _T("__Element_Specular") ) )
        m_specularElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Specular") );
    if( pcm.has_channel( _T("__Element_Specular2") ) )
        m_specular2ElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Specular2") );
    if( pcm.has_channel( _T("__Element_Specular3") ) )
        m_glintElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Specular3") );
    if( pcm.has_channel( _T("__Element_Diffuse") ) )
        m_diffuseElementWriter = pcm.get_cvt_accessor<frantic::graphics::color3f>( _T("__Element_Diffuse") );
}

void marschner_phase_function::set_particle_defaults( char* particle ) const {
    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
    if( m_specular2ElementWriter.is_valid() )
        m_specular2ElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
    if( m_glintElementWriter.is_valid() )
        m_glintElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( particle, frantic::graphics::color3f( 0 ) );
}

#if 1
color3f marschner_phase_function::shade( const vector3f& toEye, const vector3f& toLight, const color3f& incidentLight,
                                         const color3f& scatterCoefficient, const char* renderData ) const {
    vector3f tangent = vector3f::normalize( m_tangentAccessor.get( renderData ) );
    vector3f normal = vector3f::normalize( m_normalAccessor.get( renderData ) );
    vector3f binormal = vector3f::normalize( vector3f::cross( normal, tangent ) );

    float specPower = m_spec1PowerAccessor.get( renderData );
    float specLevel = m_spec1LevelAccessor.get( renderData );
    float specShift = m_spec1ShiftAccessor.get( renderData );

    float spec2Power = m_spec2PowerAccessor.get( renderData );
    float spec2Level = m_spec2LevelAccessor.get( renderData );
    float spec2Shift = m_spec2ShiftAccessor.get( renderData );

    float glintLevel = m_glintLevelAccessor.get( renderData );
    float glintWidth = m_glintSizeAccessor.get( renderData );
    float glintWidth2 = m_glintGlossinessAccessor.get( renderData );

    float diffuseLevel = m_diffuseLevelAccessor.get( renderData );

    vector3f half = vector3f::normalize( toEye + toLight );
    vector3f halfS1 = vector3f::normalize( half + specShift * tangent );

    float dotHs1_T = vector3f::dot( halfS1, tangent );
    float sinHs1_T = sqrt( 1.f - dotHs1_T * dotHs1_T );
    float s1 = specLevel * pow( sinHs1_T, specPower );

    vector3f halfS2 = vector3f::normalize( half + spec2Shift * tangent );

    float dotHs2_T = vector3f::dot( halfS2, tangent );
    float sinHs2_T = sqrt( 1.f - dotHs2_T * dotHs2_T );
    float s2 = pow( sinHs2_T, spec2Power );
    // float s2 = spec2Level * pow( sinHs2_T, spec2Power );

    float dotL_T = vector3f::dot( toLight, tangent );
    float sinL_T = sqrt( 1.f - dotL_T * dotL_T );
    float d = diffuseLevel * sinL_T;

    s2 *= ( 0.25f + 0.75f * sinL_T );

    float dotE_T = vector3f::dot( toEye, tangent );

    // float thetaI = (float)M_PI_2 - acos( dotL_T );
    // float thetaR = (float)M_PI_2 - acos( dotE_T );
    // float theta = 0.5f * (thetaR - thetaI);

    vector3f projL = vector3f::normalize( toLight - dotL_T * tangent );
    vector3f projE = vector3f::normalize( toEye - dotE_T * tangent );

    float dotProjL_N = vector3f::dot( normal, projL );
    float dotProjE_N = vector3f::dot( normal, projE );

    float dotProjL_B = vector3f::dot( binormal, projL );
    float dotProjE_B = vector3f::dot( binormal, projE );

    float phiI = atan2( dotProjL_B, dotProjL_N );
    float phiR = atan2( dotProjE_B, dotProjE_N );
    // float phiH = 0.5f * ( phiI + phiR );

    float g = 0;
    float phi = phiR - phiI;

    if( std::abs( phi ) < (float)M_PI_2 ) {
        float phiDegrees = frantic::math::radians_to_degrees( phi );
        float phiIDegrees = frantic::math::radians_to_degrees( phiI );
        float var1 = glintWidth * glintWidth;
        float var2 = glintWidth2 * glintWidth2;

        float leftGauss = gaussian2( var1, phiDegrees - 30.f * sinL_T ) *
                          ( gaussian2( var2, phiIDegrees - 30 ) + gaussian2( var2, phiIDegrees + 150 ) );
        float rightGauss = gaussian2( var1, phiDegrees + 30.f * sinL_T ) *
                           ( gaussian2( var2, phiIDegrees - 120 ) + gaussian2( var2, phiIDegrees + 60 ) );
        g = glintLevel * ( leftGauss + rightGauss );
    }

    float Ks1 = s1;
    float Ks2 = s2 * spec2Level;
    float Kglint = s2 * g;
    float Kd = d;

    frantic::graphics::color3f scatteredLight = scatterCoefficient * incidentLight;

    if( m_specularElementWriter.is_valid() )
        m_specularElementWriter.set( (char*)renderData,
                                     m_specularElementWriter.get( renderData ) + Ks1 * incidentLight );
    if( m_specular2ElementWriter.is_valid() )
        m_specular2ElementWriter.set( (char*)renderData,
                                      m_specular2ElementWriter.get( renderData ) + Ks2 * scatteredLight );
    if( m_glintElementWriter.is_valid() )
        m_glintElementWriter.set( (char*)renderData, m_glintElementWriter.get( renderData ) + Kglint * scatteredLight );
    if( m_diffuseElementWriter.is_valid() )
        m_diffuseElementWriter.set( (char*)renderData, m_diffuseElementWriter.get( renderData ) + Kd * scatteredLight );

    return s1 * incidentLight + ( Ks2 + Kglint + Kd ) * scatteredLight;
}
#endif

#if 0
color3f marschner_phase_function::shade( 
	const vector3f& toEye, 
	const vector3f& toLight, 
	const color3f& incidentLight, 
	const color3f& scatterCoefficient, 
	const char* renderData ) const 
{
	vector3f tangent = vector3f::normalize( m_tangentAccessor.get( renderData ) );
	vector3f normal = vector3f::normalize( m_normalAccessor.get( renderData ) );
	vector3f binormal = vector3f::normalize( vector3f::cross( normal, tangent ) );

	float test = vector3f::dot( tangent, normal );
	if( fabsf(test) > 1e-4f )
		std::cerr << "";

	frantic::graphics::color3f sigmaA(0.432f,0.612f,0.980f);
	float ir = 1.55f;
	float eccentricity = 0.85f;

	float alphaR = -10.f;
	float alphaTT = -alphaR * 0.5f;
	float alphaTRT = -alphaR * 1.5f;

	float betaR = 5.f;
	float betaTT = betaR * 0.5f;
	float betaTRT = betaR * 2.f;

	float kG = 0.4f;
	float wC = 1.5f;
	float deltaIR = 0.2f;
	float deltaC = 0.5f;

	float cos_t_l = vector3f::dot( tangent, toLight );
	float cos_t_e = vector3f::dot( tangent, toEye );

	//In degrees
	float theta_i = (float)M_PI_2 - acos( cos_t_l );
	float theta_r = (float)M_PI_2 - acos( cos_t_e );
	float theta_s = theta_i + theta_r;
	float theta_d = 0.5f * (theta_r - theta_i);

	//return incidentLight * gaussian( (betaR * betaR), theta_h - alphaR );

	float mR = gaussian( (betaR * betaR), ( frantic::math::radians_to_degrees(theta_s) - alphaR) );
	//float mTT = gaussian( (betaTT * betaTT), (theta_h - alphaTT) );
	//float mTRT = gaussian( (betaTRT * betaTRT), (theta_s - alphaTRT) );

	vector3f projL = vector3f::normalize( toLight - tangent * cos_t_l );
	vector3f projE = vector3f::normalize( toEye - tangent * cos_t_e );

	float cos_n_l = vector3f::dot( normal, projL );
	float cos_n_e = vector3f::dot( normal, projE );

	float cos_b_l = vector3f::dot( binormal, projL );
	float cos_b_e = vector3f::dot( binormal, projE );

	//In radians, use atan2 to get (-pi, pi] range of angle.
	float phi_i = atan2( cos_b_l, cos_n_l );
	float phi_r = atan2( cos_b_e, cos_n_e ); 

	//float phi_h = 0.5f * ( phi_r + phi_i );
	float phi = phi_r - phi_i;

	//return scatterCoefficient * incidentLight * color3f((phi + (float)M_PI) / (float)M_PI, (theta_r + 90.f) / 180.f, 0);

	float sinTheta = sin( theta_d );
	float sinTheta2 = sinTheta * sinTheta;
	float cosTheta = cos( theta_d );
	float cosTheta2 = cosTheta * cosTheta;
	float ir2 = ir * ir;
	float irPrime = sqrt( ir2 - sinTheta2 ) / cosTheta;
	float irPrime2 = ir2 / irPrime;

	float c = asin( 1.f / irPrime );

	//phi(p,gammaI) = (6pc / pi - 2) * gammaI - (8pc / pi^3) * gammaI^3 + p*(pi)
	// for R, phi(0,gammaI) = -2 * gammaI
	// Let -2 * gammaI - phi = 0
	//      gammaI = -phi / 2.f
	//d phi / dh = -2 / cos( gammaI )
	float sR = 0.f, sTRT = 0.f;
	if( std::abs(phi) < (float)M_PI ){
		float gammaI_0 = -0.5f * phi;
		float h_0 = sin( gammaI_0 );
		float dPhi_dh_0 = fabsf( -4 / sqrt(1 - h_0 * h_0) ); //|2 * d phi/dh|
		float fresnelAtten = fresnel( irPrime, irPrime2, gammaI_0 );
		sR = mR * fresnelAtten / ( dPhi_dh_0 * cosTheta2 );
	}

	/*double t1 = 12.0 * c / M_PI - 2;
	double t3 = 16.0 * c / (M_PI * M_PI * M_PI);
	double t0 = 2.0 * M_PI;

	float deltaH, phiC, t;

	if( irPrime < 2 ){
		float hC = sqrt( (4.0 - irPrime2) / 3.0 );
		float gammaC = asin( hC );
		phiC = gammaC * (t1 - t3 * gammaC * gammaC) + t0;
		
		float h = sin( gammaC );
		float asinH = asin( h );
		float asinH2 = asinH * asinH;
		float H2Inv = 1 - h * h;
		float sqrtH2Inv = sqrt( H2Inv );
		float dderiv = ( -6 * t3 * sqrtH2Inv + h * ( t1 - 3 * t3 * asinH2 ) ) / (sqrtH2Inv * H2Inv);
		deltaH = std::min( deltaC, 2 * sqrt( 2 * wC / std::abs( dderiv ) ) );
		t = 1.f;
	}else{
		phiC = 0;
		deltaH = deltaC;
		t = 1.f - frantic::math::smoothstep( irPrime, 2.f, 2.f + deltaIR );
	}

	double roots[] = { 0.0, 0.0, 0.0 };
	int nRoots = frantic::math::polynomial_roots::get_cubic_roots( 0.0, (-t0 / t3), (-t1 / t3), roots[0], roots[1], roots[2] );
	if( nRoots == 1 ){
		
	}else if( nRoots == 3 ){
	}else
		throw std::runtime_error( "" );*/

	float cosThetaI = cos( theta_i );
	return (sR * cosThetaI) * incidentLight + (sTRT * cosThetaI) * incidentLight * scatterCoefficient;
}
#endif

} // namespace krakatoa
