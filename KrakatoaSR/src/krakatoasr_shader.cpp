// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_shader.hpp>

#include <krakatoasr_renderer/params.hpp>

namespace krakatoasr {

void reset_shader_params( shader_params& shader ) { shader.params = krakatoa::shader_params(); }

shader::shader() {
    m_data = new shader_params;
    m_data->params.phaseFunction = _T( "unknown phase function" );
}

shader::~shader() { delete m_data; }

shader::shader( const shader& t ) {
    m_data = new shader_params;
    *this = t;
}

shader& shader::operator=( const shader& t ) {
    *m_data = *t.m_data;
    return *this;
}

const shader_params* shader::get_data() const { return m_data; }

shader_params* shader::get_data() { return m_data; }

shader_isotropic::shader_isotropic() { set_defaults(); }

void shader_isotropic::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Isotropic" );
}

shader_phong::shader_phong() { set_defaults(); }

void shader_phong::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Phong Surface" );
}

void shader_phong::set_specular_power( float value ) { m_data->params.specularPower = value; }

void shader_phong::set_specular_level( float value ) { m_data->params.specularLevel = value; }

void shader_phong::use_specular_power_channel( bool useChannel ) { m_data->params.allocateSpecularPower = useChannel; }

void shader_phong::use_specular_level_channel( bool useChannel ) { m_data->params.allocateSpecularLevel = useChannel; }

shader_henyey_greenstein::shader_henyey_greenstein() { set_defaults(); }

void shader_henyey_greenstein::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Henyey-Greenstein" );
}

void shader_henyey_greenstein::set_phase_eccentricity( float value ) { m_data->params.phaseEccentricity = value; }

void shader_henyey_greenstein::use_phase_eccentricity_channel( bool useChannel ) {
    m_data->params.allocatePhaseEccentricty = useChannel;
}

shader_schlick::shader_schlick() { set_defaults(); }

void shader_schlick::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Schlick" );
}

void shader_schlick::set_phase_eccentricity( float value ) { m_data->params.phaseEccentricity = value; }

void shader_schlick::use_phase_eccentricity_channel( bool useChannel ) {
    m_data->params.allocatePhaseEccentricty = useChannel;
}

shader_kajiya_kay::shader_kajiya_kay() { set_defaults(); }

void shader_kajiya_kay::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Kajiya-Kay Hair" );
}

void shader_kajiya_kay::set_specular_power( float value ) { m_data->params.specularPower = value; }

void shader_kajiya_kay::set_specular_level( float value ) { m_data->params.specularLevel = value; }

void shader_kajiya_kay::use_specular_power_channel( bool useChannel ) {
    m_data->params.allocateSpecularPower = useChannel;
}

void shader_kajiya_kay::use_specular_level_channel( bool useChannel ) {
    m_data->params.allocateSpecularLevel = useChannel;
}

shader_marschner::shader_marschner() { set_defaults(); }

void shader_marschner::set_defaults() {
    reset_shader_params( *m_data );
    m_data->params.phaseFunction = _T( "Marschner" );
}

void shader_marschner::set_specular_glossiness( float value ) { m_data->params.kingKongSpecularGlossiness = value; }

void shader_marschner::set_specular_level( float value ) { m_data->params.kingKongSpecularLevel = value; }

void shader_marschner::set_specular_shift( float value ) { m_data->params.kingKongSpecularShift = value; }

void shader_marschner::set_secondary_specular_glossiness( float value ) {
    m_data->params.kingKongSpecular2Glossiness = value;
}

void shader_marschner::set_secondary_specular_level( float value ) { m_data->params.kingKongSpecular2Level = value; }

void shader_marschner::set_secondary_specular_shift( float value ) { m_data->params.kingKongSpecular2Shift = value; }

void shader_marschner::set_glint_level( float value ) { m_data->params.kingKongGlintLevel = value; }

void shader_marschner::set_glint_size( float value ) { m_data->params.kingKongGlintSize = value; }

void shader_marschner::set_glint_glossiness( float value ) { m_data->params.kingKongGlintGlossiness = value; }

void shader_marschner::set_diffuse_level( float value ) { m_data->params.kingKongDiffuseLevel = value; }

void shader_marschner::use_specular_glossiness_channel( bool useChannel ) {
    m_data->params.kingKongSpecularGlossinessVarying = useChannel;
}

void shader_marschner::use_specular_level_channel( bool useChannel ) {
    m_data->params.kingKongSpecularLevelVarying = useChannel;
}

void shader_marschner::use_specular_shift_channel( bool useChannel ) {
    m_data->params.kingKongSpecularShiftVarying = useChannel;
}

void shader_marschner::use_secondary_specular_glossiness_channel( bool useChannel ) {
    m_data->params.kingKongSpecular2GlossinessVarying = useChannel;
}

void shader_marschner::use_secondary_specular_level_channel( bool useChannel ) {
    m_data->params.kingKongSpecular2LevelVarying = useChannel;
}

void shader_marschner::use_secondary_specular_shift_channel( bool useChannel ) {
    m_data->params.kingKongSpecular2ShiftVarying = useChannel;
}

void shader_marschner::use_glint_level_channel( bool useChannel ) {
    m_data->params.kingKongGlintLevelVarying = useChannel;
}

void shader_marschner::use_glint_size_channel( bool useChannel ) {
    m_data->params.kingKongGlintSizeVarying = useChannel;
}

void shader_marschner::use_glint_glossiness_channel( bool useChannel ) {
    m_data->params.kingKongGlintGlossinessVarying = useChannel;
}

void shader_marschner::use_diffuse_level_channel( bool useChannel ) {
    m_data->params.kingKongDiffuseLevelVarying = useChannel;
}

} // namespace krakatoasr
