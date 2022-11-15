// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <set>

#include <frantic/channels/channel_map.hpp>

#include <krakatoa/shaders.hpp>

namespace krakatoa {

class isotropic_phase_function;
class phong_phase_function;
class HG_phase_function;

/**
 * Holds parameters for the create_shader "factory" function below
 */
struct shader_params {
    frantic::tstring phaseFunction;
    bool allocatePhaseEccentricty;
    float phaseEccentricity;
    bool allocateSpecularLevel;
    float specularLevel;
    bool allocateSpecularPower;
    float specularPower;
    bool kingKongSpecularGlossinessVarying;
    float kingKongSpecularGlossiness;
    bool kingKongSpecularLevelVarying;
    float kingKongSpecularLevel;
    bool kingKongSpecularShiftVarying;
    float kingKongSpecularShift;
    bool kingKongSpecular2GlossinessVarying;
    float kingKongSpecular2Glossiness;
    bool kingKongSpecular2LevelVarying;
    float kingKongSpecular2Level;
    bool kingKongSpecular2ShiftVarying;
    float kingKongSpecular2Shift;
    bool kingKongGlintLevelVarying;
    float kingKongGlintLevel;
    bool kingKongGlintSizeVarying;
    float kingKongGlintSize;
    bool kingKongGlintGlossinessVarying;
    float kingKongGlintGlossiness;
    bool kingKongDiffuseLevelVarying;
    float kingKongDiffuseLevel;

    // constructor sets defaults
    shader_params();
    shader_params( const frantic::tstring& function );
};

/**
 * Factory function that creates a krakata_shader given a populated shader_params object
 */
boost::shared_ptr<krakatoa_shader> create_shader( const shader_params& props );

class isotropic_phase_function : public krakatoa_shader {
  public:
    isotropic_phase_function();

    virtual ~isotropic_phase_function() {}

    frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                      const frantic::graphics::vector3f& toLight,
                                      const frantic::graphics::color3f& incidentLight,
                                      const frantic::graphics::color3f& scatterCoefficient,
                                      const char* renderData ) const;

    void set_channel_map( const frantic::channels::channel_map& pcm );
};

class HG_phase_function : public krakatoa_shader {
    frantic::channels::channel_cvt_accessor<float> m_eccentricityAccessor;

  public:
    HG_phase_function();

    virtual ~HG_phase_function() {}

    frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                      const frantic::graphics::vector3f& toLight,
                                      const frantic::graphics::color3f& incidentLight,
                                      const frantic::graphics::color3f& scatterCoefficient,
                                      const char* renderData ) const;

    void set_channel_map( const frantic::channels::channel_map& pcm );
};

class schlick_phase_function : public krakatoa_shader {
    frantic::channels::channel_cvt_accessor<float> m_eccentricityAccessor;

  public:
    schlick_phase_function();

    virtual ~schlick_phase_function() {}

    frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                      const frantic::graphics::vector3f& toLight,
                                      const frantic::graphics::color3f& incidentLight,
                                      const frantic::graphics::color3f& scatterCoefficient,
                                      const char* renderData ) const;

    void set_channel_map( const frantic::channels::channel_map& pcm );
};

class phong_phase_function : public krakatoa_shader {
    frantic::channels::channel_cvt_accessor<float> m_specLevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_specPowerAccessor;
    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_normalAccessor;

    // Render element writers
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_specularElementWriter;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_diffuseElementWriter;

  public:
    phong_phase_function();

    virtual ~phong_phase_function() {}

    virtual frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                              const frantic::graphics::vector3f& toLight,
                                              const frantic::graphics::color3f& incidentLight,
                                              const frantic::graphics::color3f& scatterCoefficient,
                                              const char* renderData ) const;

    virtual void set_channel_map( const frantic::channels::channel_map& pcm );

    virtual void set_particle_defaults( char* particle ) const;
};

class kajiyakay_phase_function : public krakatoa_shader {
    frantic::channels::channel_cvt_accessor<float> m_diffuseLevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_specLevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_specPowerAccessor;
    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_tangentAccessor;

    // Render element writers
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_specularElementWriter;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_diffuseElementWriter;

  public:
    kajiyakay_phase_function();

    virtual ~kajiyakay_phase_function() {}

    frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                      const frantic::graphics::vector3f& toLight,
                                      const frantic::graphics::color3f& incidentLight,
                                      const frantic::graphics::color3f& scatterCoefficient,
                                      const char* renderData ) const;

    virtual void set_particle_defaults( char* particle ) const;

    virtual void set_channel_map( const frantic::channels::channel_map& pcm );
};

class marschner_phase_function : public krakatoa_shader {
    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_tangentAccessor;
    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_normalAccessor;

    frantic::channels::channel_cvt_accessor<float> m_spec1PowerAccessor;
    frantic::channels::channel_cvt_accessor<float> m_spec1LevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_spec1ShiftAccessor;

    frantic::channels::channel_cvt_accessor<float> m_spec2PowerAccessor;
    frantic::channels::channel_cvt_accessor<float> m_spec2LevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_spec2ShiftAccessor;

    frantic::channels::channel_cvt_accessor<float> m_glintLevelAccessor;
    frantic::channels::channel_cvt_accessor<float> m_glintSizeAccessor;
    frantic::channels::channel_cvt_accessor<float> m_glintGlossinessAccessor;

    frantic::channels::channel_cvt_accessor<float> m_diffuseLevelAccessor;

    // Render element writers
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_specularElementWriter;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_specular2ElementWriter;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_glintElementWriter;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3f> m_diffuseElementWriter;

  public:
    marschner_phase_function();

    virtual ~marschner_phase_function() {}

    virtual frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                              const frantic::graphics::vector3f& toLight,
                                              const frantic::graphics::color3f& incidentLight,
                                              const frantic::graphics::color3f& scatterCoefficient,
                                              const char* renderData ) const;

    virtual void set_particle_defaults( char* particle ) const;

    virtual void set_channel_map( const frantic::channels::channel_map& pcm );
};

} // namespace krakatoa
