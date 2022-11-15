// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_SHADER__
#define __KRAKATOASR_SHADER__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * Shader base class.
 * This is a helper class used to set shading functions within the renderer.
 * Users cannot override this class to produce new shaders. Users can only use the provided subclasses within the
 * renderer.
 */
class CLSEXPORT shader {
  protected:
    shader_params* m_data;

  public:
    shader();
    virtual ~shader();
    shader( const shader& t );
    shader& operator=( const shader& t );
    const shader_params* get_data() const;
    shader_params* get_data();
    virtual void set_defaults() = 0;
};

/**
 * Isotropic shade function.
 * Light is scattered uniformly independent of the particle, light and camera orientation in space.
 */
class CLSEXPORT shader_isotropic : public shader {
  public:
    shader_isotropic();
    virtual void set_defaults();
    // isotropic has no paramters
};

/**
 * Phong phase function.
 * Scatters light dependent on the angle between the light source's direction, the particle normal and the viewing
 * direction according to the Phong specular model. Note that the Phong model describes surface behavior but in Krakatoa
 * every particle, even deep under the surface, will be shaded using this surface shading method. When using low
 * densities, the results might appear unrealistic, producing "shiny volumes". To use this shade function, the particles
 * must have a "Normal" channel. See online documentation for example renders using this function.
 */
class CLSEXPORT shader_phong : public shader {
  public:
    shader_phong();
    virtual void set_defaults();
    /// @param value Sets a constant specular power (glossiness) value. Defaults to 10.0.
    void set_specular_power( float value );
    /// @param value Sets a constant specular level value. Defaults to 100.0.
    void set_specular_level( float value );
    /// @param useChannel Use the particle stream's "SpecularPower" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_power_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "SpecularLevel" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_level_channel( bool useChannel );
};

/**
 * Henyey-Greenstein phase function for ellipsoid-shaped scattering.
 * Scatters light dependent on the angle between the light and the viewing direction.
 * See online documentation for example renders using this function.
 */
class CLSEXPORT shader_henyey_greenstein : public shader {
  public:
    shader_henyey_greenstein();
    virtual void set_defaults();
    /// @param value Sets a constant phase eccentricity value (amount of preferential scattering). Can be between -1
    /// and 1. Defaults to 0.0.
    void set_phase_eccentricity( float value );
    /// @param useChannel Use the particle stream's "Eccentricity" channel instead of the constant value. Defaults to
    /// false.
    void use_phase_eccentricity_channel( bool useChannel );
};

/**
 * Schlick phase function approximation to the Henyey-Greenstein function.
 * Scatters light dependent on the angle between the light and the viewing direction.
 * See online documentation for example renders using this function.
 */
class CLSEXPORT shader_schlick : public shader {
  public:
    shader_schlick();
    virtual void set_defaults();
    /// @param value Sets a constant phase eccentricity value (amount of preferential scattering). Can be between -1
    /// and 1. Defaults to 0.0.
    void set_phase_eccentricity( float value );
    /// @param useChannel Use the particle stream's "Eccentricity" channel instead of the constant value. Defaults to
    /// false.
    void use_phase_eccentricity_channel( bool useChannel );
};

/**
 * Kajiya-Kay phase function for rendering of fibers such as hair or cloth.
 * To use this phase function, the particles must have a "Tangent" channel. This function also uses an optional
 * "DiffuseLevel" channel if it exists.
 */
class CLSEXPORT shader_kajiya_kay : public shader {
  public:
    shader_kajiya_kay();
    virtual void set_defaults();
    /// @param value Sets a constant specular power (glossiness) value. Defaults to 10.0.
    void set_specular_power( float value );
    /// @param value Sets a constant specular level value. Defaults to 100.0.
    void set_specular_level( float value );
    /// @param useChannel Use the particle stream's "SpecularPower" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_power_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "SpecularLevel" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_level_channel( bool useChannel );
};

/**
 * Marschner phase function for rendering of hair.
 * To use this phase function, the particles must have "Normal" and "Tangent" channels.
 */
class CLSEXPORT shader_marschner : public shader {
  public:
    shader_marschner();
    virtual void set_defaults();
    /// @param value Sets a constant specular glossiness. Defaults to 300.0.
    void set_specular_glossiness( float value );
    /// @param value Sets a constant specular level. Defaults to 25.0.
    void set_specular_level( float value );
    /// @param value Sets a constant specular shift. Can be between -1 and 1. Defaults to 0.1.
    void set_specular_shift( float value );
    /// @param value Sets a constant secondary specular glossiness. Defaults to 30.0.
    void set_secondary_specular_glossiness( float value );
    /// @param value Sets a constant secondary specular level. Defaults to 90.0.
    void set_secondary_specular_level( float value );
    /// @param value Sets a constant secondary specular shift. Can be between -1 and 1. Defaults to -0.1.
    void set_secondary_specular_shift( float value );
    /// @param value Sets a constant glint level. Defaults to 400.0.
    void set_glint_level( float value );
    /// @param value Sets a constant glint size. Can be between 0 and 360. Defaults to 0.5.
    void set_glint_size( float value );
    /// @param value Sets a constant glint glossiness. Can be between 0 and 360. Defaults to 10.0.
    void set_glint_glossiness( float value );
    /// @param value Sets a constant diffuse level. Can be between 0 and 100. Defaults to 0.0.
    void set_diffuse_level( float value );
    /// @param useChannel Use the particle stream's "SpecularGlossiness" channel instead of the constant value. Defaults
    /// to false.
    void use_specular_glossiness_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "SpecularLevel" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_level_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "SpecularShift" channel instead of the constant value. Defaults to
    /// false.
    void use_specular_shift_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "Specular2Glossiness" channel instead of the constant value.
    /// Defaults to false.
    void use_secondary_specular_glossiness_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "Specular2Level" channel instead of the constant value. Defaults to
    /// false.
    void use_secondary_specular_level_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "Specular2Shift" channel instead of the constant value. Defaults to
    /// false.
    void use_secondary_specular_shift_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "GlintLevel" channel instead of the constant value. Defaults to
    /// false.
    void use_glint_level_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "GlintSize" channel instead of the constant value. Defaults to
    /// false.
    void use_glint_size_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "GlintGlossiness" channel instead of the constant value. Defaults to
    /// false.
    void use_glint_glossiness_channel( bool useChannel );
    /// @param useChannel Use the particle stream's "DiffuseLevel" channel instead of the constant value. Defaults to
    /// false.
    void use_diffuse_level_channel( bool useChannel );
};

} // namespace krakatoasr

#endif
