// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_LIGHT__
#define __KRAKATOASR_LIGHT__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * Light source base class.
 * This is a helper class used to create lights for the renderer.
 * Users cannot override this class to produce new light types. Users can only use the provided subclasses within the
 * renderer.
 */
class CLSEXPORT light {
  protected:
    light_params* m_data;

  public:
    light();
    virtual ~light();
    light( const light& t );
    light& operator=( const light& t );
    const light_params* get_data() const;
    light_params* get_data();
    virtual void set_defaults() = 0;

    /// @param name The name of the light. Defaults to "light".
    void set_name( const char* name );
    /// @param r,g,b Specifies the "power" of the light. This is specified as individual r,g,b components. Defaults to
    /// [12.5, 12.5, 12.5].
    void set_flux( float r, float g, float b );
    /// @param decayExponent An exponent value representing how the light's intensity reduces over distance. Defaults to
    /// 0.
    void set_decay_exponent( int decayExponent );
    /// @param enableShadowMap Setting this value to true will enable shadows for this light and will produce shadows
    /// from matte objects and use imported deep image maps. Defaults to true.
    void enable_shadow_map( bool enableShadowMap );
    /// @param density Used as a multiplier on the computed shadow values. Defaults to 1.0.
    void set_shadow_density( float density );
    /// @param width This defines the resolution of the internal attenuation map and shadow maps generated from the
    /// light. Larger values produce sharper self-shadowing detail. Defaults to 512.
    void set_shadow_map_width( int width );
    /// @param useNearAttenuation Near attenuation is defined as a distance value at which the light "fades in". If this
    /// is set, you must also call "set_near_attenuation".
    void use_near_attenuation( bool useNearAttenuation );
    /// @param start,end The starting and ending distance at which the light "fades in". The light has no power before
    /// "start" and increases linearly to full power by "end".
    void set_near_attenuation( float start, float end );
    /// @param useNearAttenuation Near attenuation is defined as a distance value at which the light "fades out". If
    /// this is set, you must also call "set_far_attenuation".
    void use_far_attenuation( bool useFarAttenuation );
    /// @param start,end The starting and ending value at which the light "fades out". The light has full power before
    /// "start" and decreases linearly to no power by "end".
    void set_far_attenuation( float start, float end );
    /// @param radius The offset at which decay begins if the decay exponent is greater than 0.
    void set_decay_radius( float radius );
};

/**
 * Direct light implementation of the light interface.
 */
class CLSEXPORT direct_light : public light {
  public:
    direct_light();
    virtual void set_defaults();

    /// @param lightShape Sets the light to be either square or round in shape. Defaults to krakatoasr::SHAPE_ROUND.
    void set_light_shape( light_shape_t lightShape );
    /// @param lightAspect Only used for square lights. Sets the aspect ratio of width to height of the light rectangle.
    /// Defaults to 1.0.
    void set_light_aspect( float lightAspect );
    /// @param innerRadius,outerRadius Sets the radius for the light beam. Values between innerRadius and oterRadius are
    /// linearly interpolated from full power to no power. Defaults to 43.0 and 45.0 respectively.
    void set_rect_radius( float innerRadius, float outerRadius );
    /// @param inFilename The layered "exr" filename of the light's deep attenuation (shadow) map. Importing external
    /// maps can replace having to use meshes for shadow maps.
    void set_attenuation_map_input( const char* inFilename );
    /// @param outFilename The layered "exr" filename for outputting a deep attenuation (shadow) map from this light.
    /// @param numDepthSamples The number of depth layers to produce.
    /// @param sampleSpacing The distance between layers.
    /// @param spacingIsExponential When true, distance between layers grows exponentially instead of linearly.
    void set_attenuation_map_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                     bool spacingIsExponential );
};

/**
 * Spot light implementation of the light interface.
 */
class CLSEXPORT spot_light : public light {
  public:
    spot_light();
    virtual void set_defaults();

    /// @param lightShape Sets the light to be either square or round in shape. Defaults to krakatoasr::SHAPE_ROUND.
    void set_light_shape( light_shape_t lightShape );
    /// @param lightAspect Only used for square lights. Sets the aspect ratio of width to height of the light rectangle.
    /// Defaults to 1.0.
    void set_light_aspect( float lightAspect );
    /// @param innerAngle,outerAngle Sets the angle in degress of the spot light's cone. The values between innerAngle
    /// and outerAngle are linearly interpolated from full power to no power. Defaults to 43.0 and 45.0 respectively.
    void set_cone_angle( float innerAngle, float outerAngle );
    /// @param inFilename The layered "exr" filename of the light's deep attenuation (shadow) map. Importing external
    /// maps can replace having to use meshes for shadow maps.
    void set_attenuation_map_input( const char* inFilename );
    /// @param outFilename The layered "exr" filename for outputting a deep attenuation (shadow) map from this light.
    /// @param numDepthSamples The number of depth layers to produce.
    /// @param sampleSpacing The distance between layers.
    /// @param spacingIsExponential When true, distance between layers grows exponentially instead of linearly.
    void set_attenuation_map_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                     bool spacingIsExponential );
};

/**
 * Point light implementation of the light interface.
 */
class CLSEXPORT point_light : public light {
  public:
    point_light();
    virtual void set_defaults();

    /// @param inFilename The layered "exr" filename of the light's deep attenuation (shadow) map. All six cube renders
    /// are contained in a single file. Importing external maps can replace having to use meshes for shadow maps.
    void set_attenuation_cubemap_exr_input( const char* inFilename );
    /// @param outFilename The layered "exr" filename for outputting a deep attenuation (shadow) map from this light.
    /// @param numDepthSamples The number of depth layers to produce.
    /// @param sampleSpacing The distance between layers.
    /// @param spacingIsExponential When true, distance between layers grows exponentially instead of linearly.
    void set_attenuation_cubemap_exr_output( const char* outFilename, int numDepthSamples, float sampleSpacing,
                                             bool spacingIsExponential );
};

} // namespace krakatoasr

#endif
