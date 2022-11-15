// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <float.h>
#include <krakatoa/renderer.hpp>
#include <krakatoa/splat_renderer/filter2f.hpp>
#include <krakatoa/splat_renderer/splat_lighting.hpp>

namespace krakatoa {
namespace splat_renderer {

/**
 * This interface is a specialization of the Krakatoa renderer interface that renders particles as infintesimal splats.
 */
class splat_renderer : public renderer {
  public:
    typedef boost::shared_ptr<splat_renderer> ptr_type;

  public:
    /**
     * This factory function will create an instance of splat_renderer. This is the only method of creating one, so
     * don't get cheeky. Using this function prevents the user from having to know anything about the concrete class
     * that implements this interface.
     */
    static ptr_type create_instance();

  protected:
    filter2f_ptr m_splatFilter; // The filter used when splatting particles.

    splat_lighting_ptr m_lightingEngine; // The lighting engine used to calculate light bouncing off particles towards
                                         // the camera. Fills the "Lighting" channel.

    std::size_t m_numRenderPasses;

  public:
    virtual ~splat_renderer() {}

    /**
     * @param filter The new filter to use when splatting particles.
     */
    virtual void set_splat_filter( filter2f_ptr filter ) { m_splatFilter = filter; }

    /**
     * @param lightingEngine The new lighting engine to use for computing particle lighting during precompute_lighting()
     */
    virtual void set_lighting_engine( splat_lighting_ptr lightingEngine ) { m_lightingEngine = lightingEngine; }

    /**
     * Adjusts apparent density of particles relative to the size of a camera facing 1x1 square at the particle's
     * location. This effectively will fade distant particles and make close particles appear brighter.
     */
    virtual void apply_area_differential( bool enabled = true, float limit = FLT_MAX ) = 0;

    /**
     * Adds a render element to be filled in during the render.
     * @param renderElement the render element to add to the rendering process.
     */
    virtual void add_render_element( krakatoa::render_element_interface_ptr renderElement ) = 0;

    /**
     * Computes the per-particle lighting and stores it in the channel "Lighting". Uses the lighting engine specified in
     * set_lighting_engine().
     */
    virtual void precompute_lighting() = 0;

    /**
     * This is the main rendering function. Will use the state specified by all calls to set_XXXX() to render an image
     * of the described particles and scene.
     * @note You must call at least set_particles() before calling this.
     * @param outImage The image to draw the particles to.
     */
    virtual void render( image_type& outImage ) = 0;

  protected:
    virtual std::size_t get_num_output_images() = 0;
};

typedef splat_renderer::ptr_type splat_renderer_ptr;

splat_renderer_ptr create_cube_splat_renderer();

/**
 * This is the main routine for calculating outgoing radiance (towards the camera) from a particle.
 * \param lighting The lighting value for the particle. It is the incident light on the particle, affected by the phase
 *                 function and multiplied by the scattering coefficient.
 * \param emission The light emitted by the particle.
 * \param extinct The particle extinction (unweighted by density). Should be in the [0,1] range per-channel.
 * \param density The density of the particle. Should be in the (0,\infty) range.
 * \param areaDifferential When splatting particles, they are based off calculations with unit voxels. Since our
 * particles are drawn the same size regardless of camera position, we use the expected area at this distance to modify
 * the perceived density of the particles. It is currently used to calculate the "depth" of a particle. \return The
 * particle color and alpha for splatting.
 */
inline frantic::graphics::color6f outgoing_radiance( const frantic::graphics::color3f& lighting,
                                                     const frantic::graphics::color3f& emission,
                                                     const frantic::graphics::color3f& extinct, float density,
                                                     float areaDifferential ) {
    frantic::graphics::color6f out;

    if( extinct.r <= 0 ) {
        out.a.ar = 0;
        out.c.r = areaDifferential * ( emission.r + lighting.r * density );
    } else {
        out.a.ar = 1.f - std::exp( -density * areaDifferential * extinct.r );
        out.c.r = out.a.ar * ( lighting.r / extinct.r + emission.r / ( extinct.r * density ) );
    }

    if( extinct.g <= 0 ) {
        out.a.ag = 0;
        out.c.g = areaDifferential * ( emission.g + lighting.g * density );
    } else {
        out.a.ag = 1.f - std::exp( -density * areaDifferential * extinct.g );
        out.c.g = out.a.ag * ( lighting.g / extinct.g + emission.g / ( extinct.g * density ) );
    }

    if( extinct.b <= 0 ) {
        out.a.ab = 0;
        out.c.b = areaDifferential * ( emission.b + lighting.b * density );
    } else {
        out.a.ab = 1.f - std::exp( -density * areaDifferential * extinct.b );
        out.c.b = out.a.ab * ( lighting.b / extinct.b + emission.b / ( extinct.b * density ) );
    }

    return out;
}

inline frantic::graphics::color6f outgoing_radiance( const frantic::graphics::color3f& lighting,
                                                     const frantic::graphics::color3f& emission, float density,
                                                     float areaDifferential ) {
    frantic::graphics::color6f out;

    out.a.ar = out.a.ag = out.a.ab = 1.f - std::exp( -density * areaDifferential );
    out.c = out.a.premultiply( lighting + emission / density );

    return out;
}

} // namespace splat_renderer
} // namespace krakatoa
