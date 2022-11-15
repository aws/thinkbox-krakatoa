// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/boundbox3f.hpp>
#include <frantic/volumetrics/voxel_coord_system.hpp>

#include <krakatoa/renderer.hpp>
#include <krakatoa/splat_renderer/splat_renderer.hpp>

namespace krakatoa {
namespace raytrace_renderer {

class raytrace_renderer : public renderer {
  protected:
    frantic::volumetrics::voxel_coord_system m_vcs;
    float m_minStepSize, m_maxStepSize;

    color_type m_ambientLight;

    splat_renderer::filter2f_ptr m_splatFilter;
    splat_renderer::splat_lighting_ptr m_lightingEngine;

  public:
    typedef boost::shared_ptr<raytrace_renderer> ptr_type;

  public:
    static ptr_type create_instance( bool useOpenVDB = false );

  public:
    raytrace_renderer()
        : m_vcs( frantic::graphics::vector3f(), 1.f )
        , m_minStepSize( 1.f )
        , m_maxStepSize( 1.f ) {}

    virtual ~raytrace_renderer() {}

    void set_voxel_length( float voxelLength ) { m_vcs.set_voxel_length( voxelLength ); }

    void set_raymarch_stepsize( float minStep, float maxStep ) {
        m_minStepSize = minStep;
        m_maxStepSize = maxStep;
    }

    void set_ambient_light( const color_type& ambientLight ) { m_ambientLight = ambientLight; }

    /**
     * For ray-tracers that use splatting during the `precompute_lighting` stage, set the splat filter. This has no
     * effect for ray-tracers that don't splat.
     */
    virtual void set_splat_filter( splat_renderer::filter2f_ptr filter ) { m_splatFilter = filter; }

    /**
     * For ray-tracers that use splatting during the `precompute_lighting` stage, set the lighting engine. The lighting
     * engine fills the "Lighting" channel of the particles. This has no effect for ray-tracers that don't splat.
     */
    virtual void set_lighting_engine( splat_renderer::splat_lighting_ptr lightingEngine ) {
        m_lightingEngine = lightingEngine;
    }

    virtual void add_render_element( krakatoa::render_element_interface_ptr renderElement ) = 0;

    virtual void precompute_lighting() = 0;

    virtual void initialize() = 0;

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha ) = 0;

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha, float& nearestDepth, float alphaThreshold = 1e-4f ) = 0;

    virtual void raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1,
                                   alpha_type& accumAlpha ) = 0;

    virtual void render( image_type& outImage ) = 0;
};

typedef raytrace_renderer::ptr_type raytrace_renderer_ptr;

} // namespace raytrace_renderer
} // namespace krakatoa
