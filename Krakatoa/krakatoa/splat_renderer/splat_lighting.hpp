// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/renderer.hpp>
#include <krakatoa/splat_renderer/filter2f.hpp>

#include <frantic/graphics2d/framebuffer.hpp>

namespace krakatoa {
namespace splat_renderer {

/**
 * This interface will populate the "Lighting" channel of each particle with the amount of light that scatters off the
 * particle towards the camera. Also fills in channels used for per-light render elements and per-shader render
 * elements.
 */
class splat_lighting {
  public:
    typedef boost::shared_ptr<splat_lighting> ptr_type;

  protected:
    scene_context_ptr m_sceneContext;

    renderer::shader_ptr_type m_shader;
    bool m_useMixedShaders;
    std::vector<renderer::shader_ptr_type> m_shaders;

    renderer::progress_ptr_type m_progress;

    frantic::graphics2d::draw_point_filter::draw_point_filter_enum m_drawFilterType;

    filter2f_ptr m_splatFilter; // Should replace m_drawFilterType eventually.

    float m_densityScale;

    bool m_disableThreading;

    frantic::channels::channel_const_cvt_accessor<boost::int32_t> m_shaderAccessor;

  public:
    /**
     * Factory function for creating the default implementation of this interface. Useful so the client doesn't have a
     * dependency on the actual implementation type.
     */
    static ptr_type create_instance();

  public:
    splat_lighting()
        : m_useMixedShaders( false ) {}
    virtual ~splat_lighting() {}

    /**
     * @param progress A new progress logging object. Will be updated periodically during a call to
     * compute_particle_lighting()
     */
    virtual void set_progress_logger( renderer::progress_ptr_type progress ) { m_progress = progress; }

    /**
     * @param sceneContext A scene context that supplies the light objects to compute lighting for, as well as the
     * geometry objects that cast shadows (if enabled in the light).
     */
    virtual void set_scene_context( scene_context_ptr sceneContext ) { m_sceneContext = sceneContext; }

    /**
     * @param shader The new shader to use when computing the light that scatters off a particle towards the camera.
     */
    virtual void set_shader( renderer::shader_ptr_type shader ) { m_shader = shader; }

    virtual void set_use_mixed_shaders( bool useMixedShaders, const frantic::channels::channel_map& pcm ) = 0;

    void set_shaders( const std::vector<renderer::shader_ptr_type>& shaders ) {
        m_shaders = shaders;
        if( m_shaders.empty() ) {
            FF_LOG( warning ) << "Shaders list is empty." << std::endl;
        }
    }

    /**
     * @param densityScale A value applied as a scale to the "Density" channel of the particle. Only used for purposes
     * of shadowing.
     */
    virtual void set_density_scale( float densityScale ) { m_densityScale = densityScale; }

    /**
     * @param disableThreading If true, no threading will be disabled for splat lighting.
     */
    virtual void disable_threading( bool disableThreading ) { m_disableThreading = disableThreading; }

    virtual void set_draw_filter_type( frantic::graphics2d::draw_point_filter::draw_point_filter_enum filterType ) {
        m_drawFilterType = filterType;
    }

    virtual void set_splat_filter( filter2f_ptr filter ) { m_splatFilter = filter; }

    /**
     * @param renderElement If this is a subclass of specific_light_render_element, it will be used to determine
     *                      where the per-light lighting should be stored in the particle. All other render elements
     *                      are ignored.
     */
    virtual void add_render_element( renderer::render_element_ptr_type renderElement ) = 0;

    /**
     * Helper function for adding a group of render elements
     */
    template <class ForwardIterator>
    inline void add_render_elements( ForwardIterator begin, ForwardIterator end ) {
        for( ForwardIterator it = begin; it != end; ++it )
            add_render_element( *it );
    }

    /**
     * This function will zero out any existing values in "Lighting" and then compute the amount of light scattered off
     * each particle towards the camera.
     */
    virtual void compute_particle_lighting( renderer::particle_container_type& particles,
                                            bool useAbsorptionChannel ) = 0;
};

typedef splat_lighting::ptr_type splat_lighting_ptr;

} // namespace splat_renderer
} // namespace krakatoa
