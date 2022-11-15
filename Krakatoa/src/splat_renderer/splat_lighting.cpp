// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/light_render_element.hpp>
#include <krakatoa/splat_renderer/default_light_traits.hpp>
#include <krakatoa/splat_renderer/omni_light_traits.hpp>
#include <krakatoa/splat_renderer/splat_lighting.hpp>

#include <frantic/particles/particle_utilities.hpp>

#include <tbb/atomic.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_scan.h>
#include <tbb/tbb_thread.h>

using frantic::graphics::alpha3f;
using frantic::graphics::color3f;
using frantic::graphics::vector3f;

namespace krakatoa {
namespace splat_renderer {

class splat_lighting_impl : public splat_lighting {
    std::vector<renderer::render_element_ptr_type> m_renderElements;

    frantic::diagnostics::profiling_section m_psLightingMatte, m_psLightingSort, m_psLightingAttenuation;

  private:
    template <class LightTraits>
    void compute_lighting( renderer::particle_container_type& particles, light_object_ptr light,
                           bool useAbsorptionChannel );

    template <class LightTraits, class Range>
    void compute_lighting_pass1( const Range& range, typename LightTraits::attenuation_buffer_type& outAttenMap,
                                 light_object& light, renderer::particle_container_type& particles,
                                 bool useAbsorptionChannel );

    template <class LightTraits, class Range>
    void compute_lighting_pass2( const Range& range, typename LightTraits::attenuation_buffer_type& inoutAttenMap,
                                 light_object& light, renderer::particle_container_type& particles,
                                 bool useAbsorptionChannel, const frantic::tstring& thisLightChannel = _T("") );

    template <class LightTraits>
    void compute_deep_attenuation( light_object& light, renderer::particle_container_type& particles );

    template <class LightTraits>
    friend class compute_lighting_body;

  public:
    splat_lighting_impl();

    virtual ~splat_lighting_impl();

    virtual void add_render_element( renderer::render_element_ptr_type renderElement );

    virtual void compute_particle_lighting( renderer::particle_container_type& particles, bool useAbsorptionChannel );

    virtual void set_use_mixed_shaders( bool useMixedShaders, const frantic::channels::channel_map& pcm );
};

splat_lighting_ptr splat_lighting::create_instance() { return splat_lighting_ptr( new splat_lighting_impl ); }

splat_lighting_impl::splat_lighting_impl()
    : m_psLightingMatte( _T("Lighting:Updating Light") )
    , m_psLightingSort( _T("Lighting:Sorting Particles") )
    , m_psLightingAttenuation( _T("Lighting:Calculating") ) {
    m_progress.reset( new frantic::logging::null_render_progress_logger );
    m_densityScale = 1.f;
    m_disableThreading = false;
    m_drawFilterType = frantic::graphics2d::draw_point_filter::bilinear;
    m_shaderAccessor = frantic::channels::channel_const_cvt_accessor<boost::int32_t>( -1 );
}

splat_lighting_impl::~splat_lighting_impl() {}

void splat_lighting_impl::add_render_element( renderer::render_element_ptr_type renderElement ) {
    m_renderElements.push_back( renderElement );
}

namespace {
class clear_lighting_impl {
  private:
    clear_lighting_impl& operator=( const clear_lighting_impl& ) { return *this; } // disabled assignment operator.

  public:
    renderer::particle_container_type& m_particles;
    frantic::channels::channel_cvt_accessor<frantic::graphics::color3h> m_lightingAccessor;

    clear_lighting_impl( renderer::particle_container_type& particles )
        : m_particles( particles ) {
        m_lightingAccessor =
            m_particles.get_channel_map().get_cvt_accessor<frantic::graphics::color3h>( _T("Lighting") );
    }

    void operator()( const tbb::blocked_range<std::size_t>& range ) const {
        for( renderer::particle_container_type::iterator it = m_particles.begin() + range.begin(),
                                                         itEnd = m_particles.begin() + range.end();
             it != itEnd; ++it )
            m_lightingAccessor.set( *it, frantic::graphics::color3h::black() );
    }
};
} // namespace

void splat_lighting_impl::compute_particle_lighting( renderer::particle_container_type& particles,
                                                     bool useAbsorptionChannel ) {
    m_psLightingMatte.reset();
    m_psLightingSort.reset();
    m_psLightingAttenuation.reset();

    if( !m_sceneContext )
        return;

    if( !m_splatFilter )
        m_splatFilter = filter2f::create_instance( _T("Bilinear") );

    m_progress->update_progress( 0.f );

    frantic::channels::channel_cvt_accessor<color3f> lightAccessor =
        particles.get_channel_map().get_cvt_accessor<color3f>( _T("Lighting") );
    frantic::channels::channel_cvt_accessor<float> densityAccessor( 1.f );

    if( particles.get_channel_map().has_channel( _T("Density") ) )
        densityAccessor = particles.get_channel_map().get_cvt_accessor<float>( _T("Density") );

    frantic::channels::channel_general_accessor lightGenAccessor =
        particles.get_channel_map().get_general_accessor( _T("Lighting") );

    // Profiling shows that this is actually super slow!!!! Lighting is almost always using half anyways and the
    // conversion was killing me ... better try using memset(). Also: This should be done using tbb::parallel_for() for(
    // renderer::particle_container_type::iterator it = particles.begin(), itEnd = particles.end(); it != itEnd; ++it )
    // lightAccessor.set( *it, color3f::black() );
    // memset( lightGenAccessor.get_channel_data_pointer( *it ), 0, lightGenAccessor.primitive_size() );
    tbb::parallel_for( tbb::blocked_range<std::size_t>( 0, particles.size(), 100000ull ),
                       clear_lighting_impl( particles ), tbb::auto_partitioner() );

    for( std::size_t lightIndex = 0, numLights = m_sceneContext->get_light_objects().size(); lightIndex < numLights;
         ++lightIndex ) {
        frantic::rendering::lights::lightinterface& light =
            m_sceneContext->get_light_objects().get( (int)lightIndex )->get_light_impl();

        float progressDisplayStart = 100.f * (float)lightIndex / (float)numLights;
        float progressDisplayEnd = 100.f * (float)( lightIndex + 1 ) / (float)numLights;
        frantic::logging::progress_logger_subinterval_tracker plst( *m_progress, progressDisplayStart,
                                                                    progressDisplayEnd );

        if( !light.is_directional_light() )
            compute_lighting<omni_light_traits>( particles, m_sceneContext->get_light_objects().get( (int)lightIndex ),
                                                 useAbsorptionChannel );
        else
            compute_lighting<default_light_traits>(
                particles, m_sceneContext->get_light_objects().get( (int)lightIndex ), useAbsorptionChannel );
    }

    // Move this code to the particle splatting portion, where it can be done in a threaded manner. I'm not currently
    // sure why we care at all anyways.
    //
    //// Check that the lighting isn't invalid anywhere
    // std::size_t invalidLightingCount = 0;
    // for( renderer::particle_container_type::iterator it = particles.begin(), itEnd = particles.end(); it != itEnd;
    // ++it ){ 	color3f lightingVal = lightAccessor.get(*it); 	if( !boost::math::isfinite(lightingVal.r) ||
    //!boost::math::isfinite(lightingVal.g) || !boost::math::isfinite(lightingVal.b) ) {
    //		++invalidLightingCount;
    //		lightAccessor.set(*it, color3f(0));
    //		densityAccessor.set(*it, 0);
    //	}
    // }

    // if( invalidLightingCount > 0 )
    //	FF_LOG(warning) << invalidLightingCount << " particles had invalid lighting.  They have been set to invisible."
    //<< std::endl;

    FF_LOG( stats ) << m_psLightingMatte << '\n' << m_psLightingSort << '\n' << m_psLightingAttenuation << std::endl;
}

template <class LightTraits>
class compute_lighting_body {
    typename LightTraits::attenuation_buffer_type m_attenMap;

    light_object* m_light;
    splat_lighting_impl* m_lightingEngine;
    renderer::particle_container_type* m_particles;
    bool m_useAbsorptionChannel;

    tbb::atomic<std::size_t>* m_progressCounter;
    tbb::tbb_thread::id m_mainThreadId;

    frantic::tstring m_thisLightChannel;

  public:
    compute_lighting_body( splat_lighting_impl& lightingEngine, light_object& light,
                           renderer::particle_container_type& particles )
        : m_lightingEngine( &lightingEngine )
        , m_light( &light )
        , m_particles( &particles ) {
        m_attenMap.set_size( m_light->get_light_impl().shadow_map_size() );
        m_attenMap.fill( renderer::alpha_type( 0 ) );
        m_attenMap.set_pixel_lookup_filter( frantic::graphics2d::pixel_lookup_filter::bicubic_filter );
        m_attenMap.set_draw_point_filter( lightingEngine.m_drawFilterType );
    }

    compute_lighting_body( compute_lighting_body& rhs, tbb::split )
        : m_lightingEngine( rhs.m_lightingEngine )
        , m_light( rhs.m_light )
        , m_particles( rhs.m_particles )
        , m_thisLightChannel( rhs.m_thisLightChannel )
        , m_progressCounter( rhs.m_progressCounter )
        , m_mainThreadId( rhs.m_mainThreadId ) {
        m_attenMap.set_size( m_light->get_light_impl().shadow_map_size() );
        m_attenMap.fill( renderer::alpha_type( 0 ) );
        m_attenMap.set_pixel_lookup_filter( frantic::graphics2d::pixel_lookup_filter::bicubic_filter );
        m_attenMap.set_draw_point_filter( rhs.m_attenMap.get_draw_point_filter() );
    }

    template <class Range, class Tag>
    void operator()( const Range& range, Tag ) {
        if( !Tag::is_final_scan() ) {
            m_lightingEngine->compute_lighting_pass1<LightTraits, Range>( range, m_attenMap, *m_light, *m_particles,
                                                                          m_useAbsorptionChannel );
        } else {
            m_lightingEngine->compute_lighting_pass2<LightTraits, Range>( range, m_attenMap, *m_light, *m_particles,
                                                                          m_useAbsorptionChannel, m_thisLightChannel );
        }

        std::size_t count = m_progressCounter->fetch_and_add( range.size() );

        if( tbb::this_tbb_thread::get_id() == m_mainThreadId )
            m_lightingEngine->m_progress->update_progress( range.size() + count, 2 * m_particles->size() );
    }

    void reverse_join( compute_lighting_body& lhs ) { m_attenMap.blend_under( lhs.m_attenMap ); }

    void assign( compute_lighting_body& ) {}

    void set_light_output_channel( const frantic::tstring& thisLightElementChannel ) {
        m_thisLightChannel = thisLightElementChannel;
    }

    void compute_lighting() {
        tbb::atomic<std::size_t> progressCounter;
        progressCounter = 0;

        m_progressCounter = &progressCounter;
        m_mainThreadId = tbb::this_tbb_thread::get_id();

        tbb::parallel_scan( tbb::blocked_range<std::size_t>( 0, m_particles->size(), 100000 ), *this,
                            tbb::auto_partitioner() );

        m_lightingEngine->m_progress->update_progress( 100.f );
    }
};

template <class LightTraits>
void splat_lighting_impl::compute_lighting( renderer::particle_container_type& particles, light_object_ptr light,
                                            bool useAbsorptionChannel ) {
    const frantic::channels::channel_map& pcm = particles.get_channel_map();

    frantic::tstring lightElementChannelName;

    // TODO: This is a linear search. It doesn't happen too often so its ok for now, but this might be a problem in the
    // future
    for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i ) {
        if( krakatoa::light_render_element* pLightElem =
                dynamic_cast<krakatoa::light_render_element*>( m_renderElements[i].get() ) ) {
            if( pLightElem->get_light_name() == light->get_light_impl().name() ) {
                lightElementChannelName = pLightElem->get_channel_name();
                break;
            }
        }
    }

    FF_LOG( debug ) << _T("splat_lighting_impl::compute_lighting()\n")
                    << _T("light: ") << light->get_light_impl().name() << _T("\n")
                    << _T("resolution: ") << light->get_light_impl().shadow_map_size() << _T("\n")
                    << _T("filter: ") << m_drawFilterType << std::endl;

    {
        m_progress->set_title( _T("Updating: ") + light->get_light_impl().name() );
        frantic::logging::progress_logger_subinterval_tracker plstDepthMap( *m_progress, 0, 25 );

        frantic::diagnostics::scoped_profile spsMatte( m_psLightingMatte );
        light->begin( m_sceneContext );
        light->update( 0.5f );

        m_progress->update_progress( 100.f );
    }

    { //<--Sort the particles-->
        m_progress->set_title( _T("Sorting particles: ") + light->get_light_impl().name() );
        frantic::logging::progress_logger_subinterval_tracker plstSorting( *m_progress, 25, 50 );
        frantic::diagnostics::scoped_profile spsSorting( m_psLightingSort );

        if( light->get_light_impl().is_directional_light() ) {
            frantic::sort::parallel_sort(
                particles.begin(), particles.end(),
                frantic::particles::directed_distance(
                    pcm, static_cast<frantic::rendering::lights::directedlight_interface&>( light->get_light_impl() )
                             .direction() ),
                *m_progress, m_disableThreading );
        } else {
            frantic::sort::parallel_sort( particles.begin(), particles.end(),
                                          frantic::particles::point_distance<frantic::graphics::vector3f>(
                                              pcm, light->get_light_impl().position() ),
                                          *m_progress, m_disableThreading );
        }

        m_progress->update_progress( 100.f );
    }

    //<--Light the particles-->
    bool saveAttenMaps = light->get_light_impl().get_attenuation_saver() != NULL;

    {
        m_progress->set_title( _T("Calculating Lighting: ") + light->get_light_impl().name() );
        frantic::logging::progress_logger_subinterval_tracker plstLightingPass( *m_progress, 50,
                                                                                saveAttenMaps ? 75.f : 100.f );
        frantic::diagnostics::scoped_profile spsAttenuation( m_psLightingAttenuation );

        if( !m_disableThreading ) {
            compute_lighting_body<LightTraits> task( *this, *light, particles );

            task.set_light_output_channel( lightElementChannelName );
            task.compute_lighting();
        } else {
            typename LightTraits::attenuation_buffer_type attenMap;
            attenMap.set_size( light->get_light_impl().shadow_map_size() );
            attenMap.fill( renderer::alpha_type( 0 ) );
            attenMap.set_pixel_lookup_filter( frantic::graphics2d::pixel_lookup_filter::bicubic_filter );
            attenMap.set_draw_point_filter( m_drawFilterType );

            compute_lighting_pass2<LightTraits>( tbb::blocked_range<std::size_t>( 0, particles.size() ), attenMap,
                                                 *light, particles, useAbsorptionChannel, lightElementChannelName );
        }

        m_progress->update_progress( 100.f );
    }

    if( saveAttenMaps ) {
        m_progress->set_title( _T("Calculating Deep Attenuation: ") + light->get_light_impl().name() );
        frantic::logging::progress_logger_subinterval_tracker plstDeepAttenuationPass( *m_progress, 75.f, 100.f );

        light->update( 0.5f );

        compute_deep_attenuation<LightTraits>( *light, particles );

        m_progress->update_progress( 100.f );
    }

    // HACK: This prevents light_object from saving an empty deep atten map.
    // light->end();
}

template <class LightTraits, class Range>
void splat_lighting_impl::compute_lighting_pass1( const Range& range,
                                                  typename LightTraits::attenuation_buffer_type& outAttenMap,
                                                  light_object& light, renderer::particle_container_type& particles,
                                                  bool useAbsorptionChannel ) {
    frantic::graphics::transform4f worldToLightTM = light.get_light_impl().transform_matrix().to_inverse();

    const frantic::channels::channel_map& pcm = particles.get_channel_map();

    frantic::channels::channel_accessor<vector3f> positionAccessor = pcm.get_accessor<vector3f>( _T("Position") );
    frantic::channels::channel_cvt_accessor<color3f> lightingAccessor = pcm.get_cvt_accessor<color3f>( _T("Lighting") );
    frantic::channels::channel_cvt_accessor<float> densityAccessor = pcm.get_cvt_accessor<float>( _T("Density") );

    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::white() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;

    if( pcm.has_channel( _T("Color") ) )
        colorAccessor = pcm.get_cvt_accessor<color3f>( _T("Color") );
    if( useAbsorptionChannel && pcm.has_channel( _T("Absorption") ) )
        absorptionAccessor = pcm.get_cvt_accessor<color3f>( _T("Absorption") );

    renderer::particle_container_type::iterator it = particles.begin() + range.begin();
    renderer::particle_container_type::iterator itEnd = particles.begin() + range.end();

    for( ; it != itEnd; ++it ) {
        char* particle = (char*)( *it );

        frantic::graphics::vector3f worldPos = positionAccessor.get( particle );
        frantic::graphics::vector3f lightSpacePos = worldToLightTM * worldPos;

        typename LightTraits::screen_space_type screenSpacePos;
        if( !LightTraits::get_screen_pos( light.get_light_impl(), lightSpacePos, screenSpacePos ) )
            continue;

        float densityScale = m_densityScale * LightTraits::get_density_scale( light.get_light_impl(), lightSpacePos );
        float negDensity = -( densityAccessor.get( particle ) * densityScale );

        frantic::graphics::alpha3f particleAlpha;
        if( !absorptionAccessor.is_valid() ) {
            particleAlpha.ar = particleAlpha.ag = particleAlpha.ab = 1 - std::exp( negDensity );
        } else {
            frantic::graphics::color3f particleColor = colorAccessor.get( particle );
            frantic::graphics::color3f particleAbsorb = absorptionAccessor.get( particle );

            particleAlpha.ar = 1.f - std::exp( negDensity * ( particleColor.r + particleAbsorb.r ) );
            particleAlpha.ag = 1.f - std::exp( negDensity * ( particleColor.g + particleAbsorb.g ) );
            particleAlpha.ab = 1.f - std::exp( negDensity * ( particleColor.b + particleAbsorb.b ) );
        }

        LightTraits::draw_attenuation( particleAlpha, screenSpacePos, outAttenMap, m_splatFilter );
    }
}

template <class LightTraits, class Range>
void splat_lighting_impl::compute_lighting_pass2( const Range& range,
                                                  typename LightTraits::attenuation_buffer_type& inoutAttenMap,
                                                  light_object& light, renderer::particle_container_type& particles,
                                                  bool useAbsorptionChannel,
                                                  const frantic::tstring& thisLightChannel ) {
    frantic::graphics::transform4f worldToLightTM = light.get_light_impl().transform_matrix().to_inverse();

    const frantic::channels::channel_map& pcm = particles.get_channel_map();

    frantic::channels::channel_accessor<vector3f> positionAccessor = pcm.get_accessor<vector3f>( _T("Position") );
    frantic::channels::channel_cvt_accessor<color3f> lightingAccessor = pcm.get_cvt_accessor<color3f>( _T("Lighting") );
    frantic::channels::channel_cvt_accessor<float> densityAccessor = pcm.get_cvt_accessor<float>( _T("Density") );

    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::white() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;
    frantic::channels::channel_cvt_accessor<color3f> thisLightAccessor;

    if( pcm.has_channel( _T("Color") ) )
        colorAccessor = pcm.get_cvt_accessor<color3f>( _T("Color") );
    if( useAbsorptionChannel && pcm.has_channel( _T("Absorption") ) )
        absorptionAccessor = pcm.get_cvt_accessor<color3f>( _T("Absorption") );
    if( !thisLightChannel.empty() && pcm.has_channel( thisLightChannel ) )
        thisLightAccessor = pcm.get_cvt_accessor<color3f>( thisLightChannel );

    frantic::graphics::camera<float>& camera = m_sceneContext->get_camera();

    renderer::particle_container_type::iterator it = particles.begin() + range.begin();
    renderer::particle_container_type::iterator itEnd = particles.begin() + range.end();

    for( ; it != itEnd; ++it ) {
        char* particle = (char*)( *it );

        frantic::graphics::vector3f worldPos = positionAccessor.get( particle );
        frantic::graphics::vector3f lightSpacePos = worldToLightTM * worldPos;

        typename LightTraits::screen_space_type screenSpacePos;
        if( !LightTraits::get_screen_pos( light.get_light_impl(), lightSpacePos, screenSpacePos ) )
            continue;

        float densityScale = m_densityScale * LightTraits::get_density_scale( light.get_light_impl(), lightSpacePos );
        float negDensity = -( densityAccessor.get( particle ) * densityScale );

        frantic::graphics::alpha3f particleAlpha;
        if( !absorptionAccessor.is_valid() ) {
            particleAlpha.ar = particleAlpha.ag = particleAlpha.ab = 1 - std::exp( negDensity );
        } else {
            frantic::graphics::color3f particleColor = colorAccessor.get( particle );
            frantic::graphics::color3f particleAbsorb = absorptionAccessor.get( particle );

            particleAlpha.ar = 1.f - std::exp( negDensity * ( particleColor.r + particleAbsorb.r ) );
            particleAlpha.ag = 1.f - std::exp( negDensity * ( particleColor.g + particleAbsorb.g ) );
            particleAlpha.ab = 1.f - std::exp( negDensity * ( particleColor.b + particleAbsorb.b ) );
        }

        // TODO: Replace this with a filtering object.
        frantic::graphics::alpha3f lightAtten = inoutAttenMap.get_pixel_filtered( screenSpacePos );

        LightTraits::draw_attenuation( particleAlpha, screenSpacePos, inoutAttenMap, m_splatFilter );

        frantic::graphics::color3f scatterCoeff = colorAccessor.get( particle );
        frantic::graphics::color3f lighting = light.eval_lighting( worldPos );

        frantic::graphics::vector3f toEye =
            frantic::graphics::vector3f::normalize( camera.camera_position( worldPos ) - worldPos );
        frantic::graphics::vector3f toLight =
            frantic::graphics::vector3f::normalize( light.get_light_impl().position( worldPos ) - worldPos );

        lighting = lightAtten.occlude( lighting );
        if( !m_useMixedShaders ) {
            lighting = m_shader->shade( toEye, toLight, lighting, scatterCoeff, particle );
        } else {
            const boost::int32_t shaderIndex = m_shaderAccessor.get( particle );
            if( shaderIndex >= 0 && shaderIndex < m_shaders.size() ) {
                lighting = m_shaders[shaderIndex]->shade( toEye, toLight, lighting, scatterCoeff, particle );
            } else {
                lighting = m_shader->shade( toEye, toLight, lighting, scatterCoeff, particle );
            }
        }

        if( thisLightAccessor.is_valid() )
            thisLightAccessor.set( particle, lighting );

        lighting += lightingAccessor.get( particle );

        lightingAccessor.set( particle, lighting );
    }
}

template <class LightTraits>
void splat_lighting_impl::compute_deep_attenuation( light_object& light,
                                                    renderer::particle_container_type& particles ) {
    frantic::graphics::transform4f worldToLightTM = light.get_light_impl().transform_matrix().to_inverse();

    const frantic::channels::channel_map& pcm = particles.get_channel_map();

    frantic::channels::channel_accessor<vector3f> positionAccessor = pcm.get_accessor<vector3f>( _T("Position") );
    frantic::channels::channel_cvt_accessor<color3f> lightingAccessor = pcm.get_cvt_accessor<color3f>( _T("Lighting") );
    frantic::channels::channel_cvt_accessor<float> densityAccessor = pcm.get_cvt_accessor<float>( _T("Density") );

    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::white() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;

    if( pcm.has_channel( _T("Color") ) )
        colorAccessor = pcm.get_cvt_accessor<color3f>( _T("Color") );
    if( /*m_bUseChannelAbsorption &&*/ pcm.has_channel( _T("Absorption") ) )
        absorptionAccessor = pcm.get_cvt_accessor<color3f>( _T("Absorption") );

    renderer::particle_container_type::iterator it = particles.begin();
    renderer::particle_container_type::iterator itEnd = particles.end();

    std::size_t counter = 0, counterEnd = particles.size();
    for( ; it != itEnd; ++it ) {
        char* particle = (char*)( *it );

        frantic::graphics::vector3f worldPos = positionAccessor.get( particle );
        frantic::graphics::vector3f lightSpacePos = worldToLightTM * worldPos;

        typename LightTraits::screen_space_type screenSpacePos;
        if( !LightTraits::get_screen_pos( light.get_light_impl(), lightSpacePos, screenSpacePos ) )
            continue;

        float densityScale = m_densityScale * LightTraits::get_density_scale( light.get_light_impl(), lightSpacePos );
        float negDensity = -( densityAccessor.get( particle ) * densityScale );

        frantic::graphics::alpha3f particleAlpha;
        if( !absorptionAccessor.is_valid() ) {
            particleAlpha.ar = particleAlpha.ag = particleAlpha.ab = 1 - std::exp( negDensity );
        } else {
            frantic::graphics::color3f particleColor = colorAccessor.get( particle );
            frantic::graphics::color3f particleAbsorb = absorptionAccessor.get( particle );

            particleAlpha.ar = 1.f - std::exp( negDensity * ( particleColor.r + particleAbsorb.r ) );
            particleAlpha.ag = 1.f - std::exp( negDensity * ( particleColor.g + particleAbsorb.g ) );
            particleAlpha.ab = 1.f - std::exp( negDensity * ( particleColor.b + particleAbsorb.b ) );
        }

        LightTraits::add_deep_attenuation_sample( particleAlpha, screenSpacePos, -lightSpacePos.z,
                                                  light.get_light_impl().get_attenuation_saver() );

        m_progress->update_progress( ++counter, counterEnd );
    }

    light.get_light_impl().get_attenuation_saver()->write_file();
}

void splat_lighting_impl::set_use_mixed_shaders( bool useMixedShaders, const frantic::channels::channel_map& pcm ) {
    m_useMixedShaders = useMixedShaders;
    if( useMixedShaders ) {
        if( pcm.has_channel( _T( "PhaseFunction" ) ) ) {
            m_shaderAccessor = pcm.get_const_cvt_accessor<boost::int32_t>( _T( "PhaseFunction" ) );
        }

        if( m_shaderAccessor.is_default() || !m_shaderAccessor.is_valid() ) {
            FF_LOG( warning )
                << "Mixed-shader mode is enabled, but the \"PhaseFunction\" channel is missing or invalid."
                << std::endl;
        }
    }
}

} // namespace splat_renderer
} // namespace krakatoa
