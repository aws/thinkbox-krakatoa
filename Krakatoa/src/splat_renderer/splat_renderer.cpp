// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <algorithm>
#include <cmath>

#include <boost/algorithm/clamp.hpp>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread/tss.hpp>
#include <boost/unordered_set.hpp>

#include <frantic/rendering/depthbuffer_singleface.hpp>
#include <frantic/shading/highlights.hpp>
#include <frantic/sort/sort.hpp>

#include <krakatoa/matte_zdepth_render_element.hpp>
#include <krakatoa/min_color6f.hpp>
#include <krakatoa/occluded_layer_render_element.hpp>
#include <krakatoa/splat_renderer/parallel_progress.hpp>
#include <krakatoa/splat_renderer/splat_renderer.hpp>
#include <krakatoa/threading_functions.hpp>
#include <krakatoa/zdepth_render_element.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 4100 )
#include <tbb/atomic.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/tbb_thread.h>
#include <tbb/parallel_reduce.h>

using frantic::graphics::alpha3f;
using frantic::graphics::color3f;
using frantic::graphics::transform4f;
using frantic::graphics::vector3;
using frantic::graphics::vector3f;

using frantic::graphics2d::vector2;
using frantic::graphics2d::vector2f;

namespace krakatoa {
namespace splat_renderer {

class splat_renderer_impl : public splat_renderer {
    typedef std::vector<particle_render_element_interface_ptr> render_element_container_type;

    render_element_container_type m_renderElements;

    boost::shared_ptr<occluded_layer_render_element> m_behindMatteElement;

    boost::shared_ptr<matte_zdepth_render_element> m_matteZDepthElement;

    frantic::graphics::camera<float> m_camera;

    frantic::rendering::depthbuffer_singleface m_matteDepthmap;

    bool m_applyAreaDifferential;
    float m_areaDifferentialMax;

    float m_mblurTime;
    float m_mblurTimeSpan;

    bool m_doDOF;

    frantic::tstring
        m_progressTitle; // Set this member so that we can build a string for displaying in the progress bar.

    frantic::diagnostics::profiling_section m_psRenderMatte, m_psRenderSorting, m_psRenderDrawing;

    typedef boost::mt19937 rng_gen_type;
    typedef boost::uniform_01<float> rng_range_type;
    typedef boost::variate_generator<rng_gen_type, rng_range_type> rng_type;

  protected:
    std::vector<renderer::image_type> m_frameBuffers;
    std::vector<render_element_container_type> m_renderElementsByPartition;
    std::vector<render_element_interface_ptr> m_occludedElementByPartition;

  private:
    class parallel_render_subinterval {
      public:
        splat_renderer_impl* m_renderer;

        int m_seed;
        mutable rng_type m_rng;
        parallel_render_progress_logger_master* m_masterLogger;
        const boost::shared_array<std::size_t> m_partitionStarts;

        std::size_t m_numThreads;

      public:
        parallel_render_subinterval( splat_renderer_impl& renderer, int seed,
                                     parallel_render_progress_logger_master* masterLogger,
                                     const boost::shared_array<std::size_t> partitionSizes )
            : m_renderer( &renderer )
            , m_rng( rng_gen_type( seed ), rng_range_type() )
            , m_seed( seed )
            , m_masterLogger( masterLogger )
            , m_partitionStarts( partitionSizes ) {}

        void render_subinterval( std::size_t threadCount, render_element_container_type& renderElements,
                                 render_element_interface_ptr occludedElement, bool withThreads = true ) {
            if( withThreads ) {
                m_numThreads = threadCount;

                // This parallel_for absolutely MUST use tbb::simple_partitioner with grainsize = 1
                // We are calculating the range of particles to render manually so the number of threads calling
                // operator() must be deterministic and must be >= threadCount
                tbb::parallel_for( tbb::blocked_range<std::size_t>( 0, threadCount, 1 ), *this,
                                   tbb::simple_partitioner() );
            } else {
                m_numThreads = 1;
                ( *this )( tbb::blocked_range<std::size_t>( 0, 1, 1 ) );
            }
        }

        inline void operator()( const tbb::blocked_range<std::size_t>& range ) const {
            const std::size_t threadIndex = range.begin();

            const std::size_t particleStart = m_partitionStarts[threadIndex];
            const std::size_t particleEnd =
                threadIndex == m_numThreads - 1 ? m_renderer->m_particles->size() : m_partitionStarts[threadIndex + 1];

            tbb::blocked_range<std::size_t> particleRange( particleStart, particleEnd, 1 );

            parallel_render_progress_logger logger( *m_masterLogger, ( m_renderer->m_particles->size() / 100 ) );

            m_renderer->render_subinterval_impl(
                m_renderer->m_frameBuffers[threadIndex],
                m_renderer->m_occludedElementByPartition.size() > threadIndex &&
                        m_renderer->m_occludedElementByPartition[threadIndex]
                    ? &( m_renderer->m_occludedElementByPartition[threadIndex]->get_framebuffer() )
                    : NULL,
                m_renderer->m_renderElementsByPartition[threadIndex], m_rng, particleRange, logger );

            logger.update_image( m_renderer->m_frameBuffers[threadIndex], threadIndex );
        }
    };

    // Used for weighting each particle based on the number of bokeh samples it will have.
    struct parallel_particle_weighting_body {
        std::size_t m_totalSamples;
        std::size_t m_threadCount;
        const splat_renderer_impl* m_renderer;
        std::vector<std::size_t>& m_samplesPerParticle;

        parallel_particle_weighting_body( std::size_t threadCount, const splat_renderer_impl* renderer,
                                          std::vector<std::size_t>& samplesPerParticle )
            : m_threadCount( threadCount )
            , m_renderer( renderer )
            , m_samplesPerParticle( samplesPerParticle )
            , m_totalSamples( 0 ) {}

        parallel_particle_weighting_body( parallel_particle_weighting_body& other, tbb::split )
            : m_threadCount( other.m_threadCount )
            , m_renderer( other.m_renderer )
            , m_samplesPerParticle( other.m_samplesPerParticle )
            , m_totalSamples( 0 ) {}

        ~parallel_particle_weighting_body() {}

        void operator()( const tbb::blocked_range<std::size_t>& range ) {
            for( std::size_t index = range.begin(); index < range.end(); ++index ) {
                bool isValid = true;

                const vector3f position =
                    m_renderer->m_accessors.posAccessor.get( m_renderer->m_particles->at( index ) );
                const vector3f velocity =
                    m_renderer->m_accessors.velAccessor.is_valid()
                        ? m_renderer->m_accessors.velAccessor.get( m_renderer->m_particles->at( index ) )
                        : vector3f( 0.0f );
                const float currentParticleTime =
                    m_renderer->m_accessors.timeAccessor.get( m_renderer->m_particles->at( index ) );

                const float currentParticleTimeSeconds =
                    ( -0.5f * ( 1 - m_renderer->m_mblurBias ) + currentParticleTime ) * m_renderer->m_mblurDuration;

                const transform4f worldToCameraTM = m_renderer->m_camera.world_transform_inverse( currentParticleTime );

                const vector3f pos = position + currentParticleTimeSeconds * velocity;
                const vector3f camSpacePos = worldToCameraTM * pos;

                const vector2f screenPt2d = m_renderer->m_camera.from_cameraspace_position( camSpacePos, isValid );
                const vector3f screenPt( screenPt2d.x, screenPt2d.y, -camSpacePos.z );
                vector2f pixelRadii;
                m_renderer->m_camera.dof_ellipse_of_confusion_pixel_radii( screenPt.z, pixelRadii.x, pixelRadii.y );

                // Compute the number of samples we want to do, based on the radius
                const std::size_t depthOfFieldSampleCount = static_cast<std::size_t>(
                    ceil( M_PI * pixelRadii.x * pixelRadii.y * m_renderer->m_dofSampleRate + 0.5f ) );

                if( isValid ) {
                    m_totalSamples += depthOfFieldSampleCount;
                    m_samplesPerParticle[index] = depthOfFieldSampleCount;
                } else {
                    ++m_totalSamples;
                    m_samplesPerParticle[index] = 1;
                }
            }
        }

        void join( parallel_particle_weighting_body& other ) { m_totalSamples += other.m_totalSamples; }
    };

  private:
    virtual void render_subinterval( image_type& outImage, int seed );

    virtual void render_background( image_type& outImage );

    void render_subinterval_impl( image_type& outImage, image_type* occludedImage,
                                  render_element_container_type& outElements, rng_type& rng,
                                  const tbb::blocked_range<std::size_t>& range,
                                  parallel_render_progress_logger& logger );

    void set_jittered_mblur_time( int seed );

    void sort_particles_for_camera();

    void do_splat( image_type& outImage, image_type* occludedImage, rng_type& rng,
                   const frantic::graphics::vector3f screenPos, const pixel_type& splat, const char* particle,
                   bool useWeights = true );

    void do_splat_impl( image_type& outImage, image_type* occludedImage, const frantic::graphics::vector3f screenPos,
                        const pixel_type& splat, bool useWeights );

    struct {
        frantic::channels::channel_accessor<vector3f> posAccessor;
        frantic::channels::channel_cvt_accessor<vector3f> velAccessor;
        frantic::channels::channel_cvt_accessor<vector3f> normalAccessor;
        frantic::channels::channel_cvt_accessor<color3f> colorAccessor;
        frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;
        frantic::channels::channel_cvt_accessor<color3f> emissionAccessor;
        frantic::channels::channel_cvt_accessor<color3f> lightingAccessor;
        frantic::channels::channel_cvt_accessor<float> densityAccessor;
        frantic::channels::channel_cvt_accessor<float> timeAccessor;
        frantic::channels::channel_cvt_accessor<float> reflectionAccessor;
        frantic::channels::channel_cvt_accessor<float> bokehBlendAccessor;
        bool doReflectionStrengthScale;

        void reset( const frantic::channels::channel_map& pcm, float mblurTime, float reflectionStrength,
                    float bokehBlendAmount ) {
            posAccessor = pcm.get_accessor<vector3f>( _T("Position") );

            velAccessor.reset( vector3f( 0.f ) );
            normalAccessor.reset( vector3f( 0.f ) );
            colorAccessor.reset( color3f::white() );
            absorptionAccessor.reset();
            emissionAccessor.reset( color3f::black() );
            lightingAccessor.reset( color3f::black() );
            densityAccessor.reset( 1.f );
            timeAccessor.reset( mblurTime );
            reflectionAccessor.reset( reflectionStrength );
            doReflectionStrengthScale = false;
            bokehBlendAccessor.reset( bokehBlendAmount );
        }
    } m_accessors;

  public:
    splat_renderer_impl();

    virtual ~splat_renderer_impl();

    virtual void apply_area_differential( bool enabled = true, float maxValue = FLT_MAX );

    virtual void add_render_element( render_element_interface_ptr renderElement );

    virtual void precompute_lighting();

    virtual void render( image_type& outImage );

  protected:
    virtual std::size_t get_num_output_images();
};

splat_renderer_ptr splat_renderer::create_instance() { return splat_renderer_ptr( new splat_renderer_impl ); }

splat_renderer_impl::splat_renderer_impl()
    : m_psRenderMatte( _T("Rendering:Matte") )
    , m_psRenderSorting( _T("Rendering:Sorting") )
    , m_psRenderDrawing( _T("Rendering:Drawing") ) {
    m_mblurTime = 0.5f;
    m_mblurTimeSpan = 1.f;
    m_doDOF = false;
    m_applyAreaDifferential = true;
    m_areaDifferentialMax = FLT_MAX;
    m_numRenderPasses = 1;
}

splat_renderer_impl::~splat_renderer_impl() {}

void splat_renderer_impl::apply_area_differential( bool enabled, float maxValue ) {
    m_applyAreaDifferential = enabled;
    m_areaDifferentialMax = maxValue;
}

void splat_renderer_impl::add_render_element( render_element_interface_ptr renderElement ) {
    if( boost::shared_ptr<occluded_layer_render_element> occludedElement =
            boost::dynamic_pointer_cast<occluded_layer_render_element>( renderElement ) ) {
        ++m_numRenderPasses;
        m_behindMatteElement = occludedElement;
    } else if( boost::shared_ptr<matte_zdepth_render_element> matteElement =
                   boost::dynamic_pointer_cast<matte_zdepth_render_element>( renderElement ) ) {
        ++m_numRenderPasses;
        m_matteZDepthElement = matteElement;
    } else if( particle_render_element_interface_ptr particleElement =
                   boost::dynamic_pointer_cast<particle_render_element_interface>( renderElement ) ) {
        ++m_numRenderPasses;
        m_renderElements.push_back( particleElement );
    } else {
        FF_LOG( warning ) << _T("Discarded an unknown render element") << std::endl;
    }
}

void splat_renderer_impl::precompute_lighting() {
    if( !m_particles || !m_sceneContext || !m_lightingEngine || !m_shader )
        return;

    // Set a default splat filter if the user hasn't
    if( !m_splatFilter )
        m_splatFilter = filter2f::create_instance( _T("Bilinear") );

    // Set a default progress logger if the user hasn't
    if( !m_progress )
        m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_shader->set_channel_map( m_particles->get_channel_map() );
    for( std::vector<shader_ptr_type>::iterator it = m_shaders.begin(); it != m_shaders.end(); ++it ) {
        ( *it )->set_channel_map( m_particles->get_channel_map() );
    }

    // Cache the camera for easier reference (ie. m_camera vs. m_sceneContext->get_camera())
    m_camera = m_sceneContext->get_camera();

    m_lightingEngine->set_density_scale( m_lightDensityScale );
    m_lightingEngine->set_progress_logger( m_progress );
    m_lightingEngine->set_scene_context( m_sceneContext );
    if( m_useMixedShaders ) {
        m_lightingEngine->set_use_mixed_shaders( true, m_particles->get_channel_map() );
        m_lightingEngine->set_shaders( m_shaders );
    }
    m_lightingEngine->set_shader( m_shader );

    m_lightingEngine->add_render_elements( m_renderElements.begin(), m_renderElements.end() );

    // m_lightingEngine->set_rendering_thread_limit( m_renderingThreadLimit );
    // m_lightingEngine->set_atten_map_avail_memory_fraction( m_frameBufferAvailableMemoryFraction );

    m_lightingEngine->compute_particle_lighting( *m_particles, m_useAbsorptionChannel );
}

void splat_renderer_impl::render( image_type& outImage ) {
    if( !m_particles || !m_sceneContext || ( !m_shader && ( !m_useMixedShaders || m_shaders.empty() ) ) )
        return;

    // Set a default splat filter if the user hasn't
    if( !m_splatFilter )
        m_splatFilter = filter2f::create_instance( _T( "Bilinear" ) );

    // Set a default progress logger if the user hasn't
    if( !m_progress )
        m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_shader->set_channel_map( m_particles->get_channel_map() );
    for( std::vector<shader_ptr_type>::iterator it = m_shaders.begin(); it != m_shaders.end(); ++it ) {
        ( *it )->set_channel_map( m_particles->get_channel_map() );
    }

    m_progressTitle = _T( "Rendering" );
    m_progress->set_title( m_progressTitle );
    // m_progress->set_title("Rendering");

    // Cache the cameBra for easier reference (ie. m_camera vs. m_sceneContext->get_camera())
    m_camera = m_sceneContext->get_camera();
    m_doDOF = m_camera.has_dof() && m_dofSampleRate > 0 && m_dofEnabled;

    for( render_element_container_type::iterator it = m_renderElements.begin(), itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        // Need to set the size here, since we call clone() below, which needs the size set.
        it->get()->get_framebuffer().set_size( m_camera.get_output_size() );
        it->get()->get_framebuffer().fill( pixel_type( 0 ) );
        it->get()->set_channel_map( m_particles->get_channel_map() );
        it->get()->initialize();
    }

    if( m_behindMatteElement ) {
        m_behindMatteElement->get_framebuffer().set_size( m_camera.get_output_size() );
        m_behindMatteElement->initialize();
    }

    if( m_matteZDepthElement ) {
        m_matteZDepthElement->get_depthbuffer().set_size( m_camera.get_output_size() );
        m_matteZDepthElement->initialize();
    }

    if( m_adaptiveMblur && m_particles->get_channel_map().has_channel( _T( "Velocity" ) ) ) {
        m_mblurSamples = get_adaptive_motion_blur_passes();
        FF_LOG( stats ) << "Adaptive motion blur is on. Using " << m_mblurSamples << " motion blur samples."
                        << std::endl;
    }

    if( m_mblurSamples <= 0 ) {
        m_mblurTime = 0.5f;
        m_mblurTimeSpan = 0.f;
        render_subinterval( outImage, 42 );
    } else if( m_mblurSamples == 1 ) {
        m_mblurTime = 0.5f;
        m_mblurTimeSpan = m_mblurJittered ? 1.f : 0.f;
        render_subinterval( outImage, 42 );
    } else {
        // Create accumulation buffers for accumulating motion blur samples
        image_type accumRenderBuffer;
        accumRenderBuffer.set_size( m_camera.get_output_size() );
        accumRenderBuffer.fill( pixel_type( 0 ) );

        std::vector<boost::shared_ptr<krakatoa::render_element_interface>> accumRenderElements;
        for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i )
            accumRenderElements.push_back(
                boost::shared_ptr<krakatoa::render_element_interface>( m_renderElements[i]->clone() ) );

        boost::shared_ptr<krakatoa::render_element_interface> accumBehindMatteRenderElement;
        if( m_behindMatteElement )
            accumBehindMatteRenderElement.reset( m_behindMatteElement->clone() );

        // Don't need to accumulate for the matte zDepth since it isn't composited like normal elements. It uses a min
        // operation, which makes it commutative.

        for( int segment = 0; segment < m_mblurSamples; ++segment ) {
            m_mblurTime = ( (float)segment + 0.5f ) / (float)m_mblurSamples;
            m_mblurTimeSpan = m_mblurJittered ? ( 1.f / (float)m_mblurSamples ) : 0.f;

            frantic::logging::progress_logger_subinterval_tracker plst(
                *m_progress, (float)segment / (float)m_mblurSamples * 100.f,
                (float)( segment + 1 ) / (float)m_mblurSamples * 100.f );

            m_progressTitle = _T( "Rendering pass " ) + boost::lexical_cast<frantic::tstring>( segment + 1 ) +
                              _T( " of " ) + boost::lexical_cast<frantic::tstring>( m_mblurSamples );
            m_progress->set_title( m_progressTitle );
            // m_progress->set_title("Rendering motion blur pass " + boost::lexical_cast<std::string>(segment+1) + " of
            // " + boost::lexical_cast<std::string>(m_mblurSamples));

            if( segment > 0 ) {
                // Should this be done in render_subinterval() instead of here?
                outImage.fill( pixel_type() );
                for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i )
                    m_renderElements[i]->get_framebuffer().fill( pixel_type() );
                if( m_behindMatteElement )
                    m_behindMatteElement->get_framebuffer().fill( pixel_type() );
            }

            render_subinterval( outImage, 42 + 67 * segment );

#if defined( KRAKATOA_RT_CALLBACKS )
            if( m_mblurPassCallback ) {
                if( m_watermarkFn ) {
                    m_watermarkFn( outImage );
                    if( m_behindMatteElement )
                        m_watermarkFn( m_behindMatteElement->get_framebuffer() );
                    for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i ) {
                        m_watermarkFn( m_renderElements[i]->get_framebuffer() );
                    }
                }

                boost::optional<mblur_data::out_image_t> behindMatte;
                if( m_behindMatteElement ) {
                    behindMatte = m_behindMatteElement->get_framebuffer();
                }

                mblur_data mblurData( m_mblurTime, outImage, behindMatte, m_renderElements, m_mblurSamples );
                ( *m_mblurPassCallback )( mblurData );
            }
#endif

            outImage.apply_gain( 1.f / (float)m_mblurSamples );
            accumRenderBuffer.add_image_data( outImage );

            for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i ) {
                if( m_renderElements[i]->use_minimize_compositing() ) {
                    accumRenderElements[i]->get_framebuffer().apply_function<min_color6f>(
                        m_renderElements[i]->get_framebuffer(), min_color6f() );
                } else {
                    m_renderElements[i]->get_framebuffer().apply_gain( 1.f / (float)m_mblurSamples );
                    accumRenderElements[i]->get_framebuffer().add_image_data( m_renderElements[i]->get_framebuffer() );
                }
            }

            if( m_behindMatteElement ) {
                m_behindMatteElement->get_framebuffer().apply_gain( 1.f / (float)m_mblurSamples );
                accumBehindMatteRenderElement->get_framebuffer().add_image_data(
                    m_behindMatteElement->get_framebuffer() );
            }

            if( m_watermarkFn )
                m_watermarkFn( accumRenderBuffer );
            m_progress->update_frame_buffer( accumRenderBuffer );
        }

        accumRenderBuffer.swap( outImage );

        for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i )
            accumRenderElements[i]->get_framebuffer().swap( m_renderElements[i]->get_framebuffer() );

        if( m_behindMatteElement )
            accumBehindMatteRenderElement->get_framebuffer().swap( m_behindMatteElement->get_framebuffer() );
    }

    // Apply final watermarks.
    if( m_watermarkFn )
        m_watermarkFn( outImage );

    // outImage.fill_under( m_backgroundColor ); replaced by render_background()

    for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i ) {
        if( m_watermarkFn )
            m_watermarkFn( m_renderElements[i]->get_framebuffer() );
        m_renderElements[i]->commit();
    }

    if( m_matteZDepthElement )
        m_matteZDepthElement->commit();

    if( m_behindMatteElement ) {
        if( m_watermarkFn )
            m_watermarkFn( m_behindMatteElement->get_framebuffer() );
        m_behindMatteElement->commit();
    }

    FF_LOG( stats ) << m_psRenderMatte << '\n' << m_psRenderSorting << '\n' << m_psRenderDrawing << std::endl;
}

void splat_renderer_impl::render_subinterval( image_type& outImage, int seed ) {
    set_jittered_mblur_time( seed );

    sort_particles_for_camera();

    // Sorting is first 25% of rendering, last 75% is drawing.
    frantic::logging::progress_logger_subinterval_tracker siDrawing( *m_progress, 25, 75 );
    m_progress->set_title( m_progressTitle + _T(": Drawing ") +
                           frantic::strings::int_to_comma_seperated_string( (int)m_particles->size() ) +
                           _T(" particles") );

    m_matteDepthmap.set_size( m_matteSuperSampling * m_camera.get_output_size() );

    if( m_matteSampler ) {
        frantic::diagnostics::scoped_profile spsRenderMatte( m_psRenderMatte );
        m_matteSampler->generate_depth_map( m_sceneContext->get_camera(), m_mblurTime, m_matteDepthmap.as_framebuffer(),
                                            false );
    } else if( m_matteGenerator ) {
        m_matteGenerator.get()( m_mblurTime, m_matteDepthmap.as_framebuffer(), m_matteSuperSampling );
    }

    if( m_matteZDepthElement )
        m_matteZDepthElement->get_depthbuffer().resample_combine( m_matteDepthmap );

    const frantic::channels::channel_map& pcm = m_particles->get_channel_map();

    m_accessors.reset( pcm, m_mblurTime, m_reflectionStrength, bokeh_blend_amount().get_value_or( 1.0f ) );

    if( pcm.has_channel( _T("Velocity") ) )
        m_accessors.velAccessor = pcm.get_cvt_accessor<vector3f>( _T("Velocity") );

    if( pcm.has_channel( _T("Normal") ) )
        m_accessors.normalAccessor = pcm.get_cvt_accessor<vector3f>( _T("Normal") );

    if( pcm.has_channel( _T("Color") ) )
        m_accessors.colorAccessor = pcm.get_cvt_accessor<color3f>( _T("Color") );

    if( m_useAbsorptionChannel && pcm.has_channel( _T("Absorption") ) )
        m_accessors.absorptionAccessor = pcm.get_cvt_accessor<color3f>( _T("Absorption") );

    if( m_useEmissionChannel && pcm.has_channel( _T("Emission") ) )
        m_accessors.emissionAccessor = pcm.get_cvt_accessor<color3f>( _T("Emission") );

    if( pcm.has_channel( _T("Lighting") ) )
        m_accessors.lightingAccessor = pcm.get_cvt_accessor<color3f>( _T("Lighting") );

    if( pcm.has_channel( _T("Density") ) )
        m_accessors.densityAccessor = pcm.get_cvt_accessor<float>( _T("Density") );

    if( m_mblurJittered && pcm.has_channel( _T("MBlurTime") ) )
        m_accessors.timeAccessor = pcm.get_cvt_accessor<float>( _T("MBlurTime") );

    if( pcm.has_channel( _T( "ReflectionStrength" ) ) ) {
        m_accessors.reflectionAccessor = pcm.get_cvt_accessor<float>( _T( "ReflectionStrength" ) );
        m_accessors.doReflectionStrengthScale = true;
    }

    if( pcm.has_channel( _T( "BokehBlendInfluence" ) ) ) {
        m_accessors.bokehBlendAccessor = pcm.get_cvt_accessor<float>( _T( "BokehBlendInfluence" ) );
    }

    frantic::diagnostics::scoped_profile spsRenderDrawing( m_psRenderDrawing );

    // Get the maximum number of threads we can use based on the size of the frame buffers per thread, and the amount of
    // physical memory
    const std::size_t threadCount =
        get_max_threads( outImage.width(), outImage.height(), get_num_output_images(), m_particles->size_in_memory(),
                         sizeof( pixel_type ), m_frameBufferAvailableMemoryFraction, m_renderingThreadLimit );

    boost::shared_array<std::size_t> partitionStarts( new std::size_t[threadCount] );

    if( m_doDOF && m_particles->size() > threadCount && threadCount > 1 ) {
        // If depth of field is turned on, we have to render several splats per particle.
        // Because this number of splats is not consistent between particles
        // and is largely dependent on the particle's distance from the camera,
        // we need to take the number of bokeh samples into account when partitioning the particles to avoid a "last
        // bucket standing" issue.

        // Above, particles were partitioned into their default partitions.
        // The following will adjust these partitions to be more equal with respect to the number of splats that will
        // actually need to be drawn.

        // The first step is to determine the number of samples each particle will have, that is, the "weight" of each
        // particle.

        // Let's do this in parallel...
        std::vector<std::size_t> samplesPerParticle( m_particles->size() );
        parallel_particle_weighting_body weightingFn( threadCount, this, samplesPerParticle );
        tbb::parallel_reduce( tbb::blocked_range<std::size_t>( 0, m_particles->size() ), weightingFn );

        const std::size_t totalSamples = weightingFn.m_totalSamples;
        std::size_t remainingSamples = totalSamples;
        std::size_t remainingPartitions = static_cast<std::size_t>( threadCount );

        // At this point, samplesPerParticle and totalSamples have all been filled out for the default partitioning.
        // The next step is to adjust the partition sizes to be more even with respect to the number fo bokeh samples in
        // each partition.

        std::size_t idealSamplesPerPartition = totalSamples / static_cast<std::size_t>( threadCount );
        std::size_t currentPartition = 1;
        std::size_t currentPartitionSamples = 0;
        partitionStarts[0] = 0;
        std::size_t currentParticleIndex = 0;
        FF_LOG( stats ) << "Partition 0 starts at particle 0." << std::endl;
        for( std::vector<std::size_t>::const_iterator it = samplesPerParticle.begin(); it != samplesPerParticle.end();
             ++it, ++currentParticleIndex ) {
            currentPartitionSamples += *it;
            remainingSamples -= *it;
            if( currentPartitionSamples >= idealSamplesPerPartition ) {
                partitionStarts[currentPartition] = currentParticleIndex + 1;
                FF_LOG( stats ) << "Partition " << currentPartition << " starts at particle "
                                << currentParticleIndex + 1 << "." << std::endl;
                ++currentPartition;
                --remainingPartitions;
                currentPartitionSamples = 0;
                if( currentPartition == threadCount ) {
                    break;
                }
                idealSamplesPerPartition = remainingSamples / remainingPartitions;
            }
        }
    } else {
        const std::size_t partitionSize = m_particles->size() / threadCount;
        for( std::size_t i = 0; i < threadCount; ++i ) {
            partitionStarts[i] = i * partitionSize;
        }
    }

    parallel_render_progress_logger_master masterLogger( m_progress, m_particles->size(), m_watermarkFn );
    parallel_render_subinterval fn( *this, seed, &masterLogger, partitionStarts );

    // Create copies of the frame buffers for each channel for each thread
    for( int i = 0; i < threadCount; ++i ) {
        renderer::image_type frameBuffer;
        m_frameBuffers.push_back( frameBuffer );
        m_frameBuffers[i].set_size( outImage.size() );
        m_frameBuffers[i].fill( renderer::pixel_type( 0 ) );
        if( m_behindMatteElement ) {
            m_occludedElementByPartition.push_back(
                krakatoa::render_element_interface_ptr( m_behindMatteElement->clone() ) );
        }
        m_renderElementsByPartition.push_back( render_element_container_type() );
        for( render_element_container_type::iterator it = m_renderElements.begin(), itEnd = m_renderElements.end();
             it != itEnd; ++it ) {
            m_renderElementsByPartition[i].push_back( particle_render_element_interface_ptr(
                static_cast<krakatoa::particle_render_element_interface*>( it->get()->clone() ) ) );
        }
    }

    // Actually do the render
    fn.render_subinterval( threadCount, m_renderElements, m_behindMatteElement, !m_disableThreading );

    // Merge the copies of the frame buffers for each thread back into the originals
    for( int i = 0; i < threadCount; ++i ) {
        outImage.blend_over( m_frameBuffers[i] );
        m_frameBuffers[i].clear();
        if( m_behindMatteElement ) {
            m_behindMatteElement->combine( m_occludedElementByPartition[i].get() );
        }
        for( render_element_container_type::iterator it = m_renderElements.begin(),
                                                     it2 = m_renderElementsByPartition[i].begin(),
                                                     itEnd = m_renderElements.end();
             it != itEnd; ++it, ++it2 ) {
            it->get()->combine( it2->get() );
        }

        masterLogger.update_progress_and_image( 0, outImage, i );
    }

    m_renderElementsByPartition.clear();
    m_occludedElementByPartition.clear();

    render_background( outImage );

    return;
}

void splat_renderer_impl::render_subinterval_impl( image_type& outImage, image_type* occludedImage,
                                                   render_element_container_type& outElements, rng_type& rng,
                                                   const tbb::blocked_range<std::size_t>& range,
                                                   parallel_render_progress_logger& progressLogger ) {
    bool doEnvRefl = m_environment && m_enableReflections;

    for( particle_container_type::iterator it = m_particles->begin() + range.begin(),
                                           itEnd = m_particles->begin() + range.end();
         it != itEnd; ++it ) {
        char* p = *it;

        float currentParticleTime = m_accessors.timeAccessor.get( p );
        float currentParticleTimeSeconds = ( -0.5f * ( 1 - m_mblurBias ) + currentParticleTime ) * m_mblurDuration;

        transform4f worldToCameraTM = m_camera.world_transform_inverse( currentParticleTime );

        vector3f pos = m_accessors.posAccessor.get( p ) + currentParticleTimeSeconds * m_accessors.velAccessor.get( p );
        vector3f rayStart = m_camera.camera_position( currentParticleTime, pos ); // WorldSpace
        vector3f camSpacePos = worldToCameraTM * pos;                             // CameraSpace

        // Do not process particles outside of the clipping frustrum
        if( ( -camSpacePos.z < m_camera.near_distance() &&
              m_camera.projection_mode() != frantic::graphics::projection_mode::spherical ) ||
            -camSpacePos.z > m_camera.far_distance() )
            continue;

        // float areaDifferential = m_applyAreaDifferential ?
        // m_camera.area_differential_from_cameraspace_position(camSpacePos) : 1.f; float areaDifferential =
        // m_camera.area_differential_from_cameraspace_position(camSpacePos); if( !m_applyAreaDifferential &&
        // areaDifferential > 10.f ) 	areaDifferential = 10.f;
        float areaDifferential = 1.f;
        if( m_applyAreaDifferential ) {
            areaDifferential = m_camera.area_differential_from_cameraspace_position( camSpacePos );
            if( areaDifferential > m_areaDifferentialMax )
                areaDifferential = m_areaDifferentialMax;
        }

        float density = m_accessors.densityAccessor.get( p );
        float effectiveDensity = density * m_cameraDensityScale;
        color3f particleColor = m_accessors.colorAccessor.get( p );
        // color3f particleAbsorb = absorptionAccessor.get(p);
        color3f lighting = m_accessors.lightingAccessor.get( p );
        color3f emission = m_cameraEmissionScale * m_accessors.emissionAccessor.get( p );

        // If there is an infinite lighting value, we don't want it and we skip the particle.
        if( !boost::math::isfinite( lighting.r ) || !boost::math::isfinite( lighting.g ) ||
            !boost::math::isfinite( lighting.b ) )
            continue;

        pixel_type particleDrawingColor;

        if( doEnvRefl ) {
            vector3f normal = vector3f::normalize( m_accessors.normalAccessor.get( p ) );
            vector3f view = vector3f::normalize( pos - rayStart );
            vector3f envDir = frantic::shading::compute_reflection( view, normal );

            float refStrength = m_accessors.reflectionAccessor.get( p );
            if( m_accessors.doReflectionStrengthScale ) {
                refStrength *= m_reflectionStrength;
            }

            lighting += refStrength * m_environment->lookup_environment( envDir );
        }

        if( m_lightingCallback ) {
            const vector3f normal = m_accessors.normalAccessor.get( p );
            const krakatoa::lighting_data lightingData( emission, lighting, normal, pos );
            const color3f callbackLight = ( *m_lightingCallback )( lightingData );
            lighting.r = std::max( lighting.r, callbackLight.r );
            lighting.g = std::max( lighting.g, callbackLight.g );
            lighting.b = std::max( lighting.b, callbackLight.b );
        }

        switch( m_renderMode ) {
        case mode_type::additive:
            particleDrawingColor = pixel_type( effectiveDensity * particleColor, frantic::graphics::alpha3f( 0 ) );
            break;
        case mode_type::normal:
            if( effectiveDensity <= 0 ) {
                particleDrawingColor.a.ar = particleDrawingColor.a.ag = particleDrawingColor.a.ab = 0;
                particleDrawingColor.c = areaDifferential * emission;
            } else if( !m_accessors.absorptionAccessor.is_valid() ) {
                particleDrawingColor.a.ar = particleDrawingColor.a.ag = particleDrawingColor.a.ab =
                    1.f - std::exp( -effectiveDensity * areaDifferential );
                particleDrawingColor.c = particleDrawingColor.a.premultiply( lighting + emission / effectiveDensity );
                // particleDrawingColor.c  = particleDrawingColor.a.premultiply( lighting );
                // particleDrawingColor.c += (float)( (1.0 - std::exp( -(double)effectiveDensity )) /
                // (double)effectiveDensity ) * emission;
            } else {
                color3f particleAbsorb = m_accessors.absorptionAccessor.get( p );
                color3f particleExtinction = particleAbsorb + particleColor;

                if( particleExtinction.r <= 0 ) {
                    particleDrawingColor.a.ar = 0;
                    particleDrawingColor.c.r = areaDifferential * ( emission.r + lighting.r * effectiveDensity );
                } else {
                    particleDrawingColor.a.ar =
                        1.f - std::exp( -effectiveDensity * areaDifferential * particleExtinction.r );
                    particleDrawingColor.c.r =
                        particleDrawingColor.a.ar * ( lighting.r / particleExtinction.r +
                                                      emission.r / ( particleExtinction.r * effectiveDensity ) );
                }

                if( particleExtinction.g <= 0 ) {
                    particleDrawingColor.a.ag = 0;
                    particleDrawingColor.c.g = areaDifferential * ( emission.g + lighting.g * effectiveDensity );
                } else {
                    particleDrawingColor.a.ag =
                        1.f - std::exp( -effectiveDensity * areaDifferential * particleExtinction.g );
                    particleDrawingColor.c.g =
                        particleDrawingColor.a.ag * ( lighting.g / particleExtinction.g +
                                                      emission.g / ( particleExtinction.g * effectiveDensity ) );
                }

                if( particleExtinction.b <= 0 ) {
                    particleDrawingColor.a.ab = 0;
                    particleDrawingColor.c.b = areaDifferential * ( emission.b + lighting.b * effectiveDensity );
                } else {
                    particleDrawingColor.a.ab =
                        1.f - std::exp( -effectiveDensity * areaDifferential * particleExtinction.b );
                    particleDrawingColor.c.b =
                        particleDrawingColor.a.ab * ( lighting.b / particleExtinction.b +
                                                      emission.b / ( particleExtinction.b * effectiveDensity ) );
                }
            }
            break;
        }

        if( m_atmosphere )
            m_atmosphere->apply_atomsphere( particleDrawingColor.c, rayStart, pos );

        bool isValid = true;
        vector2f screenPt = m_camera.from_cameraspace_position( camSpacePos, isValid );
        if( !isValid )
            continue;

        do_splat( outImage, occludedImage, rng, vector3f( screenPt.x, screenPt.y, -camSpacePos.z ),
                  particleDrawingColor, p );

        for( render_element_container_type::iterator it = outElements.begin(), itEnd = outElements.end(); it != itEnd;
             ++it ) {
            pixel_type outPixel( it->get()->evaluate( p ), particleDrawingColor.a );

            bool useWeights = true;
            particle_render_element_interface::draw_type dt = it->get()->get_drawing_type();

            if( dt == particle_render_element_interface::draw_type_shader ) {
                if( effectiveDensity <= 0 ) {
                    outPixel.c = color3f( 0 ); // outPixel.c = areaDifferential * emission;
                } else if( !m_accessors.absorptionAccessor.is_valid() ) {
                    outPixel.c = outPixel.a.premultiply( outPixel.c /*+ emission / effectiveDensity*/ );
                } else {
                    color3f particleAbsorb = m_accessors.absorptionAccessor.get( p );
                    color3f particleExtinction = particleAbsorb + particleColor;

                    if( particleDrawingColor.a.ar <= 0 ) {
                        outPixel.c.r = areaDifferential * ( /*emission.r + */ outPixel.c.r * effectiveDensity );
                    } else {
                        outPixel.c.r =
                            outPixel.a.ar *
                            ( outPixel.c.r /
                              particleExtinction.r /*+ emission.r / ( particleExtinction.r * effectiveDensity )*/ );
                    }

                    if( particleDrawingColor.a.ag <= 0 ) {
                        outPixel.c.g = areaDifferential * ( /*emission.g + */ outPixel.c.g * effectiveDensity );
                    } else {
                        outPixel.c.g =
                            outPixel.a.ag *
                            ( outPixel.c.g /
                              particleExtinction.g /*+ emission.g / ( particleExtinction.g * effectiveDensity )*/ );
                    }

                    if( particleDrawingColor.a.ab <= 0 ) {
                        outPixel.c.b = areaDifferential * ( /*emission.b + */ outPixel.c.b * effectiveDensity );
                    } else {
                        outPixel.c.b =
                            outPixel.a.ab *
                            ( outPixel.c.b /
                              particleExtinction.b /*+ emission.b / ( particleExtinction.b * effectiveDensity )*/ );
                    }
                }
            } else if( dt == particle_render_element_interface::draw_type_normal ) {
                outPixel.c *= m_cameraEmissionScale;
                if( effectiveDensity <= 0 ) {
                    outPixel.c = areaDifferential * outPixel.c;
                } else if( !m_accessors.absorptionAccessor.is_valid() ) {
                    outPixel.c = outPixel.a.premultiply( /*lighting +*/ outPixel.c / effectiveDensity );
                } else {
                    color3f particleAbsorb = m_accessors.absorptionAccessor.get( p );
                    color3f particleExtinction = particleAbsorb + particleColor;

                    if( particleDrawingColor.a.ar <= 0 ) {
                        outPixel.c.r = areaDifferential * ( outPixel.c.r /*+ lighting.r * effectiveDensity*/ );
                    } else {
                        outPixel.c.r = outPixel.a.ar * ( /*lighting.r / particleExtinction.r +*/ outPixel.c.r /
                                                         ( particleExtinction.r * effectiveDensity ) );
                    }

                    if( particleDrawingColor.a.ag <= 0 ) {
                        outPixel.c.g = areaDifferential * ( outPixel.c.g /*+ lighting.g * effectiveDensity*/ );
                    } else {
                        outPixel.c.g = outPixel.a.ag * ( /*lighting.g / particleExtinction.g +*/ outPixel.c.g /
                                                         ( particleExtinction.g * effectiveDensity ) );
                    }

                    if( particleDrawingColor.a.ab <= 0 ) {
                        outPixel.c.b = areaDifferential * ( outPixel.c.b /*+ lighting.b * effectiveDensity*/ );
                    } else {
                        outPixel.c.b = outPixel.a.ab * ( /*lighting.b / particleExtinction.b +*/ outPixel.c.b /
                                                         ( particleExtinction.b * effectiveDensity ) );
                    }
                }
            } else if( dt == particle_render_element_interface::draw_type_antialias ) {
                outPixel.a = alpha3f( 1.f );
            } else {
                outPixel.a = alpha3f( 1.f );
                useWeights = false;
            }

            useWeights = useWeights && !it->get()->use_minimize_compositing();

            do_splat( it->get()->get_framebuffer(),
                      it->get()->use_minimize_compositing() && m_saveOccludedZDepth ? &it->get()->get_framebuffer()
                                                                                    : NULL,
                      rng, vector3f( screenPt.x, screenPt.y, -camSpacePos.z ), outPixel, p, useWeights );
        }

        progressLogger.tick();
    }
}

void splat_renderer_impl::render_background( image_type& outImage ) {
    if( m_environment && !m_defaultBackground ) {
        m_environment->render_background( m_camera, outImage, m_mblurTime );

        // frantic::graphics::transform4f toWorld = m_camera.world_transform(m_mblurTime);
        //
        ////TODO: This doesn't do any sort of antialiasing. It probably should. Maybe. Ya know. Also this should
        ///definitely be done in parallel.
        // for( int y = 0, yEnd = outImage.height(); y < yEnd; ++y ){
        //	for( int x = 0, xEnd = outImage.width(); x < xEnd; ++x ){
        //		bool isValid = true;
        //		frantic::graphics2d::vector2f p( (float)x + 0.5f, (float)y + 0.5f );
        //		//frantic::graphics2d::vector2f screenUV( (float)x / (float)(outImage.width() - 1), -(float)y /
        //(float)(outImage.height() - 1) ); 		frantic::graphics::vector3f dir = toWorld.transform_no_translation(
        //m_camera.to_cameraspace_direction( p, isValid ) );

        //		color_type c = m_environment->lookup_environment( dir );
        //		//frantic::graphics::color_with_alpha<color_type,frantic::graphics::alpha1f> c =
        //m_environment->lookup_background( /*dir, screenUV*/x + 0.5f, y + 0.5f );

        //		outImage.blend_over( x, y, pixel_type( c )/*pixel_type(c.c, alpha_type(c.a.a))*/ );
        //	}
        //}
    } else {
        outImage.fill_under( m_backgroundColor );
    }
}

void splat_renderer_impl::set_jittered_mblur_time( int seed ) {
    if( !m_mblurJittered || !m_particles->get_channel_map().has_channel( _T("MBlurTime") ) )
        return;

    boost::mt19937 generator( seed );
    boost::variate_generator<boost::mt19937&, boost::uniform_01<float>> rng( generator, boost::uniform_01<float>() );

    frantic::channels::channel_cvt_accessor<float> timeAccessor =
        m_particles->get_channel_map().get_cvt_accessor<float>( _T("MBlurTime") );

    for( particle_container_type::iterator it = m_particles->begin(), itEnd = m_particles->end(); it != itEnd; ++it )
        timeAccessor.set( *it, m_mblurTime + ( rng() - 0.5f ) * m_mblurTimeSpan );
}

class jittered_mblur_sort_op {
    float m_mblurBias;
    float m_mblurDurationSeconds;

    const frantic::graphics::camera<float>* m_camera;

    frantic::channels::channel_accessor<vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<vector3f> m_velAccessor;
    frantic::channels::channel_cvt_accessor<float> m_timeAccessor;

    const bool m_wideSpherical;
    const frantic::graphics::transform4f m_camspaceXForm;

  public:
    jittered_mblur_sort_op()
        : m_camera( NULL )
        , m_wideSpherical( false )
        , m_camspaceXForm( frantic::graphics::transform4f() ) {}

    jittered_mblur_sort_op( float defaultTime, float mblurBias, float mblurDurationSeconds, bool isJittered,
                            const frantic::channels::channel_map& map,
                            const frantic::graphics::camera<float>& theCamera )
        : m_camera( &theCamera )
        , m_timeAccessor( defaultTime )
        , m_mblurBias( mblurBias )
        , m_mblurDurationSeconds( mblurDurationSeconds )
        , m_wideSpherical( m_camera->projection_mode() == frantic::graphics::projection_mode::spherical &&
                           ( m_camera->vertical_fov() >= boost::math::constants::pi<float>() ||
                             m_camera->horizontal_fov() >= boost::math::constants::pi<float>() ) )
        , m_camspaceXForm( theCamera.world_transform_inverse() ) {
        m_posAccessor = map.get_accessor<vector3f>( _T("Position") );
        m_velAccessor = map.get_cvt_accessor<vector3f>( _T("Velocity") );

        if( isJittered && map.has_channel( _T("MBlurTime") ) )
            m_timeAccessor = map.get_cvt_accessor<float>( _T("MBlurTime") );
    }

    bool operator()( const char* lhs, const char* rhs ) const {
        float lhsTime = m_timeAccessor.get( lhs );
        float lhsTimeSeconds = ( -0.5f * ( 1 - m_mblurBias ) + lhsTime ) * m_mblurDurationSeconds;

        float rhsTime = m_timeAccessor.get( rhs );
        float rhsTimeSeconds = ( -0.5f * ( 1 - m_mblurBias ) + rhsTime ) * m_mblurDurationSeconds;

        if( m_wideSpherical ) {
            const vector3f lhsCSPos =
                m_camspaceXForm * ( m_posAccessor.get( lhs ) + lhsTimeSeconds * m_velAccessor.get( lhs ) );
            const vector3f rhsCSPos =
                m_camspaceXForm * ( m_posAccessor.get( rhs ) + rhsTimeSeconds * m_velAccessor.get( rhs ) );
            return lhsCSPos.get_magnitude() < rhsCSPos.get_magnitude();
        } else {
            return vector3f::dot( m_camera->view_direction( lhsTime ),
                                  m_posAccessor.get( lhs ) + lhsTimeSeconds * m_velAccessor.get( lhs ) ) <
                   vector3f::dot( m_camera->view_direction( rhsTime ),
                                  m_posAccessor.get( rhs ) + rhsTimeSeconds * m_velAccessor.get( rhs ) );
        }
    }
};

class mblur_sort_op {
    vector3f m_sortDir;
    float m_sortTimeSeconds;

    frantic::channels::channel_accessor<vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<vector3f> m_velAccessor;

    const frantic::graphics::camera<float>* m_camera;
    const bool m_wideSpherical;
    const frantic::graphics::transform4f m_camspaceXForm;

  public:
    mblur_sort_op()
        : m_sortTimeSeconds( 0.f )
        , m_camera( NULL )
        , m_wideSpherical( false )
        , m_camspaceXForm( frantic::graphics::transform4f() ) {}

    mblur_sort_op( float timeOffsetSeconds, const frantic::channels::channel_map& map, const vector3f& sortDir,
                   const frantic::graphics::camera<float>& camera )
        : m_sortDir( sortDir )
        , m_sortTimeSeconds( timeOffsetSeconds )
        , m_camera( &camera )
        , m_wideSpherical( m_camera->projection_mode() == frantic::graphics::projection_mode::spherical &&
                           ( m_camera->vertical_fov() >= boost::math::constants::pi<float>() ||
                             m_camera->horizontal_fov() >= boost::math::constants::pi<float>() ) )
        , m_camspaceXForm( camera.world_transform_inverse() ) {
        m_posAccessor = map.get_accessor<vector3f>( _T("Position") );
        m_velAccessor = map.get_cvt_accessor<vector3f>( _T("Velocity") );
    }

    bool operator()( const char* lhs, const char* rhs ) const {
        if( m_wideSpherical ) {
            const vector3f lhsCSPos =
                m_camspaceXForm * ( m_posAccessor.get( lhs ) + m_sortTimeSeconds * m_velAccessor.get( lhs ) );
            const vector3f rhsCSPos =
                m_camspaceXForm * ( m_posAccessor.get( rhs ) + m_sortTimeSeconds * m_velAccessor.get( rhs ) );
            return lhsCSPos.get_magnitude() < rhsCSPos.get_magnitude();
        } else {
            return vector3f::dot( m_sortDir, m_posAccessor.get( lhs ) + m_sortTimeSeconds * m_velAccessor.get( lhs ) ) <
                   vector3f::dot( m_sortDir, m_posAccessor.get( rhs ) + m_sortTimeSeconds * m_velAccessor.get( rhs ) );
        }
    }
};

class default_sort_op {
    vector3f m_sortDir;
    frantic::channels::channel_accessor<vector3f> m_posAccessor;
    const frantic::graphics::camera<float>* m_camera;
    const bool m_wideSpherical;
    const frantic::graphics::transform4f m_camspaceXForm;

  public:
    default_sort_op()
        : m_camera( NULL )
        , m_wideSpherical( false )
        , m_camspaceXForm( frantic::graphics::transform4f() ) {}

    default_sort_op( const frantic::channels::channel_map& map, const vector3f& sortDir,
                     const frantic::graphics::camera<float>& camera )
        : m_sortDir( sortDir.to_normalized() )
        , m_posAccessor( map.get_accessor<vector3f>( _T("Position") ) )
        , m_camera( &camera )
        , m_wideSpherical( m_camera->projection_mode() == frantic::graphics::projection_mode::spherical &&
                           ( m_camera->vertical_fov() >= boost::math::constants::pi<float>() ||
                             m_camera->horizontal_fov() >= boost::math::constants::pi<float>() ) )
        , m_camspaceXForm( camera.world_transform_inverse() ) {}

    bool operator()( const char* lhs, const char* rhs ) const {
        if( m_wideSpherical ) {
            const vector3f lhsCSPos = m_camspaceXForm * m_posAccessor.get( lhs );
            const vector3f rhsCSPos = m_camspaceXForm * m_posAccessor.get( rhs );
            return lhsCSPos.get_magnitude() < rhsCSPos.get_magnitude();
        } else {
            return vector3f::dot( m_sortDir, m_posAccessor.get( lhs ) ) <
                   vector3f::dot( m_sortDir, m_posAccessor.get( rhs ) );
        }
    }
};

void splat_renderer_impl::sort_particles_for_camera() {
    if( this->m_renderMode == krakatoa::renderer::mode_type::additive )
        return;

    frantic::logging::progress_logger_subinterval_tracker plstSorting( *m_progress, 0, 25 );
    frantic::diagnostics::scoped_profile spsRenderSorting( m_psRenderSorting );

    m_progress->set_title( m_progressTitle + _T(": Sorting ") +
                           frantic::strings::int_to_comma_seperated_string( (int)m_particles->size() ) +
                           _T(" particles") );

    if( m_mblurJittered && m_particles->get_channel_map().has_channel( _T("MBlurTime") ) ) {
        frantic::sort::parallel_sort( m_particles->begin(), m_particles->end(),
                                      jittered_mblur_sort_op( m_mblurTime, m_mblurBias, m_mblurDuration,
                                                              m_mblurJittered, m_particles->get_channel_map(),
                                                              m_camera ),
                                      *m_progress, !m_doThreadedSorting );
    } else {
        float offsetTimeSeconds = ( -0.5f * ( 1 - m_mblurBias ) + m_mblurTime ) * m_mblurDuration;

        if( std::abs( offsetTimeSeconds ) >
            0.0001f ) { // If we need to offset the position using velocity, there is a separate code-path
            frantic::sort::parallel_sort( m_particles->begin(), m_particles->end(),
                                          mblur_sort_op( offsetTimeSeconds, m_particles->get_channel_map(),
                                                         m_camera.view_direction( m_mblurTime ), m_camera ),
                                          *m_progress, !m_doThreadedSorting );
        } else { // Are we centered?
            frantic::sort::parallel_sort(
                m_particles->begin(), m_particles->end(),
                default_sort_op( m_particles->get_channel_map(), m_camera.view_direction( m_mblurTime ), m_camera ),
                *m_progress, !m_doThreadedSorting );
        }
    }
}

void splat_renderer_impl::do_splat( image_type& outImage, image_type* occludeImage, rng_type& rng,
                                    const frantic::graphics::vector3f screenPt, const pixel_type& splat,
                                    const char* particle, bool useWeights ) {
    if( m_doDOF ) {
        vector2f pixelRadii;
        m_camera.dof_ellipse_of_confusion_pixel_radii( screenPt.z, pixelRadii.x, pixelRadii.y );

        // Compute the number of samples we want to do, based on the radius
        const int depthOfFieldSampleCount =
            std::min( (int)ceil( M_PI * pixelRadii.x * pixelRadii.y * m_dofSampleRate + 0.5f ), 100000 );
        const float invDOFSampleCount = 1.f / static_cast<float>( depthOfFieldSampleCount );
        const std::size_t resolution = 2 * static_cast<std::size_t>( std::max( pixelRadii.x, pixelRadii.y ) );
        const std::size_t resolutionSquared = resolution * resolution;

        const boost::optional<frantic::graphics2d::image_channel<float>>& bokehMask = bokeh_mask();
        const float bokehBlendAmount =
            boost::algorithm::clamp( m_accessors.bokehBlendAccessor( particle ), 0.0f, 1.0f );
        const boost::optional<const frantic::graphics2d::image_channel<frantic::graphics::color3f>&> bokehBlend =
            bokeh_blend_map( std::max( static_cast<std::size_t>( 1 ), resolution ) * m_mipmapResolutionCoefficient );

        const vector2f maskCenter = bokehMask ? vector2f( static_cast<float>( bokehMask.get().width() ) / 2.0f,
                                                          static_cast<float>( bokehMask.get().height() ) / 2.0f )
                                              : vector2f( 0.f );

        const vector2f blendCenter = bokehBlend ? vector2f( static_cast<float>( bokehBlend.get().width() ) / 2.0f,
                                                            static_cast<float>( bokehBlend.get().height() ) / 2.0f )
                                                : vector2f( 0.f );

        const float density = m_renderMode == mode_type::additive ? m_accessors.densityAccessor.get( particle ) : 0.f;
        const float effectiveDensity = density * m_cameraDensityScale;

        const float anamorphicSqueeze = anamorphic_squeeze().get_value_or( 1.0f );

        for( int dofSample = 0; dofSample < depthOfFieldSampleCount; ++dofSample ) {
            pixel_type weightedSplat;
            vector2f offsetPt;

            if( bokehMask ) {
                int mapX = 0;
                int mapY = 0;
                vector2f sampleLocation;

                std::size_t total = 0;
                do {
                    sampleLocation = frantic::graphics2d::vector2f( 2 * rng() - 1, 2 * rng() - 1 );
                    const vector2f offset =
                        vector2f( sampleLocation.x * maskCenter.x, sampleLocation.y * maskCenter.y ) + maskCenter;
                    mapX = boost::algorithm::clamp( static_cast<int>( std::floor( offset.x ) ), 0,
                                                    bokehMask.get().width() - 1 );
                    mapY = boost::algorithm::clamp( static_cast<int>( std::floor( offset.y ) ), 0,
                                                    bokehMask.get().height() - 1 );
                    ++total;
                } while( bokehMask.get().at( mapX, mapY ) <= static_cast<float>( rng() ) && total < resolutionSquared );

                offsetPt = pixelRadii * sampleLocation;
            } else {
                offsetPt = pixelRadii * frantic::graphics2d::vector2f::from_unit_disk_random( rng );
            }

            if( bokehBlend && bokehBlendAmount > 0.0f && !useWeights ) {
                const vector2f sampleLocation( offsetPt.x / pixelRadii.x, offsetPt.y / pixelRadii.y );
                const vector2f offset =
                    vector2f( sampleLocation.x * blendCenter.x, sampleLocation.y * blendCenter.y ) + blendCenter;
                const int blendMapX = boost::algorithm::clamp( static_cast<int>( std::floor( offset.x ) ), 0,
                                                               bokehBlend.get().width() - 1 );
                const int blendMapY = boost::algorithm::clamp( static_cast<int>( std::floor( offset.y ) ), 0,
                                                               bokehBlend.get().height() - 1 );
                color3f blendColor = bokehBlend.get().at( blendMapX, blendMapY );

                if( m_renderMode == mode_type::additive ) {
                    blendColor *= effectiveDensity;
                } else {
                    // Blend hue and saturation, but not value.
                    // This is to attempt to preserve the lighting of the particles
                    // because we can't re-calculate lighting for each bokeh splat.
                    blendColor = color3f::from_hsv( blendColor.hue(), blendColor.saturation(), splat.c.value(),
                                                    frantic::graphics::color3f::unclamped );
                }

                const color3f blendedColor =
                    ( bokehBlendAmount * blendColor ) + ( ( 1.0f - bokehBlendAmount ) * splat.c );
                weightedSplat = pixel_type( blendedColor, splat.a ) * invDOFSampleCount;
            } else {
                weightedSplat = useWeights ? splat * ( 1.f / depthOfFieldSampleCount ) : splat;
            }

            if( anamorphicSqueeze != 1.0f ) {
                if( anamorphicSqueeze > 1.0f ) {
                    offsetPt.y *= anamorphicSqueeze;
                } else {
                    offsetPt.x /= anamorphicSqueeze;
                }
            }

            do_splat_impl( outImage, occludeImage,
                           vector3f( screenPt.x + offsetPt.x, screenPt.y + offsetPt.y, screenPt.z ), weightedSplat,
                           useWeights );
        }
    } else {
        do_splat_impl( outImage, occludeImage, screenPt, splat, useWeights );
    }
}

void splat_renderer_impl::do_splat_impl( image_type& outImage, image_type* occludedImage,
                                         const frantic::graphics::vector3f screenPt, const pixel_type& splat,
                                         bool useWeights ) {
    int filterWidth = m_splatFilter->get_width();

    float* pixelWeights = (float*)alloca( filterWidth * filterWidth * sizeof( float ) );

    vector2 pixel;
    m_splatFilter->do_filter( vector2f( screenPt.x, screenPt.y ), pixel, pixelWeights );

    for( int y = 0; y < filterWidth; ++y ) {
        int realY = pixel.y + y;

        if( (unsigned)realY < (unsigned)outImage.height() ) {
            for( int x = 0; x < filterWidth; ++x ) {
                int realX = pixel.x + x;
                if( (unsigned)realX < (unsigned)outImage.width() ) {
                    float weight = useWeights ? pixelWeights[x + y * filterWidth] : 1;

                    bool occluded = false;

                    if( m_matteSampler || m_matteGenerator ) {
                        int baseX = realX * m_matteSuperSampling;
                        int baseY = realY * m_matteSuperSampling;

                        // check all the samples in the depthmap that overlap this screen position.
                        // if at least one is occluded, consider it fully occluded
                        for( int j = 0; j < m_matteSuperSampling && !occluded; ++j ) {
                            for( int i = 0; i < m_matteSuperSampling && !occluded; ++i ) {
                                if( !m_matteDepthmap.is_visible( baseX + i, baseY + j, screenPt.z ) )
                                    occluded = true;
                            }
                        }
                    }

                    if( !occluded ) {
                        // now check deep matte image!
                        if( m_deepMatte ) {

                            int width = outImage.width();
                            int height = outImage.height();
                            int imgWidth = m_deepMatte->size().xsize;
                            int imgHeight = m_deepMatte->size().ysize;

                            // TODO: This works when the image and the deep holdout matte are of the same ratio
                            // (normally the same dimensions)
                            // TODO: what the heck to do when images are of different ratio
                            if( screenPt.z >
                                m_deepMatte->get_zdepth_bilinear( screenPt.x * ( (float)imgWidth / width ),
                                                                  screenPt.y * ( (float)imgHeight / height ) ) )
                                occluded = true;

                            /*
                            int w = outImage.width();
                            int h = outImage.height();
                            int imgDim = m_deepMatte->size().xsize;

                            //deep images are always square images. in prman, if the output image resolution is size
                            [w,h], and the deep image is size [n,n], the mapping from image to matte is
                            (pixelX/w*n,pixelY/w*n).
                            //note that the deep image does not fully cover the output image when h > w.
                            float lookupScale = (float)imgDim / w;
                            float yLookupOffset = ( imgDim - h * lookupScale ) * 0.5f;
                            vector2f imgLookup( screenPt.x * lookupScale, screenPt.y * lookupScale + yLookupOffset );
                            if( imgLookup.y >= 0 && imgLookup.y < imgDim ) {
                                    if( screenPt.z > m_deepMatte->get_zdepth_bilinear( imgLookup.x, imgLookup.y ) )
                                            occluded = true;
                            }
                            */
                        }
                    }

                    // draw the splat to either the outImage or the occludedImage depending on if it's unoccluded or
                    // fully/partially occluded.
                    if( !occluded ) {
                        outImage.blend_over( realX, realY, weight * splat );
                    } else {
                        if( occludedImage )
                            occludedImage->blend_over( realX, realY, weight * splat );
                    }
                }
            }
        }
    }
}

std::size_t splat_renderer_impl::get_num_output_images() { return m_numRenderPasses; }

} // namespace splat_renderer
} // namespace krakatoa
