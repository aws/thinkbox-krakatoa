// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <algorithm>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/thread/tss.hpp>

#include <frantic/rendering/depthbuffer_singleface.hpp>
#include <frantic/shading/highlights.hpp>
#include <frantic/sort/sort.hpp>

#include <krakatoa/matte_zdepth_render_element.hpp>
#include <krakatoa/occluded_layer_render_element.hpp>
#include <krakatoa/splat_renderer/parallel_progress.hpp>
#include <krakatoa/splat_renderer/splat_renderer.hpp>
#include <krakatoa/threading_functions.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 4100 )
#include <tbb/atomic.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/tbb_thread.h>

using frantic::graphics::alpha3f;
using frantic::graphics::color3f;
using frantic::graphics::transform4f;
using frantic::graphics::vector3;
using frantic::graphics::vector3f;

using frantic::graphics2d::vector2;
using frantic::graphics2d::vector2f;

namespace krakatoa {
namespace splat_renderer {

class splat_cube_renderer_impl : public splat_renderer {
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
        splat_cube_renderer_impl* m_renderer;

        int m_seed;
        mutable rng_type m_rng;

        parallel_render_progress_logger_master* m_masterLogger;

        std::size_t m_numThreads;

      public:
        parallel_render_subinterval( splat_cube_renderer_impl& renderer, int seed,
                                     parallel_render_progress_logger_master* masterLogger )
            : m_renderer( &renderer )
            , m_rng( rng_gen_type( seed ), rng_range_type() )
            , m_seed( seed )
            , m_masterLogger( NULL ) {}

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
            std::size_t threadIndex = range.begin();
            std::size_t partitionSize = m_renderer->m_particles->size() / m_numThreads;
            std::size_t particleStart = threadIndex * partitionSize;
            std::size_t particleEnd = std::min( m_renderer->m_particles->size(), particleStart + partitionSize );
            // Since the number of particles may not be evenly divisible by the number of threads, we need to render the
            // remainder in the last partition
            if( threadIndex == m_numThreads - 1 )
                particleEnd += m_renderer->m_particles->size() % m_numThreads;

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

  private:
    virtual void render_subinterval( image_type& outImage, int seed );

    virtual void render_background( image_type& outImage );

    void render_subinterval_impl( image_type& outImage, image_type* occludedImage,
                                  render_element_container_type& outElements, rng_type& rng,
                                  const tbb::blocked_range<std::size_t>& range,
                                  parallel_render_progress_logger& logger );

    void set_jittered_mblur_time( int seed );

    void sort_particles_for_camera();

    void do_splat( int cubeSize, image_type& outImage, image_type* occludedImage, rng_type& rng,
                   const frantic::graphics::vector3f& camSpacePos, const pixel_type& splat, bool useWeights = true );

    void do_splat_impl( int cubeSize, image_type& outImage, image_type* occludedImage,
                        const frantic::graphics::vector3f& screenPos, const pixel_type& splat, bool useWeights );

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
        bool doReflectionStrengthScale;

        void reset( const frantic::channels::channel_map& pcm, float mblurTime, float reflectionStrength ) {
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
        }
    } m_accessors;

  public:
    splat_cube_renderer_impl();

    virtual ~splat_cube_renderer_impl();

    virtual void apply_area_differential( bool enabled = true, float maxValue = FLT_MAX );

    virtual void add_render_element( render_element_interface_ptr renderElement );

    virtual void precompute_lighting();

    virtual void render( image_type& outImage );

  protected:
    std::size_t get_num_output_images();
};

splat_renderer_ptr create_cube_splat_renderer() { return splat_renderer_ptr( new splat_cube_renderer_impl ); }

splat_cube_renderer_impl::splat_cube_renderer_impl()
    : m_psRenderMatte( _T("Rendering:Matte") )
    , m_psRenderSorting( _T("Rendering:Sorting") )
    , m_psRenderDrawing( _T("Rendering:Drawing") ) {
    m_mblurTime = 0.5f;
    m_mblurTimeSpan = 1.f;
    m_doDOF = false;
    m_applyAreaDifferential = true;
    m_areaDifferentialMax = FLT_MAX;
}

splat_cube_renderer_impl::~splat_cube_renderer_impl() {}

void splat_cube_renderer_impl::apply_area_differential( bool enabled, float maxValue ) {
    m_applyAreaDifferential = enabled;
    m_areaDifferentialMax = maxValue;
}

void splat_cube_renderer_impl::add_render_element( render_element_interface_ptr renderElement ) {
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

void splat_cube_renderer_impl::precompute_lighting() {
    if( !m_particles || !m_sceneContext || !m_lightingEngine || !m_shader )
        return;

    // Set a default splat filter if the user hasn't
    if( !m_splatFilter )
        m_splatFilter = filter2f::create_instance( _T("Bilinear") );

    // Set a default progress logger if the user hasn't
    if( !m_progress )
        m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_shader->set_channel_map( m_particles->get_channel_map() );

    // Cache the camera for easier reference (ie. m_camera vs. m_sceneContext->get_camera())
    m_camera = m_sceneContext->get_camera();

    m_lightingEngine->set_density_scale( m_lightDensityScale );
    m_lightingEngine->set_progress_logger( m_progress );
    m_lightingEngine->set_scene_context( m_sceneContext );
    m_lightingEngine->set_shader( m_shader );

    m_lightingEngine->add_render_elements( m_renderElements.begin(), m_renderElements.end() );

    m_lightingEngine->compute_particle_lighting( *m_particles, m_useAbsorptionChannel );
}

void splat_cube_renderer_impl::render( image_type& outImage ) {
    if( !m_particles || !m_sceneContext || !m_shader )
        return;

    // Set a default splat filter if the user hasn't
    if( !m_splatFilter )
        m_splatFilter = filter2f::create_instance( _T("Bilinear") );

    // Set a default progress logger if the user hasn't
    if( !m_progress )
        m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_shader->set_channel_map( m_particles->get_channel_map() );

    m_progressTitle = _T("Rendering");
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

            m_progressTitle = _T("Rendering pass ") + boost::lexical_cast<frantic::tstring>( segment + 1 ) +
                              _T(" of ") + boost::lexical_cast<frantic::tstring>( m_mblurSamples );
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

            outImage.apply_gain( 1.f / (float)m_mblurSamples );
            accumRenderBuffer.add_image_data( outImage );

            for( std::size_t i = 0, iEnd = m_renderElements.size(); i < iEnd; ++i ) {
                m_renderElements[i]->get_framebuffer().apply_gain( 1.f / (float)m_mblurSamples );
                accumRenderElements[i]->get_framebuffer().add_image_data( m_renderElements[i]->get_framebuffer() );
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

void splat_cube_renderer_impl::render_subinterval( image_type& outImage, int seed ) {
    set_jittered_mblur_time( seed );

    sort_particles_for_camera();

    // Sorting is first 25% of rendering, last 75% is drawing.
    frantic::logging::progress_logger_subinterval_tracker siDrawing( *m_progress, 25, 75 );
    m_progress->set_title( m_progressTitle + _T(": Drawing ") +
                           frantic::strings::int_to_comma_seperated_string( (int)m_particles->size() ) +
                           _T(" particles") );

    if( m_matteSampler ) {
        frantic::diagnostics::scoped_profile spsRenderMatte( m_psRenderMatte );

        int cubeSize = m_matteSuperSampling * std::min( outImage.width() / 2, outImage.height() / 3 );

        m_matteDepthmap.set_size( m_matteSuperSampling * outImage.size() );

        frantic::rendering::depthbuffer_singleface singleCubeFaceDepth( cubeSize );

        const frantic::graphics::camera<float>& sceneCamera = m_sceneContext->get_camera();

        // Create six cameras to render the depth maps. TODO: Should I make 'm_matteSampler' support cube rendering
        // directly? Depends on if we want z-depth or actual distance. The particles are definitely rendered in radial
        // distance order and not z-order ...
        for( int i = 0; i < 6; ++i ) {
            frantic::graphics::camera<float> cubeCamera = frantic::graphics::camera<float>::from_cube_face(
                sceneCamera.world_transform( m_mblurTime ),
                static_cast<frantic::graphics::cube_face::default_cube_face>( i ) );

            cubeCamera.set_far( sceneCamera.far_distance() );
            cubeCamera.set_near( sceneCamera.near_distance() );
            cubeCamera.set_focal_distance( sceneCamera.focal_distance() );
            cubeCamera.set_focal_length( sceneCamera.focal_length() );
            cubeCamera.set_fstop( sceneCamera.fstop() );
            cubeCamera.set_output_size( frantic::graphics2d::size2( cubeSize ) );

            m_matteSampler->generate_depth_map( cubeCamera, m_mblurTime, singleCubeFaceDepth.as_framebuffer(), false );

            int xOff = ( i % 2 ) * cubeSize;
            int yOff = ( i / 2 ) * cubeSize;

            for( int y = 0; y < cubeSize; ++y ) {
                for( int x = 0; x < cubeSize; ++x )
                    m_matteDepthmap.set_depth_value( x + xOff, y + yOff, singleCubeFaceDepth.get_depth_value( x, y ) );
            }
        }

        if( m_matteZDepthElement )
            m_matteZDepthElement->get_depthbuffer().resample_combine( m_matteDepthmap );
    }

    const frantic::channels::channel_map& pcm = m_particles->get_channel_map();

    m_accessors.reset( pcm, m_mblurTime, m_reflectionStrength );

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

    parallel_render_progress_logger_master masterLogger( m_progress, m_particles->size(), m_watermarkFn );
    parallel_render_subinterval fn( *this, seed, &masterLogger );

    frantic::diagnostics::scoped_profile spsRenderDrawing( m_psRenderDrawing );

    // Get the maximum number of threads we can use based on the size of the frame buffers per thread, and the amount of
    // physical memory
    std::size_t threadCount =
        get_max_threads( outImage.width(), outImage.height(), get_num_output_images(), m_particles->size_in_memory(),
                         sizeof( pixel_type ), 100.0f, m_renderingThreadLimit );

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

    render_background( outImage );

    return;
}

void splat_cube_renderer_impl::render_subinterval_impl( image_type& outImage, image_type* occludedImage,
                                                        render_element_container_type& outElements, rng_type& rng,
                                                        const tbb::blocked_range<std::size_t>& range,
                                                        parallel_render_progress_logger& progressLogger ) {
    bool doEnvRefl = m_environment && m_enableReflections;

    int cubeSize = std::min( outImage.width() / 2, outImage.height() / 3 );

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
        float camSpaceDist = camSpacePos.get_magnitude();

        // Do not process particles outside of the clipping frustrum
        if( camSpaceDist < m_camera.near_distance() || camSpaceDist > m_camera.far_distance() )
            continue;

        float areaDifferential = 1.f;
        if( m_applyAreaDifferential ) {
            areaDifferential = static_cast<float>( cubeSize * cubeSize ) / ( 4.f * camSpaceDist * camSpaceDist );
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

        switch( m_renderMode ) {
        case mode_type::additive:
            particleDrawingColor = pixel_type( effectiveDensity * particleColor, frantic::graphics::alpha3f( 0 ) );
            break;
        case mode_type::normal:
            if( effectiveDensity <= 0 ) {
                particleDrawingColor.a.ar = particleDrawingColor.a.ag = particleDrawingColor.a.ab = 0;
                particleDrawingColor.c = areaDifferential * emission;
            } else if( !m_accessors.absorptionAccessor.is_valid() ) {
                particleDrawingColor = outgoing_radiance( lighting, emission, effectiveDensity, areaDifferential );
            } else {
                color3f particleAbsorb = m_accessors.absorptionAccessor.get( p );
                color3f particleExtinction = particleAbsorb + particleColor;

                particleDrawingColor =
                    outgoing_radiance( lighting, emission, particleExtinction, effectiveDensity, areaDifferential );
            }
            break;
        }

        if( m_atmosphere )
            m_atmosphere->apply_atomsphere( particleDrawingColor.c, rayStart, pos );

        do_splat( cubeSize, outImage, occludedImage, rng, camSpacePos, particleDrawingColor );

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

            do_splat( cubeSize, it->get()->get_framebuffer(), NULL, rng, camSpacePos, outPixel, useWeights );
        }

        progressLogger.tick();
    }
}

void splat_cube_renderer_impl::render_background( image_type& outImage ) {
    if( m_environment && !m_defaultBackground ) {
        int cubeSize = std::min( outImage.width() / 2, outImage.height() / 3 );

        frantic::rendering::framebuffer_cubeface<pixel_type> cubeImage( cubeSize );

        m_environment->render_cube_background( m_camera.world_transform( m_mblurTime ), cubeImage );

        // Convert the temporary cubeface_buffer back to our rendering format.
        // TODO: Add an overload to the env renderer that draws directly to the specified format.
        int yOff = 0;
        for( int i = 0; i < 6; i += 2, yOff += cubeSize ) {
            for( int y = 0; y < cubeSize; ++y ) {
                for( int x = 0; x < cubeSize; ++x )
                    outImage.set_pixel( x, y + yOff, cubeImage.at( i, frantic::graphics2d::vector2( x, y ) ) );
                for( int x = 0; x < cubeSize; ++x )
                    outImage.set_pixel( x + cubeSize, y + yOff,
                                        cubeImage.at( i + 1, frantic::graphics2d::vector2( x, y ) ) );
            }
        }
    } else {
        outImage.fill_under( m_backgroundColor );
    }
}

void splat_cube_renderer_impl::set_jittered_mblur_time( int seed ) {
    if( !m_mblurJittered || !m_particles->get_channel_map().has_channel( _T("MBlurTime") ) )
        return;

    boost::mt19937 generator( seed );
    boost::variate_generator<boost::mt19937&, boost::uniform_01<float>> rng( generator, boost::uniform_01<float>() );

    frantic::channels::channel_cvt_accessor<float> timeAccessor =
        m_particles->get_channel_map().get_cvt_accessor<float>( _T("MBlurTime") );

    for( particle_container_type::iterator it = m_particles->begin(), itEnd = m_particles->end(); it != itEnd; ++it )
        timeAccessor.set( *it, m_mblurTime + ( rng() - 0.5f ) * m_mblurTimeSpan );
}

namespace {
class jittered_mblur_sort_op {
    float m_mblurBias;
    float m_mblurDurationSeconds;

    const frantic::graphics::camera<float>* m_camera;

    frantic::channels::channel_accessor<vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<vector3f> m_velAccessor;
    frantic::channels::channel_cvt_accessor<float> m_timeAccessor;

  public:
    jittered_mblur_sort_op()
        : m_camera( NULL ) {}

    jittered_mblur_sort_op( float defaultTime, float mblurBias, float mblurDurationSeconds, bool isJittered,
                            const frantic::channels::channel_map& map,
                            const frantic::graphics::camera<float>& theCamera )
        : m_camera( &theCamera )
        , m_timeAccessor( defaultTime )
        , m_mblurBias( mblurBias )
        , m_mblurDurationSeconds( mblurDurationSeconds ) {
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

        return frantic::graphics::vector3f::distance_squared( m_posAccessor.get( lhs ) +
                                                                  lhsTimeSeconds * m_velAccessor.get( lhs ),
                                                              m_camera->camera_position( lhsTime ) ) <
               frantic::graphics::vector3f::distance_squared( m_posAccessor.get( rhs ) +
                                                                  rhsTimeSeconds * m_velAccessor.get( rhs ),
                                                              m_camera->camera_position( rhsTime ) );
    }
};

class mblur_sort_op {
    vector3f m_sortPos;
    float m_sortTimeSeconds;

    frantic::channels::channel_accessor<vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<vector3f> m_velAccessor;

  public:
    mblur_sort_op()
        : m_sortTimeSeconds( 0.f ) {}

    mblur_sort_op( float timeOffsetSeconds, const frantic::channels::channel_map& map, const vector3f& sortPos )
        : m_sortPos( sortPos )
        , m_sortTimeSeconds( timeOffsetSeconds ) {
        m_posAccessor = map.get_accessor<vector3f>( _T("Position") );
        m_velAccessor = map.get_cvt_accessor<vector3f>( _T("Velocity") );
    }

    bool operator()( const char* lhs, const char* rhs ) const {
        return frantic::graphics::vector3f::distance_squared(
                   m_posAccessor.get( lhs ) + m_sortTimeSeconds * m_velAccessor.get( lhs ), m_sortPos ) <
               frantic::graphics::vector3f::distance_squared(
                   m_posAccessor.get( rhs ) + m_sortTimeSeconds * m_velAccessor.get( rhs ), m_sortPos );
    }
};

class default_sort_op {
    vector3f m_sortPos;

    frantic::channels::channel_accessor<vector3f> m_posAccessor;

  public:
    default_sort_op() {}

    default_sort_op( const frantic::channels::channel_map& map, const vector3f& sortPos )
        : m_sortPos( sortPos )
        , m_posAccessor( map.get_accessor<vector3f>( _T("Position") ) ) {}

    bool operator()( const char* lhs, const char* rhs ) const {
        return frantic::graphics::vector3f::distance_squared( m_posAccessor.get( lhs ), m_sortPos ) <
               frantic::graphics::vector3f::distance_squared( m_posAccessor.get( rhs ), m_sortPos );
    }
};
} // namespace

void splat_cube_renderer_impl::sort_particles_for_camera() {
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
                                                         m_camera.camera_position( m_mblurTime ) ),
                                          *m_progress, !m_doThreadedSorting );
        } else { // Are we centered?
            frantic::sort::parallel_sort(
                m_particles->begin(), m_particles->end(),
                default_sort_op( m_particles->get_channel_map(), m_camera.camera_position( m_mblurTime ) ), *m_progress,
                !m_doThreadedSorting );
        }
    }
}

void splat_cube_renderer_impl::do_splat( int cubeSize, image_type& outImage, image_type* occludeImage, rng_type& rng,
                                         const frantic::graphics::vector3f& camSpacePos, const pixel_type& splat,
                                         bool useWeights ) {
    // TODO: What depth of field calculations make sense for a 360 degree cube image? Possibly just the usual, with
    // minor adjustments.

    if( m_doDOF ) {
        float dist = camSpacePos.get_magnitude();

        float lensRadius = 0.5f * m_camera.focal_length() / m_camera.fstop();
        float imageDistance = 1 / ( 1 / m_camera.focal_length() - 1 / m_camera.focal_distance() );
        float objectImageDistance = 1 / ( 1 / m_camera.focal_length() - 1 / dist );
        float circleOfConfusionWorldRadius =
            lensRadius * fabsf( imageDistance - objectImageDistance ) / objectImageDistance;
        float circleOfConfusionPixelRadius = circleOfConfusionWorldRadius * cubeSize / ( 2 * imageDistance );

        // Compute the number of samples we want to do, based on the radius
        int depthOfFieldSampleCount =
            (int)ceil( M_PI * frantic::math::square( circleOfConfusionPixelRadius ) * m_dofSampleRate + 0.5f );
        if( depthOfFieldSampleCount > 100000 ) // Clamp the number of samples at 100K
            depthOfFieldSampleCount = 100000;

        // Calculate the Tangent & Binormal vectors for the disk around the actual position.
        frantic::graphics::vector3f t, bn;
        frantic::shading::compute_tangent_binormal( camSpacePos / dist, t, bn );

        pixel_type weightedSplat = splat * ( 1.f / depthOfFieldSampleCount );
        for( int dofSample = 0; dofSample < depthOfFieldSampleCount; ++dofSample ) {
            // For each point, offset by a random point in the appropriate disk around the actual position.
            // TODO: Should this be a z-disk, or a half-sphere such that we maintain correct draw ordering?!?
            vector2f offsetPt =
                circleOfConfusionWorldRadius * frantic::graphics2d::vector2f::from_unit_disk_random( rng );
            vector3f newPos = camSpacePos + offsetPt.x * t + offsetPt.y * bn;

            do_splat_impl( cubeSize, outImage, occludeImage, newPos, weightedSplat, useWeights );
        }
    } else {
        do_splat_impl( cubeSize, outImage, occludeImage, camSpacePos, splat, useWeights );
    }
}

void splat_cube_renderer_impl::do_splat_impl( int cubeSize, image_type& outImage, image_type* occludedImage,
                                              const frantic::graphics::vector3f& camSpacePos, const pixel_type& splat,
                                              bool useWeights ) {
    int filterWidth = m_splatFilter->get_width();

    float* pixelWeights = (float*)alloca( filterWidth * filterWidth * sizeof( float ) );

    float ratioTolerance = static_cast<float>( cubeSize - filterWidth ) / static_cast<float>( cubeSize );

    frantic::graphics::cube_face::default_cube_face cubeFaces[6];

    int numFaces = frantic::graphics::get_cube_faces( camSpacePos, ratioTolerance, cubeFaces );

    // Can safely assume the size of the array is at least 1
    frantic::graphics2d::vector2f coordinate0 =
        frantic::graphics::get_cube_face_coordinate( camSpacePos, cubeFaces[0] );

    // NOTE: Stolen mercilessly from framebuffer_cubeface::draw_point.
    //
    // The compensation factor is cos^3(theta).  This calculation depends on the fact that the face has a 90 degree
    // field of view.
    //
    // The reason for the cos^3(theta) is twofold.  First, consider the ratio of the distance to the point being drawn
    // versus the perpendicular distance. If you draw out the little triangle, it becomes clear that this distance is
    // cos(theta).  Because the inverse square falloff behavior of point sources, this means that the ray density at
    // that point is decreased by 1/cos^2(theta). Next, consider the angle at which the rays are hitting the surface. If
    // you compute the ratio of area of the stretched surface versus the area of he perpendicular surface, you get that
    // the surface area is increased by cos(theta).  Multiplying these two factors together as a compensation for the
    // decrease in density gives us cos^3(theta).
    //
    // This compensation works correctly for both additive drawing of particles and alpha blending of particles.
    float compensationFactor = ( coordinate0.x * coordinate0.x + coordinate0.y * coordinate0.y + 1 );

    compensationFactor *= sqrt( compensationFactor );

    pixel_type compensatedSplat = frantic::rendering::cubeface_compensation( splat, compensationFactor );

    for( int i = 0; i < numFaces; ++i ) {
        float depth = 0;

        frantic::graphics2d::vector2f p =
            frantic::graphics::get_cube_face_coordinate_and_zdepth( camSpacePos, cubeFaces[i], depth );

        p.x = ( 1.f + p.x ) * 0.5f * static_cast<float>( cubeSize );
        p.y = ( 1.f + p.y ) * 0.5f * static_cast<float>( cubeSize );

        frantic::graphics2d::vector2 pixel;
        m_splatFilter->do_filter( p, pixel, pixelWeights );

        // Assuming we have a vertical stacking of cube images, with +X, -X at the bottom and +Z, -Z at the top.
        int xOff = ( cubeFaces[i] % 2 ) * cubeSize;
        int yOff = ( cubeFaces[i] / 2 ) * cubeSize;

        for( int y = 0; y < filterWidth; ++y ) {
            int realY = pixel.y + y;

            if( (unsigned)realY >= (unsigned)cubeSize )
                continue;

            realY += yOff;

            for( int x = 0; x < filterWidth; ++x ) {
                int realX = pixel.x + x;
                if( (unsigned)realX >= (unsigned)cubeSize )
                    continue;

                realX += xOff;

                float weight = useWeights ? pixelWeights[x + y * filterWidth] : 1;

                bool occluded = false;

                if( m_matteSampler ) {
                    int baseX = realX * m_matteSuperSampling;
                    int baseY = realY * m_matteSuperSampling;

                    // check all the samples in the depthmap that overlap this screen position.
                    // if at least one is occluded, consider it fully occluded
                    for( int j = 0; j < m_matteSuperSampling && !occluded; ++j ) {
                        for( int i = 0; i < m_matteSuperSampling && !occluded; ++i ) {
                            if( !m_matteDepthmap.is_visible( baseX + i, baseY + j, depth ) )
                                occluded = true;
                        }
                    }
                }

                // if( !occluded ) {
                //	//now check deep matte image!
                //	if( m_deepMatte ) {

                //		int width = outImage.width();
                //		int height = outImage.height();
                //		int imgWidth = m_deepMatte->size().xsize;
                //		int imgHeight = m_deepMatte->size().ysize;

                //		//TODO: This works when the image and the deep holdout matte are of the same ratio
                //(normally the same dimensions)
                //		//TODO: what the heck to do when images are of different ratio
                //		if( depth > m_deepMatte->get_zdepth_bilinear( screenPt.x * ( (float)imgWidth / width ),
                //screenPt.y * ( (float)imgHeight / height ) ) ) 			occluded = true;
                //	}
                //}

                // draw the splat to either the outImage or the occludedImage depending on if it's unoccluded or
                // fully/partially occluded.
                if( !occluded ) {
                    outImage.blend_over( realX, realY, weight * compensatedSplat );
                } else {
                    if( occludedImage )
                        occludedImage->blend_over( realX, realY, weight * compensatedSplat );
                }
            } // for( int x = 0; x < filterWidth; ++x ){
        }     // for( int y = 0; y < filterWidth; ++y ){
    }         // for( int i = 0; i < numFaces; ++i )
}

std::size_t splat_cube_renderer_impl::get_num_output_images() { return m_numRenderPasses; }

} // namespace splat_renderer
} // namespace krakatoa
