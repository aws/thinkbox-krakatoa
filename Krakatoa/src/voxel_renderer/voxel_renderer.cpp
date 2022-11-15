// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <boost/algorithm/clamp.hpp>
#include <boost/bind.hpp>
#include <boost/call_traits.hpp>

#pragma warning( push, 3 )
#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#pragma warning( pop )

#include <frantic/diagnostics/profiling_section.hpp>
#include <frantic/graphics2d/boundrect2.hpp>
#include <frantic/rendering/deep_attenuation_savers.hpp>
#include <frantic/rendering/depthbuffer_singleface.hpp>
#include <frantic/shading/highlights.hpp>

#include <krakatoa/light_render_element.hpp>
#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/triangle_rasterizer.hpp>
#include <krakatoa/voxel_renderer/default_filter2f.hpp>
#include <krakatoa/voxel_renderer/image_wrapper.hpp>
#include <krakatoa/voxel_renderer/particle_data_source.hpp>
#include <krakatoa/voxel_renderer/sample.hpp>
#include <krakatoa/voxel_renderer/slice_container_impl.hpp>
#include <krakatoa/voxel_renderer/slice_coordsys_impl.hpp>
#include <krakatoa/voxel_renderer/voxel_renderer.hpp>

// temp
#include <frantic/particles/particle_array.hpp>
#include <frantic/particles/particle_file_stream_factory.hpp>

//#define FRANTIC_DISABLE_THREADS

#ifndef FRANTIC_DISABLE_THREADS
#define PROFILING_SECTION_ENTER( section )
#define PROFILING_SECTION_EXIT( section )
#else
#define PROFILING_SECTION_ENTER( section ) ( section ).enter()
#define PROFILING_SECTION_EXIT( section ) ( section ).exit()
#endif

using frantic::graphics::alpha3f;
using frantic::graphics::color3f;
using frantic::graphics::vector3f;
using frantic::graphics2d::boundrect2;
using frantic::graphics2d::vector2;
using frantic::graphics2d::vector2f;

extern frantic::diagnostics::profiling_section g_voxelIndexCreation;
extern int g_filterRadius;

namespace krakatoa {
namespace voxel_renderer {

class renderer_impl : public voxel_renderer {
    struct ray_data {
        frantic::graphics::ray3f ray;
        float prevDensity;
        color_type prevLight;
        color_type prevExtinction;
        // int sliceId; Removed to improve cache performance. There is now a parallel array of ints 'm_rayDataSliceId'.
    };

    class activator : public new_voxel_callback_interface {
        renderer_impl* m_owner;

      public:
        activator( renderer_impl& owner )
            : m_owner( &owner ) {}

        virtual ~activator() {}

        virtual void on_new_voxel( const frantic::graphics2d::boundrect2& bounds ) {
            m_owner->activate_pixels_under_rect( bounds );
        }
    };

    // Object describing the viewing camera
    frantic::graphics::camera<float> m_camera;

    krakatoa::light_object_ptr m_lightObject;

    std::vector<ray_data> m_rayData;
    boost::scoped_array<int> m_rayDataSliceId;

    std::vector<ray_data> m_lightRayData;
    boost::scoped_array<int> m_lightRayDataSliceId;

    boost::shared_ptr<data_source> m_dataSource;
    boost::shared_ptr<slice_container> m_data;
    boost::shared_ptr<slice_coordsys> m_coordsys;

    frantic::channels::channel_map
        m_shaderElementMap; // When evaluating the shaders we want extra output space to get extra data from the shader.

    frantic::rendering::depthbuffer_singleface m_matteDepthMap;

    std::vector<light_render_element_ptr>
        m_lightRenderElements; // Keep these separate because they need special treatment.
    std::vector<particle_render_element_interface_ptr> m_renderElements;

    struct render_element_data {
        particle_render_element_interface_ptr element;
        std::vector<color_type> prevData;
    };

    std::vector<render_element_data> m_renderElementData;

    /*std::vector< particle_render_element_interface_ptr > m_shaderRenderElements;
    std::vector< particle_render_element_interface_ptr > m_basicRenderElements;


    struct shader_exta_data{
            particle_render_element_interface_ptr element;
            std::vector< color_type > prevData;
    };

    std::vector< shader_exta_data > m_shaderElementsExtra;*/

    typedef image_wrapper<pixel_type> image_type;
    typedef frantic::graphics2d::framebuffer<alpha_type> atten_image_type;

    frantic::diagnostics::profiling_section m_psTotal, m_psInit, m_psSplat, m_psRender, m_psSample, m_psCalculate,
        m_psIntersect, m_psCreateRay, m_psBlend;

  private:
    void render_attenuation( const boundrect2& rect, atten_image_type& attenBuffer,
                             frantic::rendering::singleface_atten_saver* pDeepAttenSaver );
    void render_scattered_light( const boundrect2& rect, image_type& outImage, const atten_image_type& curAtten );
    void render_emission( const boundrect2& rect, image_type& outImage );
    void render_elements( const boundrect2& rect, image_type& outImage );

    inline void do_render_attenuation( atten_image_type& attenBuffer,
                                       frantic::rendering::singleface_atten_saver* pDeepAttenSaver );
    inline void do_render_scattered_light( image_type& outImage, const atten_image_type& curAtten );
    inline void do_render_emission( image_type& outImage );
    inline void do_blend_over( image_type& dest, const image_type& src );

    bool check_matte_depthmap( int x, int y, int width, int height, float& t0, float& t1, float rayZComponent ) const;

    void apply_dof( image_type& outImage ); //<-- Not threaded!

    void init_ray_data( frantic::graphics2d::size2 size );

    void init_light_ray_data( frantic::graphics2d::size2 size );

    inline void set_pixel( int x, int y, image_type& outImage, const pixel_type& p ) const;

    inline void activate_pixels_under_rect( const frantic::graphics2d::boundrect2& bounds );

    void tbb_render_attenuation( const tbb::blocked_range2d<int>& range, atten_image_type& attenBuffer,
                                 frantic::rendering::singleface_atten_saver* pDeepAttenSaver ) {
        render_attenuation(
            boundrect2( range.cols().begin(), range.cols().end(), range.rows().begin(), range.rows().end() ),
            attenBuffer, pDeepAttenSaver );
    }

    void tbb_render_scattered_light( const tbb::blocked_range2d<int>& range, image_type& outImage,
                                     const atten_image_type& curAtten ) {
        render_scattered_light(
            boundrect2( range.cols().begin(), range.cols().end(), range.rows().begin(), range.rows().end() ), outImage,
            curAtten );
    }

    void tbb_render_emission( const tbb::blocked_range2d<int>& range, image_type& outImage ) {
        render_emission(
            boundrect2( range.cols().begin(), range.cols().end(), range.rows().begin(), range.rows().end() ),
            outImage );
    }

    static void tbb_blend_over( const tbb::blocked_range2d<int>& range, image_type& dest, const image_type& src ) {
        for( int y = range.rows().begin(), yEnd = range.rows().end(); y < yEnd; ++y ) {
            for( int x = range.cols().begin(), xEnd = range.cols().end(); x < xEnd; ++x )
                dest.blend_over( x, y, src.get_pixel( x, y ) );
        }
    }

    void render_subinterval( voxel_renderer::image_type& outImage );

    void render_subinterval_with_light( voxel_renderer::image_type& outImage, light_object_ptr lightObject,
                                        bool allowDeepAtten = false );

    void render_background( voxel_renderer::image_type& outImage );

  public:
    renderer_impl();

    virtual void add_render_element( render_element_interface_ptr renderElement );

    virtual void precompute_lighting();

    virtual void render( renderer::image_type& outImage );
};

voxel_renderer* voxel_renderer::create_instance() { return new renderer_impl; }

void renderer_impl::do_render_attenuation( renderer_impl::atten_image_type& attenImage,
                                           frantic::rendering::singleface_atten_saver* pDeepAttenSaver ) {
    m_psRender.enter();

    frantic::graphics2d::size2 imgSize = m_lightObject->get_light_impl().shadow_map_size();

#ifndef FRANTIC_DISABLE_THREADS
    tbb::parallel_for(
        tbb::blocked_range2d<int>( 0, imgSize.ysize, 32, 0, imgSize.xsize, 32 ),
        boost::bind( &renderer_impl::tbb_render_attenuation, this, _1, boost::ref( attenImage ), pDeepAttenSaver ) );
#else
    render_attenuation( boundrect2( vector2( 0, 0 ), imgSize ), attenImage, pDeepAttenSaver );
#endif
    m_psRender.exit();
}

void renderer_impl::do_render_scattered_light( renderer_impl::image_type& outImage,
                                               const renderer_impl::atten_image_type& curAtten ) {
#ifndef FRANTIC_DISABLE_THREADS
    tbb::parallel_for( tbb::blocked_range2d<int>( 0, outImage.height(), 32, 0, outImage.width(), 32 ),
                       boost::bind( &renderer_impl::tbb_render_scattered_light, this, _1, boost::ref( outImage ),
                                    boost::ref( curAtten ) ) );
#else
    render_scattered_light( boundrect2( vector2( 0, 0 ), outImage.size() ), outImage, curAtten );
#endif
}

void renderer_impl::do_render_emission( renderer_impl::image_type& outImage ) {
    m_psRender.enter();
#ifndef FRANTIC_DISABLE_THREADS
    tbb::parallel_for( tbb::blocked_range2d<int>( 0, outImage.height(), 32, 0, outImage.width(), 32 ),
                       boost::bind( &renderer_impl::tbb_render_emission, this, _1, boost::ref( outImage ) ) );
#else
    render_emission( boundrect2( vector2( 0, 0 ), outImage.size() ), outImage );
#endif
    m_psRender.exit();
}

void renderer_impl::do_blend_over( image_type& dest, const image_type& src ) {
    m_psBlend.enter();
#ifndef FRANTIC_DISABLE_THREADS
    tbb::parallel_for( tbb::blocked_range2d<int>( 0, dest.height(), 32, 0, dest.width(), 32 ),
                       boost::bind( &renderer_impl::tbb_blend_over, _1, boost::ref( dest ), boost::ref( src ) ) );
#else
    dest.blend_over( src );
#endif
    m_psBlend.exit();
}

renderer_impl::renderer_impl()
    : m_psTotal( _T("Voxel:Total") )
    , m_psInit( _T("Voxel:Init") )
    , m_psSplat( _T("Voxel:Splatting") )
    , m_psRender( _T("Voxel:Rendering") )
    , m_psSample( _T("Voxel:Rendering:Sampling") )
    , m_psCalculate( _T("Voxel:Rendering:Calculating") )
    , m_psIntersect( _T("Voxel:Rendering:Intersecting") )
    , m_psCreateRay( _T("Voxel:Rendering:CreateRay") )
    , m_psBlend( _T("Voxel:Rendering:BlendingBuffers") ) {
    m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_coordsys.reset( new slice_coordsys_impl );

    m_data.reset( new slice_container_impl );
    m_data->set_read_filter( new default_filter2f );
    m_data->set_new_voxel_callback( new activator( *this ) );
}

void renderer_impl::add_render_element( render_element_interface_ptr renderElement ) {
    if( particle_render_element_interface_ptr particleElement =
            boost::dynamic_pointer_cast<particle_render_element_interface>( renderElement ) ) {
        if( light_render_element_ptr lightElement =
                boost::dynamic_pointer_cast<light_render_element>( renderElement ) ) {
            m_lightRenderElements.push_back( lightElement );
        } else {
            m_renderElements.push_back( particleElement );
            // switch( particleElement->get_drawing_type() ){
            // case particle_render_element_interface::draw_type_shader:
            //	m_shaderRenderElements.push_back( particleElement );
            //	break;
            ////case particle_render_element_interface::draw_type_normal:
            // default:
            //	m_basicRenderElements.push_back( particleElement );
            //	break;
            ////default:
            ////	break;
            //}
        }
    }
}

void renderer_impl::precompute_lighting() {}

void renderer_impl::render( renderer::image_type& outImage ) {
    if( !m_particles || !m_sceneContext || !m_shader )
        return;

    m_psTotal.reset();
    m_psInit.reset();
    m_psSplat.reset();
    m_psRender.reset();
    m_psSample.reset();
    m_psCalculate.reset();
    m_psIntersect.reset();
    m_psCreateRay.reset();
    m_psBlend.reset();
    g_voxelIndexCreation.reset();

    m_psTotal.enter();
    m_psInit.enter();

    boost::shared_ptr<particle_data_source> particleSource(
        new particle_data_source( *m_particles, static_cast<std::size_t>( m_doThreadedSorting ? 0u : 1u ) ) );
    particleSource->set_draw_filter( m_particleFilter );
    particleSource->set_progress_logger( m_progress );

    m_dataSource = particleSource;

    frantic::channels::channel_map voxelChannels;

    // Density is a required voxel channel. Sort of.
    voxelChannels.define_channel<float>( _T("Density") );

    const frantic::channels::channel_map& srcMap = m_dataSource->get_channel_map();

    if( srcMap.has_channel( _T("Color") ) /*&& enableColor*/ )
        voxelChannels.define_channel<color3f>( _T("Color") );
    if( srcMap.has_channel( _T("Emission") ) && m_useEmissionChannel )
        voxelChannels.define_channel<color3f>( _T("Emission") );
    if( srcMap.has_channel( _T("Absorption") ) && m_useAbsorptionChannel )
        voxelChannels.define_channel<color3f>( _T("Absorption") );
    if( srcMap.has_channel( _T("Normal") ) && m_environment )
        voxelChannels.define_channel<vector3f>( _T("Normal") );
    if( srcMap.has_channel( _T( "BokehBlendInfluence" ) ) ) {
        voxelChannels.define_channel<float>( _T( "BokehBlendInfluence" ) );
    }

    m_shader->define_required_channels( voxelChannels, frantic::channels::data_type_float32 );

    for( std::vector<particle_render_element_interface_ptr>::iterator it = m_renderElements.begin(),
                                                                      itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        if( ( *it )->get_drawing_type() != particle_render_element_interface::draw_type_shader )
            ( *it )->add_required_channels( voxelChannels );
    }

    // if( voxelChannels.has_channel( "Position" ) )
    //	voxelChannels.delete_channel( "Position" );

    // We want all the channels to be float32 (see slice_container_impl.cpp), so adjust any float16 ones that sneaked
    // through.
    for( std::size_t i = 0, iEnd = voxelChannels.channel_count(); i < iEnd; ++i ) {
        if( !frantic::channels::is_channel_data_type_float( voxelChannels[i].data_type() ) )
            BOOST_THROW_EXCEPTION( std::runtime_error( "Invalid channel for voxel renderer: " +
                                                       frantic::strings::to_string( voxelChannels[i].name() ) ) );

        if( voxelChannels[i].data_type() != frantic::channels::data_type_float32 )
            voxelChannels.set_channel_data_type( voxelChannels[i].name(), frantic::channels::data_type_float32 );
    }

    voxelChannels.end_channel_definition();

    m_data->reset( voxelChannels );

    // We only want to add the channels for the shader render elements to a special channel map, so that we don't
    // allocate voxel space for it. This requires a bit of trickery since element::add_required_channels() wants
    // a non-finished channel map.
    m_shaderElementMap = voxelChannels;

    // Shader render elements will add extra channels for the shader to write to. I do it here, so that
    frantic::channels::channel_map shaderElementChannels;
    for( std::vector<particle_render_element_interface_ptr>::iterator it = m_renderElements.begin(),
                                                                      itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        if( ( *it )->get_drawing_type() == particle_render_element_interface::draw_type_shader )
            ( *it )->add_required_channels( shaderElementChannels );
    }

    for( std::size_t i = 0, iEnd = shaderElementChannels.channel_count(); i < iEnd; ++i ) {
        if( !m_shaderElementMap.has_channel( shaderElementChannels[i].name() ) )
            m_shaderElementMap.append_channel( shaderElementChannels[i].name(), shaderElementChannels[i].arity(),
                                               shaderElementChannels[i].data_type() );
    }

    m_shader->set_channel_map( m_shaderElementMap );

    m_camera = m_sceneContext->get_camera();
    m_doDOF = m_dofEnabled && m_sceneContext->get_camera().has_dof();

    outImage.set_size( m_camera.get_output_size() );
    outImage.fill( pixel_type( 0 ) );

    bool hasCameraPassRenderElements = false;

    for( std::vector<particle_render_element_interface_ptr>::iterator it = m_renderElements.begin(),
                                                                      itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        ( *it )->get_framebuffer().set_size( m_camera.get_output_size() );
        ( *it )->get_framebuffer().fill( pixel_type( 0 ) );
        if( ( *it )->get_drawing_type() == particle_render_element_interface::draw_type_shader )
            ( *it )->set_channel_map( m_shaderElementMap );
        else {
            ( *it )->set_channel_map( voxelChannels );
            hasCameraPassRenderElements = true;
        }
        ( *it )->initialize();
    }

    if( m_renderMode == renderer::mode_type::normal ) {
        for( int i = 0, iEnd = (int)m_sceneContext->get_light_objects().size(); i < iEnd; ++i ) {
            light_object_ptr light = m_sceneContext->get_light_objects().get( i );
            if( !light->get_light_impl().is_directional_light() )
                throw std::runtime_error(
                    "Krakatoa does not support point lights in voxel rendering mode. Please disable light: \"" +
                    frantic::strings::to_string( light->get_light_impl().name() ) + "\"" );

            light->begin( m_sceneContext );
        }

        for( std::vector<light_render_element_ptr>::iterator it = m_lightRenderElements.begin(),
                                                             itEnd = m_lightRenderElements.end();
             it != itEnd; ++it ) {
            ( *it )->get_framebuffer().set_size( m_camera.get_output_size() );
            ( *it )->get_framebuffer().fill( pixel_type( 0 ) );
            ( *it )->initialize();
        }
    }

    int totalPasses = 0;

    if( m_adaptiveMblur && m_particles->get_channel_map().has_channel( _T( "Velocity" ) ) ) {
        m_mblurSamples = get_adaptive_motion_blur_passes();
        FF_LOG( stats ) << "Adaptive motion blur is on. Using " << m_mblurSamples << " motion blur samples."
                        << std::endl;
    }

    int numMBlurPasses = std::max( 1, m_mblurSamples );

    if( m_renderMode == renderer::mode_type::normal )
        totalPasses += numMBlurPasses * (int)m_sceneContext->get_light_objects().size();

    bool doCameraPass = ( m_environment || m_renderMode == renderer::mode_type::additive ||
                          m_particles->get_channel_map().has_channel( _T("Emission") ) || hasCameraPassRenderElements );

    if( doCameraPass )
        totalPasses += numMBlurPasses;

    float numLightPasses = (float)( numMBlurPasses * (int)m_sceneContext->get_light_objects().size() );

    frantic::graphics2d::framebuffer<frantic::graphics::color6f> passBuffer( outImage.size() );
    frantic::graphics2d::framebuffer<frantic::graphics::color6f> backBuffer( outImage.size() );

    m_renderElementData.clear();

    for( std::vector<particle_render_element_interface_ptr>::iterator it = m_renderElements.begin(),
                                                                      itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        m_renderElementData.push_back( render_element_data() );
        m_renderElementData.back().element = particle_render_element_interface_ptr(
            static_cast<particle_render_element_interface*>( ( *it )->clone() ) );
    }

    m_psInit.exit();

    int totalPassCounter = 0;
    for( int mblurPass = 0; mblurPass < numMBlurPasses; ++mblurPass ) {
        bool isCanonicalTime = ( mblurPass == numMBlurPasses / 2 );

        float mblurTime = ( (float)mblurPass + 0.5f ) / (float)numMBlurPasses;

        if( m_matteSampler ) {
            m_matteDepthMap.set_size( m_sceneContext->get_camera().get_output_size() * m_matteSuperSampling );
            m_matteSampler->generate_depth_map( m_sceneContext->get_camera(), mblurTime,
                                                m_matteDepthMap.as_framebuffer(), false );
        }

        m_mblurTime = mblurTime;
        m_mblurRange = m_mblurSamples > 0 ? 1.f / (float)numMBlurPasses : 0.f;

        // We have a current motion blue time [0,1] and optionally a width [0,1] to sample over. Convert these
        // quatities to seconds.
        float mblurTimeSeconds = ( -0.5f * ( 1 - m_mblurBias ) + m_mblurTime ) * m_mblurDuration;
        float mblurRangeSeconds = m_mblurRange * m_mblurDuration;

        if( !m_mblurJittered )
            m_mblurRange = 0;

        // Update the particle source to the current time.
        particleSource->set_motion_blur_time( mblurTimeSeconds, mblurRangeSeconds, 42 * totalPassCounter );

        if( m_renderMode == renderer::mode_type::normal ) {
            for( int i = 0, iEnd = (int)m_sceneContext->get_light_objects().size(); i < iEnd;
                 ++i, ++totalPassCounter ) {
                passBuffer.fill( pixel_type( 0 ) );

                for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                itEnd = m_renderElementData.end();
                     it != itEnd; ++it ) {
                    if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader ) {
                        it->element->get_framebuffer().fill( pixel_type( 0 ) );
                        it->prevData.assign( m_camera.get_output_size().get_area(), color_type( 0 ) );
                    }
                }

                light_object_ptr light = m_sceneContext->get_light_objects().get( i );

                light->update( mblurTime );

                frantic::logging::progress_logger_subinterval_tracker lightProgIv(
                    *m_progress, 100.f * (float)totalPassCounter / (float)totalPasses,
                    100.f * (float)( totalPassCounter + 1 ) / (float)totalPasses );
                m_progress->set_title( _T("Rendering pass ") + boost::lexical_cast<frantic::tstring>( mblurPass + 1 ) +
                                       _T(" of ") + boost::lexical_cast<frantic::tstring>( numMBlurPasses ) +
                                       _T(" for light ") + light->get_light_impl().name() );

                render_subinterval_with_light( passBuffer, light, isCanonicalTime );

                // Add the incoming light into the render buffer
                for( int y = 0; y < outImage.height(); ++y ) {
                    for( int x = 0; x < outImage.width(); ++x ) {
                        pixel_type bufferVal = passBuffer.get_pixel( x, y );
                        outImage.at( x, y ) +=
                            pixel_type( bufferVal.c / (float)numMBlurPasses, bufferVal.a / (float)totalPasses );
                    }
                }

                // Look to see if there is a render element that wants to accumulate only this light's drawing.
                for( std::vector<light_render_element_ptr>::iterator it = m_lightRenderElements.begin(),
                                                                     itEnd = m_lightRenderElements.end();
                     it != itEnd; ++it ) {
                    if( ( *it )->get_light_name() == light->get_light_impl().name() ) {
                        renderer::image_type& fb = ( *it )->get_framebuffer();

                        // Add the incoming light into the render buffer
                        for( int y = 0; y < outImage.height(); ++y ) {
                            for( int x = 0; x < outImage.width(); ++x ) {
                                pixel_type bufferVal = passBuffer.get_pixel( x, y );
                                fb.at( x, y ) += pixel_type( bufferVal.c / (float)numMBlurPasses,
                                                             bufferVal.a / (float)numMBlurPasses );
                            }
                        }

                        break;
                    }
                }

                std::vector<particle_render_element_interface_ptr>::iterator itDest = m_renderElements.begin();

                for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                itEnd = m_renderElementData.end();
                     it != itEnd; ++it, ++itDest ) {
                    if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader ) {
                        renderer::image_type& src = it->element->get_framebuffer();
                        renderer::image_type& dest = ( *itDest )->get_framebuffer();

                        // Add the incoming light into the render buffer
                        for( int y = 0; y < outImage.height(); ++y ) {
                            for( int x = 0; x < outImage.width(); ++x ) {
                                pixel_type bufferVal = src.get_pixel( x, y );
                                dest.at( x, y ) +=
                                    pixel_type( bufferVal.c / (float)numMBlurPasses, bufferVal.a / numLightPasses );
                            }
                        }
                    }
                }
            }
        }

        if( doCameraPass ) {
            frantic::logging::progress_logger_subinterval_tracker lightProgIv(
                *m_progress, 100.f * (float)totalPassCounter / (float)totalPasses,
                100.f * (float)( totalPassCounter + 1 ) / (float)totalPasses );
            m_progress->set_title( _T("Rendering pass ") + boost::lexical_cast<frantic::tstring>( mblurPass + 1 ) +
                                   _T(" of ") + boost::lexical_cast<frantic::tstring>( numMBlurPasses ) +
                                   _T(" for camera") );

            passBuffer.fill( pixel_type( 0 ) );

            for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                            itEnd = m_renderElementData.end();
                 it != itEnd; ++it ) {
                if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader ) {
                    it->element->get_framebuffer().fill( pixel_type( 0 ) );

                    if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_normal )
                        it->prevData.assign( m_camera.get_output_size().get_area(), color_type( 0 ) );
                }
            }

            render_subinterval( passBuffer );

            // We need to render the background layer, unless its a simple solid color.
            if( m_environment )
                render_background( passBuffer );

            for( int y = 0; y < outImage.height(); ++y ) {
                for( int x = 0; x < outImage.width(); ++x ) {
                    pixel_type bufferVal = passBuffer.get_pixel( x, y );
                    outImage.at( x, y ) +=
                        pixel_type( bufferVal.c / (float)numMBlurPasses, bufferVal.a / (float)totalPasses );
                }
            }

            std::vector<particle_render_element_interface_ptr>::iterator itDest = m_renderElements.begin();

            for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                            itEnd = m_renderElementData.end();
                 it != itEnd; ++it, ++itDest ) {
                if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader ) {
                    renderer::image_type& src = it->element->get_framebuffer();
                    renderer::image_type& dest = ( *itDest )->get_framebuffer();

                    // Add the incoming light into the render buffer
                    for( int y = 0; y < outImage.height(); ++y ) {
                        for( int x = 0; x < outImage.width(); ++x ) {
                            pixel_type bufferVal = src.get_pixel( x, y );
                            dest.at( x, y ) +=
                                pixel_type( bufferVal.c / (float)numMBlurPasses, bufferVal.a / (float)numMBlurPasses );
                        }
                    }
                }
            }

            ++totalPassCounter;
        }
    }

    // This temp data is not needed anymore.
    m_renderElementData.clear();

    // If we didn't do a camera pass, that means we just dump a single render of the background.
    if( !doCameraPass || !m_environment )
        outImage.fill_under( m_backgroundColor );

    if( m_watermarkFn )
        m_watermarkFn( outImage );

    for( std::vector<light_render_element_ptr>::iterator it = m_lightRenderElements.begin(),
                                                         itEnd = m_lightRenderElements.end();
         it != itEnd; ++it ) {
        if( m_watermarkFn )
            m_watermarkFn( ( *it )->get_framebuffer() );
        ( *it )->commit();
    }

    for( std::vector<particle_render_element_interface_ptr>::iterator it = m_renderElements.begin(),
                                                                      itEnd = m_renderElements.end();
         it != itEnd; ++it ) {
        if( m_watermarkFn )
            m_watermarkFn( ( *it )->get_framebuffer() );
        ( *it )->commit();
    }

    if( m_renderMode == renderer::mode_type::normal ) {
        for( int i = 0, iEnd = (int)m_sceneContext->get_light_objects().size(); i < iEnd; ++i )
            m_sceneContext->get_light_objects().get( i )->end();
    }

    m_psTotal.exit();

    FF_LOG( stats ) << m_psTotal << _T("\n") << m_psInit << _T("\n") << m_psSplat << _T("\n") << g_voxelIndexCreation
                    << _T("\n") << m_psRender << _T("\n") << m_psSample << _T("\n") << m_psCreateRay << _T("\n")
                    << m_psIntersect << _T("\n") << m_psCalculate << _T("\n") << m_psBlend << std::endl;
}

void renderer_impl::init_ray_data( frantic::graphics2d::size2 size ) {
    m_rayData.resize( static_cast<std::size_t>( size.get_area() ) );
    m_rayDataSliceId.reset( new int[size.get_area()] );

    for( int y = 0; y < size.ysize; ++y ) {
        for( int x = 0; x < size.xsize; ++x ) {
            ray_data& rd = m_rayData[size.get_index( x, y )];

            bool isValid = true;
            rd.ray = m_camera.get_worldspace_ray( vector2f( (float)x + 0.5f, (float)y + 0.5f ), m_mblurTime, isValid );
            rd.prevDensity = 0.f;
            rd.prevExtinction = color_type::black();
            rd.prevLight = color_type::black();
            // rd.sliceId = std::numeric_limits<int>::max();

            m_rayDataSliceId[size.get_index( x, y )] = std::numeric_limits<int>::max();
        }
    }
}

void renderer_impl::init_light_ray_data( frantic::graphics2d::size2 size ) {
    m_lightRayData.resize( static_cast<std::size_t>( size.get_area() ) );
    m_lightRayDataSliceId.reset( new int[size.get_area()] );

    const frantic::graphics::camera<float>& lightCam =
        dynamic_cast<const frantic::rendering::lights::directedlight_interface&>( m_lightObject->get_light_impl() )
            .get_camera();

    for( int y = 0; y < size.ysize; ++y ) {
        for( int x = 0; x < size.xsize; ++x ) {
            ray_data& rd = m_lightRayData[size.get_index( x, y )];

            bool isValid = true;
            rd.ray = lightCam.get_worldspace_ray( vector2f( (float)x + 0.5f, (float)y + 0.5f ), m_mblurTime, isValid );
            rd.prevDensity = 0.f;
            rd.prevExtinction = color_type::black();
            rd.prevLight = color_type::black();
            // rd.sliceId = std::numeric_limits<int>::max();

            m_lightRayDataSliceId[size.get_index( x, y )] = std::numeric_limits<int>::max();
        }
    }
}

void renderer_impl::render_subinterval( voxel_renderer::image_type& outImage ) {
    m_lightObject.reset();

    // Wrap the output image in an image_wrapper object that improves performance.
    image_type sliceImage( outImage );

    init_ray_data( outImage.size() );

    int camSlice = m_coordsys->reset( m_camera.world_transform( m_mblurTime ), m_voxelSize );

    // Find the range of slices that need to be processed.
    std::pair<int, int> dataRange = m_dataSource->reset( m_coordsys );

    int nearSlice = std::min( dataRange.first, camSlice );
    int farSlice = dataRange.second;

    m_psInit.exit();

    bool lastHadData = false;

    int counter = 0;
    int updateCount = ( nearSlice - farSlice + 1 ) / 10;
    if( updateCount <= 0 )
        updateCount = 1;

    renderer::image_type tempBuffer;
    std::vector<renderer::image_type> tempElementBuffers( m_renderElementData.size() );

    if( m_doDOF ) {
        tempBuffer.set_size( sliceImage.size() );

        std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                        itEnd = m_renderElementData.end();
             it != itEnd; ++it, ++itTemp ) {
            if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader )
                itTemp->set_size( m_camera.output_size() );
        }
    }

    for( int slice = nearSlice; slice >= farSlice; --slice ) {
        m_coordsys->set_slice_index( slice );

        m_psSplat.enter();
        bool nextHasData = m_dataSource->do_step( m_data );
        m_psSplat.exit();

        if( !lastHadData && !nextHasData )
            continue;
        lastHadData = nextHasData;

        if( m_doDOF ) {
            image_type tempImage( tempBuffer );

            tempBuffer.fill( pixel_type( 0 ) );

            for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                            itEnd = m_renderElementData.end();
                 it != itEnd; ++it ) {
                if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader )
                    it->element->get_framebuffer().fill( pixel_type( 0 ) );
            }

            do_render_emission( tempImage );

            apply_dof( tempImage );
            do_blend_over( sliceImage, tempImage );

            std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
            for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                            itEnd = m_renderElementData.end();
                 it != itEnd; ++it, ++itTemp ) {
                if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader ) {
                    image_type src( it->element->get_framebuffer() ), dest( *itTemp );

                    apply_dof( src );
                    do_blend_over( dest, src );
                }
            }
        } else {
            do_render_emission( sliceImage );
        }

        // Update framebuffer
        int count = nearSlice - slice;
        int total = nearSlice - farSlice + 1;

        m_progress->update_progress( count, total );
        if( ++counter == updateCount ) {
            counter = 0;
            if( m_watermarkFn )
                m_watermarkFn( outImage );
            m_progress->update_frame_buffer( outImage );
        }
    }

    if( m_doDOF ) {
        std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                        itEnd = m_renderElementData.end();
             it != itEnd; ++it, ++itTemp ) {
            if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader )
                itTemp->swap( it->element->get_framebuffer() );
        }
    }
}

void renderer_impl::render_subinterval_with_light( voxel_renderer::image_type& outImage, light_object_ptr lightObject,
                                                   bool /*allowDeepAtten*/ ) {
    m_lightObject = lightObject;

    // Wrap the output image in an image_wrapper object that improves performance.
    image_type sliceImage( outImage );

    atten_image_type attenImage( m_lightObject->get_light_impl().shadow_map_size() );
    attenImage.set_draw_point_filter( frantic::graphics2d::draw_point_filter::bicubic );
    attenImage.set_pixel_lookup_filter( frantic::graphics2d::pixel_lookup_filter::bicubic_filter );

    // get our attenuation saver object (note that this is always a single face saver. no cube maps.)
    frantic::rendering::singleface_atten_saver* deepAttenSaver = NULL;
    if( m_lightObject->get_light_impl().get_attenuation_saver() &&
        !m_lightObject->get_light_impl().get_attenuation_saver()->is_cubeface() )
        deepAttenSaver = static_cast<frantic::rendering::singleface_atten_saver*>(
            m_lightObject->get_light_impl().get_attenuation_saver().get() );

    init_ray_data( outImage.size() );
    init_light_ray_data( attenImage.size() );

    // Initialize the slicing plane, and transforms from world-space to slice-space.
    std::pair<int, int> camLightRange =
        m_coordsys->reset( m_camera.world_transform( m_mblurTime ),
                           m_lightObject->get_light_impl().transform_matrix( m_mblurTime ), m_voxelSize );

    int camSlice = camLightRange.first;
    int lightSlice = camLightRange.second;

    // Find the range of slices that need to be processed.
    std::pair<int, int> dataRange = m_dataSource->reset( m_coordsys );

    int nearSlice = dataRange.first;
    int farSlice = dataRange.second;

    renderer::image_type tempBuffer;
    std::vector<renderer::image_type> tempElementBuffers( m_renderElementData.size() );

    if( m_doDOF ) {
        tempBuffer.set_size( sliceImage.size() );

        std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                        itEnd = m_renderElementData.end();
             it != itEnd; ++it, ++itTemp ) {
            if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader )
                itTemp->set_size( m_camera.output_size() );
        }
    }

    if( m_coordsys->is_forward() ) {
        // Check that something is actually visible
        if( nearSlice >= farSlice && farSlice <= camSlice ) {
            bool lastHadData = false;

            int slice = nearSlice;

            for( /*nothing*/; slice > camSlice; --slice ) {
                m_coordsys->set_slice_index( slice );

                m_psSplat.enter();
                bool nextHasData = m_dataSource->do_step( m_data );
                m_psSplat.exit();

                if( !lastHadData && !nextHasData )
                    continue;
                lastHadData = nextHasData;

                do_render_attenuation( attenImage, deepAttenSaver );

                // Update framebuffer
                int count = nearSlice - slice;
                int total = nearSlice - farSlice + 1;

                m_progress->update_progress( count, total );
            }

            int counter = 0;
            int updateCount = ( slice - farSlice + 1 ) / 10;
            if( updateCount <= 0 )
                updateCount = 1;

            for( /*nothing*/; slice >= farSlice; --slice ) {
                m_coordsys->set_slice_index( slice );

                m_psSplat.enter();
                bool nextHasData = m_dataSource->do_step( m_data );
                m_psSplat.exit();

                if( !lastHadData && !nextHasData )
                    continue;
                lastHadData = nextHasData;

                if( slice <= lightSlice )
                    do_render_attenuation( attenImage, deepAttenSaver );

                if( m_doDOF ) {
                    image_type tempImage( tempBuffer );

                    tempBuffer.fill( pixel_type( 0 ) );

                    for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                    itEnd = m_renderElementData.end();
                         it != itEnd; ++it ) {
                        if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader )
                            it->element->get_framebuffer().fill( pixel_type( 0 ) );
                    }

                    do_render_scattered_light( tempImage, attenImage );

                    apply_dof( tempImage );
                    do_blend_over( sliceImage, tempImage );

                    std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
                    for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                    itEnd = m_renderElementData.end();
                         it != itEnd; ++it, ++itTemp ) {
                        if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader ) {
                            image_type src( it->element->get_framebuffer() ), dest( *itTemp );

                            apply_dof( src );
                            do_blend_over( dest, src );
                        }
                    }
                } else {
                    do_render_scattered_light( sliceImage, attenImage );
                }

                // Update framebuffer
                int count = nearSlice - slice;
                int total = nearSlice - farSlice + 1;

                m_progress->update_progress( count, total );
                if( ++counter == updateCount ) {
                    counter = 0;
                    if( m_watermarkFn )
                        m_watermarkFn( outImage );
                    m_progress->update_frame_buffer( outImage );
                }
            }
        }
    } else {
        if( camSlice > farSlice )
            farSlice = camSlice;

        // Check that something is actually visible. near is close to light, far is closer to camera.
        if( nearSlice >= farSlice && nearSlice >= camSlice ) {
            bool lastHadData = false;

            int counter = 0;
            int updateCount = ( nearSlice - farSlice + 1 ) / 10;
            if( updateCount <= 0 )
                updateCount = 1;

            for( int slice = nearSlice; slice >= farSlice; --slice ) {
                m_coordsys->set_slice_index( slice );

                m_psSplat.enter();
                bool nextHasData = m_dataSource->do_step( m_data );
                m_psSplat.exit();

                if( !lastHadData && !nextHasData )
                    continue;
                lastHadData = nextHasData;

                if( slice <= lightSlice )
                    do_render_attenuation( attenImage, deepAttenSaver );

                if( m_doDOF ) {
                    image_type tempImage( tempBuffer );

                    tempBuffer.fill( pixel_type( 0 ) );

                    for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                    itEnd = m_renderElementData.end();
                         it != itEnd; ++it ) {
                        if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader )
                            it->element->get_framebuffer().fill( pixel_type( 0 ) );
                    }

                    do_render_scattered_light( tempImage, attenImage );

                    apply_dof( tempImage );
                    do_blend_over( sliceImage, tempImage );

                    std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
                    for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                    itEnd = m_renderElementData.end();
                         it != itEnd; ++it, ++itTemp ) {
                        if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader ) {
                            image_type src( it->element->get_framebuffer() ), dest( *itTemp );

                            apply_dof( src );
                            do_blend_over( dest, src );
                        }
                    }
                } else {
                    do_render_scattered_light( sliceImage, attenImage );
                }

                // Update framebuffer
                int count = nearSlice - slice;
                int total = nearSlice - farSlice + 1;

                m_progress->update_progress( count, total );
                if( ++counter == updateCount ) {
                    counter = 0;
                    if( m_watermarkFn )
                        m_watermarkFn( outImage );
                    m_progress->update_frame_buffer( outImage );
                }
            }
        }
    }

    if( deepAttenSaver )
        deepAttenSaver->write_file();

    if( m_doDOF ) {
        std::vector<renderer::image_type>::iterator itTemp = tempElementBuffers.begin();
        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                        itEnd = m_renderElementData.end();
             it != itEnd; ++it, ++itTemp ) {
            if( it->element->get_drawing_type() == particle_render_element_interface::draw_type_shader )
                itTemp->swap( it->element->get_framebuffer() );
        }
    }
}

void renderer_impl::render_background( voxel_renderer::image_type& outImage ) {
    if( m_environment && !m_defaultBackground ) {
        m_environment->render_background( m_camera, outImage, m_mblurTime );
    } else {
        outImage.fill_under( m_backgroundColor );
    }
}

void renderer_impl::render_attenuation( const boundrect2& rect, renderer_impl::atten_image_type& attenBuffer,
                                        frantic::rendering::singleface_atten_saver* pDeepAttenSaver ) {
    sample sample;
    m_data->construct_sample( sample );

    frantic::channels::channel_accessor<float> densityAccessor =
        sample.get_channel_map().get_accessor<float>( _T("Density") );
    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::black() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;

    if( sample.has_property( _T("Color") ) )
        colorAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Color") );

    if( sample.has_property( _T("Absorption") ) )
        absorptionAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Absorption") );

    int sliceIndex = m_coordsys->get_slice_index();
    int sliceIndexPlusOne = sliceIndex + 1;

    const frantic::graphics::camera<float>& lightCam =
        static_cast<frantic::rendering::lights::directedlight_interface&>( m_lightObject->get_light_impl() )
            .get_camera();

    vector3f viewDirection = vector3f::normalize( lightCam.view_direction( m_mblurTime ) );

    for( int y = rect.minimum().y; y < rect.maximum().y; ++y ) {
        for( int x = rect.minimum().x; x < rect.maximum().x; ++x ) {
            int pixelId = x + attenBuffer.width() * y;

            // This relies on sliceIndex always getting more negative.
            if( m_lightRayDataSliceId[pixelId] <= sliceIndexPlusOne ) {
                ray_data& rd = m_lightRayData[pixelId];
                frantic::graphics::ray3f& pixelRay = rd.ray;

                PROFILING_SECTION_ENTER( m_psIntersect );
                std::pair<float, float> rayIntersect = m_coordsys->intersect_ray_with_slice( pixelRay );

                if( m_lightRayDataSliceId[pixelId] == m_coordsys->get_slice_index() ) {
                    frantic::graphics::vector3f worldPos = pixelRay.at( rayIntersect.second );
                    frantic::graphics::vector3f voxelPos = m_coordsys->transform_from_world( worldPos );
                    frantic::graphics2d::vector2f voxelPos2D( voxelPos.x, voxelPos.y );
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    PROFILING_SECTION_ENTER( m_psSample );
                    m_data->get_sample( voxelPos2D, sample );
                    PROFILING_SECTION_EXIT( m_psSample );
                } else {
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    // Set all values to zero, since there wasn't anything to sample.
                    memset( (char*)sample.get_raw_buffer(), 0, sample.get_channel_map().structure_size() );
                }

                float density = densityAccessor.get( sample.get_raw_buffer() );
                float prevDensity = rd.prevDensity;

                if( density > 0 || prevDensity > 0 ) {
                    rd.prevDensity = density;

                    float step = fabsf( rayIntersect.second - rayIntersect.first ) *
                                 pixelRay.direction().get_magnitude() * m_voxelSize;
                    float scalarFactor =
                        -0.5f * step * m_lightDensityScale * m_lightObject->get_light_impl().shadow_density();

                    alpha_type alpha;
                    if( !absorptionAccessor.is_valid() ) {
                        alpha.ar = alpha.ag = alpha.ab = 1 - std::exp( ( density + prevDensity ) * scalarFactor );
                    } else {
                        color3f color = colorAccessor.get( sample.get_raw_buffer() );
                        if( colorAccessor.is_default() )
                            color *= density;

                        color3f extinction = color + absorptionAccessor.get(
                                                         sample.get_raw_buffer() ); // Has density multiplied in already
                        color3f prevExtinction = rd.prevExtinction;

                        rd.prevExtinction = extinction;

                        alpha.ar = 1 - std::exp( ( prevExtinction.r + extinction.r ) * scalarFactor );
                        alpha.ag = 1 - std::exp( ( prevExtinction.g + extinction.g ) * scalarFactor );
                        alpha.ab = 1 - std::exp( ( prevExtinction.b + extinction.b ) * scalarFactor );
                    }

                    attenBuffer.blend_over( x, y, alpha );

                    if( pDeepAttenSaver ) {
                        float zDepth = vector3f::dot( pixelRay.direction(), viewDirection ) * rayIntersect.second;
                        pDeepAttenSaver->add_sample( x, y, zDepth, alpha );
                    }
                }
            }
        } // x
    }     // y
}

/**
 * This function does recursive Simpson's rule integration of of a linearly interpolated density function and light
 * function. It recursively divides the interval until it an absolute error tolerance is exceeded. This recurses on two
 * intervals to only require extensive recursion on parts of the interval that require it, since a exp() function tends
 * to bias towards a small region at one of the interval.
 *
 * This solves the equation: integral(0,1)[ (lightA + x (lightB - lightA)) * e^-integral(0,x){extinctionA + y
 * (extinctionB - extinctionA) dy} dx ] by using integral(0,y){extinctionA + t (extinctionB - extinctionA) dt} = y *
 * extinctionA + 0.5 y^2 (extinctionB - extinctionA)
 *
 * @param lightA the amount of light reflected at the start of the interval
 * @param lightB the amount of light reflected at the end of the interval
 * @param extinctionA the coefficient of extinction at the start of the interval
 * @param extinctionB the coefficient of exrinction at the end of the interval
 * @return the total amount of reflected light, and the alpha transparency associated with this interval. These values
 * can be considered pre-multiplied color values for the purposes of alpha-blending.
 */
inline std::pair<float, float> integrate_scattered_light( float lightA, float lightB, float extinctionA,
                                                          float extinctionB ) {
    struct impl {
        float lightA, lightB;
        float extinctionA, extinctionB;

        typedef float value_type;
        typedef boost::call_traits<value_type>::param_type param_type;

        std::pair<value_type, float> apply() {
            float endT = std::exp( -0.5f * ( extinctionA + extinctionB ) );

            value_type middleLight = std::exp( -( 0.5f * extinctionA + 0.125f * ( extinctionB - extinctionA ) ) ) *
                                     0.5f * ( lightA + lightB );
            value_type endLight = endT * lightB;

            value_type sum = lightA + endLight + 4 * middleLight;

            return std::pair<value_type, float>( apply_recursive( 0, 0, 1.f, lightA, middleLight, endLight, sum ) / 6.f,
                                                 1.f - endT );
        }

        value_type apply_recursive( int depth, float a, float b, param_type left, param_type middle, param_type right,
                                    param_type sum ) {
            float tLeft = a + 0.25f * ( b - a );
            float tLeft2_2 = 0.5f * tLeft * tLeft;

            value_type leftLight = std::exp( -( tLeft * extinctionA + tLeft2_2 * ( extinctionB - extinctionA ) ) ) *
                                   ( lightA + tLeft * ( lightB - lightA ) );
            value_type leftSum = left + middle + 4 * leftLight;

            float tRight = a + 0.75f * ( b - a );
            float tRight2_2 = 0.5f * tRight * tRight;

            value_type rightLight = std::exp( -( tRight * extinctionA + tRight2_2 * ( extinctionB - extinctionA ) ) ) *
                                    ( lightA + tRight * ( lightB - lightA ) );
            value_type rightSum = middle + right + 4 * rightLight;

            value_type totalSum = 0.5f * ( leftSum + rightSum );

            if( depth > 10 || fabsf( sum - totalSum ) < 1e-3f )
                return totalSum;

            float result;

            float c = 0.5f * ( a + b );
            result = apply_recursive( depth + 1, a, c, left, leftLight, middle, leftSum );
            result += apply_recursive( depth + 1, c, b, middle, rightLight, right, rightSum );

            return 0.5f * result;
        }
    } theImpl;

    theImpl.lightA = lightA;
    theImpl.lightB = lightB;
    theImpl.extinctionA = extinctionA;
    theImpl.extinctionB = extinctionB;

    return theImpl.apply();
}

inline void prep_sample_for_shader( float density, sample& inoutSample ) {
    // Prep the sample for the shader (ie. divide the Density out of everything)
    for( std::size_t i = 0, iEnd = inoutSample.get_channel_map().channel_count(); i < iEnd; ++i ) {
        const frantic::channels::channel& ch = inoutSample.get_channel_map()[i];
        if( ch.name() != _T("Density") && ch.name() != _T("Emission") ) {
            float* pData = reinterpret_cast<float*>( (char*)inoutSample.get_raw_buffer() + ch.offset() );
            for( std::size_t j = 0, jEnd = ch.arity(); j < jEnd; ++j )
                pData[j] /= density;
        }
    }
}

void renderer_impl::render_scattered_light( const boundrect2& rect, renderer_impl::image_type& outImage,
                                            const renderer_impl::atten_image_type& curAtten ) {
    sample sample;
    // m_data->construct_sample( sample ); //This was a better architecture, but I wanted to craft in render elements in
    // a bizarre way so I broke this.

    sample.reset( m_shaderElementMap );

    frantic::channels::channel_accessor<float> densityAccessor =
        sample.get_channel_map().get_accessor<float>( _T("Density") );
    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::white() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;

    if( sample.has_property( _T("Color") ) )
        colorAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Color") );

    if( sample.has_property( _T("Absorption") ) )
        absorptionAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Absorption") );

    const frantic::graphics::camera<float>& lightCamera =
        static_cast<const frantic::rendering::lights::directedlight_interface&>( m_lightObject->get_light_impl() )
            .get_camera();

    vector3f viewDirection = m_camera.view_direction( m_mblurTime );

    int sliceIndex = m_coordsys->get_slice_index();
    int sliceIndexPlusOne = sliceIndex + 1;

    for( int y = rect.minimum().y; y < rect.maximum().y; ++y ) {
        for( int x = rect.minimum().x; x < rect.maximum().x; ++x ) {

            int pixelId = x + outImage.width() * y;

            // This relies on sliceIndex always getting more negative.
            if( m_rayDataSliceId[pixelId] <= sliceIndexPlusOne ) {
                ray_data& rd = m_rayData[pixelId];
                frantic::graphics::ray3f& pixelRay = rd.ray;

                PROFILING_SECTION_ENTER( m_psIntersect );
                std::pair<float, float> rayIntersect = m_coordsys->intersect_ray_with_slice( pixelRay );

                // We may need to shorten the interval to account for matte objects.
                float zComponent = vector3f::dot( pixelRay.direction(), viewDirection );
                if( !check_matte_depthmap( x, y, outImage.width(), outImage.height(), rayIntersect.first,
                                           rayIntersect.second, zComponent ) )
                    continue;

                frantic::graphics::vector3f worldPos = pixelRay.at( rayIntersect.second );

                if( m_rayDataSliceId[pixelId] == m_coordsys->get_slice_index() ) {
                    frantic::graphics::vector3f voxelPos = m_coordsys->transform_from_world( worldPos );
                    frantic::graphics2d::vector2f voxelPos2D( voxelPos.x, voxelPos.y );
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    PROFILING_SECTION_ENTER( m_psSample );
                    m_data->get_sample( voxelPos2D, sample );
                    m_shader->set_particle_defaults(
                        sample.get_raw_buffer() ); // This zeroes out the parts of sample that are used for accumulating
                                                   // render element stuff.
                    PROFILING_SECTION_EXIT( m_psSample );
                } else {
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    // Set all values to zero, since there wasn't anything to sample.
                    memset( (char*)sample.get_raw_buffer(), 0, sample.get_channel_map().structure_size() );
                }

                float density = densityAccessor.get( sample.get_raw_buffer() );
                float prevDensity = rd.prevDensity;

                if( density > 0 || prevDensity > 0 ) {
                    rd.prevDensity = density;

                    color3f color = colorAccessor.get( sample.get_raw_buffer() );
                    color3f absorb;

                    if( colorAccessor.is_default() )
                        color *= density;

                    if( absorptionAccessor.is_valid() )
                        absorb = absorptionAccessor.get( sample.get_raw_buffer() );

                    float step = fabsf( rayIntersect.second - rayIntersect.first ) *
                                 pixelRay.direction().get_magnitude() * m_voxelSize;

                    color3f lightVal( 0 );

                    if( density > 0 ) {
                        lightVal = m_lightObject->eval_lighting( worldPos );

                        bool isValid = true;
                        vector3f lightSpacePos = lightCamera.world_transform_inverse( m_mblurTime ) * worldPos;
                        vector2f lightScreenPos = lightCamera.from_cameraspace_position( lightSpacePos, isValid );

                        lightVal = curAtten.get_pixel_filtered( lightScreenPos ).occlude( lightVal );

                        if( lightVal.r > 0 || lightVal.g > 0 || lightVal.b > 0 ) {
                            vector3f lightPos = m_lightObject->get_light_impl().position( m_mblurTime, worldPos );

                            vector3f toEye = vector3f::normalize( -pixelRay.direction() );
                            vector3f toLight = vector3f::normalize( lightPos - worldPos );

                            prep_sample_for_shader( density, sample );

                            lightVal =
                                ( m_cameraDensityScale * density ) *
                                m_shader->shade( toEye, toLight, lightVal, color / density, sample.get_raw_buffer() );
                        }
                    }

                    color3f prevLight = rd.prevLight;
                    rd.prevLight = lightVal;

                    alpha_type alpha;

                    if( !absorptionAccessor.is_valid() ) {
                        float extinction = density * m_cameraDensityScale * step;
                        float prevExtinction = prevDensity * m_cameraDensityScale * step;

                        // TODO: Consider doing this on single3 color object instead of per-channel, since the density
                        // is the same for each.
                        boost::tie( lightVal.r, alpha.ar ) =
                            integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction, extinction );
                        boost::tie( lightVal.g, alpha.ag ) =
                            integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction, extinction );
                        boost::tie( lightVal.b, alpha.ab ) =
                            integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction, extinction );
                    } else {
                        color3f extinction =
                            color + absorb; // Use previously grabbed value before shader prep divided out the density.
                        color3f prevExtinction = rd.prevExtinction;

                        rd.prevExtinction = extinction;

                        extinction *= ( m_cameraDensityScale * step );
                        prevExtinction *= ( m_cameraDensityScale * step );

                        boost::tie( lightVal.r, alpha.ar ) =
                            integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction.r, extinction.r );
                        boost::tie( lightVal.g, alpha.ag ) =
                            integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction.g, extinction.g );
                        boost::tie( lightVal.b, alpha.ab ) =
                            integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction.b, extinction.b );
                    }

                    lightVal *= step;

                    // Aply APM extinction from camera origin to start of integration interval.
                    if( m_atmosphere )
                        m_atmosphere->apply_atomsphere( lightVal, pixelRay.origin(), worldPos );

                    set_pixel( x, y, outImage, pixel_type( lightVal, alpha ) );

                    // For each shader render element, accumulate its results here too.
                    for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                    itEnd = m_renderElementData.end();
                         it != itEnd; ++it ) {
                        if( it->element->get_drawing_type() != particle_render_element_interface::draw_type_shader )
                            continue;

                        color_type lightVal =
                            ( m_cameraDensityScale * density ) * it->element->evaluate( sample.get_raw_buffer() );
                        color_type prevLight = it->prevData[pixelId];

                        it->prevData[pixelId] = lightVal;

                        if( !absorptionAccessor.is_valid() ) {
                            float extinction = density * m_cameraDensityScale * step;
                            float prevExtinction = prevDensity * m_cameraDensityScale * step;

                            boost::tie( lightVal.r, alpha.ar ) =
                                integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction, extinction );
                            boost::tie( lightVal.g, alpha.ag ) =
                                integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction, extinction );
                            boost::tie( lightVal.b, alpha.ab ) =
                                integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction, extinction );
                        } else {
                            color3f extinction =
                                color +
                                absorb; // Use previously grabbed value before shader prep divided out the density.
                            color3f prevExtinction = rd.prevExtinction;

                            rd.prevExtinction = extinction;

                            extinction *= ( m_cameraDensityScale * step );
                            prevExtinction *= ( m_cameraDensityScale * step );

                            boost::tie( lightVal.r, alpha.ar ) =
                                integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction.r, extinction.r );
                            boost::tie( lightVal.g, alpha.ag ) =
                                integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction.g, extinction.g );
                            boost::tie( lightVal.b, alpha.ab ) =
                                integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction.b, extinction.b );
                        }

                        lightVal *= step;

                        // set_pixel( x, y, it->element->get_framebuffer(), pixel_type( lightVal, alpha ) );
                        if( m_coordsys->is_forward() ) {
                            it->element->get_framebuffer().blend_over( x, y, pixel_type( lightVal, alpha ) );
                        } else {
                            it->element->get_framebuffer().blend_under( x, y, pixel_type( lightVal, alpha ) );
                        }
                    }
                }
            }
        } // x
    }     // y
}

void renderer_impl::render_emission( const boundrect2& rect, renderer_impl::image_type& outImage ) {
    sample sample;
    m_data->construct_sample( sample );

    frantic::channels::channel_cvt_accessor<float> densityAccessor( 1.f );
    frantic::channels::channel_cvt_accessor<color3f> emissionAccessor( color3f::black() );
    frantic::channels::channel_cvt_accessor<vector3f> normalAccessor( vector3f( 0, 0, 0 ) );
    frantic::channels::channel_cvt_accessor<color3f> colorAccessor( color3f::white() );
    frantic::channels::channel_cvt_accessor<color3f> absorptionAccessor;
    frantic::channels::channel_cvt_accessor<float> reflectionAccessor( m_reflectionStrength );
    bool doReflectionStrengthScale = false;

    if( m_renderMode == renderer::mode_type::normal ) {
        if( sample.has_property( _T("Density") ) )
            densityAccessor = sample.get_channel_map().get_cvt_accessor<float>( _T("Density") );

        if( sample.has_property( _T("Emission") ) && m_useEmissionChannel )
            emissionAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Emission") );

        if( sample.has_property( _T("Normal") ) )
            normalAccessor = sample.get_channel_map().get_cvt_accessor<vector3f>( _T("Normal") );

        if( sample.has_property( _T("Color") ) )
            colorAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Color") );

        if( sample.has_property( _T("Absorption") ) && m_useAbsorptionChannel )
            absorptionAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Absorption") );

        if( sample.has_property( _T( "ReflectionStrength" ) ) ) {
            reflectionAccessor = sample.get_channel_map().get_cvt_accessor<float>( _T( "ReflectionStrength" ) );
            doReflectionStrengthScale = true;
        }
    } else { // m_renderMode = renderer_mode::additive
        densityAccessor = frantic::channels::channel_cvt_accessor<float>( 0.f );
        colorAccessor = frantic::channels::channel_cvt_accessor<color3f>( color3f::black() );

        if( sample.has_property( _T("Color") ) )
            emissionAccessor = sample.get_channel_map().get_cvt_accessor<color3f>( _T("Color") );
    }

    vector3f viewDirection = m_camera.view_direction( m_mblurTime );

    int sliceIndex = m_coordsys->get_slice_index();
    int sliceIndexPlusOne = sliceIndex + 1;

    bool doEnvReflection = m_environment && m_enableReflections;

    color3f extinction, prevExtinction;

    for( int y = rect.minimum().y; y < rect.maximum().y; ++y ) {
        for( int x = rect.minimum().x; x < rect.maximum().x; ++x ) {

            int pixelId = x + outImage.width() * y;

            // This relies on sliceIndex always getting more negative.
            if( m_rayDataSliceId[pixelId] <= sliceIndexPlusOne ) {
                ray_data& rd = m_rayData[pixelId];
                frantic::graphics::ray3f& pixelRay = rd.ray;

                PROFILING_SECTION_ENTER( m_psIntersect );
                std::pair<float, float> rayIntersect = m_coordsys->intersect_ray_with_slice( pixelRay );

                // We may need to shorten the interval to account for matte objects.
                float zComponent = vector3f::dot( pixelRay.direction(), viewDirection );
                if( !check_matte_depthmap( x, y, outImage.width(), outImage.height(), rayIntersect.first,
                                           rayIntersect.second, zComponent ) )
                    continue;

                frantic::graphics::vector3f worldPos = pixelRay.at( rayIntersect.second );

                if( m_rayDataSliceId[pixelId] == m_coordsys->get_slice_index() ) {
                    frantic::graphics::vector3f voxelPos = m_coordsys->transform_from_world( worldPos );
                    frantic::graphics2d::vector2f voxelPos2D( voxelPos.x, voxelPos.y );
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    PROFILING_SECTION_ENTER( m_psSample );
                    m_data->get_sample( voxelPos2D, sample );
                    PROFILING_SECTION_EXIT( m_psSample );
                } else {
                    PROFILING_SECTION_EXIT( m_psIntersect );

                    // Set all values to zero, since there wasn't anything to sample.
                    memset( (char*)sample.get_raw_buffer(), 0, sample.get_channel_map().structure_size() );
                }

                float density = densityAccessor.get( sample.get_raw_buffer() );
                float prevDensity = rd.prevDensity;

                color3f emission = emissionAccessor.get( sample.get_raw_buffer() );
                color3f prevLight = rd.prevLight;

                if( density > 0.f || prevDensity > 0.f || emission.r > 0 || emission.g > 0 || emission.b > 0 ||
                    prevLight.r > 0 || prevLight.g > 0 || prevLight.b > 0 ) {
                    PROFILING_SECTION_ENTER( m_psCalculate );

                    rd.prevDensity = density;

                    float step = fabsf( rayIntersect.second - rayIntersect.first ) *
                                 pixelRay.direction().get_magnitude() * m_voxelSize;

                    alpha_type alpha;
                    color3f lightVal;

                    lightVal = m_cameraEmissionScale * emission;

                    if( doEnvReflection ) {
                        vector3f normal = vector3f::normalize( normalAccessor.get( sample.get_raw_buffer() ) );
                        vector3f toEye = vector3f::normalize( -pixelRay.direction() );
                        vector3f envDir = frantic::shading::compute_reflection( toEye, normal );

                        float refStrength = reflectionAccessor.get( sample.get_raw_buffer() );
                        if( doReflectionStrengthScale ) {
                            refStrength *= m_reflectionStrength;
                        }
                        // TODO: Should the shader process this? At the very least the scattering color should come into
                        // play.
                        lightVal += ( m_cameraDensityScale * refStrength * density ) *
                                    m_environment->lookup_environment( -envDir );
                    }

                    rd.prevLight = lightVal;

                    if( !absorptionAccessor.is_valid() ) {
                        float extinction = density * m_cameraDensityScale * step;
                        float prevExtinction = prevDensity * m_cameraDensityScale * step;

                        // TODO: Consider doing this on single3 color object instead of per-channel, since the density
                        // is the same for each.
                        boost::tie( lightVal.r, alpha.ar ) =
                            integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction, extinction );
                        boost::tie( lightVal.g, alpha.ag ) =
                            integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction, extinction );
                        boost::tie( lightVal.b, alpha.ab ) =
                            integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction, extinction );
                    } else {
                        color3f color = colorAccessor.get( sample.get_raw_buffer() );
                        if( colorAccessor.is_default() )
                            color *= density;

                        /*color3f */ extinction = color + absorptionAccessor.get( sample.get_raw_buffer() );
                        /*color3f */ prevExtinction = rd.prevExtinction;

                        extinction *= ( m_cameraDensityScale * step );

                        rd.prevExtinction = extinction;

                        boost::tie( lightVal.r, alpha.ar ) =
                            integrate_scattered_light( prevLight.r, lightVal.r, prevExtinction.r, extinction.r );
                        boost::tie( lightVal.g, alpha.ag ) =
                            integrate_scattered_light( prevLight.g, lightVal.g, prevExtinction.g, extinction.g );
                        boost::tie( lightVal.b, alpha.ab ) =
                            integrate_scattered_light( prevLight.b, lightVal.b, prevExtinction.b, extinction.b );
                    }

                    lightVal *= step;

                    // Aply APM extinction from camera origin to start of integration interval.
                    if( m_atmosphere )
                        m_atmosphere->apply_atomsphere( lightVal, pixelRay.origin(), worldPos );

                    set_pixel( x, y, outImage, pixel_type( lightVal, alpha ) );
                    PROFILING_SECTION_EXIT( m_psCalculate );

                    bool hasDivided = false;

                    if( density > 1e-3f ) {
                        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                        itEnd = m_renderElementData.end();
                             it != itEnd; ++it ) {
                            int type = it->element->get_drawing_type();
                            if( type == particle_render_element_interface::draw_type_solid ||
                                type == particle_render_element_interface::draw_type_antialias ) {
                                if( !hasDivided ) {
                                    hasDivided = true;
                                    prep_sample_for_shader( density, sample );
                                }

                                color_type c = it->element->evaluate( sample.get_raw_buffer() );

                                // Always marching forward in the camera-space pass.
                                it->element->get_framebuffer().blend_over( x, y, pixel_type( c, alpha_type( 1.f ) ) );
                            }
                        }
                    }

                    if( density > 0 || emission.r > 0 || emission.g > 0 || emission.b > 0 || prevLight.r > 0 ||
                        prevLight.g > 0 || prevLight.b > 0 ) {
                        for( std::vector<render_element_data>::iterator it = m_renderElementData.begin(),
                                                                        itEnd = m_renderElementData.end();
                             it != itEnd; ++it ) {
                            if( it->element->get_drawing_type() ==
                                particle_render_element_interface::draw_type_normal ) {
                                if( !hasDivided ) {
                                    hasDivided = true;
                                    prep_sample_for_shader( density, sample );
                                }

                                color_type lightVal =
                                    m_cameraEmissionScale * it->element->evaluate( sample.get_raw_buffer() );
                                color_type prevLight = it->prevData[pixelId];

                                it->prevData[pixelId] = lightVal;

                                if( !absorptionAccessor.is_valid() ) {
                                    float extinction = density * m_cameraDensityScale * step;
                                    float prevExtinction = prevDensity * m_cameraDensityScale * step;

                                    // TODO: Consider doing this on single3 color object instead of per-channel, since
                                    // the density is the same for each.
                                    boost::tie( lightVal.r, alpha.ar ) = integrate_scattered_light(
                                        prevLight.r, lightVal.r, prevExtinction, extinction );
                                    boost::tie( lightVal.g, alpha.ag ) = integrate_scattered_light(
                                        prevLight.g, lightVal.g, prevExtinction, extinction );
                                    boost::tie( lightVal.b, alpha.ab ) = integrate_scattered_light(
                                        prevLight.b, lightVal.b, prevExtinction, extinction );
                                } else {
                                    boost::tie( lightVal.r, alpha.ar ) = integrate_scattered_light(
                                        prevLight.r, lightVal.r, prevExtinction.r, extinction.r );
                                    boost::tie( lightVal.g, alpha.ag ) = integrate_scattered_light(
                                        prevLight.g, lightVal.g, prevExtinction.g, extinction.g );
                                    boost::tie( lightVal.b, alpha.ab ) = integrate_scattered_light(
                                        prevLight.b, lightVal.b, prevExtinction.b, extinction.b );
                                }

                                lightVal *= step;

                                // Always marching forward in the camera-space pass.
                                it->element->get_framebuffer().blend_over( x, y, pixel_type( lightVal, alpha ) );
                            }
                        }
                    }
                }
            }
        } // x
    }     // y
}

void renderer_impl::set_pixel( int x, int y, image_type& outImage, const pixel_type& p ) const {
    if( m_coordsys->is_forward() ) {
        outImage.blend_over( x, y, p );
    } else {
        outImage.blend_under( x, y, p );
    }
}

void renderer_impl::apply_dof( renderer_impl::image_type& renderBuffer ) {
    // We will be swapping 'renderBuffer' with this buffer at the end.
    image_type resultBuffer( renderBuffer.size() );

    boost::mt19937 generator( 42 );
    boost::uniform_real<float> range( 0, 1 );
    boost::variate_generator<boost::mt19937&, boost::uniform_real<float>> rng( generator, range );

    std::vector<char> pixelEnabled;
    pixelEnabled.reserve( 1024 );

    vector3f viewDirection = m_camera.view_direction( m_mblurTime );

    int sliceIndex = m_coordsys->get_slice_index();
    int sliceIndexPlusOne = sliceIndex + 1;
    sample sample;
    frantic::channels::channel_cvt_accessor<float> blendAmountAcc( bokeh_blend_amount().get_value_or( 1.0f ) );
    frantic::channels::channel_cvt_accessor<float> densityAcc( 1.0f );
    const bool doBokehBlend = bokeh_blend_map( 1 ) && bokeh_blend_amount().get_value_or( 0.0f ) != 0.0f;
    if( doBokehBlend ) {
        m_data->construct_sample( sample );
        if( sample.has_property( _T( "BokehBlendInfluence" ) ) ) {
            blendAmountAcc = sample.get_channel_map().get_cvt_accessor<float>( _T( "BokehBlendInfluence" ) );
        }

        if( sample.has_property( _T( "Density" ) ) ) {
            densityAcc = sample.get_channel_map().get_cvt_accessor<float>( _T( "Density" ) );
        }
    }

    for( int y = 0; y < renderBuffer.height(); ++y ) {
        for( int x = 0; x < renderBuffer.width(); ++x ) {
            int pixelId = x + renderBuffer.width() * y;

            // This relies on sliceIndex always getting more negative.
            if( m_rayDataSliceId[pixelId] <= sliceIndexPlusOne ) {
                pixel_type c = renderBuffer.get_pixel( x, y );

                ray_data& rd = m_rayData[pixelId];
                frantic::graphics::ray3f& pixelRay = rd.ray;

                PROFILING_SECTION_ENTER( m_psIntersect );
                std::pair<float, float> rayIntersect = m_coordsys->intersect_ray_with_slice( pixelRay );
                if( doBokehBlend ) {
                    frantic::graphics::vector3f worldPos = pixelRay.at( rayIntersect.second );

                    if( m_rayDataSliceId[pixelId] == m_coordsys->get_slice_index() ) {
                        frantic::graphics::vector3f voxelPos = m_coordsys->transform_from_world( worldPos );
                        frantic::graphics2d::vector2f voxelPos2D( voxelPos.x, voxelPos.y );
                        PROFILING_SECTION_EXIT( m_psIntersect );

                        PROFILING_SECTION_ENTER( m_psSample );
                        m_data->get_sample( voxelPos2D, sample );
                        PROFILING_SECTION_EXIT( m_psSample );
                    } else {
                        PROFILING_SECTION_EXIT( m_psIntersect );

                        // Set all values to zero, since there wasn't anything to sample.
                        memset( (char*)sample.get_raw_buffer(), 0, sample.get_channel_map().structure_size() );
                    }
                } else {
                    PROFILING_SECTION_EXIT( m_psIntersect );
                }

                float zComponent = vector3f::dot( pixelRay.direction(), viewDirection );
                float z = zComponent * rayIntersect.second;

                vector2f pixel( (float)x, (float)y );
                vector2f pixelRadii;
                m_camera.dof_ellipse_of_confusion_pixel_radii( z, pixelRadii.x, pixelRadii.y );

                if( anamorphic_squeeze() && anamorphic_squeeze().get() != 1.0f ) {
                    const float squeeze = anamorphic_squeeze().get();
                    if( squeeze > 1.0f ) {
                        pixelRadii.y *= squeeze;
                    } else {
                        pixelRadii.x /= squeeze;
                    }
                }

                int radX = (int)std::ceil( pixelRadii.x ); // I think floor would work
                int radY = (int)std::ceil( pixelRadii.y ); // I think floor would work

                pixelEnabled.resize( ( 2 * radX + 1 ) *
                                     ( 2 * radY + 1 ) ); // ex. for rad 2 we visit -2,-1,0,1,2 = 2*rad+1

                float denomX = pixelRadii.x * pixelRadii.x;
                float denomY = pixelRadii.y * pixelRadii.y;

                float n = 0;
                char* p = &pixelEnabled[0];
                for( int j = -radY; j <= radY; ++j ) {
                    for( int i = -radX; i <= radX; ++i, ++p ) {
                        if( bokeh_mask() ) {
                            const int width = 2 * radX;
                            const int height = 2 * radY;
                            const int maskX = ( ( i + radX ) * ( bokeh_mask().get().width() - 1 ) ) / width;
                            const int maskY = ( ( j + radY ) * ( bokeh_mask().get().height() - 1 ) ) / height;
                            if( bokeh_mask().get().at( maskX, maskY ) > rng() ) {
                                n += 1.f;
                                *p = 1;
                            } else {
                                *p = 0;
                            }
                        } else {
                            float dist = float( i * i ) / denomX + float( j * j ) / denomY;
                            if( dist <= 1.f ) {
                                n += 1.f;
                                *p = 1;
                            } else {
                                *p = 0;
                            }
                        }
                    }
                }

                const float invN = n == 0.0f ? 0.0f : 1.f / n;
                if( !doBokehBlend ) {
                    c *= invN;
                }
                p = &pixelEnabled[0];

                const pixel_type originalColor = c;

                for( int j = -radY; j <= radY; ++j ) {
                    for( int i = -radX; i <= radX; ++i, ++p ) {
                        int realX = x + i;
                        int realY = y + j;
                        if( unsigned( realX ) >= (unsigned)renderBuffer.width() ||
                            unsigned( realY ) >= (unsigned)renderBuffer.height() )
                            continue;

                        const boost::optional<const frantic::graphics2d::image_channel<frantic::graphics::color3f>&>
                            bokehBlend = bokeh_blend_map(
                                std::max( 1, std::max( renderBuffer.width(), renderBuffer.height() ) ) *
                                m_mipmapResolutionCoefficient );
                        if( bokehBlend && bokeh_blend_amount().get_value_or( 0.0f ) != 0.0f ) {
                            const int width = 2 * radX;
                            const int height = 2 * radY;
                            const int maskX = ( ( i + radX ) * ( bokehBlend.get().width() - 1 ) ) / width;
                            const int maskY = ( ( j + radY ) * ( bokehBlend.get().height() - 1 ) ) / height;
                            color3f blendColor = bokehBlend.get().at( maskX, maskY );

                            if( m_renderMode == mode_type::additive ) {
                                const float density = densityAcc.get( sample.get_raw_buffer() );
                                const float effectiveDensity = density * m_cameraDensityScale;

                                blendColor *= effectiveDensity;
                            } else {
                                // Blend hue and saturation, but not value.
                                // This is to attempt to preserve the lighting of the particles
                                // because we can't re-calculate lighting for each bokeh pixel.
                                blendColor =
                                    color3f::from_hsv( blendColor.hue(), blendColor.saturation(),
                                                       originalColor.c.value(), frantic::graphics::color3f::unclamped );
                            }
                            const float bokehBlendAmount =
                                boost::algorithm::clamp( blendAmountAcc( sample.get_raw_buffer() ), 0.0f, 1.0f );
                            c = pixel_type( ( bokehBlendAmount * blendColor ) +
                                                ( ( 1.0f - bokehBlendAmount ) * originalColor.c ),
                                            originalColor.a ) *
                                invN;
                        }

                        if( *p != 0 )
                            resultBuffer.set_pixel( realX, realY, resultBuffer.get_pixel( realX, realY ) + c );
                    }
                }
            }
        }
    }

    resultBuffer.swap( renderBuffer );
}

bool renderer_impl::check_matte_depthmap( int x, int y, int width, int height, float& t0, float& t1,
                                          float rayZComponent ) const {

    if( !m_matteSampler && !m_deepMatte )
        return true;

    float z0 = t0 * rayZComponent;
    float z1 = t1 * rayZComponent;
    float zMatte = std::numeric_limits<float>::max();

    if( m_matteSampler ) {

        if( m_matteSuperSampling <= 1 ) {
            zMatte = m_matteDepthMap.get_depth_value( x, y );
        } else {
            int realX = x * m_matteSuperSampling;
            int realY = y * m_matteSuperSampling;
            for( int offY = 0; offY < m_matteSuperSampling; ++offY ) {
                for( int offX = 0; offX < m_matteSuperSampling; ++offX )
                    zMatte = std::min( zMatte, m_matteDepthMap.get_depth_value( realX + offX, realY + offY ) );
            }
        }
    }

    // now check deep matte image!
    if( m_deepMatte ) {

        int imgWidth = m_deepMatte->size().xsize;
        int imgHeight = m_deepMatte->size().ysize;

        // TODO: This works when the image and the deep holdout matte are of the same ratio (normally the same
        // dimensions)
        // TODO: what the heck to do when images are of different ratio
        zMatte = std::min( zMatte, m_deepMatte->get_zdepth_bilinear( x * ( (float)imgWidth / width ),
                                                                     y * ( (float)imgHeight / height ) ) );

        /*
        //deep images are always square images. in prman, if the output image resolution is size [w,h], and the deep
        image is size [n,n], the mapping from image to matte is (pixelX/w*n,pixelY/w*n).
        //note that the deep image does not fully cover the output image when h > w.
        int imgDim = m_deepMatte->size().xsize;
        float lookupScale = (float)imgDim / width;
        float yLookupOffset = ( imgDim - height * lookupScale ) * 0.5f;
        vector2f imgLookup( x * lookupScale, y * lookupScale + yLookupOffset );
        if( imgLookup.y >= 0 && imgLookup.y < imgDim )
                zMatte = std::min( zMatte, m_deepMatte->get_zdepth_bilinear( imgLookup.x, imgLookup.y ) );
        */
    }

    if( m_coordsys->is_forward() ) {
        if( z1 > zMatte ) {
            if( z0 > zMatte ) // This ray is occluded completely
                return false;

            // Move the worldPos to the intersection with the matte object.
            t1 = zMatte / rayZComponent;
        }
    } else {
        if( z1 > zMatte ) {
            if( 2 * z1 - z0 > zMatte ) // This ray is occluded completely
                return false;

            // We need to record the data sampled at this location, since we will need it next step. We don't want
            // anything to actually be rendered though, so set the interval to be 0 length (ie. t0 == t1).
            t0 = t1 = zMatte / rayZComponent;
        } else if( z0 > zMatte ) {
            t0 = zMatte / rayZComponent;
        }
    }

    return true;
}

namespace {
class coverage_shader {
  public:
    typedef krakatoa::clipped_triangle triangle_type;

  private:
    int m_y;
    int m_sliceId;

    int* m_rayData;
    frantic::graphics2d::size2 m_rayDataSize;

  public:
    coverage_shader( int* rayData, frantic::graphics2d::size2 rayDataSize, int sliceId )
        : m_rayData( rayData )
        , m_rayDataSize( rayDataSize )
        , m_sliceId( sliceId ) {}

    int get_buffer_width() const { return m_rayDataSize.xsize; }
    int get_buffer_height() const { return m_rayDataSize.ysize; }

    void new_scan( int y, float, float ) { m_y = y; }

    void evaluate( int x, float ) { m_rayData[m_rayDataSize.get_index( x, m_y )] = m_sliceId; }
};
} // namespace

void renderer_impl::activate_pixels_under_rect( const frantic::graphics2d::boundrect2& bounds ) {
    const int lookupRadius = 1;

    vector3f p[] = { m_coordsys->transform_to_world( vector3f( (float)bounds.minimum().x - lookupRadius,
                                                               (float)bounds.minimum().y - lookupRadius, 0 ) ),
                     m_coordsys->transform_to_world( vector3f( (float)bounds.maximum().x + lookupRadius,
                                                               (float)bounds.minimum().y - lookupRadius, 0 ) ),
                     m_coordsys->transform_to_world( vector3f( (float)bounds.minimum().x - lookupRadius,
                                                               (float)bounds.maximum().y + lookupRadius, 0 ) ),
                     m_coordsys->transform_to_world( vector3f( (float)bounds.maximum().x + lookupRadius,
                                                               (float)bounds.maximum().y + lookupRadius, 0 ) ) };

    krakatoa::triangle_rasterizer<coverage_shader> rasterizer( m_camera, m_mblurTime );
    coverage_shader theShader( m_rayDataSliceId.get(), m_camera.get_output_size(), m_coordsys->get_slice_index() );
    rasterizer.raster_triangle( theShader, p[0], p[1], p[3] );
    rasterizer.raster_triangle( theShader, p[3], p[2], p[0] );

    if( m_lightObject ) {
        const frantic::graphics::camera<float>& lightCam =
            static_cast<const frantic::rendering::lights::directedlight_interface&>( m_lightObject->get_light_impl() )
                .get_camera();

        krakatoa::triangle_rasterizer<coverage_shader> rasterizer( lightCam, m_mblurTime );
        coverage_shader theShader( m_lightRayDataSliceId.get(), m_lightObject->get_light_impl().shadow_map_size(),
                                   m_coordsys->get_slice_index() );
        rasterizer.raster_triangle( theShader, p[0], p[1], p[3] );
        rasterizer.raster_triangle( theShader, p[3], p[2], p[0] );
    }
}

void renderer_impl::render_elements( const boundrect2& /*rect*/, image_type& /*outImage*/ ) {}

#if 0
#define M_4PI 12.566370614359172953850573533118
#define CUBEFACE_OVERLAP_DEGREES 1.f

class omni_part_light : public frantic::rendering::lights::directedlight_interface{
public:
	omni_part_light(
		frantic::rendering::lights::lightinterface& omni,
		frantic::graphics::cube_face::default_cube_face cubeFace,
		int x, int y
	)	:	directedlight_interface(
		     omni.name(),
			 frantic::graphics::color3f( (float)M_4PI ),
			 0,
			 omni.is_shadows_enabled(),
			 omni.shadow_density(),
			 false, false, 0, 0, 0, 0, 
			 frantic::graphics::camera(
			   frantic::graphics::projection_mode::perspective,
			   omni.transform_matrix(),
			   2.0f * ::atanf( (1+4.f/(float)omni.shadow_map_size().xsize) * ::tanf(atan(0.5f) + frantic::math::degrees_to_radians(CUBEFACE_OVERLAP_DEGREES)) ),
			   omni.shadow_map_size(),
			   0.001f,
			   1e+10,
			   1.f
			 ),
			 frantic::rendering::lights::LIGHT_SHAPE_SQUARE,
			 1.f,
			 atan(0.5f) - frantic::math::degrees_to_radians(CUBEFACE_OVERLAP_DEGREES),
			 atan(0.5f) + frantic::math::degrees_to_radians(CUBEFACE_OVERLAP_DEGREES)
			)
	{
		m_camera.set_transform( 
			m_camera.world_transform() * 
			frantic::graphics::transform4f::from_cubeface(cubeFace) * 
			frantic::graphics::transform4f( vector3f(1,0,0), vector3f(0,1,0), vector3f(0.5f*(float)(2*x-1), 0.5f*(float)(2*y-1), 1.f) ) );
	}
	
	virtual ~omni_part_light()
	{}

	virtual void write_xml( std::ostream& /*out*/, const std::string& /*prefix*/ ) const{
	}

	// Calculates the irradiance from the light at a given point in space
	// bIsValid is true if the light shines on the given shadingPosition
	virtual frantic::graphics::color3f irradiance( const frantic::graphics::vector3f& shadingPosition, float motionSegmentTime, bool& bIsValid) const{
		using frantic::graphics::vector3f;
		using frantic::graphics::color3f;
		
		vector3f camSpacePos = get_camera().world_transform_inverse(motionSegmentTime) * shadingPosition;
		float rayDist = camSpacePos.get_magnitude();

		float xAngle = atan( std::abs(camSpacePos.x / camSpacePos.z / m_sqrtLightAspect) );
		float yAngle = atan( std::abs(camSpacePos.y / camSpacePos.z / m_sqrtLightAspect * m_lightAspect) );

		if( xAngle > m_outerRadius || yAngle > m_outerRadius || camSpacePos.z > 0){
			bIsValid = false;
			return frantic::graphics::color3f();
		}
			
		float falloffWeight;
		falloffWeight  = 1.f - frantic::math::smoothstep(xAngle, m_innerRadius, m_outerRadius);
		falloffWeight *= 1.f - frantic::math::smoothstep(yAngle, m_innerRadius, m_outerRadius);

		if (use_near_attenuation()) {
			if ( rayDist < m_nearAttenuationStart) {
				bIsValid = false;
				return color3f();
			}
			else if( rayDist < m_nearAttenuationEnd)
				falloffWeight *= frantic::math::smoothstep(rayDist, m_nearAttenuationStart, m_nearAttenuationEnd );
		}

		if(use_far_attenuation()) {
			if ( rayDist > m_farAttenuationEnd ) {
				bIsValid = false;
				return frantic::graphics::color3f();
			}
			else if( rayDist > m_farAttenuationStart)
				falloffWeight *= 1.f - frantic::math::smoothstep(rayDist, m_farAttenuationStart, m_farAttenuationEnd );
		}

		if( m_decayExponent == 2 ) {
			return falloffWeight * m_flux_over_4pi / (rayDist * rayDist);
		} else if( m_decayExponent == 1 ) {
			return falloffWeight * m_flux_over_4pi / rayDist;
		} else {
			return falloffWeight * m_flux_over_4pi;
		}
	}

	// Calcualates the irradiance on a surface at a given point with a given normal
	// This is irradiance(position) * cos(angle between normal and direction to light)
	virtual frantic::graphics::color3f radiance( const frantic::graphics::vector3f& shadingPosition, float motionSegmentTime, const frantic::graphics::vector3f& normal, bool& bIsValid ) const{
		using frantic::graphics::vector3f;
		using frantic::graphics::color3f;
		
		vector3f vectorToLight = directedlight_interface::position(motionSegmentTime) - shadingPosition;
		
		float cosTheta = vector3f::dot( normal, vectorToLight ) / vectorToLight.get_magnitude();
		return (cosTheta > 0) ? cosTheta * irradiance(shadingPosition, motionSegmentTime, bIsValid) : color3f();
	}
};
#endif

/*
void renderer_impl::render_omni_light( renderer::image_type& outImage ){
        frantic::graphics::cube_face::default_cube_face faces[] = {
                frantic::graphics::cube_face::CF_Z_NEG, frantic::graphics::cube_face::CF_Z_POS,
                frantic::graphics::cube_face::CF_X_NEG, frantic::graphics::cube_face::CF_X_POS,
                frantic::graphics::cube_face::CF_Y_NEG, frantic::graphics::cube_face::CF_Y_POS
        };

        frantic::rendering::lights::lightinterface& omniLight = m_lightObject.get_impl().get_light_impl();

        for( int i = 0; i < 6; ++i ){
                for(int j = 0; j <= 1; ++j){
                        for(int k = 0; k <= 1; ++k){
                                frantic::logging::progress_logger_subinterval_tracker siOmni( *m_progress, 100.f *
((float)(i*4+j*2+k) / 24.f), 100.f * ((float)(i*4+j*2+k+1) / 24.f)  );

                                boost::shared_ptr<omni_part_light> tempLight( new omni_part_light(omniLight, faces[i],
j, k) );

                                m_light = tempLight.get();
                                m_lightObject.reset( light_object::create(tempLight), false );
                                m_lightObject.get_impl().begin( m_sceneContext );
                                m_lightObject.get_impl().update( m_mblurTime );

                                renderer::image_type sliceFramebuffer( outImage.size() );
                                render_subinterval( sliceFramebuffer );

                                m_lightObject.get_impl().end();

                                //Add the incoming light into the render buffer
                                for(int y = 0; y < outImage.height(); ++y){
                                        for(int x = 0; x < outImage.width(); ++x){
                                                pixel_type bufferVal = sliceFramebuffer.get_pixel( x, y );
                                                outImage.set_pixel(x, y, outImage.get_pixel( x, y ) + pixel_type(
bufferVal.c, bufferVal.a / 24.f ) );
                                        }
                                }
                        }
                }
        }
}
*/

} // namespace voxel_renderer
} // namespace krakatoa
