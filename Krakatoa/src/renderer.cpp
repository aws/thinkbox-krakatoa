// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoa/camera_manager.hpp>
#include <krakatoa/renderer.hpp>

#include <frantic/graphics2d/mipmap.hpp>

#include <boost/algorithm/clamp.hpp>
#include <boost/unordered_set.hpp>

namespace krakatoa {

class default_shader : public krakatoa_shader {
  public:
    virtual ~default_shader() {}

    virtual void set_channel_map( const frantic::channels::channel_map& /*pcm*/ ) {}

    virtual frantic::graphics::color3f shade( const frantic::graphics::vector3f& /*toEye*/,
                                              const frantic::graphics::vector3f& /*toLight*/,
                                              const frantic::graphics::color3f& incidentLight,
                                              const frantic::graphics::color3f& scatterCoefficient,
                                              const char* /*renderData*/ ) const {
        return scatterCoefficient * incidentLight;
    }
};

class default_scene_context : public scene_context {
    camera_manager_ptr m_defaultCameraManager;

    template <class T>
    class empty_collection : public collection_interface<T> {
      public:
        virtual std::size_t size() const { return 0; }

        virtual typename collection_interface<T>::return_type get( int /*index*/ ) {
            throw std::domain_error( "Index out of bounds" );
        }

        virtual typename collection_interface<T>::const_return_type get( int /*index*/ ) const {
            throw std::domain_error( "Index out of bounds" );
        }
    };

    empty_collection<matte_primitive_ptr> m_matteCollection;
    empty_collection<light_object_ptr> m_lightCollection;

  public:
    default_scene_context()
        : m_defaultCameraManager( new camera_manager ) {}

    virtual double get_time() const { return 0; }
    virtual void set_time( double /*time*/ ) {}

    virtual const frantic::graphics::camera<float>& get_camera() const {
        return m_defaultCameraManager->get_current_camera();
    }
    virtual frantic::graphics::camera<float>& get_camera() { return m_defaultCameraManager->get_current_camera(); }

    virtual void set_camera( const frantic::graphics::camera<float>& cam ) {
        m_defaultCameraManager = camera_manager_ptr( new camera_manager( cam ) );
    }

    virtual void set_camera( const std::vector<frantic::graphics::camera<float>>& cams ) {
        m_defaultCameraManager = camera_manager_ptr( new camera_manager( cams ) );
    }

    virtual const matte_collection& get_matte_objects() const { return m_matteCollection; }

    virtual const light_collection& get_light_objects() const { return m_lightCollection; }
};

renderer::renderer() {
    m_cameraDensityScale = m_lightDensityScale = 1.f;
    m_cameraEmissionScale = 1.f;
    m_renderMode = mode_type::normal;
    m_mblurDuration = 0;
    m_mblurBias = 0;
    m_mblurSamples = 0;
    m_mblurJittered = false;
    m_adaptiveMblur = false;
    m_adaptiveMBlurSampleLowerBound = 0;
    m_adaptiveMBlurSampleUpperBound = 0;
    m_adaptiveMBlurSmoothness = 1.0f;
    m_adaptiveMBlurExponent = 1.0f;
    m_particles = NULL;
    m_disableThreading = false;
    m_doThreadedSorting = true;
    m_matteSuperSampling = 1;
    m_dofEnabled = false;
    m_dofSampleRate = 0.1f;
    m_backgroundColor = pixel_type( color_type::black(), alpha_type( 0 ) );
    m_enableReflections = true;
    m_reflectionStrength = 1.f;
    m_defaultBackground = false;
    m_renderingThreadLimit = -1;
    m_frameBufferAvailableMemoryFraction = 0.75f;
    m_useEmissionChannel = true;
    m_useAbsorptionChannel = true;
    m_mipmapResolutionCoefficient = 1;
    m_saveOccludedZDepth = false;

    m_shader.reset( new default_shader );
    m_sceneContext.reset( new default_scene_context );
    m_useMixedShaders = false;
}

void renderer::set_atmosphere( atmosphere_interface_ptr theAtmosphere ) { m_atmosphere = theAtmosphere; }

void renderer::set_environment( renderer::environment_ptr_type theEnvironment, float reflectionStrength ) {
    m_environment = theEnvironment;
    m_enableReflections = ( reflectionStrength > 0.f );
    m_reflectionStrength = std::max( 0.f, reflectionStrength );
}

void renderer::set_matte_sampler( matte_interface_ptr matteSampler, int superSampling ) {
    m_matteSampler = matteSampler;
    m_matteSuperSampling = std::max( 1, superSampling );
}

void renderer::set_deep_matte_map( boost::shared_ptr<frantic::rendering::singleface_atten_loader> deepMatteMap ) {
    m_deepMatte = deepMatteMap;
}

void renderer::set_particles( renderer::particle_container_type* theParticles ) { m_particles = theParticles; }

void renderer::set_shader( renderer::shader_ptr_type theShader ) { m_shader = theShader; }

void renderer::set_scene_context( scene_context_ptr theContext ) { m_sceneContext = theContext; }

void renderer::set_progress_logger( renderer::progress_ptr_type prog ) { m_progress = prog; }

void renderer::set_density_scale( float cameraScale, float lightScale ) {
    m_cameraDensityScale = cameraScale;
    m_lightDensityScale = lightScale;
}

void renderer::set_emission_scale( float emissionScale ) { m_cameraEmissionScale = emissionScale; }

void renderer::disable_threading( bool disableThreading ) { m_disableThreading = disableThreading; }

void renderer::disable_threading_for_sort( bool disableThreadingForSort ) {
    m_doThreadedSorting = !disableThreadingForSort;
}

void renderer::set_render_mode( renderer::mode_type::enum_t renderMode ) { m_renderMode = renderMode; }

void renderer::set_motion_blur_interval( float duration, float bias ) {
    m_mblurDuration = duration;
    m_mblurBias = bias;
}

void renderer::set_motion_blur_samples( int numSamples ) { m_mblurSamples = numSamples; }

void renderer::set_motion_blur_jittered( bool isJittered ) { m_mblurJittered = isJittered; }

void renderer::set_enable_adaptive_motion_blur( bool enable ) { m_adaptiveMblur = enable; }

void renderer::set_adaptive_motion_blur_min_samples( int numSamples ) { m_adaptiveMBlurSampleLowerBound = numSamples; }

void renderer::set_adaptive_motion_blur_max_samples( int numSamples ) { m_adaptiveMBlurSampleUpperBound = numSamples; }

void renderer::set_adaptive_motion_blur_smoothness( float smoothness ) {
    m_adaptiveMBlurSmoothness = std::max( smoothness, 0.0f );
}

void renderer::set_adaptive_motion_blur_exponent( float exponent ) { m_adaptiveMBlurExponent = exponent; }

void renderer::set_depth_of_field_enabled( bool enabled, float sampleRate ) {
    m_dofEnabled = enabled;
    m_dofSampleRate = std::max( 0.f, sampleRate );
}

void renderer::set_watermark( const watermark_fn& theWatermark ) { m_watermarkFn = theWatermark; }

void renderer::set_background_color( const pixel_type& backgroundColor, bool useEnvironBackground ) {
    m_backgroundColor = backgroundColor;
    m_defaultBackground = !useEnvironBackground;
}

std::size_t renderer::get_num_output_images() { return 1; }

void renderer::set_use_emission( bool useEmission ) { m_useEmissionChannel = useEmission; }

void renderer::set_use_absorption( bool useAbsorption ) { m_useAbsorptionChannel = useAbsorption; }

namespace {
struct vector3f_hash {
    std::size_t operator()( const frantic::graphics::vector3f& in ) const {
        std::size_t seed = 0;
        boost::hash_combine( seed, in.x );
        boost::hash_combine( seed, in.y );
        boost::hash_combine( seed, in.z );

        return seed;
    }
};
} // namespace

void renderer::set_bokeh_mask( const frantic::graphics2d::image_channel<float>& bokehMask ) { m_bokehMask = bokehMask; }

void renderer::set_bokeh_blend_map(
    const frantic::graphics2d::image_channel<frantic::graphics::color3f>& bokehBlendMap ) {
    m_bokehBlendMap = frantic::graphics2d::generate_mipmap<
        frantic::graphics::color3f, frantic::graphics2d::four_pixel_mean<frantic::graphics::color3f>>( bokehBlendMap );
}

void renderer::set_bokeh_blend_amount( float amount ) { m_bokehBlendAmount = amount; }

void renderer::set_anamorphic_squeeze( float squeeze ) { m_anamorphicSqueeze = squeeze; }

void renderer::set_mipmap_resolution_coefficient( int coefficient ) {
    m_mipmapResolutionCoefficient = boost::algorithm::clamp( coefficient, 1, 256 );
}

void renderer::set_save_occluded_ZDepth( bool save ) { m_saveOccludedZDepth = save; }

const boost::optional<frantic::graphics2d::image_channel<float>>& renderer::bokeh_mask() const { return m_bokehMask; }

boost::optional<const frantic::graphics2d::image_channel<frantic::graphics::color3f>&>
renderer::bokeh_blend_map( std::size_t resolution ) const {
    if( !m_bokehBlendMap ) {
        return boost::none;
    }

    return frantic::graphics2d::nearest_mipmap_resolution<frantic::graphics::color3f>( m_bokehBlendMap.get(),
                                                                                       resolution );
}

boost::optional<float> renderer::bokeh_blend_amount() const { return m_bokehBlendAmount; }

boost::optional<float> renderer::anamorphic_squeeze() const { return m_anamorphicSqueeze; }

#if defined( KRAKATOA_RT_CALLBACKS )
void renderer::set_mblur_pass_callback( const boost::function<void( const mblur_data& )>& callback ) {
    m_mblurPassCallback = callback;
}
#endif

void renderer::set_lighting_callback(
    const boost::function<frantic::graphics::color3f( const krakatoa::lighting_data& )> callback ) {
    m_lightingCallback = callback;
}

void renderer::set_matte_generator(
    const boost::function<void( double, frantic::graphics2d::framebuffer<float>&, int )> matteGenerator ) {
    m_matteGenerator = matteGenerator;
}

void renderer::set_use_mixed_shaders( bool useMixedShaders ) { m_useMixedShaders = useMixedShaders; }

void renderer::add_shader( shader_ptr_type shader ) { m_shaders.push_back( shader ); }

int renderer::get_adaptive_motion_blur_passes() const {
    using frantic::graphics::transform4f;
    using frantic::graphics::vector3f;
    using frantic::graphics2d::vector2f;

    if( m_adaptiveMBlurSampleUpperBound <= m_adaptiveMBlurSampleLowerBound ) {
        return m_adaptiveMBlurSampleUpperBound;
        FF_LOG( warning )
            << "Adaptive motion blur upper bound less than or equal to lower bound. Using upper bound segments."
            << std::endl;
    }

    const frantic::channels::channel_map& pcm = m_particles->get_channel_map();
    frantic::channels::channel_const_cvt_accessor<vector3f> posAccessor =
        pcm.get_const_cvt_accessor<vector3f>( _T( "Position" ) );
    frantic::channels::channel_const_cvt_accessor<vector3f> velAccessor =
        pcm.get_const_cvt_accessor<vector3f>( _T( "Velocity" ) );

    int maxDistance = m_adaptiveMBlurSampleLowerBound;

    // Particles
    for( particle_container_type::const_iterator it = m_particles->begin(); it != m_particles->end(); ++it ) {
        const vector3f position = posAccessor.get( *it );
        const vector3f velocity = velAccessor.get( *it );

        bool valid = true;
        const vector3f velocityProduct = velocity * 0.5f * m_mblurDuration;
        const int tempDistance =
            get_point_recommended_motion_blur_passes( position - velocityProduct, position + velocityProduct, valid );
        if( valid ) {
            maxDistance = std::max( tempDistance, maxDistance );
            if( maxDistance == m_adaptiveMBlurSampleUpperBound ) {
                return maxDistance;
            }
        }
    }

    // Matte Objects
    bool triangleCountWarning = false;

    const scene_context::matte_collection& mattes = m_sceneContext->get_matte_objects();
    for( std::size_t i = 0; i < mattes.size(); ++i ) {
        const krakatoa::matte_primitive_ptr currMatte = mattes.get( i );
        currMatte->set_time( 0.25f );
        const std::size_t numTriangles = currMatte->get_triangle_count();
        boost::unordered_set<vector3f, vector3f_hash> alreadyConsidered;

        for( std::size_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex ) {
            vector3f triangle[3];
            vector3f offsetTriangle[3];

            currMatte->set_time( 0.25f );
            currMatte->get_triangle( triangleIndex, triangle );
            const transform4f xform = currMatte->get_transform();

            currMatte->set_time( 0.75f );
            if( currMatte->get_triangle_count() <= triangleIndex ) {
                if( !triangleCountWarning ) {
                    FF_LOG( warning ) << "Matte object triangle count varries per-sub-frame. Adaptive motion blur pass "
                                         "count may be inaccurate."
                                      << std::endl;
                    triangleCountWarning = true;
                }
                continue;
            }

            currMatte->get_triangle( triangleIndex, offsetTriangle );
            const transform4f offsetXform = currMatte->get_transform();

            for( std::size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex ) {
                if( alreadyConsidered.find( triangle[vertexIndex] ) == alreadyConsidered.end() ) {
                    alreadyConsidered.insert( triangle[vertexIndex] );

                    const vector3f minVertex = xform * triangle[vertexIndex];
                    const vector3f maxVertex = offsetXform * offsetTriangle[vertexIndex];

                    bool valid = true;
                    const int tempDistance = get_point_recommended_motion_blur_passes( minVertex, maxVertex, valid );
                    if( valid ) {
                        maxDistance = std::max( tempDistance, maxDistance );
                        if( maxDistance == m_adaptiveMBlurSampleUpperBound ) {
                            return maxDistance;
                        }
                    }
                }
            }
        }
    }

    return maxDistance;
}

int renderer::get_point_recommended_motion_blur_passes( const frantic::graphics::vector3f& minPoint,
                                                        const frantic::graphics::vector3f& maxPoint,
                                                        bool& valid ) const {
    using frantic::graphics::vector3f;
    using frantic::graphics2d::vector2f;

    const frantic::graphics::camera<float> camera = m_sceneContext->get_camera();

    const frantic::graphics2d::vector2f screenPositionMin = camera.from_worldspace_position( minPoint, valid );
    const frantic::graphics2d::vector2f screenPositionMax = camera.from_worldspace_position( maxPoint, valid );
    if( !valid ) {
        return 0;
    }

    // We use half of the Taxicab Distance and then factor in the given smoothness setting.
    const vector2f difference = screenPositionMax - screenPositionMin;
    const float taxiDistance = std::abs( difference.x ) + std::abs( difference.y );
    const int pixelDistance = static_cast<int>( std::ceil( m_adaptiveMBlurSmoothness * 0.5f * taxiDistance ) );

    const int rangeLength = m_adaptiveMBlurSampleUpperBound - m_adaptiveMBlurSampleLowerBound;
    if( m_adaptiveMBlurSampleUpperBound == 0 ) {
        return 0;
    }

    const int clampedPixelDistance =
        boost::algorithm::clamp( pixelDistance, m_adaptiveMBlurSampleLowerBound, m_adaptiveMBlurSampleUpperBound );

    const float normalizedDistance = static_cast<float>( pixelDistance - m_adaptiveMBlurSampleLowerBound ) /
                                     static_cast<float>( m_adaptiveMBlurSampleUpperBound );

    const float exponentiated = std::pow( normalizedDistance, m_adaptiveMBlurExponent );
    const float unnormalized = ( exponentiated * static_cast<float>( m_adaptiveMBlurSampleUpperBound ) ) +
                               static_cast<float>( m_adaptiveMBlurSampleLowerBound );
    return boost::algorithm::clamp( static_cast<int>( std::floor( unnormalized ) ), m_adaptiveMBlurSampleLowerBound,
                                    m_adaptiveMBlurSampleUpperBound );
}

} // namespace krakatoa
