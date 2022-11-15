// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_renderer.hpp>

#include <krakatoasr_renderer/params.hpp>
#include <krakatoasr_renderer/renderer.hpp>

#include <frantic/math/utils.hpp>
#include <frantic/strings/tstring.hpp>

#include <boost/bind.hpp>

using namespace frantic::graphics;
using namespace frantic::graphics2d;
using namespace frantic::geometry;
using namespace frantic::channels;
using namespace frantic::particles;

namespace krakatoasr {

std::string get_filter_string( filter_t filter, int size ) {
    // valid options are: "Nearest Neighbor", "Bilinear", "Bicubic" or "Bilinear#"
    if( filter == FILTER_BILINEAR && size == 1 ) {
        return "Bilinear";
    } else if( filter == FILTER_BILINEAR && size > 1 ) {
        return "Bilinear" + boost::lexical_cast<std::string>( size );
    } else if( filter == FILTER_BICUBIC ) {
        return "Bicubic";
    } else if( filter == FILTER_NEAREST_NEIGHBOR ) {
        return "Nearest Neighbor";
    } else {
        throw std::runtime_error(
            "Invalid filter specified. Valid options are bilinear, bicubic, and nearest neighbor" );
    }
}

krakatoa_renderer::krakatoa_renderer() {
    m_data = new krakatoa_renderer_params;
    reset_renderer(); // sets defaults
}

krakatoa_renderer::~krakatoa_renderer() { delete m_data; }

krakatoa_renderer::krakatoa_renderer( const krakatoa_renderer& t ) {
    m_data = new krakatoa_renderer_params;
    *this = t;
}

krakatoa_renderer& krakatoa_renderer::operator=( const krakatoa_renderer& t ) {
    *m_data = *t.m_data;
    return *this;
}

const krakatoa_renderer_params* krakatoa_renderer::get_params() const { return m_data; }

krakatoa_renderer_params* krakatoa_renderer::get_params() { return m_data; }

void krakatoa_renderer::set_error_on_missing_license( bool errorOnMissingLicense ) {
    m_data->errorOnMissingLicense = errorOnMissingLicense;
}

void krakatoa_renderer::set_background_color( float r, float g, float b ) { m_data->backgroundColor.set( r, g, b ); }

void krakatoa_renderer::set_density_per_particle( float densityPerParticle ) {
    m_data->densityPerParticle = densityPerParticle;
}

void krakatoa_renderer::set_density_exponent( int densityExponent ) { m_data->densityExponent = densityExponent; }

void krakatoa_renderer::set_lighting_density_per_particle( float lightingDensityPerParticle ) {
    m_data->lightingDensityPerParticle = lightingDensityPerParticle;
}

void krakatoa_renderer::set_lighting_density_exponent( int lightingDensityExponent ) {
    m_data->lightingDensityExponent = lightingDensityExponent;
}

void krakatoa_renderer::use_emission( bool useEmission ) { m_data->useEmissionColor = useEmission; }

void krakatoa_renderer::set_emission_strength( float emissionStrength ) { m_data->emissionStrength = emissionStrength; }

void krakatoa_renderer::set_emission_strength_exponent( int emissionStrengthExponent ) {
    m_data->emissionStrengthExponent = emissionStrengthExponent;
}

void krakatoa_renderer::use_absorption_color( bool useAbsorptionColor ) {
    m_data->useAbsorptionColor = useAbsorptionColor;
}

void krakatoa_renderer::set_rendering_method( rendering_method_t method ) { m_data->renderingMethod = method; }

void krakatoa_renderer::set_voxel_filter_radius( int radius ) { m_data->voxelFilterRadius = radius; }

void krakatoa_renderer::set_voxel_size( float voxelSize ) { m_data->voxelSize = voxelSize; }

void krakatoa_renderer::set_draw_point_filter( filter_t filter, int filterSize ) {
    m_data->drawPointFilter = get_filter_string( filter, filterSize );
}

void krakatoa_renderer::set_attenuation_lookup_filter( filter_t filter, int filterSize ) {
    m_data->attenuationLookupFilter = get_filter_string( filter, filterSize );
}

void krakatoa_renderer::set_additive_mode( bool additiveMode ) { m_data->additiveMode = additiveMode; }

void krakatoa_renderer::set_matte_renderer_supersampling( int subDivisions ) {
    m_data->matteSuperSampling = subDivisions;
}

void krakatoa_renderer::set_deep_matte_filename( const char* filename ) { m_data->deepMatteFilename = filename; }

void krakatoa_renderer::set_shader( const shader* shader ) { m_data->shaderParams = shader->get_data()->params; }

void krakatoa_renderer::enable_z_depth_render( bool enable ) { m_data->enableZDepthElement = enable; }

void krakatoa_renderer::enable_normal_render( bool enable ) { m_data->enableNormalElement = enable; }

void krakatoa_renderer::enable_velocity_render( bool enable ) { m_data->enableVelocityElement = enable; }

void krakatoa_renderer::enable_occluded_rgba_render( bool enable ) { m_data->enableOccludedRgbaElement = enable; }

void krakatoa_renderer::push_camera( bool reset ) {
    krakatoa_camera newCamera;

    newCamera.tm = m_data->cameraTm;
    newCamera.type = m_data->cameraType;
    newCamera.fov = m_data->cameraFov;
    newCamera.orthographicWidth = m_data->cameraOrthographicWidth;
    newCamera.nearClipping = m_data->cameraNearClipping;
    newCamera.farClipping = m_data->cameraFarClipping;
    newCamera.screenOffset = m_data->screenOffset;

    newCamera.renderResolution = m_data->renderResolution;
    newCamera.pixelAspectRatio = m_data->pixelAspectRatio;

    newCamera.renderSaveCallback = m_data->renderSaveCallback;

    m_data->cameras.push_back( newCamera );

    if( reset ) {
        krakatoa_renderer::reset_camera();
    }
}

void krakatoa_renderer::reset_camera() {
    m_data->cameraTm = animated_transform();
    m_data->cameraType = CAMERA_PERSPECTIVE;
    m_data->cameraFov = 90.0f / 180.0f * static_cast<float>( M_PI );
    m_data->cameraOrthographicWidth = 30.0f;
    m_data->cameraNearClipping = 0.001f;
    m_data->cameraFarClipping = 1e+10f;
    m_data->screenOffset.set( 0.0f, 0.0f );

    m_data->renderResolution = frantic::graphics2d::size2( 640, 480 );
    m_data->pixelAspectRatio = 1.0f;

    m_data->renderSaveCallback = NULL;
}

void krakatoa_renderer::add_camera( const animated_transform& tm, camera_type_t type, float fov,
                                    float orthographicWidth, float nearClipping, float farClipping,
                                    float horizontalOffset, float verticalOffset, int xResolution, int yResolution,
                                    float pixelAspectRatio, render_save_interface* renderSaveCallback ) {
    krakatoa_camera newCamera;

    newCamera.tm = tm;
    newCamera.type = type;
    newCamera.fov = fov;
    newCamera.orthographicWidth = orthographicWidth;
    newCamera.nearClipping = nearClipping;
    newCamera.farClipping = farClipping;
    newCamera.screenOffset = vector2f( horizontalOffset, verticalOffset );

    newCamera.renderResolution = size2( xResolution, yResolution );
    newCamera.pixelAspectRatio = pixelAspectRatio;

    newCamera.renderSaveCallback = renderSaveCallback;

    m_data->cameras.push_back( newCamera );
}

void krakatoa_renderer::clear_cameras() { m_data->cameras.clear(); }

void krakatoa_renderer::set_camera_tm( const animated_transform& tm ) { m_data->cameraTm = tm; }

void krakatoa_renderer::set_camera_type( camera_type_t type ) { m_data->cameraType = type; }

void krakatoa_renderer::set_camera_perspective_fov( float fov ) { m_data->cameraFov = fov; }

void krakatoa_renderer::set_camera_orthographic_width( float orthographicWidth ) {
    m_data->cameraOrthographicWidth = orthographicWidth;
}

void krakatoa_renderer::set_camera_clipping( float nearClipping, float farClipping ) {
    m_data->cameraNearClipping = nearClipping;
    m_data->cameraFarClipping = farClipping;
}

void krakatoa_renderer::set_screen_offset( float horizontalOffset, float verticalOffset ) {
    m_data->screenOffset = vector2f( horizontalOffset, verticalOffset );
}

void krakatoa_renderer::set_render_resolution( int xResolution, int yResolution ) {
    m_data->renderResolution = size2( xResolution, yResolution );
}

void krakatoa_renderer::set_pixel_aspect_ratio( float ratio ) { m_data->pixelAspectRatio = ratio; }

void krakatoa_renderer::enable_motion_blur( bool enableMotionBlur ) { m_data->enableMotionBlur = enableMotionBlur; }

void krakatoa_renderer::set_motion_blur( float shutterBegin, float shutterEnd, int numSamples,
                                         bool jitteredMotionBlur ) {
    m_data->shutterBegin = shutterBegin;
    m_data->shutterEnd = shutterEnd;
    m_data->numMotionBlurSamples = numSamples;
    m_data->useJitteredMotionBlur = jitteredMotionBlur;
}

void krakatoa_renderer::enable_camera_blur( bool enableCameraBlur ) { m_data->disableCameraBlur = !enableCameraBlur; }

void krakatoa_renderer::enable_depth_of_field( bool enableDepthOfField ) { m_data->enableDof = enableDepthOfField; }

void krakatoa_renderer::set_depth_of_field( float fStop, float focalLength, float focalDistance, float sampleRate ) {
    m_data->dofFStop = fStop;
    m_data->dofFocalLength = focalLength;
    m_data->dofFocalDistance = focalDistance;
    m_data->dofSampleRate = sampleRate;
}

void krakatoa_renderer::enable_adaptive_motion_blur( bool enable ) { m_data->enableAdaptiveMotionBlur = enable; }

void krakatoa_renderer::set_adaptive_motion_blur_min_samples( int minSamples ) {
    m_data->adaptiveMotionBlurMinSamples = minSamples;
}

void krakatoa_renderer::set_adaptive_motion_blur_max_samples( int maxSamples ) {
    m_data->adaptiveMotionBlurMaxSamples = maxSamples;
}

void krakatoa_renderer::set_adaptive_motion_blur_smoothness( float smoothness ) {
    m_data->adaptiveMotionBlurSmoothness = smoothness;
}

void krakatoa_renderer::set_adaptive_motion_blur_exponent( float exponent ) {
    m_data->adaptiveMotionBlurExponent = exponent;
}

void krakatoa_renderer::set_bokeh_shape_map( const texture_data& texture ) {
    if( !frantic::math::is_power_of_two( texture.width ) ) {
        throw std::runtime_error( "set_bokeh_shape_map(): Error: Texture width must be a power of 2." );
    }
    m_data->bokehShapeMap = texture;
    m_data->useBokehShapeMap = true;
}

void krakatoa_renderer::set_bokeh_blend_map( const texture_data& texture ) {
    if( !frantic::math::is_power_of_two( texture.width ) ) {
        throw std::runtime_error( "set_bokeh_blend_map(): Error: Texture width must be a power of 2." );
    }
    m_data->bokehBlendMap = texture;
    m_data->useBokehBlendMap = true;
}

void krakatoa_renderer::set_bokeh_blend_influence( float influence ) { m_data->bokehBlendInfluence = influence; }

void krakatoa_renderer::set_anamorphic_squeeze( float squeeze ) {
    m_data->useAnamorphicSqueeze = true;
    m_data->anamorphicSqueeze = boost::algorithm::clamp( squeeze, 0.1f, 10.0f );
}

void krakatoa_renderer::set_bokeh_blend_mipmap_scale( int scale ) {
    m_data->bokehBlendMipmapScale = boost::algorithm::clamp( scale, 1, 256 );
}

void krakatoa_renderer::set_allocate_bokeh_blend_influence( bool allocate ) {
    m_data->allocateBokehBlendInfluenceChannel = allocate;
}

void krakatoa_renderer::add_light(
    const light* lt,
    const animated_transform&
        tm ) { // HAVENT TESTED IT WITH ANIMATED TM. ONLY STATIC. HAVENT TESTED SHADOW MAPS (WAIT FOR MATTE OBJECTS)
    m_data->lights.push_back( std::make_pair( *lt->get_data(), tm ) );
}

void krakatoa_renderer::remove_all_lights() { m_data->lights.clear(); }

void krakatoa_renderer::add_mesh( const triangle_mesh* mesh, const animated_transform& tm ) {
    m_data->meshes.push_back( std::make_pair( mesh->get_data(), tm ) );
}

void krakatoa_renderer::remove_all_meshes() { m_data->meshes.clear(); }

void krakatoa_renderer::add_particle_stream( particle_stream stream ) {
    // sanity check to make sure the user didn't pass an NULL stream.
    if( !stream.get_data()->stream ) {
        throw std::runtime_error( "This particle stream has not been initalized. The user must initialize the stream "
                                  "by using the \"create_from\" member functions in the particle_stream class." );
    }
    // see if this stream already exists in the renderer.
    for( int i = 0; i < m_data->particles.size(); ++i ) {
        if( m_data->particles[i].get_data()->stream == stream.get_data()->stream )
            throw std::runtime_error(
                "This particle stream has already been added to the renderer object. Streams can only be added once. "
                "To use the same data in multiple streams, use a cached stream object." );
    }
    m_data->particles.push_back( stream );
}

void krakatoa_renderer::remove_all_particle_streams() { m_data->particles.clear(); }

void krakatoa_renderer::save_output_prt( const char* prtFilename, bool computeLightingChannel, bool useDefaultChannels,
                                         float unitLength, coordinate_system_type_t coordinateSystem, int fps ) {
    m_data->particleOutputFilename = prtFilename;
    m_data->particleOutputComputeLighting = computeLightingChannel;
    m_data->particleOutputUseDefaultChannels = useDefaultChannels;
    m_data->particleOutputMetadataUnits = unitLength;
    m_data->particleOutputMetadataCoordSys = coordinateSystem;
    m_data->particleOutputMetadataFramerate = fps;
}

void krakatoa_renderer::append_output_prt_channel( const char* channelName, data_type_t channelDataType,
                                                   int channelArity ) {
    particle_output_channel channel;
    channel.name = channelName;
    channel.type = channelDataType;
    channel.arity = channelArity;
    m_data->particleOutputChannels.push_back( channel );
}

void krakatoa_renderer::set_progress_logger_update( progress_logger_interface* progressLoggerUpdater ) {
    m_data->progressLoggerUpdater = progressLoggerUpdater;
}

void krakatoa_renderer::set_frame_buffer_update( frame_buffer_interface* frameBufferUpdater ) {
    m_data->frameBufferUpdater = frameBufferUpdater;
}

void krakatoa_renderer::set_cancel_render_callback( cancel_render_interface* cancelRenderCheck ) {
    m_data->cancelRenderCheck = cancelRenderCheck;
}

void krakatoa_renderer::set_render_save_callback( render_save_interface* renderSaveCallback ) {
    m_data->renderSaveCallback = renderSaveCallback;
}

void krakatoa_renderer::set_number_of_threads( int numThreads ) { m_data->numThreads = numThreads; }

void krakatoa_renderer::set_frame_buffer_available_memory_fraction( float fraction ) {
    m_data->frameBufferAvailableMemoryFraction = fraction;
}

void krakatoa_renderer::set_enable_multi_shader_mode( const shader_isotropic* isotropic, const shader_phong* phong,
                                                      const shader_henyey_greenstein* henyey,
                                                      const shader_schlick* schlick, const shader_kajiya_kay* kajiya,
                                                      const shader_marschner* marschner ) {
    m_data->useMultiShaderMode = true;
    m_data->shaders.push_back( isotropic ? isotropic->get_data()->params
                                         : krakatoa::shader_params( _T( "Isotropic" ) ) );
    m_data->shaders.push_back( phong ? phong->get_data()->params : krakatoa::shader_params( _T( "Phong Surface" ) ) );
    m_data->shaders.push_back( henyey ? henyey->get_data()->params
                                      : krakatoa::shader_params( _T( "Henyey-Greenstein" ) ) );
    m_data->shaders.push_back( schlick ? schlick->get_data()->params : krakatoa::shader_params( _T( "Schlick" ) ) );
    m_data->shaders.push_back( kajiya ? kajiya->get_data()->params
                                      : krakatoa::shader_params( _T( "Kajiya-Kay Hair" ) ) );
    m_data->shaders.push_back( marschner ? marschner->get_data()->params
                                         : krakatoa::shader_params( _T( "Marschner Hair" ) ) );
}

bool krakatoa_renderer::render() { return render_scene( *m_data ); }

void krakatoa_renderer::reset_renderer() { reset_krakatoa_renderer_params( *m_data ); }

} // namespace krakatoasr
