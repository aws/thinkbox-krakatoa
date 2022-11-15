// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_renderer/renderer.hpp>

#include <boost/assign/list_of.hpp>

#include <krakatoasr_renderer/params.hpp>
#include <krakatoasr_renderer/progress_logger.hpp>

#include <krakatoasr_particles.hpp>
#include <krakatoasr_render_saving.hpp>

#include <krakatoa/camera_manager.hpp>
#include <krakatoa/light_object.hpp>
#include <krakatoa/renderer.hpp>
#include <krakatoa/splat_renderer/splat_renderer.hpp>
#include <krakatoa/voxel_renderer/default_filter3f.hpp>
#include <krakatoa/voxel_renderer/voxel_renderer.hpp>
#include <krakatoa/watermark.hpp>

#include <krakatoa/channel_render_element.hpp>
#include <krakatoa/emission_render_element.hpp>
#include <krakatoa/light_render_element.hpp>
#include <krakatoa/normal_render_element.hpp>
#include <krakatoa/occluded_layer_render_element.hpp>
#include <krakatoa/specular_render_element.hpp>
#include <krakatoa/velocity_render_element.hpp>
#include <krakatoa/zdepth_render_element.hpp>

#include <boost/filesystem.hpp>
#include <frantic/graphics2d/framebuffer.hpp>
#include <frantic/logging/progress_logger.hpp>
#include <frantic/particles/streams/apply_function_particle_istream.hpp>
#include <frantic/particles/streams/particle_array_particle_istream.hpp>
#include <frantic/particles/streams/transformed_particle_istream.hpp>
#include <frantic/rendering/lights/directlight.hpp>
#include <frantic/rendering/lights/pointlight.hpp>
#include <frantic/rendering/lights/spotlight.hpp>

#include <tbb/parallel_for.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/tbb_exception.h>

using namespace frantic::graphics;
using namespace frantic::graphics2d;
using namespace frantic::geometry;
using namespace frantic::channels;
using namespace frantic::particles::streams;
using frantic::strings::to_tstring;

namespace krakatoasr {

namespace detail {

/// a scene context container class needed by krakatoa
class krakatoasr_scene_context : public krakatoa::scene_context {
  private:
    double m_time;
    krakatoa::camera_manager_ptr m_cameraManager;
    krakatoa::collection_wrapper<krakatoa::matte_primitive_ptr, std::vector> m_matteObjects;
    krakatoa::collection_wrapper<boost::shared_ptr<krakatoa::light_object>, std::vector> m_lightObjects;

  public:
    krakatoasr_scene_context()
        : m_cameraManager( new krakatoa::camera_manager ) {}

    double get_time() const { return m_time; }
    void set_time( double time ) { m_time = time; }

    const camera<float>& get_camera() const { return m_cameraManager->get_current_camera(); }
    camera<float>& get_camera() { return m_cameraManager->get_current_camera(); }

    void set_camera( const camera<float>& cam ) {
        m_cameraManager = krakatoa::camera_manager_ptr( new krakatoa::camera_manager( cam ) );
    }

    void set_camera( const std::vector<camera<float>>& cams ) {
        m_cameraManager = krakatoa::camera_manager_ptr( new krakatoa::camera_manager( cams ) );
    }

    const matte_collection& get_matte_objects() const { return m_matteObjects; }
    const light_collection& get_light_objects() const { return m_lightObjects; }
    void add_matte_object( krakatoa::matte_primitive_ptr pObj ) { m_matteObjects.push_back( pObj ); }
    void add_light_object( boost::shared_ptr<krakatoa::light_object> pLightObj ) {
        m_lightObjects.push_back( pLightObj );
    }
};

/// a matte object wrapper class needed by krakatoa
class krakatoasr_matte_primitive : public krakatoa::matte_primitive {
  private:
    const triangle_mesh_params* m_params;
    animated_transform m_tm;
    float m_shutterBegin;
    float m_shutterEnd;

    float m_currentShutterTime;
    bool m_shutterTimeIsValid;

  public:
    krakatoasr_matte_primitive( const triangle_mesh_params* params, const animated_transform& tm, float shutterBegin,
                                float shutterEnd )
        : m_params( params )
        , m_tm( tm )
        , m_shutterBegin( shutterBegin )
        , m_shutterEnd( shutterEnd ) {
        m_currentShutterTime = 0.0f;
        m_shutterTimeIsValid = false; // set_time needs to be called before we can use this mesh. ok?
    }
    virtual void set_time( float motionSegmentTime ) {
        krakatoa::matte_primitive::set_time( motionSegmentTime ); // needed
        m_currentShutterTime =
            m_shutterBegin +
            motionSegmentTime * ( m_shutterEnd - m_shutterBegin ); // current shutter time (motionSegmentTime was a
                                                                   // [0-1] value between shutter begin and end)
        m_shutterTimeIsValid = ( m_currentShutterTime >= m_params->shutterValidityBegin &&
                                 m_currentShutterTime < m_params->shutterValidityEnd );
    }
    virtual std::size_t get_triangle_count() const {
        if( m_shutterTimeIsValid )
            return m_params->mesh->face_count();
        return 0;
    }
    virtual void get_triangle( std::size_t index, frantic::graphics::vector3f outVertices[] ) const {
        if( index < get_triangle_count() ) {
            const vector3& faceIndices = m_params->mesh->get_face( index );
            float velocityMult = m_currentShutterTime - m_params->shutterTime;
            if( velocityMult != 0.0f && m_params->velocityAcc.valid() ) {
                outVertices[0] =
                    m_params->mesh->get_vertex( faceIndices[0] ) + m_params->velocityAcc[faceIndices[0]] * velocityMult;
                outVertices[1] =
                    m_params->mesh->get_vertex( faceIndices[1] ) + m_params->velocityAcc[faceIndices[1]] * velocityMult;
                outVertices[2] =
                    m_params->mesh->get_vertex( faceIndices[2] ) + m_params->velocityAcc[faceIndices[2]] * velocityMult;
            } else {
                outVertices[0] = m_params->mesh->get_vertex( faceIndices[0] );
                outVertices[1] = m_params->mesh->get_vertex( faceIndices[1] );
                outVertices[2] = m_params->mesh->get_vertex( faceIndices[2] );
            }
        } else
            throw std::runtime_error( "Attempting to get invalid triangle index." );
    }
    virtual float get_opacity( const frantic::geometry::raytrace_intersection& /*ri*/ ) const { return 1.0f; }
    virtual frantic::graphics::transform4f get_transform() const {
        frantic::graphics::transform4f tm;
        m_tm.get_transform( &tm[0], m_currentShutterTime );
        return tm;
    }
    virtual void get_mesh_copy( frantic::geometry::trimesh3& /*outMesh*/ ) const {
        // NOTE: This currently only gets called if we're raytracing the mesh, and that ONLY happens when the camera is
        // NON-perspetive and NON-orthographic (eg vray spherical). As a result, this function is currently NEVER called
        // in Krakatoa SR.
        throw std::runtime_error( "Implement get_mesh_copy if we need raytracing of matte meshes in Krakatoa SR." );
    }
    virtual bool is_visible_to_lights() const { return m_params->visibleToLights; }
    virtual bool is_visible_to_cameras() const { return m_params->visibleToCamera; }
};

// a matte interface wrapper that adds an initial depth map to the existing matte interface.
class krakatoasr_matte_interface : public krakatoa::matte_interface {
    std::vector<float> m_initialDepthmap;
    frantic::graphics2d::size2 m_size;

  public:
    krakatoasr_matte_interface( const krakatoa::scene_context_ptr context )
        : matte_interface( context ) {}

    // allow the user to select a pre-existing depth image to be combined with
    // note: this object takes ownership of the initialDepthmap passed in via a swap.
    void set_initial_depthmap( std::vector<float>& initialDepthmap, frantic::graphics2d::size2 size ) {
        m_initialDepthmap.swap( initialDepthmap );
        m_size = size;
    }

    // this implementation calls the superclass, but adds in the initial depthmap (as defined above)
    virtual void generate_depth_map( const frantic::graphics::camera<float>& renderCam, float mblurTime,
                                     frantic::graphics2d::framebuffer<float>& outDepthImg, bool forLight ) {

        // call parent class's function (it does all the work)
        // this renders the triangle meshes if there are any
        matte_interface::generate_depth_map( renderCam, mblurTime, outDepthImg, forLight );

        // boost::shared_ptr<frantic::graphics2d::framebuffer<float> > m_initialMatteDepthmap;
        // boost::shared_ptr< frantic::graphics2d::framebuffer<float> > depthBuffer( new
        // frantic::graphics2d::framebuffer<float>( initialDepthmap, size ) );

        // combine with the user-defined initial matte depth map
        if( !m_initialDepthmap.empty() && !forLight ) {

            // our input to this is a vector<float> and a framebuffer<float>
            // we will convert them into depthbuffer_singleface objects (using swaps where possible) to use the
            // "resample_combine" member function. then get the framebuffer back out and reassign to out output
            // outDepthImg parameter.

            frantic::graphics2d::framebuffer<float> initialFramebuffer( m_initialDepthmap, m_size ); // does a deep copy
            frantic::rendering::depthbuffer_singleface initialDepthbuffer;
            initialDepthbuffer.set_with_swap( initialFramebuffer );

            frantic::rendering::depthbuffer_singleface recombinedDepthbuffer;
            recombinedDepthbuffer.set_with_swap( outDepthImg );

            recombinedDepthbuffer.resample_combine( initialDepthbuffer );
            outDepthImg.swap( recombinedDepthbuffer.as_framebuffer() );
        }
    }
};

} // namespace detail

void get_render_particle_channels( const krakatoa_renderer_params& params, channel_map& outPcm ) {
    if( !outPcm.has_channel( _T( "Position" ) ) )
        outPcm.define_channel( _T( "Position" ), 3, frantic::channels::data_type_float32 );

    if( !outPcm.has_channel( _T( "Color" ) ) )
        outPcm.define_channel( _T( "Color" ), 3, frantic::channels::data_type_float16 );

    if( !outPcm.has_channel( _T( "Density" ) ) )
        outPcm.define_channel( _T( "Density" ), 1, frantic::channels::data_type_float16 );

    if( params.renderingMethod != METHOD_VOXEL ) {
        if( !outPcm.has_channel( _T( "Lighting" ) ) ) {
            outPcm.define_channel( _T( "Lighting" ), 3, frantic::channels::data_type_float16 );
        }
    }

    if( params.enableMotionBlur && params.shutterBegin < params.shutterEnd ) {
        if( !outPcm.has_channel( _T( "Velocity" ) ) )
            outPcm.define_channel( _T( "Velocity" ), 3, frantic::channels::data_type_float16 );

        if( params.useJitteredMotionBlur ) {
            if( !outPcm.has_channel( _T( "MBlurTime" ) ) )
                outPcm.define_channel( _T( "MBlurTime" ), 1, frantic::channels::data_type_float16 );
        }
    }

    if( params.useAbsorptionColor ) {
        if( !outPcm.has_channel( _T( "Absorption" ) ) )
            outPcm.define_channel( _T( "Absorption" ), 3, frantic::channels::data_type_float16 );
    }

    if( params.useEmissionColor ) {
        if( !outPcm.has_channel( _T( "Emission" ) ) )
            outPcm.define_channel( _T( "Emission" ), 3, frantic::channels::data_type_float16 );
    }

    if( params.useBokehBlendMap && params.enableDof && params.allocateBokehBlendInfluenceChannel ) {
        if( !outPcm.has_channel( _T( "BokehBlendInfluence" ) ) )
            outPcm.define_channel( _T( "BokehBlendInfluence" ), 1, frantic::channels::data_type_float16 );
    }

    if( params.doReflection ) {
        if( !outPcm.has_channel( _T( "Normal" ) ) ) {
            outPcm.define_channel<vector3f>( _T( "Normal" ) );
        }

        if( params.allocateReflectionStrength && !outPcm.has_channel( _T( "ReflectionStrength" ) ) ) {
            outPcm.define_channel<float>( _T( "ReflectionStrength" ) );
        }
    }

    if( params.useMultiShaderMode ) {
        if( !outPcm.has_channel( _T( "PhaseFunction" ) ) ) {
            outPcm.define_channel<boost::int32_t>( _T( "PhaseFunction" ) );
        }
    }
}

void retrieve_particles( krakatoa_renderer_params& params, krakatoa::renderer::particle_container_type& outParticles ) {

    const frantic::channels::channel_map& pcm = outParticles.get_channel_map();

    // create a default particle for the outgoing stream.
    boost::scoped_array<char> defaultParticle( new char[pcm.structure_size()] );
    memset( defaultParticle.get(), 0, pcm.structure_size() );
    if( pcm.has_channel( _T( "Density" ) ) )
        pcm.get_cvt_accessor<float>( _T( "Density" ) ).set( defaultParticle.get(), 1.0f );
    if( pcm.has_channel( _T( "Selection" ) ) )
        pcm.get_cvt_accessor<float>( _T( "Selection" ) ).set( defaultParticle.get(), 1.0f ); // what is "Selection"?
    if( pcm.has_channel( _T( "Color" ) ) )
        pcm.get_cvt_accessor<color3f>( _T( "Color" ) ).set( defaultParticle.get(), color3f( 1.0f ) );
    // TODO: These should not be hard-coded, but set by the shader.
    if( pcm.has_channel( _T( "Eccentricity" ) ) )
        pcm.get_cvt_accessor<float>( _T( "Eccentricity" ) )
            .set( defaultParticle.get(), params.shaderParams.phaseEccentricity );
    if( pcm.has_channel( _T( "SpecularPower" ) ) )
        pcm.get_cvt_accessor<float>( _T( "SpecularPower" ) )
            .set( defaultParticle.get(), params.shaderParams.specularPower );
    if( pcm.has_channel( _T( "SpecularLevel" ) ) )
        pcm.get_cvt_accessor<float>( _T( "SpecularLevel" ) )
            .set( defaultParticle.get(), params.shaderParams.specularLevel / 100.0f );
    if( pcm.has_channel( _T( "BokehBlendInfluence" ) ) )
        pcm.get_cvt_accessor<float>( _T( "BokehBlendInfluence" ) )
            .set( defaultParticle.get(), params.bokehBlendInfluence );
    if( pcm.has_channel( _T( "PhaseFunction" ) ) )
        pcm.get_cvt_accessor<boost::int32_t>( _T( "PhaseFunction" ) ).set( defaultParticle.get(), -1 );

    // add transformation stream. this could not be done originally, because we didn't know what shutterBegin or
    // shutterEnd would be at render time.
    for( int i = 0; i < params.particles.size(); ++i ) {
        krakatoasr::particle_stream_data* streamData = params.particles[i].get_data();

        // get the base transformation matrix
        float mblurCenterTime = ( params.shutterBegin + params.shutterEnd ) *
                                0.5f; // the "center" of the interval is always the base time for all scene objects
                                      // (including particles) in krakatoa sr.
        float tmElements[16];
        streamData->tm.get_transform( tmElements, mblurCenterTime );
        transform4f particleTm( tmElements );
        if( !particleTm.is_identity() ) {
            boost::shared_ptr<particle_istream> transformedStream(
                new transformed_particle_istream<float>( streamData->stream, particleTm ) );
            streamData->stream = transformedStream;
        }

        if( pcm.has_channel( _T( "Velocity" ) ) ) {

            // get the transformation matrices for computing new particle velocity
            if( streamData->tm.is_animated() ) {
                // currently we are only using the the first and last transformation matrix in the motion block.
                // this is because krakatoa particles are only allowed to have linear motion.
                // our function adds the velocity generated from the transformation motion onto the existing velocity
                // channel.
                float shutterBegin = params.shutterBegin;
                float shutterEnd = params.shutterEnd;

                // this is a typical scene *without* motion blur, but still has a "Velocity" channel.
                // this happens when the scene has a "velocity render element", but no shutter times. so, we just use
                // the first and last transform in the animation.
                if( shutterEnd == shutterBegin ) {
                    shutterBegin = streamData->tm.get_data()->tm.begin()->first;
                    shutterEnd = streamData->tm.get_data()->tm.rbegin()->first;
                }

                float shutterLength = shutterEnd - shutterBegin;
                if( shutterLength > 0.0f ) {
                    float tmElementsBegin[16];
                    float tmElementsEnd[16];
                    streamData->tm.get_transform( tmElementsBegin, shutterBegin );
                    streamData->tm.get_transform( tmElementsEnd, shutterEnd );
                    transform4f objectTmInv = particleTm.to_inverse();
                    transform4f tm1 = transform4f( tmElementsBegin ) * objectTmInv;
                    transform4f tm2 = transform4f( tmElementsEnd ) * objectTmInv;

                    // inline static struct/function for the function istream below
                    static boost::array<frantic::tstring, 2> affectedChannels = { _T( "Velocity" ), _T( "Position" ) };
                    struct modify_velocity {
                        static vector3f fn( const vector3f& velocity, const vector3f& position, const transform4f& tm1,
                                            const transform4f& tm2, float timeStep ) {
                            return velocity + ( tm2 * position - tm1 * position ) * timeStep;
                        }
                    };
                    boost::shared_ptr<particle_istream> tmVelocityStream(
                        new apply_function_particle_istream<vector3f( vector3f, vector3f )>(
                            streamData->stream,
                            boost::bind( &modify_velocity::fn, boost::placeholders::_1, boost::placeholders::_2, tm1,
                                         tm2, 1.0f / shutterLength ),
                            _T( "Velocity" ), affectedChannels ) );
                    streamData->stream = tmVelocityStream;
                }
            }

            // advect the particles so that time zero is the center of the motion blur interval.
            // because krakatoa sr always presents itself to the renderer as centered motion blur, we must move the
            // particles forward or backward in time based on where the center of the motion blur interval is. we might
            // want to consider changing this (and using the built-in motion blur bias), but if we do, we must find
            // every place that motionBegin and motionEnd are accessed and fix the newly introduced offsetting errors.
            if( fabsf( mblurCenterTime ) >
                10e-7f ) { // not a great check. this offset is generally in seconds, so this seems appropriate.
                krakatoasr::add_time_offset( params.particles[i], mblurCenterTime );
            }
        }
    }

    if( params.progressLoggerUpdater )
        params.progressLoggerUpdater->set_title( "Retrieving Particles" );
    frantic::diagnostics::profiling_section psRetrieve( _T( "Retrieving Particles" ) );

    // each stream provided to the render will go in this list.
    std::vector<boost::shared_ptr<particle_istream>> listOfStreams;

    // deterimine how many particles to reserve for our final particle array
    boost::int64_t numParticlesToReserve = 0;
    bool hasUnknownParticleCountStream = false;

    // go though each of the streams and do a particle count
    for( int i = 0; i < params.particles.size(); ++i ) {
        boost::shared_ptr<particle_istream> pin = params.particles[i].get_data()->stream;

        listOfStreams.push_back( pin );
        boost::int64_t particleCount = pin->particle_count();
        if( particleCount >= 0 ) {
            numParticlesToReserve += particleCount;
        } else {
            // unknown number of particles
            boost::int64_t particleCountGuess = pin->particle_count_guess();
            if( particleCountGuess >= 0 )
                numParticlesToReserve += particleCountGuess;
            else
                hasUnknownParticleCountStream = true;
        }
    }

    // if there are streams that have an unknown number of particles, we should make our own guess.
    // this is kind of how max-krakatoa does it, so i'm continuing the tradition
    if( hasUnknownParticleCountStream )
        numParticlesToReserve += 1000000;

    FF_LOG( debug ) << "Internally reserving memory for " << numParticlesToReserve << " particles.\n";

    psRetrieve.enter();

    // reserve what we believe to be enough for the particles in all streams.
    // this is generally a pretty big chunk right here.
    outParticles.reserve( numParticlesToReserve );

    // retrieve the particles
    // Note: Previously, this was done in parallel. But it caused more problems that it solved.
    frantic::logging::null_progress_logger nullProgress;
    for( size_t i = 0; i < listOfStreams.size(); ++i ) {
        if( listOfStreams[i]->particle_count() != 0 ) {
            // quick sanity check before we begin.
            if( !listOfStreams[i]->get_native_channel_map().has_channel( _T( "Position" ) ) ) {
                FF_LOG( warning ) << "Warning: The stream \"" << listOfStreams[i]->name()
                                  << "\" does not have a \"Position\" channel. All particle streams must provide a "
                                     "\"Position\" channel to be used by Krakatoa. This stream will be skipped.\n";
            } else {
                // insert those particles!
                outParticles.insert_particles( listOfStreams[i], defaultParticle.get(), nullProgress );
            }
        }
    }

    psRetrieve.exit();

    // Do a check for infinite or NaN positions, deleting these particles.
    frantic::channels::channel_accessor<vector3f> posAccessor = pcm.get_accessor<vector3f>( _T( "Position" ) );
    size_t particleSize = pcm.structure_size();
    for( krakatoa::renderer::particle_container_type::iterator it = outParticles.begin(), itEnd = outParticles.end();
         it != itEnd;
         /*nothing*/ ) {
        if( !posAccessor.get( *it ).is_finite() ) {
            if( --itEnd != it )
                memcpy( *it, *itEnd, particleSize );
            outParticles.pop_back();
        } else
            ++it;
    }

    FF_LOG( stats ) << psRetrieve << std::endl;

    size_t numParticles = outParticles.size();
    FF_LOG( stats ) << "Total number of particles to be rendered: "
                    << frantic::strings::int_to_comma_seperated_string( (int)numParticles ) << std::endl;
    size_t particleMemory = particleSize * numParticles;
    FF_LOG( stats ) << "Total Memory Allocated to particles: "
                    << frantic::strings::int_bytes_to_string( particleMemory ) << std::endl;
}

static std::vector<camera<float>> create_render_camera_helper( const krakatoa_renderer_params& params,
                                                               bool hasVelocityChannel,
                                                               const std::vector<krakatoa_camera>& cameras ) {
    std::vector<camera<float>> renderCameras;

    for( std::vector<krakatoa_camera>::const_iterator it = cameras.begin(); it != cameras.end(); ++it ) {
        camera<float> newCamera;

        // set camera transformation
        if( hasVelocityChannel && it->tm.is_animated() ) {
            // create an animated transformation. it uses 10 evenly samples between the beginning and ending shutter
            // times. this is to match max krakatoa which also uses 10 samples.
            std::vector<transform4f> animatedCameraTm( 10 );
            for( int i = 0; i < 10; ++i ) {
                float shutterTime = params.shutterBegin + ( params.shutterEnd - params.shutterBegin ) * ( i / 9.0f );
                float tm[16];
                it->tm.get_transform( tm, shutterTime );
                animatedCameraTm[i] = transform4f( tm );
            }
            newCamera.set_transform( animatedCameraTm );
        } else {
            // unanimated transform
            float tm[16];
            it->tm.get_transform( tm, ( params.shutterBegin + params.shutterEnd ) * 0.5f ); // the "center" of the
                                                                                            // interval is always the
                                                                                            // base time for all scene
                                                                                            // objects (including the
                                                                                            // camera) in krakatoa sr.
            newCamera.set_transform( transform4f( tm ) );
        }

        newCamera.set_projection_mode( it->type == CAMERA_PERSPECTIVE ? projection_mode::perspective
                                                                      : projection_mode::orthographic );

        newCamera.set_output_size( it->renderResolution );
        newCamera.set_pixel_aspect( it->pixelAspectRatio );

        newCamera.set_near( it->nearClipping );
        newCamera.set_far( it->farClipping );
        newCamera.set_horizontal_fov( it->fov );
        newCamera.set_orthographic_width( it->orthographicWidth );
        newCamera.set_subpixel_offset( it->screenOffset );

        // dof parameters of camera
        newCamera.set_fstop( params.dofFStop );
        newCamera.set_focal_length( params.dofFocalLength );
        newCamera.set_focal_distance( params.dofFocalDistance );

        renderCameras.push_back( newCamera );
    }

    return renderCameras;
}

std::vector<camera<float>> create_render_camera( const krakatoa_renderer_params& params, bool hasVelocityChannel ) {
    std::vector<camera<float>> renderCameras;

    if( params.cameras.empty() ) {
        std::vector<krakatoa_camera> camerasToCopy;

        krakatoa_camera camera;
        camera.tm = params.cameraTm;
        camera.type = params.cameraType;
        camera.fov = params.cameraFov;
        camera.orthographicWidth = params.cameraOrthographicWidth;
        camera.nearClipping = params.cameraNearClipping;
        camera.farClipping = params.cameraFarClipping;
        camera.screenOffset = params.screenOffset;

        camera.renderResolution = params.renderResolution;
        camera.pixelAspectRatio = params.pixelAspectRatio;

        camerasToCopy.push_back( camera );

        renderCameras = create_render_camera_helper( params, hasVelocityChannel, camerasToCopy );
    } else {
        renderCameras = create_render_camera_helper( params, hasVelocityChannel, params.cameras );
    }

    return renderCameras;
}

boost::shared_ptr<frantic::rendering::lights::lightinterface>
create_light_interface( const light_params& params, const animated_transform& tm,
                        const krakatoa_renderer_params& rendererParams ) {

    // create transformation
    motion_blurred_transform<float> animTm;
    if( rendererParams.shutterBegin < rendererParams.shutterEnd ) {
        // currently we're only doing a linear animation on the transform, even if the animated transform contains
        // multiple samples.
        float matrixBegin[16];
        float matrixEnd[16];
        tm.get_transform( matrixBegin, rendererParams.shutterBegin );
        tm.get_transform( matrixEnd, rendererParams.shutterEnd );
        animTm = motion_blurred_transform<float>( transform4f( matrixBegin ), transform4f( matrixEnd ) );
    } else {
        float matrix[16];
        tm.get_transform(
            matrix,
            rendererParams
                .shutterBegin ); // shutter begin and end are equal in this case. when this happens, usually they are
                                 // zero (but not always). in this case, shutterBegin is equal to shutterEnd.
        animTm = motion_blurred_transform<float>( transform4f( matrix ) );
    }

    // create light interface object
    boost::shared_ptr<frantic::rendering::lights::lightinterface> myLight;

    // create the correct interface type based on the "lightType" parameter.
    if( params.lightType == "direct" ) {
        // turns a light_params object into a directlight
        myLight.reset( new frantic::rendering::lights::directlight(
            to_tstring( params.name ), animTm, params.flux, params.decayExponent, params.enableShadows,
            params.shadowDensity, params.shadowMapWidth, params.useNearAtten, params.useFarAtten,
            params.nearAttenuationStart, params.nearAttenuationEnd, params.farAttenuationStart,
            params.farAttenuationEnd,
            (frantic::rendering::lights::LIGHT_SHAPE)params.lightShape, // assume same enum
            params.lightAspect,
            params.innerRectRadius, // specified in degrees
            params.outerRectRadius, params.decayRadius ) );

        if( !params.attenuationMapSavingPath.empty() ) {
            boost::filesystem::create_directory(
                boost::filesystem::path( frantic::files::directory_from_path( params.attenuationMapSavingPath ) ) );
            frantic::graphics2d::size2 mapDim( params.shadowMapWidth );
            boost::shared_ptr<frantic::rendering::atten_saver> attenSaver(
                new frantic::rendering::singleface_atten_exr_saver(
                    to_tstring( params.attenuationMapSavingPath ), mapDim, params.attenuationMapDepthSamples,
                    params.attenuationMapSampleSpacing, params.attenuationMapExponentialSampleSpacing ) );
            myLight->set_attenuation_saver( attenSaver );
        }

        // set attenuation loading
        if( !params.attenMapLoadingPath.empty() )
            myLight->set_attenuation_loader(
                frantic::rendering::create_singleface_atten_loader( to_tstring( params.attenMapLoadingPath ) ) );

    } else if( params.lightType == "spot" ) {
        // turns a light_params object into a spotlight
        float innerConeHalfAngle =
            params.innerConeAngle / 180.0f * (float)M_PI * 0.5f; // convert to radiens, and half it
        float outerConeHalfAngle = params.outerConeAngle / 180.0f * (float)M_PI * 0.5f;

        myLight.reset( new frantic::rendering::lights::spotlight(
            to_tstring( params.name ), animTm, params.flux, params.decayExponent, params.enableShadows,
            params.shadowDensity, params.shadowMapWidth, params.useNearAtten, params.useFarAtten,
            params.nearAttenuationStart, params.nearAttenuationEnd, params.farAttenuationStart,
            params.farAttenuationEnd,
            (frantic::rendering::lights::LIGHT_SHAPE)params.lightShape, // assume same enum
            params.lightAspect,
            innerConeHalfAngle, // IMPORTANT: do not pass in the whole angle, half of it only. specified in radians
            outerConeHalfAngle, params.decayRadius ) );

        if( !params.attenuationMapSavingPath.empty() ) {
            boost::filesystem::create_directory(
                boost::filesystem::path( frantic::files::directory_from_path( params.attenuationMapSavingPath ) ) );
            frantic::graphics2d::size2 mapDim( params.shadowMapWidth );
            boost::shared_ptr<frantic::rendering::atten_saver> attenSaver(
                new frantic::rendering::singleface_atten_exr_saver(
                    to_tstring( params.attenuationMapSavingPath ), mapDim, params.attenuationMapDepthSamples,
                    params.attenuationMapSampleSpacing, params.attenuationMapExponentialSampleSpacing ) );
            myLight->set_attenuation_saver( attenSaver );
        }

        // set attenuation loading
        if( !params.attenMapLoadingPath.empty() )
            myLight->set_attenuation_loader(
                frantic::rendering::create_singleface_atten_loader( to_tstring( params.attenMapLoadingPath ) ) );

    } else if( params.lightType == "point" ) {
        // turns a light_params object into a pointlight
        myLight.reset( new frantic::rendering::lights::pointlight(
            to_tstring( params.name ), animTm, params.flux, params.decayExponent, params.enableShadows,
            params.shadowDensity, params.shadowMapWidth, params.useNearAtten, params.useFarAtten,
            params.nearAttenuationStart, params.nearAttenuationEnd, params.farAttenuationStart,
            params.farAttenuationEnd, params.decayRadius ) );

        if( !params.attenuationMapSavingPath.empty() ) {
            boost::filesystem::create_directory(
                boost::filesystem::path( frantic::files::directory_from_path( params.attenuationMapSavingPath ) ) );
            boost::shared_ptr<frantic::rendering::atten_saver> attenSaver(
                new frantic::rendering::cubeface_atten_exr_saver(
                    to_tstring( params.attenuationMapSavingPath ), params.shadowMapWidth,
                    params.attenuationMapDepthSamples, params.attenuationMapSampleSpacing,
                    params.attenuationMapExponentialSampleSpacing ) );
            myLight->set_attenuation_saver( attenSaver );
        }

        // set attenuation loading
        if( !params.attenMapLoadingPath.empty() )
            myLight->set_attenuation_loader( frantic::rendering::create_cubeface_atten_loader(
                frantic::strings::to_tstring( params.attenMapLoadingPath ) ) );

    } else {
        throw std::runtime_error( "Light with name \"" + params.name + "\" has an invalid light type: \"" +
                                  params.lightType + "\". Valid types are \"direct\", \"spot\", and \"point\"." );
    }

    // add light interface to renderer
    return myLight;
}

void write_particles( const krakatoa::renderer::particle_container_type& parray, const channel_map& outputChannelMap,
                      const std::string& prtFilename, float unitLength, coordinate_system_type_t coordinates, int fps,
                      boost::shared_ptr<frantic::logging::render_progress_logger> renderProgress ) {
    int prtCompressionLevel = -1; // what should this value be?

    frantic::particles::particle_file_stream_factory_object streamFactory;
    if( coordinates != COORDINATE_SYSTEM_UNSPECIFIED )
        streamFactory.set_coordinate_system( static_cast<frantic::graphics::coordinate_system::option>( coordinates ) );
    if( unitLength != 0 )
        streamFactory.set_length_unit_in_meters( unitLength );
    if( fps != 0 )
        streamFactory.set_frame_rate( fps, 1 );

    boost::shared_ptr<frantic::particles::streams::particle_array_particle_istream> pin(
        new frantic::particles::streams::particle_array_particle_istream( parray ) );
    boost::shared_ptr<frantic::particles::streams::particle_ostream> pout =
        streamFactory.create_ostream( to_tstring( prtFilename ), pin->get_channel_map(), outputChannelMap,
                                      pin->particle_count(), prtCompressionLevel );

    // could possibly throw a cancel exception. which will be caught later.
    frantic::particles::save_particle_stream( pin, pout, *renderProgress );
    FF_LOG( stats ) << "Particle file written: " << to_tstring( prtFilename ) << std::endl;
}

void render_scene_internal( krakatoa_renderer_params& params ) {
    tbb::task_scheduler_init scheduler;

    // this is the object that handles displaying progress updates, frame buffer updates, and handling throwing "cancel"
    // exceptions (caught by calling function)
    boost::shared_ptr<krakatoasr_progress_logger> renderProgress( new krakatoasr_progress_logger(
        params.progressLoggerUpdater, params.frameBufferUpdater, params.cancelRenderCheck ) );

    // right off the bat, check if it's already been cancelled. this will throw if cancelled.
    renderProgress->check_for_abort();

    boost::intrusive_ptr<detail::krakatoasr_scene_context> sceneContext( new detail::krakatoasr_scene_context );
    sceneContext->set_time( 0.0 );

    // sanity checking
    if( params.shutterBegin > params.shutterEnd )
        throw std::runtime_error( "The shutter begin time was greater than the shutter end time. The shutter begin "
                                  "time must be less than or equal to the shutter end time." );

    boost::shared_ptr<krakatoa::renderer> krakRenderer;

    color6f backgroundColor( params.backgroundColor.r, params.backgroundColor.g, params.backgroundColor.b );
    boost::function<void( framebuffer<color6f>& )> watermarkFunc = &krakatoa::null_watermark;

    // if we aren't saving the image or saving the prt file, there's nothing to do.
    bool noSaveCallback = true;
    for( std::vector<krakatoa_camera>::const_iterator it = params.cameras.begin(); it != params.cameras.end(); ++it ) {
        if( it->renderSaveCallback != NULL ) {
            noSaveCallback = false;
            break;
        }
    }
    noSaveCallback = noSaveCallback && !params.renderSaveCallback;

    if( noSaveCallback && params.particleOutputFilename == "" && !params.lightingOnly ) {
        FF_LOG( warning )
            << "The renderer has no output callback set (by calling krakatoa_renderer::set_render_save_callback), nor "
               "does it have an output particle file set (by calling save_output_prt). No render will be performed "
               "because there are no outputs."
            << std::endl;
        return;
    }

    float renderDensity = params.densityPerParticle;
    renderDensity *= powf( 10.0f, (float)params.densityExponent );

    float lightDensity = params.lightingDensityPerParticle;
    lightDensity *= powf( 10.0f, (float)params.lightingDensityExponent );

    float emissionStrength = params.emissionStrength;
    emissionStrength *= powf( 10.f, (float)params.emissionStrengthExponent );
    if( emissionStrength < 0.0f )
        emissionStrength = 0.0f;

    // set up the krakRenderer base object
    if( params.renderingMethod == METHOD_VOXEL ) {
        krakatoa::voxel_renderer::voxel_renderer_ptr renderer(
            krakatoa::voxel_renderer::voxel_renderer::create_instance() );

        boost::shared_ptr<krakatoa::voxel_renderer::default_filter3f> particleFilter(
            new krakatoa::voxel_renderer::default_filter3f( (float)params.voxelFilterRadius ) );
        renderer->set_particle_filter( particleFilter );

        float voxelSize = params.voxelSize;
        renderer->set_voxel_size( voxelSize );

        float spacingFactor = ( voxelSize * voxelSize * voxelSize );
        renderDensity /= spacingFactor;
        lightDensity /= spacingFactor;
        emissionStrength /= spacingFactor;

        krakRenderer = renderer;
    } else if( params.renderingMethod == METHOD_PARTICLE ) {
        krakatoa::splat_renderer::splat_renderer_ptr renderer(
            krakatoa::splat_renderer::splat_renderer::create_instance() );
        // Set the rendering thread limit
        // This is just the pysical number of threads that Krakatoa could use (usually pass in the default of -1,
        // meaning the TBB thread pool size) This actual number of threads used is optimized based on the size of the
        // frame buffer later on
        renderer->set_rendering_thread_limit( params.numThreads );
        renderer->set_fram_buffer_available_memory_fraction( params.frameBufferAvailableMemoryFraction );

        renderer->set_splat_filter(
            krakatoa::splat_renderer::filter2f::create_instance( to_tstring( params.drawPointFilter ) ) );

        if( !params.additiveMode ) {
            krakatoa::splat_renderer::splat_lighting_ptr lightEngine =
                krakatoa::splat_renderer::splat_lighting::create_instance();
            lightEngine->set_splat_filter(
                krakatoa::splat_renderer::filter2f::create_instance( to_tstring( params.attenuationLookupFilter ) ) );
            renderer->set_lighting_engine( lightEngine );
        }

        krakRenderer = renderer;
    } else {
        throw std::runtime_error( "Unknown renderer type. Please use either particle or voxel rendering method." );
    }

    // create shader object
    boost::shared_ptr<krakatoa::krakatoa_shader> pShader = krakatoa::create_shader( params.shaderParams );

    // set up the channel map
    frantic::channels::channel_map pcm;

    get_render_particle_channels( params, pcm );
    pShader->define_required_channels( pcm );

    if( params.useMultiShaderMode ) {
        krakRenderer->set_use_mixed_shaders( true );
        for( std::vector<krakatoa::shader_params>::const_iterator it = params.shaders.begin();
             it != params.shaders.end(); ++it ) {
            boost::shared_ptr<krakatoa::krakatoa_shader> shader = krakatoa::create_shader( *it );
            shader->define_required_channels( pcm );
            krakRenderer->add_shader( shader );
        }
    }

    // create render elements and add them as needed
    boost::shared_ptr<krakatoa::zdepth_render_element> zDepthElements;
    boost::shared_ptr<krakatoa::normal_render_element> normalElement;
    boost::shared_ptr<krakatoa::velocity_render_element> velocityElement;
    boost::shared_ptr<krakatoa::occluded_layer_render_element> occludedRbgaElement;
    boost::shared_ptr<krakatoa::emission_render_element> emissionRenderElement;
    boost::shared_ptr<krakatoa::specular_render_element> specularRenderElement;
    std::vector<boost::shared_ptr<krakatoa::channel_render_element>> channelRenderElements;
    if( params.enableZDepthElement ) {
        zDepthElements.reset( new krakatoa::zdepth_render_element( sceneContext, false, false, 0.0f, 1.0f ) );
        zDepthElements->add_required_channels( pcm );
        krakRenderer->add_render_element( zDepthElements );
    }
    if( params.enableNormalElement ) {
        normalElement.reset( new krakatoa::normal_render_element( sceneContext, true, false ) );
        normalElement->add_required_channels( pcm );
        krakRenderer->add_render_element( normalElement );
    }
    if( params.enableVelocityElement ) {
        velocityElement.reset( new krakatoa::velocity_render_element( sceneContext, true, 1.0f, false,
                                                                      std::numeric_limits<float>::max() ) );

        // sets a constant derivative for world-to-camera matrix (for camera motion blur)
        if( params.cameraTm.is_animated() ) {
            const std::map<float, transform4f>& animatedTms = params.cameraTm.get_data()->tm;
            float animationLength = animatedTms.rbegin()->first - animatedTms.begin()->first;
            if( animationLength > 0.0f ) {
                transform4f tmDeriv =
                    ( animatedTms.rbegin()->second.to_inverse() - animatedTms.begin()->second.to_inverse() ) /
                    animationLength;
                velocityElement->set_world_to_camera_deriv( tmDeriv );
            }
        }

        velocityElement->add_required_channels( pcm );
        krakRenderer->add_render_element( velocityElement );
    }
    if( params.enableOccludedRgbaElement ) {
        occludedRbgaElement.reset( new krakatoa::occluded_layer_render_element );
        krakRenderer->add_render_element( occludedRbgaElement );
    }
    if( params.enableEmissionElement ) {
        emissionRenderElement.reset( new krakatoa::emission_render_element );
        krakRenderer->add_render_element( emissionRenderElement );
        emissionRenderElement->add_required_channels( pcm );
    }
    if( params.enableSpecularElement || params.enableSpecular2Element ) {
        specularRenderElement.reset( new krakatoa::specular_render_element );
        krakRenderer->add_render_element( specularRenderElement );
        specularRenderElement->add_required_channels( pcm );
    }
    for( std::vector<particle_output_channel>::const_iterator it = params.customChannelElements.begin();
         it != params.customChannelElements.end(); ++it ) {
        static const std::map<krakatoasr::data_type_t, frantic::channels::data_type_t> ksrToFranticType =
            boost::assign::map_list_of<krakatoasr::data_type_t, frantic::channels::data_type_t>(
                krakatoasr::DATA_TYPE_FLOAT16, frantic::channels::data_type_float16 )(
                krakatoasr::DATA_TYPE_FLOAT32, frantic::channels::data_type_float32 )(
                krakatoasr::DATA_TYPE_FLOAT64, frantic::channels::data_type_float64 );
        std::map<krakatoasr::data_type_t, frantic::channels::data_type_t>::const_iterator findIt =
            ksrToFranticType.find( it->type );
        channelRenderElements.push_back( boost::make_shared<krakatoa::channel_render_element>(
            true, frantic::strings::to_tstring( it->name ),
            findIt != ksrToFranticType.end() ? findIt->second : frantic::channels::data_type_float16, it->arity,
            color3f( 0.f ) ) );
        channelRenderElements.back()->add_required_channels( pcm );
        krakRenderer->add_render_element( channelRenderElements.back() );
    }

    // create channel output's channel map, and extra channels that were requested by the users's prt saver to the main
    // channel map.
    channel_map particleOutputChannelMap;
    if( !params.particleOutputFilename.empty() ) {
        if( params.particleOutputUseDefaultChannels ) {
            // if we are saving out the prt as a pre-render (no lighting), remove the "Lighting" channel
            for( size_t i = 0; i < pcm.channel_count(); ++i ) {
                frantic::tstring name;
                frantic::channels::data_type_t dataType;
                size_t arity;
                pcm.get_channel_definition( i, name, dataType, arity );
                if( name != _T( "Lighting" ) || params.particleOutputComputeLighting )
                    particleOutputChannelMap.define_channel( name, arity, dataType );
            }
        } else {
            if( params.particleOutputChannels.empty() ) {
                FF_LOG( warning )
                    << _T( "A PRT particle output file was specified (\"" ) +
                           to_tstring( params.particleOutputFilename ) +
                           _T( "\"). The user has requested to use a custom channel layout for this file, however no channels have been specified. As a result, no .PRT file will be written. To write a .PRT file, either specify channels using \"append_output_prt_channel\", or set \"useDefaultChannels\" when calling \"save_output_prt\"." )
                    << std::endl;
                params.particleOutputFilename = ""; // stop saving of channels;
            } else {
                // define all the custom channels the user has requested
                for( size_t i = 0; i < params.particleOutputChannels.size(); ++i ) {
                    const particle_output_channel& channel = params.particleOutputChannels[i];
                    frantic::channels::data_type_t dataType =
                        (frantic::channels::data_type_t)channel.type; // warning: enums must be identical.
                    // special-case "Position", because when we union the channel maps, we do NOT want position to be
                    // upgraded past 32bit (the renderer will crash).
                    if( channel.name == "Position" && dataType > frantic::channels::data_type_float32 )
                        dataType = frantic::channels::data_type_float32;
                    // add to our particle output channel map
                    particleOutputChannelMap.define_channel( to_tstring( channel.name ), channel.arity, dataType );
                }

                // add these channels to our main channel map
                pcm.union_channel_map( particleOutputChannelMap );

                FF_LOG( debug ) << "Overriding default .prt file output channel map. Using channel map:\n"
                                << particleOutputChannelMap << std::endl;
            }
        }
    }

    // finished the channel definitions at this point.
    pcm.end_channel_definition();
    particleOutputChannelMap.end_channel_definition();

    FF_LOG( debug ) << "Using particle layout:\n" << pcm << std::endl;

    // create and set camera
    std::vector<camera<float>> renderCameras = create_render_camera( params, pcm.has_channel( _T( "Velocity" ) ) );
    sceneContext->set_camera( renderCameras );

    // set up motion blur
    int mblurSamples = 0;
    bool mblurJittered = false;
    float mblurDuration = 0.0f;
    if( params.enableMotionBlur ) {
        mblurSamples = params.numMotionBlurSamples;
        mblurDuration = params.shutterEnd - params.shutterBegin;
        mblurJittered = params.useJitteredMotionBlur;
    }

    bool dofAllowed = params.enableDof;
    float dofSampleRate = params.dofSampleRate;

    krakatoa::renderer::mode_type::enum_t renderMode = krakatoa::renderer::mode_type::normal;
    if( params.additiveMode )
        renderMode = krakatoa::renderer::mode_type::additive;

    // add matte meshes to the renderer
    for( int i = 0; i < params.meshes.size(); ++i ) {
        boost::shared_ptr<krakatoa::matte_primitive> matteMesh( new detail::krakatoasr_matte_primitive(
            params.meshes[i].first, params.meshes[i].second, params.shutterBegin, params.shutterEnd ) );
        sceneContext->add_matte_object( matteMesh );
    }

    // create a new matte interface. this will be given to the renderer.
    // the matte interface will get its mesh objects though the sceneContext.
    // the way this works is really run around and stupid. but here it is anyway.
    boost::shared_ptr<detail::krakatoasr_matte_interface> matteInterface;
    if( params.meshes.size() > 0 || !params.initialMatteDepthBuffer.empty() ) {
        matteInterface.reset( new detail::krakatoasr_matte_interface( sceneContext ) );
        matteInterface->set_num_threads(
            params.numThreads ); // separate from the rest, because it is the only thing that does not use TBB.
    }

    // lights
    for( size_t i = 0; i < params.lights.size(); ++i ) {
        boost::shared_ptr<frantic::rendering::lights::lightinterface> lightInterface =
            create_light_interface( params.lights[i].first, params.lights[i].second, params );
        krakatoa::light_object_ptr lightObj = krakatoa::light_object::create( lightInterface );
        // lightObj->set_atmosphere( atmosphere ); //not using apme
        boost::shared_ptr<krakatoa::shadow_map_generator> shadowMapGen;
        if( matteInterface )
            shadowMapGen.reset( new krakatoa::shadow_map_generator( matteInterface ) );
        lightObj->set_shadow_map_generator( shadowMapGen );
        sceneContext->add_light_object( lightObj );
    }

    // load initial depth image from buffer
    if( !params.initialMatteDepthBuffer.empty() )
        matteInterface->set_initial_depthmap( params.initialMatteDepthBuffer, params.initialMatteDepthBufferSize );

    // load deep matte image
    boost::shared_ptr<frantic::rendering::singleface_atten_loader> deepMatteMap;
    if( !params.deepMatteFilename.empty() )
        deepMatteMap = frantic::rendering::create_singleface_atten_loader( to_tstring( params.deepMatteFilename ) );

    // setup the common renderer attributes
    // krakRenderer->set_atmosphere( atmosphere ); //not using apme
    // krakRenderer->set_environment( environment ); //we don't have a way of doing environment in SR yet
    krakRenderer->set_matte_sampler( matteInterface, params.matteSuperSampling );
    krakRenderer->set_deep_matte_map( deepMatteMap );
    krakRenderer->set_scene_context( sceneContext );
    krakRenderer->set_watermark( watermarkFunc );

    krakRenderer->set_background_color( backgroundColor );
    krakRenderer->set_density_scale( renderDensity, lightDensity );
    krakRenderer->set_depth_of_field_enabled( dofAllowed, dofSampleRate );
    krakRenderer->set_emission_scale( emissionStrength );
    krakRenderer->set_motion_blur_interval( mblurDuration, 0.0f );
    krakRenderer->set_motion_blur_samples( mblurSamples );
    krakRenderer->set_motion_blur_jittered( mblurJittered );

    if( params.enableMotionBlur && params.enableAdaptiveMotionBlur ) {
        krakRenderer->set_enable_adaptive_motion_blur( true );
        krakRenderer->set_adaptive_motion_blur_min_samples( params.adaptiveMotionBlurMinSamples );
        krakRenderer->set_adaptive_motion_blur_max_samples( params.adaptiveMotionBlurMaxSamples );
        krakRenderer->set_adaptive_motion_blur_smoothness( params.adaptiveMotionBlurSmoothness );
        krakRenderer->set_adaptive_motion_blur_exponent( params.adaptiveMotionBlurExponent );
    } else {
        krakRenderer->set_enable_adaptive_motion_blur( false );
    }

    if( params.useBokehShapeMap ) {
        frantic::graphics2d::image_channel<float> shapeMap;
        const int width = params.bokehShapeMap.width;
        frame_buffer_pixel_data* pixel = params.bokehShapeMap.pixels;
        shapeMap.set_size( frantic::graphics2d::size2( width, width ) );
        for( int i = 0; i < width; ++i ) {
            for( int j = 0; j < width; ++j, ++pixel ) {
                const float alpha = ( pixel->r_alpha + pixel->g_alpha + pixel->b_alpha ) / 3.0f;
                shapeMap.set_pixel( i, j, alpha );
            }
        }

        krakRenderer->set_bokeh_mask( shapeMap );
    }

    if( params.useBokehBlendMap ) {
        frantic::graphics2d::image_channel<frantic::graphics::color3f> blendMap;
        const int width = params.bokehBlendMap.width;
        frame_buffer_pixel_data* pixel = params.bokehBlendMap.pixels;
        blendMap.set_size( frantic::graphics2d::size2( width, width ) );
        for( int i = 0; i < width; ++i ) {
            for( int j = 0; j < width; ++j, ++pixel ) {
                const frantic::graphics::color3f color( pixel->r, pixel->g, pixel->b );
                blendMap.set_pixel( i, j, color );
            }
        }

        krakRenderer->set_bokeh_blend_map( blendMap );
        krakRenderer->set_bokeh_blend_amount( params.bokehBlendInfluence );
    }

    if( params.useAnamorphicSqueeze ) {
        krakRenderer->set_anamorphic_squeeze( params.anamorphicSqueeze );
    }

#if defined( KRAKATOA_RT_CALLBACKS )
    if( params.mblurPassCallback ) {
        krakRenderer->set_mblur_pass_callback( *params.mblurPassCallback );
    }

    if( params.lightingCallback ) {
        krakRenderer->set_lighting_callback( *params.lightingCallback );
    }

    if( params.matteGenerator ) {
        krakRenderer->set_matte_generator( *params.matteGenerator );
    }
#endif

    if( params.saveOccludedZDepth ) {
        krakRenderer->set_save_occluded_ZDepth( true );
    }

    krakRenderer->set_mipmap_resolution_coefficient( params.bokehBlendMipmapScale );

    krakRenderer->set_render_mode( renderMode );

    krakRenderer->set_shader( pShader );
    krakRenderer->set_progress_logger( renderProgress );

    // check for a cancel before the particle retrieving starts. this will throw if cancelled.
    renderProgress->check_for_abort();

    // set particles. these calls put ALL the particles from the streams into memory
    krakatoa::renderer::particle_container_type parray( pcm );
    retrieve_particles(
        params, parray ); // this call will take all the particles from all the inputted streams and stick 'em in parray
    krakRenderer->set_particles( &parray );

    // check for a cancel after the particle retrieving is done. this will throw if cancelled.
    renderProgress->check_for_abort();

    // pre-render write particles (if the user requested)
    if( !params.particleOutputFilename.empty() && !params.particleOutputComputeLighting ) {
        // write out the particles.
        write_particles( parray, particleOutputChannelMap, params.particleOutputFilename,
                         params.particleOutputMetadataUnits, params.particleOutputMetadataCoordSys,
                         params.particleOutputMetadataFramerate, renderProgress );

        // if we aren't going on to save any rendered image, just return at this point.
        // this happens only with no image savers, and particleOutputComputeLighting set to false.
        bool hasSaveCallback = false;
        for( std::vector<krakatoa_camera>::const_iterator it = params.cameras.begin(); it != params.cameras.end();
             ++it ) {
            if( it->renderSaveCallback != NULL ) {
                hasSaveCallback = true;
                break;
            }
        }
        hasSaveCallback = hasSaveCallback || params.renderSaveCallback;
        if( !hasSaveCallback ) {
            return;
        }
    }

    if( !params.additiveMode )
        krakRenderer->precompute_lighting();

    frantic::graphics2d::framebuffer<color6f> renderFramebuffer( sceneContext->get_camera().get_output_size() );

    // do actual render. this is where all the work is done.
    if( !params.lightingOnly )
        krakRenderer->render( renderFramebuffer );

    renderProgress->update_progress( 100.0 );

    // update the frame buffer with the final image (this might have already happened, but maybe not)
    renderProgress->update_frame_buffer( renderFramebuffer );

    // post-render write particles (if the user requested)
    if( !params.particleOutputFilename.empty() && params.particleOutputComputeLighting ) {
        // write out the particles. if this returns false, that means the user CANCELLED the render.
        write_particles( parray, particleOutputChannelMap, params.particleOutputFilename,
                         params.particleOutputMetadataUnits, params.particleOutputMetadataCoordSys,
                         params.particleOutputMetadataFramerate, renderProgress );
    }

    // write out final image(s) to files
    std::size_t i = 0;
    for( std::vector<camera<float>>::const_iterator cameraIterator = renderCameras.begin();
         cameraIterator != renderCameras.end(); ++cameraIterator, ++i ) {
        render_save_interface* renderSaveCallback;
        if( params.cameras.empty() ) {
            renderSaveCallback = params.renderSaveCallback;
        } else {
            renderSaveCallback = params.cameras[i].renderSaveCallback;
        }

        if( renderSaveCallback ) {
            // NOTE: the following calls assume color6f has the same memory layout as frame_buffer_pixel_data. Which is
            // does. but if that changes, we're boned.
            int imageCount = 0;
            std::vector<output_type_t> listOfTypes;
            std::vector<frame_buffer_pixel_data*> listOfImages;
            std::vector<std::string> listOfCustomChannelNames;

            // add rbga
            listOfTypes.push_back( OUTPUT_RGBA );
            listOfImages.push_back( (frame_buffer_pixel_data*)( &renderFramebuffer.data()[0] ) );
            ++imageCount;
            // add z
            if( params.enableZDepthElement ) {
                listOfTypes.push_back( OUTPUT_Z );
                listOfImages.push_back( (frame_buffer_pixel_data*)( &zDepthElements->get_framebuffer().data()[0] ) );
                ++imageCount;
            }
            // add normal
            if( params.enableNormalElement ) {
                listOfTypes.push_back( OUTPUT_NORMAL );
                listOfImages.push_back( (frame_buffer_pixel_data*)( &normalElement->get_framebuffer().data()[0] ) );
                ++imageCount;
            }
            // add velocity
            if( params.enableVelocityElement ) {
                listOfTypes.push_back( OUTPUT_VELOCITY );
                listOfImages.push_back( (frame_buffer_pixel_data*)( &velocityElement->get_framebuffer().data()[0] ) );
                ++imageCount;
            }
            // add occluded rbga
            if( params.enableOccludedRgbaElement ) {
                if( params.renderingMethod == METHOD_VOXEL ) {
                    FF_LOG( warning ) << "Occluded RGBA pass is not supported in \"voxel\" rendering mode.\n";
                } else {
                    listOfTypes.push_back( OUTPUT_RGBA_OCCLUDED );
                    listOfImages.push_back(
                        (frame_buffer_pixel_data*)( &occludedRbgaElement->get_framebuffer().data()[0] ) );
                    ++imageCount;
                }
            }
            if( params.enableEmissionElement ) {
                listOfTypes.push_back( OUTPUT_EMISSION );
                listOfImages.push_back(
                    (frame_buffer_pixel_data*)( &emissionRenderElement->get_framebuffer().data()[0] ) );
                ++imageCount;
            }
            if( params.enableSpecularElement || params.enableSpecular2Element ) {
                listOfTypes.push_back( OUTPUT_SPECULAR );
                listOfImages.push_back(
                    (frame_buffer_pixel_data*)( &specularRenderElement->get_framebuffer().data()[0] ) );
                ++imageCount;
            }
            std::size_t i = 0;
            for( std::vector<boost::shared_ptr<krakatoa::channel_render_element>>::const_iterator it =
                     channelRenderElements.begin();
                 it != channelRenderElements.end(); ++it, ++i ) {
                listOfTypes.push_back( OUTPUT_CUSTOM );
                listOfImages.push_back( (frame_buffer_pixel_data*)( &( *it )->get_framebuffer().data()[0] ) );
                listOfCustomChannelNames.push_back( params.customChannelElements[i].name );
                ++imageCount;
            }

            krakatoasr::multi_channel_exr_file_saver* exrSaver =
                dynamic_cast<krakatoasr::multi_channel_exr_file_saver*>( renderSaveCallback );
            if( exrSaver ) {
                exrSaver->get_data()->customChannelNames = listOfCustomChannelNames;
                std::size_t i = 0;
                for( std::vector<std::string>::const_iterator it = listOfCustomChannelNames.begin();
                     it != listOfCustomChannelNames.end(); ++it, ++i ) {
                    krakatoasr::custom_channel_exr_file_saver_data saverData;
                    saverData.r = *it + ".R";
                    saverData.g = *it + ".G";
                    saverData.b = *it + ".B";
                    saverData.bitDepth = krakatoasr::BIT_DEPTH_HALF;
                    exrSaver->get_data()->customChannels[*it] = saverData;
                }
            }

            // call the save function
            renderSaveCallback->save_render_data( cameraIterator->get_output_size().xsize,
                                                  cameraIterator->get_output_size().ysize, imageCount, &listOfTypes[0],
                                                  &listOfImages[0] );
        }
    }
}

bool render_scene( krakatoa_renderer_params& params ) {

    // Wrap in a special try-catch, we will be calling into the progress logger, which means we might receive an abort
    // exception.
    bool isCancelled = false;
    try {

        // Launch the internal function!
        render_scene_internal( params );

    } catch( frantic::logging::progress_cancel_exception& e ) {
        FF_LOG( debug ) << e.what() << std::endl;
        isCancelled = true;
    } catch( tbb::captured_exception& e ) {
        // this is a little bit hacky, but it's a neccessary consequence of throwing exceptions inside tbb handled code
        if( std::string( e.name() ) == typeid( frantic::logging::progress_cancel_exception ).name() ) {
            FF_LOG( debug ) << e.what() << std::endl;
            isCancelled = true;
        } else {
            // otherwise, re-throw it as a standard runtime error, we'll lose the runtime type, but that's the best we
            // can do
            params.particles.clear(); // must clear
            throw std::runtime_error( e.what() );
        }
    } catch( std::exception& e ) {
        params.particles.clear(); // must clear
        throw;
    }

    params.particles.clear(); // must clear the particle streams after the render call. always.
    return !isCancelled;
}

boost::shared_ptr<krakatoa::raytrace_renderer::raytrace_renderer>
setup_raytrace_renderer( krakatoa_renderer_params& params,
                         krakatoa::renderer::particle_container_type& particleBuffer ) {
    tbb::task_scheduler_init scheduler;

    // this is the object that handles displaying progress updates, frame buffer updates, and handling throwing "cancel"
    // exceptions (caught by calling function)
    boost::shared_ptr<krakatoasr_progress_logger> renderProgress( new krakatoasr_progress_logger(
        params.progressLoggerUpdater, params.frameBufferUpdater, params.cancelRenderCheck ) );

    // right off the bat, check if it's already been cancelled. this will throw if cancelled.
    renderProgress->check_for_abort();

    boost::intrusive_ptr<detail::krakatoasr_scene_context> sceneContext( new detail::krakatoasr_scene_context );
    sceneContext->set_time( 0.0 );

    // sanity checking
    if( params.shutterBegin > params.shutterEnd )
        throw std::runtime_error( "The shutter begin time was greater than the shutter end time. The shutter begin "
                                  "time must be less than or equal to the shutter end time." );

    float renderDensity = params.densityPerParticle;
    renderDensity *= powf( 10.0f, (float)params.densityExponent );

    float lightDensity = params.lightingDensityPerParticle;
    lightDensity *= powf( 10.0f, (float)params.lightingDensityExponent );

    float emissionStrength = params.emissionStrength;
    emissionStrength *= powf( 10.f, (float)params.emissionStrengthExponent );
    if( emissionStrength < 0.0f )
        emissionStrength = 0.0f;
    // TODO: Take spacing factor into account.

    // create shader object
    boost::shared_ptr<krakatoa::krakatoa_shader> pShader = krakatoa::create_shader( params.shaderParams );

    // set up the channel map
    frantic::channels::channel_map pcm;

    get_render_particle_channels( params, pcm );
    pShader->define_required_channels( pcm );

    pcm.end_channel_definition();

    // set up motion blur
    int mblurSamples = 0;
    bool mblurJittered = false;
    float mblurDuration = 0.0f;
    if( params.enableMotionBlur ) {
        mblurSamples = params.numMotionBlurSamples;
        mblurDuration = params.shutterEnd - params.shutterBegin;
        mblurJittered = params.useJitteredMotionBlur;
    }

    bool dofAllowed = params.enableDof;
    float dofSampleRate = params.dofSampleRate;

    // create a new matte interface. this will be given to the renderer.
    // the matte interface will get its mesh objects though the sceneContext.
    // the way this works is really run around and stupid. but here it is anyway.
    boost::shared_ptr<detail::krakatoasr_matte_interface> matteInterface;
    if( params.meshes.size() > 0 || !params.initialMatteDepthBuffer.empty() ) {
        matteInterface.reset( new detail::krakatoasr_matte_interface( sceneContext ) );
        matteInterface->set_num_threads(
            params.numThreads ); // separate from the rest, because it is the only thing that does not use TBB.
    }

    // lights
    for( size_t i = 0; i < params.lights.size(); ++i ) {
        boost::shared_ptr<frantic::rendering::lights::lightinterface> lightInterface =
            create_light_interface( params.lights[i].first, params.lights[i].second, params );
        krakatoa::light_object_ptr lightObj = krakatoa::light_object::create( lightInterface );
        // lightObj->set_atmosphere( atmosphere ); //not using apme
        boost::shared_ptr<krakatoa::shadow_map_generator> shadowMapGen;
        if( matteInterface )
            shadowMapGen.reset( new krakatoa::shadow_map_generator( matteInterface ) );
        lightObj->set_shadow_map_generator( shadowMapGen );
        sceneContext->add_light_object( lightObj );
    }

    // load initial depth image from buffer
    if( !params.initialMatteDepthBuffer.empty() )
        matteInterface->set_initial_depthmap( params.initialMatteDepthBuffer, params.initialMatteDepthBufferSize );

    // load deep matte image
    boost::shared_ptr<frantic::rendering::singleface_atten_loader> deepMatteMap;
    if( !params.deepMatteFilename.empty() )
        deepMatteMap = frantic::rendering::create_singleface_atten_loader( to_tstring( params.deepMatteFilename ) );

    krakatoa::raytrace_renderer::raytrace_renderer_ptr krakRenderer =
        krakatoa::raytrace_renderer::raytrace_renderer::create_instance( true ); // use openvdb_renderer

    krakRenderer->set_render_mode( params.additiveMode ? krakatoa::renderer::mode_type::additive
                                                       : krakatoa::renderer::mode_type::normal );

    krakRenderer->set_matte_sampler( matteInterface, params.matteSuperSampling );
    krakRenderer->set_deep_matte_map( deepMatteMap );
    krakRenderer->set_scene_context( sceneContext );

    krakRenderer->set_density_scale( renderDensity, lightDensity );
    krakRenderer->set_depth_of_field_enabled( dofAllowed, dofSampleRate );
    krakRenderer->set_emission_scale( emissionStrength );
    krakRenderer->set_motion_blur_interval( mblurDuration, 0.0f );
    krakRenderer->set_motion_blur_samples( mblurSamples );
    krakRenderer->set_motion_blur_jittered( mblurJittered );
    krakRenderer->set_use_emission( params.useEmissionColor );
    krakRenderer->set_use_absorption( params.useAbsorptionColor );

    if( params.enableMotionBlur && params.enableAdaptiveMotionBlur ) {
        krakRenderer->set_enable_adaptive_motion_blur( true );
        krakRenderer->set_adaptive_motion_blur_min_samples( params.adaptiveMotionBlurMinSamples );
        krakRenderer->set_adaptive_motion_blur_max_samples( params.adaptiveMotionBlurMaxSamples );
        krakRenderer->set_adaptive_motion_blur_smoothness( params.adaptiveMotionBlurSmoothness );
        krakRenderer->set_adaptive_motion_blur_exponent( params.adaptiveMotionBlurExponent );
    } else {
        krakRenderer->set_enable_adaptive_motion_blur( false );
    }

    krakRenderer->set_shader( pShader );
    krakRenderer->set_splat_filter(
        krakatoa::splat_renderer::filter2f::create_instance( to_tstring( params.drawPointFilter ) ) );

    if( !params.additiveMode ) {
        krakatoa::splat_renderer::splat_lighting_ptr lightEngine =
            krakatoa::splat_renderer::splat_lighting::create_instance();
        lightEngine->set_splat_filter(
            krakatoa::splat_renderer::filter2f::create_instance( to_tstring( params.attenuationLookupFilter ) ) );
        krakRenderer->set_lighting_engine( lightEngine );
    }

    krakRenderer->set_progress_logger( renderProgress );

    // check for a cancel before the particle retrieving starts. this will throw if cancelled.
    renderProgress->check_for_abort();

    // set particles. these calls put ALL the particles from the streams into memory
    particleBuffer = krakatoa::renderer::particle_container_type( pcm );
    retrieve_particles(
        params,
        particleBuffer ); // this call will take all the particles from all the inputted streams and stick 'em in parray

    krakRenderer->set_particles( &particleBuffer );

    // check for a cancel after the particle retrieving is done. this will throw if cancelled.
    renderProgress->check_for_abort();

    krakRenderer->precompute_lighting();

    renderProgress->update_progress( 100.0 );

    return krakRenderer;
}

} // namespace krakatoasr
