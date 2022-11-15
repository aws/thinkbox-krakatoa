// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_particles.hpp>

#include <krakatoasr_renderer/params.hpp>
#include <krakatoasr_renderer/progress_logger.hpp>

#include <krakatoa/particle_repopulation.hpp>
#include <krakatoa/particle_volume.hpp>
#include <krakatoa/prt_maker.hpp>

#include <frantic/geometry/triangle_utils.hpp>
#include <frantic/particles/particle_file_stream_factory.hpp>
#include <frantic/particles/streams/apply_function_particle_istream.hpp>
#include <frantic/particles/streams/channel_scale_particle_istream.hpp>
#include <frantic/particles/streams/duplicate_channel_particle_istream.hpp>
#include <frantic/particles/streams/fractional_particle_istream.hpp>
#include <frantic/particles/streams/rle_levelset_particle_istream.hpp>
#include <frantic/particles/streams/set_channel_particle_istream.hpp>
#include <frantic/particles/streams/surface_particle_istream.hpp>

#include <boost/bind.hpp>

using namespace frantic::graphics;
using namespace frantic::channels;
using namespace frantic::particles;
using namespace frantic::geometry;

using frantic::strings::to_tstring;

namespace krakatoasr {

//
//
// user defined custom particle streams
//
//

particle_stream_interface::particle_stream_interface() {
    m_data = new particle_stream_interface_data;

    // this is done sort of weirdly. i'm finializing the channel map right away so that we can use "append_channel" to
    // add channels. the reason i want to use "append_channel" is so that the position and offset of the channels do not
    // change after they are added.
    m_data->channelMap.end_channel_definition();
    m_data->hasBeenAddedToStream = false;
}

particle_stream_interface::~particle_stream_interface() { delete m_data; }

particle_stream_interface::particle_stream_interface( const particle_stream_interface& t ) {
    m_data = new particle_stream_interface_data;
    *this = t;
}

particle_stream_interface& particle_stream_interface::operator=( const particle_stream_interface& t ) {
    *m_data = *t.m_data;
    return *this;
}
const particle_stream_interface_data* particle_stream_interface::get_data() const { return m_data; }

particle_stream_interface_data* particle_stream_interface::get_data() { return m_data; }

channel_data particle_stream_interface::append_channel( const char* name, data_type_t dataType, int arity ) {
    if( m_data->hasBeenAddedToStream )
        throw std::runtime_error( "Cannot append more channels to the particle_stream_interface once it has been added "
                                  "to a particle_stream object." );

    // create channel data object to be passed to the user
    channel_data channelData;
    channelData.channelByteSize = (int)sizeof_channel_data_type( (frantic::channels::data_type_t)dataType ) * arity;
    channelData.byteOffsetInParticle = (int)m_data->channelMap.structure_size_without_padding();

    // append this channel to the end of our channel map
    m_data->channelMap.append_channel(
        to_tstring( name ), (size_t)arity,
        (frantic::channels::data_type_t)dataType ); // this assumes the enums are the same. note i'm specifically
                                                    // assigning an offset. maybe not neccessary in this case.

    return channelData;
}

void particle_stream_interface::set_channel_value( const channel_data& channelData, void* particleData,
                                                   const void* inValue ) const {
    memcpy( (char*)particleData + channelData.byteOffsetInParticle, (char*)inValue, channelData.channelByteSize );
}

void particle_stream_interface::get_channel_value( const channel_data& channelData, const void* particleData,
                                                   void* outValue ) const {
    memcpy( (char*)outValue, (char*)particleData + channelData.byteOffsetInParticle, channelData.channelByteSize );
}

namespace detail {
// this is a class that converts our krakatoasr::particle_stream_interface into a
// frantic::particles::streams::particle_istream it is written this way so we don't have to expose our entire
// particle_istream class and helper classes.
class particle_stream_interface_wrapper : public streams::particle_istream {
  private:
    particle_stream_interface* m_delegateStream;
    channel_map m_channelMap;                // channel map set by user
    channel_map m_nativeChannelMap;          // channel map from m_delegateStream
    channel_map_adaptor m_channelMapAdaptor; // adaptor to convert from m_nativeChannelMap to m_channelMap
    boost::int64_t m_particleCount;          // count from m_delegateStream
    boost::int64_t m_currentIndex;           // keeps track of how many particles have been retrieved
    std::vector<char> m_defaultParticle;     // set by user
    char* m_rawDefaultParticle;
    std::vector<char> m_particleScratchSpace; // space of memory used when retrieving particles
    char* m_rawParticleScratchSpace;

  public:
    particle_stream_interface_wrapper( particle_stream_interface* delegateStream ) {
        if( delegateStream->get_data()->hasBeenAddedToStream )
            throw std::runtime_error( "This particle_stream_interface has already been added to a particle_stream "
                                      "object. It cannot be added to a different particle_stream object." );
        delegateStream->get_data()->hasBeenAddedToStream = true;

        const channel_map& delegateChannelMap = delegateStream->get_data()->channelMap;

        if( !delegateChannelMap.has_channel( _T( "Position" ) ) ) {
            throw std::runtime_error(
                "The particle_stream_interface object provided does not have a \"Position\" channel. A \"Position\" "
                "channel is required. Please add it with particle_stream_interface::append_channel()." );
        }
        // set data members of our wrapper stream
        m_delegateStream = delegateStream;
        m_nativeChannelMap = delegateChannelMap;
        set_channel_map( delegateChannelMap );
        m_particleCount = delegateStream->particle_count();
        m_currentIndex = -1;
        m_defaultParticle.resize( m_channelMap.structure_size() );
        m_rawDefaultParticle = &m_defaultParticle[0];
        m_particleScratchSpace.resize( m_nativeChannelMap.structure_size() );
        m_rawParticleScratchSpace = &m_particleScratchSpace[0];
    }
    virtual void close() { m_delegateStream->close(); }
    virtual frantic::tstring name() const { return _T( "<Custom Krakatoa API particle stream>" ); }
    virtual std::size_t particle_size() const { return m_channelMap.structure_size(); }
    virtual boost::int64_t particle_count() const { return m_particleCount; }
    virtual boost::int64_t particle_index() const { return m_currentIndex; }
    virtual boost::int64_t particle_count_left() const {
        if( m_particleCount > -1 )
            return m_particleCount - m_currentIndex - 1;
        return -1;
    }
    virtual boost::int64_t particle_progress_count() const { return particle_count(); }
    virtual boost::int64_t particle_progress_index() const { return particle_index(); }
    virtual void set_channel_map( const frantic::channels::channel_map& particleChannelMap ) {
        m_channelMap = particleChannelMap;
        // create an adaptor between the native map and our requested map
        m_channelMapAdaptor.set( m_channelMap, m_nativeChannelMap );
        // create new, cleared default particle
        m_defaultParticle.resize( m_channelMap.structure_size() );
        m_rawDefaultParticle = &m_defaultParticle[0];
        memset( m_rawDefaultParticle, 0, m_channelMap.structure_size() );
    }
    virtual const frantic::channels::channel_map& get_channel_map() const { return m_channelMap; }
    virtual const frantic::channels::channel_map& get_native_channel_map() const { return m_nativeChannelMap; }
    virtual void set_default_particle( char* rawParticleBuffer ) {
        memcpy( m_rawDefaultParticle, rawParticleBuffer, m_channelMap.structure_size() );
    }
    virtual bool get_particle( char* rawParticleBuffer ) {
        bool success = false;
        if( m_particleCount != -1 &&
            m_currentIndex >=
                m_particleCount ) // this is in here because our stream will keep taking particles out past the stated
                                  // particle count. seems like a bug higher up since we will end up going over our
                                  // particle_array length. but instead of fixing it higher up, it's checked here.
            return false;
        ++m_currentIndex;
        if( m_channelMapAdaptor.is_identity() ) {
            success = m_delegateStream->get_next_particle( rawParticleBuffer );
        } else {
            success = m_delegateStream->get_next_particle( m_rawParticleScratchSpace );
            if( success )
                m_channelMapAdaptor.copy_structure( rawParticleBuffer, m_rawParticleScratchSpace,
                                                    m_rawDefaultParticle );
        }
        return success;
    }
    virtual bool get_particles( char* rawParticleBuffer, std::size_t& numParticles ) {
        size_t particleSize = m_channelMap.structure_size();
        for( std::size_t i = 0; i < numParticles; ++i ) {
            if( !get_particle( rawParticleBuffer ) ) {
                numParticles = i;
                return false;
            }
            rawParticleBuffer += particleSize;
        }
        return true;
    }
};
} // namespace detail

//
//
// particle streams
//
//

particle_stream::particle_stream() { m_data = new particle_stream_data; }

particle_stream::~particle_stream() { delete m_data; }

particle_stream::particle_stream( const particle_stream& t ) {
    m_data = new particle_stream_data;
    *this = t;
}

particle_stream& particle_stream::operator=( const particle_stream& t ) {
    *m_data = *t.m_data;
    return *this;
}

const particle_stream_data* particle_stream::get_data() const { return m_data; }

particle_stream_data* particle_stream::get_data() { return m_data; }

void particle_stream::set_transform( const animated_transform& tm ) { m_data->tm = tm; }

particle_stream particle_stream::create_from_file( const char* filename ) {
    particle_file_stream_factory_object factoryObject;
    factoryObject.set_coordinate_system(
        frantic::graphics::coordinate_system::unspecified ); // must do this, otherwise it defaults to right-hand z-up.
                                                             // In the future, this would be a scene option.
    particle_stream newStream;
    newStream.m_data->stream = factoryObject.create_istream( to_tstring( filename ) );
    return newStream;
}

particle_stream particle_stream::create_from_mesh_volume(
    const triangle_mesh& mesh, float voxelSpacing, unsigned int subdivCount, bool particleJitteringEnabled,
    int numJitteredParticlesPerVoxel, int randomJitterSeed, bool wellDistributedJittering, bool enableShell,
    float shellStart, float shellThickness, cancel_render_interface* cancelCheckCallback, bool* wasCancelled ) {
    using namespace frantic::particles::streams;

    int wellDistributedRandomCount = 1024; // hard coded for now

    krakatoasr_progress_logger cancelChecker( NULL, NULL, cancelCheckCallback );
    if( cancelCheckCallback != NULL ) {
        if( wasCancelled == NULL )
            throw std::runtime_error(
                "When creating a particle stream using create_from_mesh_volume, the user passed in a "
                "cancel_render_interface. To use a cancel callback, the user must also provide a \"wasCancelled\" "
                "boolean which will be set to true if the process was cancelled." );
        *wasCancelled = false;
    }

    // for motion blur, we have this shutterTime parameter. it is so we can provide several meshes at different shutter
    // times for changing topology motion blur. however, this does not really make sense in the context of particle
    // volumes, so let's ignore that parameter.
    if( mesh.get_data()->shutterTime != 0.0f )
        FF_LOG( warning ) << "When creating a particle stream using create_from_mesh_volume, the shutter time of the "
                             "input mesh should be set to zero. The mesh provided has a non-zero shutter time. "
                             "Krakatoa will act as though it is set to zero."
                          << std::endl;

    // construct pLevelset (and check for cancel message)
    boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set> pLevelset;
    try {
        pLevelset = krakatoa::get_particle_volume_levelset( *mesh.get_data()->mesh, voxelSpacing, cancelChecker );
    } catch( frantic::logging::progress_cancel_exception& ) {
        FF_LOG( debug ) << "particle_stream::create_from_mesh_volume was cancelled." << std::endl;
        *wasCancelled = true;
        particle_stream emptyStream;
        return emptyStream;
    }

    std::vector<frantic::tstring> channelNames;
    pLevelset->get_channel_names( channelNames );

    channel_map channelMap;
    channelMap.define_channel( _T( "Position" ), 3, data_type_float32 );
    for( size_t i = 0; i < channelNames.size(); i++ ) {
        const frantic::tstring name = channelNames[i];
        frantic::volumetrics::levelset::rle_channel_general_accessor acc =
            pLevelset->get_channel_general_accessor( name );
        if( !channelMap.has_channel( name ) )
            channelMap.define_channel( name, acc.arity(), acc.data_type() );
    }
    channelMap.end_channel_definition();

    // create voxel sampler (for istream)
    // NOTE: this is cached in the 3dsmax version, unless the parameters change. it is cached because using
    // "wellDistributedJittering" is a little slow.
    boost::shared_ptr<frantic::volumetrics::voxel_sampler_interface> voxelSampler =
        krakatoa::get_particle_volume_voxel_sampler( subdivCount, particleJitteringEnabled,
                                                     numJitteredParticlesPerVoxel, randomJitterSeed,
                                                     wellDistributedRandomCount, wellDistributedJittering );
    boost::shared_ptr<particle_istream> outputStream;

    // if the user has explicity disabled the shell, then we have to set the shell parameters back to default.
    if( !enableShell ) {
        shellStart = 0.0f;
        shellThickness = 1e10f;
    }

    boost::shared_ptr<particle_istream> voxelStream =
        boost::shared_ptr<particle_istream>( new rle_levelset_particle_istream(
            channelMap, pLevelset, voxelSampler, -( shellStart + shellThickness ), -shellStart, true ) );

    particle_stream newStream;
    newStream.m_data->stream = voxelStream;
    return newStream;
}

namespace detail {

/*
 * Wrapper used for "create_from_surface" function below.
 * It wraps the frantic::geometry::trimesh3 object for use with surface_particle_istream.
 */
class trimesh3_wrapper {
    boost::shared_ptr<frantic::geometry::trimesh3> m_mesh;
    frantic::channels::channel_accessor<frantic::graphics::vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_normalAccessor;

  public:
    trimesh3_wrapper( boost::shared_ptr<frantic::geometry::trimesh3> mesh ) { m_mesh = mesh; }

    ~trimesh3_wrapper() {}

    void get_native_map( frantic::channels::channel_map& outNativeMap ) {
        outNativeMap.define_channel<frantic::graphics::vector3f>( _T("Position") );
        outNativeMap.define_channel<frantic::graphics::vector3f>( _T("Normal") );
    }

    void set_channel_map( const frantic::channels::channel_map& seedMap ) {
        m_normalAccessor.reset();
        m_posAccessor.reset();

        if( seedMap.has_channel( _T( "Position" ) ) )
            m_posAccessor = seedMap.get_accessor<frantic::graphics::vector3f>( _T( "Position" ) );

        if( seedMap.has_channel( _T( "Normal" ) ) )
            m_normalAccessor = seedMap.get_cvt_accessor<frantic::graphics::vector3f>( _T( "Normal" ) );
    }

    std::size_t surface_count() { return m_mesh->face_count(); }

    std::size_t element_count( std::size_t surfaceIndex ) {
        if( 0 <= surfaceIndex && surfaceIndex < m_mesh->face_count() )
            return 1; // Triangles is only split into 1 triangle
        return 0;
    }

    float element_area( std::size_t surfaceIndex, std::size_t /*elementIndex*/ ) {
        float area = -1.0f;
        frantic::geometry::vector3 vertices;
        frantic::geometry::vector3f vertex1;
        frantic::geometry::vector3f vertex2;
        frantic::geometry::vector3f vertex3;

        if( 0 <= surfaceIndex && surfaceIndex < m_mesh->face_count() ) {
            vertices = m_mesh->get_face( surfaceIndex );

            vertex1 = m_mesh->get_vertex( vertices.x );
            vertex2 = m_mesh->get_vertex( vertices.y );
            vertex3 = m_mesh->get_vertex( vertices.z );

            area = 0.5f * frantic::geometry::vector3f::cross( vertex1 - vertex2, vertex3 - vertex2 ).get_magnitude();
        }

        return area;
    }

    template <class RandomGen>
    void seed_particle( char* pOutParticle, std::size_t surfaceIndex, std::size_t /*elementIndex*/,
                        RandomGen& randomnessGenerator ) {
        float baryCentricCoord[3];
        frantic::graphics::vector3f triVerts[3];
        frantic::geometry::vector3 vertices;
        frantic::geometry::vector3f particle;

        frantic::geometry::random_barycentric_coordinate( baryCentricCoord, randomnessGenerator );

        vertices = m_mesh->get_face( surfaceIndex );

        triVerts[0] = m_mesh->get_vertex( vertices.x );
        triVerts[1] = m_mesh->get_vertex( vertices.y );
        triVerts[2] = m_mesh->get_vertex( vertices.z );

        particle = ( baryCentricCoord[0] * triVerts[0] + baryCentricCoord[1] * triVerts[1] +
                     baryCentricCoord[2] * triVerts[2] );

        m_posAccessor.get( pOutParticle ) = particle;

        if( m_normalAccessor.is_valid() )
            m_normalAccessor.set( pOutParticle,
                                  frantic::graphics::triangle_normal( triVerts[0], triVerts[1], triVerts[2] ) );
    }
};

} // namespace detail

particle_stream particle_stream::create_from_surface( const triangle_mesh& mesh, INT64 particleCount,
                                                      float particleSpacing, bool useParticleSpacing, int randomSeed ) {
    using namespace frantic::particles::streams;
    frantic::channels::channel_map channelMap;

    if( mesh.get_data()->shutterTime != 0.0f )
        FF_LOG( warning ) << "When creating a particle stream using create_from_mesh_volume, the shutter time of the "
                             "input mesh should be set to zero. The mesh provided has a non-zero shutter time. "
                             "Krakatoa will act as though it is set to zero."
                          << std::endl;

    channelMap.define_channel( _T( "Position" ), 3, frantic::channels::data_type_float32 );
    channelMap.define_channel( _T( "Normal" ), 3, frantic::channels::data_type_float32 );
    channelMap.end_channel_definition();

    detail::trimesh3_wrapper trimeshWrapper( mesh.get_data()->mesh );
    boost::shared_ptr<surface_particle_istream<detail::trimesh3_wrapper>> meshSeedingStream(
        new surface_particle_istream<detail::trimesh3_wrapper>( trimeshWrapper ) );

    meshSeedingStream->set_channel_map( channelMap );
    meshSeedingStream->set_random_seed( randomSeed );

    if( useParticleSpacing )
        meshSeedingStream->set_particle_spacing( particleSpacing );
    else
        meshSeedingStream->set_particle_count( particleCount );

    particle_stream outStream;
    outStream.get_data()->stream = meshSeedingStream;
    return outStream;
}

particle_stream particle_stream::create_from_fractals( INT64 particleCount, const fractal_parameters& fractalParams ) {

    const fractal_parameters_data& params = *fractalParams.get_data();

    boost::shared_ptr<streams::particle_istream> outputStream;

    int numTMs = (int)params.position.size();
    int numColors = (int)params.colors.size();

    // error check the inputs
    if( params.rotation.size() != numTMs || params.scale.size() != numTMs || params.skewOrientation.size() != numTMs ||
        params.skewAngle.size() != numTMs || params.weight.size() != numTMs )
        throw std::runtime_error(
            "Parameter input error. Ensure each affine transform component array is the same length." );
    if( numTMs == 0 )
        throw std::runtime_error( "No affine transformations were specified for the fractal particles. There must be "
                                  "two or more affine transformations." );
    if( numTMs == 1 )
        throw std::runtime_error( "Only one affine transformation was specified for fractal particles. There must be "
                                  "two or more affine transformations." );
    if( params.colorPositions.size() != numColors )
        throw std::runtime_error(
            "Parameter input error. Ensure color positions and color value arrays are the same length." );

    channel_map emptyCM;
    emptyCM.define_channel<vector3f>( _T( "Position" ) );
    emptyCM.end_channel_definition();

    // we can pass in an empty channel map, since we don't know the final channel map yet, and the renderer will call
    // "set_channel_map" on this stream afterwards.
    boost::shared_ptr<krakatoa::ifs_particle_istream> ifsStream( new krakatoa::ifs_particle_istream(
        emptyCM, particleCount,
        0 ) ); // zero as the random seed. this seed drives the "RandomValue" channel in the particle stream. I don't
               // think i need to expose the random seed, why would anyone need to change it?

    FF_LOG( debug ) << "Generating particles from " << numTMs << " affine transformations:" << std::endl;

    for( int i = 0; i < numTMs; ++i ) {

        FF_LOG( debug ) << "Affine transformation #" << i + 1 << " position: " << to_tstring( params.position[i].str() )
                        << std::endl;
        FF_LOG( debug ) << "Affine transformation #" << i + 1 << " rotation: " << to_tstring( params.rotation[i].str() )
                        << std::endl;
        FF_LOG( debug ) << "Affine transformation #" << i + 1 << " scale: " << to_tstring( params.scale[i].str() )
                        << std::endl;
        FF_LOG( debug ) << "Affine transformation #" << i + 1
                        << " skew orientation: " << to_tstring( params.skewOrientation[i].str() ) << std::endl;
        FF_LOG( debug ) << "Affine transformation #" << i + 1 << " skew angle: " << params.skewAngle[i] << std::endl;

        // NOTE: This is a direct adaptation of MakerController::Evaluate in MaxKrakatoaPRTMaker.cpp
        transform4f prsTM;

        // create a TM from the position, rotation, scale
        quat4f rotationNormalize = params.rotation[i];
        rotationNormalize.normalize();
        transform4f rotationTM;
        rotationNormalize.as_transform4f( rotationTM );
        prsTM = transform4f::from_translation( params.position[i] ) * rotationTM *
                transform4f::from_scale( params.scale[i] );

        // skew portion
        quat4f currSkewOrientation = params.skewOrientation[i];
        currSkewOrientation.normalize();
        float currSkewAngle =
            frantic::math::clamp( frantic::math::degrees_to_radians( params.skewAngle[i] ), 0.f, (float)M_PI );

        transform4f outTM;

        // outTM.SetRow( 2, Point3( sinf( skewAngle ), 0.f, cosf( skewAngle ) ) ); //from MAX
        outTM[8] = sinf( currSkewAngle );
        outTM[10] = cosf( currSkewAngle );

        // RotateMatrix( outTM, skewOrienation ); //from MAX
        transform4f rotateTM;
        currSkewOrientation.as_transform4f( rotateTM );
        outTM = outTM * rotateTM;

        // PreRotateMatrix( outTM, skewOrienation.Invert() ); //from MAX
        transform4f rotateTMInv;
        quat4f currSkewOrientationInv = currSkewOrientation;
        currSkewOrientationInv.invert();
        currSkewOrientationInv.as_transform4f( rotateTMInv );
        outTM = rotateTMInv * outTM;

        // add this transform to our affine particle stream
        outTM = prsTM * outTM;
        ifsStream->add_affine_transform( outTM, params.weight[i] );
    }

    outputStream = ifsStream;

    // handling setting color channel
    if( numColors > 1 ) {

        static boost::array<frantic::tstring, 1> modifyColorAffectedChannels = { _T( "ColorScalar" ) };
        struct modify_color {
            static vector3f fn( float colorScalar, const std::map<float, vector3f>& colorGradientMap ) {
                vector3f val;
                std::map<float, vector3f>::const_iterator iter = colorGradientMap.upper_bound( colorScalar );
                if( iter == colorGradientMap.begin() ) {
                    val = iter->second;
                } else if( iter == colorGradientMap.end() ) {
                    val = ( --iter )->second;
                } else {
                    std::map<float, vector3f>::const_iterator i1 = iter;
                    std::map<float, vector3f>::const_iterator i0 = --iter;
                    float alpha = ( colorScalar - i0->first ) / ( i1->first - i0->first );
                    val = i0->second * ( 1.0f - alpha ) + i1->second * alpha;
                }
                return val;
            }
        };

        // construct a map of color gradients
        std::map<float, vector3f> colorGradientMap;
        for( int i = 0; i < numColors; ++i ) {
            float pos = params.colorPositions[i];
            colorGradientMap[pos] = params.colors[i];
        }

        // this only works with 2+ colors.
        boost::shared_ptr<streams::particle_istream> modifyColorStream(
            new streams::apply_function_particle_istream<vector3f( float )>(
                outputStream, boost::bind( &modify_color::fn, _1, colorGradientMap ), _T( "Color" ),
                modifyColorAffectedChannels ) );
        outputStream = modifyColorStream;

    } else if( numColors == 1 ) {
        // set to the only color in the list
        boost::shared_ptr<streams::particle_istream> modifyColorStream(
            new streams::set_channel_particle_istream<vector3f>( outputStream, _T( "Color" ),
                                                                 fractalParams.get_data()->colors[0] ) );
        outputStream = modifyColorStream;
    } else {
        // set to white
        boost::shared_ptr<streams::particle_istream> modifyColorStream(
            new streams::set_channel_particle_istream<vector3f>( outputStream, _T( "Color" ), vector3f( 1.0f ) ) );
        outputStream = modifyColorStream;
    }

    // copy color to emission
    static boost::array<frantic::tstring, 1> modifyEmissionAffectedChannels = { _T( "Color" ) };
    struct modify_emission {
        static vector3f fn( vector3f color ) { return color; }
    };
    boost::shared_ptr<streams::particle_istream> copyColorToEmissionStream(
        new streams::apply_function_particle_istream<vector3f( vector3f )>(
            outputStream, boost::bind( &modify_emission::fn, _1 ), _T( "Emission" ), modifyEmissionAffectedChannels ) );
    outputStream = copyColorToEmissionStream;

    // create the returning particle_stream object and set its internal stream to be the one we just set up.
    particle_stream newStream;
    newStream.m_data->stream = outputStream;
    return newStream;
}

particle_stream particle_stream::create_from_particle_stream_interface( particle_stream_interface* userStream ) {
    if( !userStream->get_data()->channelMap.has_channel( _T( "Position" ) ) )
        throw std::runtime_error(
            "The particle_stream_interface object provided does not have a \"Position\" channel. A \"Position\" "
            "channel is required. Please add it with particle_stream_interface::append_channel()." );
    particle_stream newStream;
    newStream.m_data->stream.reset( new detail::particle_stream_interface_wrapper( userStream ) );
    return newStream;
}

//
//
// Modification functions for existing particle streams
//
//

void channelop_scale( particle_stream& stream, const char* channelName, float scaleValue ) {
    if( stream.get_data()->stream->get_native_channel_map().has_channel( to_tstring( channelName ) ) ) {
        boost::shared_ptr<streams::particle_istream> scaledChannelStream( new streams::channel_scale_particle_istream(
            stream.get_data()->stream, to_tstring( channelName ), scaleValue ) );
        stream.get_data()->stream = scaledChannelStream;
    }
}

void channelop_copy( particle_stream& stream, const char* destChannelName, const char* sourceChannelName ) {
    if( stream.get_data()->stream->get_native_channel_map().has_channel( to_tstring( sourceChannelName ) ) ) {
        boost::shared_ptr<streams::particle_istream> duplicateChannelStream(
            new streams::duplicate_channel_particle_istream( stream.get_data()->stream, to_tstring( sourceChannelName ),
                                                             to_tstring( destChannelName ), true ) );
        stream.get_data()->stream = duplicateChannelStream;
    }
}

void channelop_set( particle_stream& stream, const char* channelName, data_type_t dataTypeIn, int arity,
                    const void* data ) {
    // this function is a little hacky. it doesn't not handle all possible combinations of channels. however, krakatoa
    // itself uses a finite number of combinations. so it shouldn't matter for right meow.

    // this assumes that the enums are the same. which they should be. if they're not, we're screwed in more than just
    // this place.
    frantic::channels::data_type_t dataType = (frantic::channels::data_type_t)dataTypeIn;

    boost::shared_ptr<streams::particle_istream> origStream = stream.get_data()->stream;
    boost::shared_ptr<streams::particle_istream> newStream = origStream;

    if( frantic::channels::is_channel_data_type_int( dataType ) ) {
        // handle integers. does this work for to/from unsigned/signed conversions?
        channel_type_convertor_function_t cvt = get_channel_type_convertor_function(
            dataType, frantic::channels::data_type_int64, to_tstring( channelName ) );
        if( arity == 1 ) {
            boost::int64_t newValue;
            cvt( (char*)&newValue, (char*)data, 1 ); // promote to largest integer to not lose precision on set.
            newStream.reset( new streams::set_channel_particle_istream<boost::int64_t>(
                origStream, to_tstring( channelName ), newValue ) );
        }
    } else if( frantic::channels::is_channel_data_type_float( dataType ) ) {
        // handle floats
        channel_type_convertor_function_t cvt = get_channel_type_convertor_function(
            dataType, frantic::channels::data_type_float32, to_tstring( channelName ) );
        if( arity == 1 ) {
            float newValue;
            cvt( (char*)&newValue, (char*)data, 1 ); // set to 32 bit float
            newStream.reset(
                new streams::set_channel_particle_istream<float>( origStream, to_tstring( channelName ), newValue ) );
        } else if( arity == 3 ) {
            float newValue[3];
            cvt( (char*)newValue, (char*)data, 3 ); // set to 32 bit float
            newStream.reset( new streams::set_channel_particle_istream<vector3f>(
                origStream, to_tstring( channelName ), vector3f( newValue[0], newValue[1], newValue[2] ) ) );
        }
    }

    stream.get_data()->stream = newStream;
}

void create_vector_magnitude_channel( particle_stream& stream, const char* destChannelName, int destChannelArity,
                                      const char* sourceChannelName ) {
    boost::shared_ptr<streams::particle_istream> inStream = stream.get_data()->stream;

    // do some error checking on the type and arity of the input/output channels
    // most of this function is error checking.
    bool sanityCheckFailed = false;
    const channel_map& cm = inStream->get_native_channel_map();
    if( destChannelArity != 1 && destChannelArity != 3 ) {
        FF_LOG( error )
            << "create_vector_magnitude_channel: The destination channel arity must be set to either 1 or 3.\n";
        sanityCheckFailed = true;
    }
    if( cm.has_channel( to_tstring( destChannelName ) ) ) {
        frantic::channels::data_type_t existingDataType;
        size_t existingArity;
        cm.get_channel_definition( to_tstring( destChannelName ), existingDataType, existingArity );
        if( !frantic::channels::is_channel_data_type_float( existingDataType ) ) {
            FF_LOG( error ) << "create_vector_magnitude_channel: The channel \"" << destChannelName
                            << "\" already exists in the stream, however, it is not a floating point type. Channel "
                               "magnitude requires the destination be a floating point channel.\n";
            sanityCheckFailed = true;
        }
        if( existingArity != destChannelArity ) {
            FF_LOG( error ) << "create_vector_magnitude_channel: The channel \"" << destChannelName
                            << "\" already exists in the stream, however, the output arity requested is "
                            << destChannelArity << " and the arity of the existing channel is " << existingArity
                            << ". These arity values must match.\n";
            sanityCheckFailed = true;
        }
    }
    if( !cm.has_channel( to_tstring( sourceChannelName ) ) ) {
        FF_LOG( error ) << "create_vector_magnitude_channel: The source channel \"" << sourceChannelName
                        << "\" does not exist in the stream.\n";
        sanityCheckFailed = true;
    }
    frantic::channels::data_type_t sourceDataType;
    size_t sourceArity;
    cm.get_channel_definition( to_tstring( sourceChannelName ), sourceDataType, sourceArity );
    if( !frantic::channels::is_channel_data_type_float( sourceDataType ) ) {
        FF_LOG( error )
            << "create_vector_magnitude_channel: The channel \"" << sourceChannelName
            << "\" is not a floating point channel. Magnitude can only be computed from a float[3] channel.\n";
        sanityCheckFailed = true;
    }
    if( sourceArity != 3 ) {
        FF_LOG( error ) << "create_vector_magnitude_channel: The channel \"" << sourceChannelName
                        << "\" does not have arity 3. Magnitude can only be computed from a float[3] channel.\n";
        sanityCheckFailed = true;
    }

    // as a first step, add the channel and set it to zero if it doesn't exist.
    // in the case where sanity check has failed, also add it and set it to zero. it seems like the right thing to do?
    if( !cm.has_channel( to_tstring( destChannelName ) ) || sanityCheckFailed ) {
        if( destChannelArity == 1 )
            inStream.reset(
                new streams::set_channel_particle_istream<float>( inStream, to_tstring( destChannelName ), 0.0f ) );
        else if( destChannelArity == 3 )
            inStream.reset( new streams::set_channel_particle_istream<vector3f>(
                inStream, to_tstring( destChannelName ), vector3f( 0.0f ) ) );
    }

    if( !sanityCheckFailed ) {

        if( destChannelArity == 1 ) {
            // inline static struct/function for the function istream below
            boost::array<frantic::tstring, 1> affectedChannels = { to_tstring( sourceChannelName ) };
            struct compute_magnitude {
                static float fn( const vector3f& sourceValue ) { return sourceValue.get_magnitude(); }
            };
            inStream.reset( new streams::apply_function_particle_istream<float( vector3f )>(
                inStream, boost::bind( &compute_magnitude::fn, _1 ), to_tstring( destChannelName ),
                affectedChannels ) );
        } else {
            // inline static struct/function for the function istream below
            boost::array<frantic::tstring, 1> affectedChannels = { to_tstring( sourceChannelName ) };
            struct compute_magnitude {
                static vector3f fn( const vector3f& sourceValue ) {
                    float m = sourceValue.get_magnitude();
                    return vector3f( m, m, m );
                }
            };
            inStream.reset( new streams::apply_function_particle_istream<vector3f( vector3f )>(
                inStream, boost::bind( &compute_magnitude::fn, _1 ), to_tstring( destChannelName ),
                affectedChannels ) );
        }
    }

    stream.get_data()->stream = inStream;
}

void add_particle_repopulation( particle_stream& stream, float fillRadius, int fillRadiusSubdivs,
                                int numParticlesPerSubdiv, float densityFalloffStart, unsigned randomSeed,
                                krakatoasr::progress_logger_interface* progressLogger,
                                krakatoasr::cancel_render_interface* cancelCheck ) {

    // Create the frantic logger object (wraps the SR-exposed logger/canceller objects).
    // I like the idea of defaulting to the console logger, but this may be contentious as it doesn't allow the user to
    // easily disable logging.
    if( !progressLogger )
        progressLogger = krakatoasr::get_console_progress_logger();

    krakatoasr_progress_logger logger( progressLogger, NULL, cancelCheck );

    // create a new stream. this will exhaust the existing stream in "originalParticleStream".
    boost::shared_ptr<frantic::particles::streams::particle_istream> outputStream =
        krakatoa::create_particle_repopulation_istream( stream.get_data()->stream, fillRadius, fillRadiusSubdivs,
                                                        numParticlesPerSubdiv, densityFalloffStart, randomSeed,
                                                        logger );
    // replace the stream in "originalParticleStream" to be the newly created stream.
    stream.get_data()->stream = outputStream;
}

void add_time_offset( particle_stream& stream, float timeOffset ) {

    boost::shared_ptr<streams::particle_istream> outputStream = stream.get_data()->stream;
    if( timeOffset != 0.0f ) {
        if( !outputStream->get_channel_map().has_channel( _T( "Velocity" ) ) ) {
            FF_LOG( warning ) << "Time offset attempted on a stream without a \"Velocity\" channel. The time offset "
                                 "will have no effect.\n";
        } else {

            // inline static struct/function for the function istream below
            static boost::array<frantic::tstring, 2> affectedChannels = { _T( "Position" ), _T( "Velocity" ) };
            struct modify_position {
                static vector3f fn( const vector3f& position, const vector3f& velocity, float offsetInSeconds ) {
                    return position + velocity * offsetInSeconds;
                }
            };
            outputStream.reset( new streams::apply_function_particle_istream<vector3f( vector3f, vector3f )>(
                outputStream, boost::bind( &modify_position::fn, _1, _2, timeOffset ), _T( "Position" ),
                affectedChannels ) );
        }
    }
    stream.get_data()->stream = outputStream;
}

void add_fractional_particle_stream( particle_stream& stream, float fraction ) {
    boost::shared_ptr<streams::particle_istream> inputStream = stream.get_data()->stream;
    boost::shared_ptr<streams::particle_istream> outputStream =
        streams::apply_fractional_particle_istream( inputStream, fraction );
    stream.get_data()->stream = outputStream;
}

//
//
// Class for creating fractal-style particles
//
//

fractal_parameters::fractal_parameters() { m_data = new fractal_parameters_data; }
fractal_parameters::~fractal_parameters() { delete m_data; }
fractal_parameters::fractal_parameters( const fractal_parameters& t ) {
    m_data = new fractal_parameters_data;
    *this = t;
}
fractal_parameters& fractal_parameters::operator=( const fractal_parameters& t ) {
    *m_data = *t.m_data;
    return *this;
}
const fractal_parameters_data* fractal_parameters::get_data() const { return m_data; }
fractal_parameters_data* fractal_parameters::get_data() { return m_data; }

void fractal_parameters::append_affine_transform( float positionX, float positionY, float positionZ, float rotationX,
                                                  float rotationY, float rotationZ, float rotationW, float scaleX,
                                                  float scaleY, float scaleZ, float skewOrientationX,
                                                  float skewOrientationY, float skewOrientationZ,
                                                  float skewOrientationW, float skewAngle, float weight ) {
    m_data->position.push_back( vector3f( positionX, positionY, positionZ ) );
    m_data->rotation.push_back( quat4f( rotationW, rotationX, rotationY, rotationZ ) );
    m_data->scale.push_back( vector3f( scaleX, scaleY, scaleZ ) );
    m_data->skewOrientation.push_back(
        quat4f( skewOrientationW, skewOrientationX, skewOrientationY, skewOrientationZ ) );
    m_data->skewAngle.push_back( skewAngle );
    m_data->weight.push_back( frantic::math::clamp( weight, 0.0f, 1.0f ) );
}

void fractal_parameters::append_color_gradient( float r, float g, float b, float position ) {
    m_data->colors.push_back( vector3f( r, g, b ) );
    m_data->colorPositions.push_back( position );
}

void fractal_parameters::set_from_random( int affineTMCount, int colorGradientCount, int randomSeed ) {
    set_default_random_fractal_parameters( affineTMCount, colorGradientCount, randomSeed, *m_data );
}

} // namespace krakatoasr
