// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include "krakatoa/prt_stream_builder.hpp"

#include <cmath>
#include <utility>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <frantic/math/utils.hpp>

#include <frantic/files/filename_sequence.hpp>
#include <frantic/files/paths.hpp>
#include <frantic/graphics/color3f.hpp>
#include <frantic/graphics/vector3f.hpp>
#include <frantic/logging/logging_level.hpp>
#include <frantic/particles/particle_file_stream_factory.hpp>
#include <frantic/particles/streams/apply_function_particle_istream.hpp>
#include <frantic/particles/streams/concatenated_parallel_particle_istream.hpp>
#include <frantic/particles/streams/concatenated_particle_istream.hpp>
#include <frantic/particles/streams/empty_particle_istream.hpp>
#include <frantic/particles/streams/fractional_by_id_particle_istream.hpp>
#include <frantic/particles/streams/fractional_particle_istream.hpp>
#include <frantic/particles/streams/set_channel_particle_istream.hpp>
#include <frantic/particles/streams/time_interpolation_particle_istream.hpp>

using namespace frantic;
using namespace frantic::graphics;
using namespace frantic::channels;
using namespace frantic::particles;
using namespace frantic::particles::streams;

namespace {

/**
 * Extracts the two closest frames to 'time', given that time is expressed
 * in 'ticksPerFrame'
 *
 * @param  time input time
 * @param  ticksPerFrame current frame = (time / ticksPerFrame)
 * @param  epsilon the largest distance at which 'time' will be considered
 *         to be exactly at a single frame, rather than between two
 * @return a pair, containing the (closest frame, second closest frame),
 * if the distance to the closest frame is less than epsilon, it just
 * returns the closest frame twice
 */
std::pair<int, int> get_closest_frames( double time, double ticksPerFrame, double epsilon = 0.000001 ) {
    int frame = static_cast<int>( floor( double( ( time / ticksPerFrame ) + 0.5 ) ) );
    double frameTime = static_cast<double>( frame ) * ticksPerFrame;
    int otherFrame;

    if( fabs( frameTime - time ) < epsilon )
        otherFrame = frame;
    else if( frameTime < time )
        otherFrame = frame + 1;
    else
        otherFrame = frame - 1;

    return std::make_pair( frame, otherFrame );
}

/**
 * gets the interpolation distance of 'currentTime' between the two specified frames, where
 * 'currentTime' is measured in 'ticksPerFrame'
 *
 * @param  frame0 a frame before or after 'currentTime'
 * @param  frame1 a frame before or after 'currentTime'
 * @param  currentTime the time to evaluate the interpolation at
 * @param  ticksPerFrame the scale of 'currentTime' in frames
 * @return 0 if the two frames are the same, otherwise, the interpolation value between the two frames
 */
double get_time_interpolate( int frame0, int frame1, double currentTime, double ticksPerFrame ) {
    if( frame0 != frame1 ) {
        const int left = frame0;
        const int right = frame1;
        double tLeft = static_cast<double>( left ) * ticksPerFrame;
        double tRight = static_cast<double>( right ) * ticksPerFrame;
        return ( currentTime - tLeft ) / ( tRight - tLeft );
    } else {
        return 0.0;
    }
}

// operations used by the prt stream builder
struct ops {
    static vector3f add_velocity_to_pos( const vector3f& pos, const vector3f& velocity, float scale ) {
        return pos + ( scale * velocity );
    }

    static vector3f scale_vector( const vector3f& v, float scale ) { return scale * v; }

    static vector3f uvw_from_density( float density ) { return vector3f( density ); }

    static float square_density( float density ) { return density * density; }

    static color3f clamp_color( const color3f& color ) {
        return color3f( frantic::math::clamp( color.r, 0.f, 1.f ), frantic::math::clamp( color.g, 0.f, 1.f ),
                        frantic::math::clamp( color.b, 0.f, 1.f ) );
    }
};

} // namespace

namespace krakatoa {

prt_stream_builder::prt_stream_builder() { reset_to_defaults(); }

prt_stream_builder::~prt_stream_builder() {}

/**
 * Sets the builder to a known, default state, that will generally just pass
 * files through unchanged.
 */
void prt_stream_builder::reset_to_defaults() {
    m_frameOffset = 0;
    m_currentTime = 0.0;
    m_framesPerSecond = 24.0;
    m_timeDerivative = 1.0f;

    m_coordinateSystem = frantic::graphics::coordinate_system::unspecified;
    m_scaleToMeters = 1.0;

    m_loadFraction = 1.0;
    m_loadLimit = std::numeric_limits<boost::int64_t>::max();
    m_loadMode = render::PARTICLELOAD_EVENLY_DISTRIBUTE;
    m_rangeStart = std::numeric_limits<int>::min();
    m_rangeEnd = std::numeric_limits<int>::max();
    m_rangeStartClampMode = clamp_mode::hold;
    m_rangeEndClampMode = clamp_mode::hold;
    m_interpolateSubFrames = false;
    m_keepVelocityChannel = false;
    m_fileSequences.clear();
    m_singleFiles.clear();
    m_threadingSupport = false;
    m_throwOnError = false;
    m_logErrors = true;
    m_cacheErrors = true;
    clear_errors();
}

/**
 * Performs file pattern replacement on the input file sequences
 *
 * @param currentFrame the frame to be rendered
 * @param outFilenames will be filled with the replaced file names from the sequences
 */
void prt_stream_builder::get_files_at_frame( int currentFrame, std::vector<frantic::tstring>& outFilenames ) {
    outFilenames.clear();

    for( size_t i = 0; i < m_fileSequences.size(); ++i ) {
        files::filename_pattern filePattern( m_fileSequences[i] );
        outFilenames.push_back( filePattern[(int)currentFrame] );
    }
}

size_t prt_stream_builder::num_errors() { return m_errorMessages.size(); }

void prt_stream_builder::get_error( size_t index, frantic::tstring& outError ) {
    if( index < m_errorMessages.size() )
        outError = m_errorMessages[index];
    else
        outError = _T("Invalid error index specified.");
}

void prt_stream_builder::clear_errors() { m_errorMessages.clear(); }

void prt_stream_builder::error_message( const frantic::tstring& error ) {
    if( m_cacheErrors ) {
        m_errorMessages.push_back( error );
    }

    if( m_logErrors ) {
        FF_LOG( warning ) << error << std::endl;
    }
}

void prt_stream_builder::handle_error( const std::exception& error ) {
    error_message( frantic::strings::to_tstring( error.what() ) );

    if( m_throwOnError ) {
        throw;
    }
}

/**
 * Primary stream construction method.  Uses all of this object's members to generate a single
 * concatenated particle stream, applying interpolation, reduction and clamping as specified.
 */
boost::shared_ptr<frantic::particles::streams::particle_istream> prt_stream_builder::generate_stream() {
    clear_errors();
    std::pair<int, int> interpolationFrames = get_closest_frames( m_currentTime, 1.0 );
    interpolationFrames.first += m_frameOffset;
    interpolationFrames.second += m_frameOffset;

    frantic::channels::channel_map emptyChannelMap;
    emptyChannelMap.define_channel( _T("Position"), 3, data_type_float32 );
    emptyChannelMap.end_channel_definition();

    if( interpolationFrames.first < m_rangeStart ) {
        if( m_rangeStartClampMode == clamp_mode::blank )
            return boost::shared_ptr<particle_istream>( new empty_particle_istream( emptyChannelMap ) );
        else if( m_rangeStartClampMode == clamp_mode::hold )
            interpolationFrames.first = interpolationFrames.second = m_rangeStart;
        else
            FF_LOG( warning ) << "Unknown clamp mode : " << m_rangeStartClampMode << " at " << __FILE__ << ":"
                              << __FUNCTION__ << "(" << __LINE__ << ")" << std::endl;
    } else if( interpolationFrames.first > m_rangeEnd ) {
        if( m_rangeEndClampMode == clamp_mode::blank )
            return boost::shared_ptr<particle_istream>( new empty_particle_istream( emptyChannelMap ) );
        else if( m_rangeEndClampMode == clamp_mode::hold )
            interpolationFrames.first = interpolationFrames.second = m_rangeEnd;
        else
            FF_LOG( warning ) << "Unknown clamp mode : " << m_rangeStartClampMode << " at " << __FILE__ << ":"
                              << __FUNCTION__ << "(" << __LINE__ << ")" << std::endl;
    }

    if( !m_interpolateSubFrames )
        interpolationFrames.second = interpolationFrames.first;

    std::vector<frantic::tstring> primaryFiles, secondaryFiles;
    get_files_at_frame( interpolationFrames.first, primaryFiles );

    if( interpolationFrames.second != interpolationFrames.first )
        get_files_at_frame( interpolationFrames.second, secondaryFiles );

    // set up factory object with the settings from the maya scene.
    frantic::particles::particle_file_stream_factory_object factory;
    factory.set_coordinate_system( m_coordinateSystem );
    factory.set_length_unit_in_meters( m_scaleToMeters );
    factory.set_frame_rate( (unsigned)m_framesPerSecond, 1 );

    std::vector<boost::shared_ptr<particle_istream>> particleStreams;

    for( size_t i = 0; i < primaryFiles.size(); ++i ) {
        try {

            boost::shared_ptr<particle_istream> stream = factory.create_istream( primaryFiles[i] );
            particleStreams.push_back( stream );

            if( interpolationFrames.second != interpolationFrames.first ) {
                const double interp =
                    get_time_interpolate( interpolationFrames.first, interpolationFrames.second, m_currentTime, 1.0 );
                const double timeStepSeconds =
                    ( interpolationFrames.first < interpolationFrames.second ? 1 : -1 ) * ( 1.0 / m_framesPerSecond );
                boost::shared_ptr<particle_istream> secondStream = factory.create_istream( secondaryFiles[i] );

                particleStreams.back().reset( new time_interpolation_particle_istream(
                    particleStreams.back(), secondStream, float( timeStepSeconds ), float( interp ) ) );
            }

            bool hasVelocity = particleStreams.back()->get_channel_map().has_channel( _T( "Velocity" ) );

            if( hasVelocity && m_rangeStart == m_rangeEnd )
                particleStreams.back().reset( new set_channel_particle_istream<vector3f>(
                    particleStreams.back(), _T( "Velocity" ), vector3f( 0.f ) ) );
        } catch( std::exception& error ) {
            handle_error( error );
        }
    }

    for( size_t i = 0; i < m_singleFiles.size(); ++i ) {
        try {

            boost::shared_ptr<particle_istream> stream = factory.create_istream( m_singleFiles[i] );
            particleStreams.push_back( stream );

            bool hasVelocity = particleStreams.back()->get_channel_map().has_channel( _T( "Velocity" ) );

            if( hasVelocity && ( !m_keepVelocityChannel || m_rangeStart == m_rangeEnd ) )
                particleStreams.back().reset( new set_channel_particle_istream<vector3f>(
                    particleStreams.back(), _T( "Velocity" ), vector3f( 0.f ) ) );
        } catch( std::exception& error ) {
            handle_error( error );
            // TODO: implement other error response options
            // FF_LOG(warning) << _T("Krakatoa is ignoring an invalid or missing file: \"") << primaryFiles[i] <<
            // _T("\"") << std::endl;
        }
    }

    boost::shared_ptr<particle_istream> particleInput;

    // build the file stream
    if( particleStreams.size() == 0 ) {
        particleInput = boost::shared_ptr<particle_istream>( new empty_particle_istream( emptyChannelMap ) );
    } else if( particleStreams.size() == 1 ) {
        particleInput = boost::shared_ptr<particle_istream>( particleStreams[0] );
    } else {
        if( m_threadingSupport )
            particleInput =
                boost::shared_ptr<particle_istream>( new concatenated_parallel_particle_istream( particleStreams ) );
        else
            particleInput = boost::shared_ptr<particle_istream>( new concatenated_particle_istream( particleStreams ) );
    }

    // apply load reduction
    if( m_loadMode == render::PARTICLELOAD_FIRST_N ) {
        particleInput = apply_fractional_particle_istream( particleInput, m_loadFraction, m_loadLimit, false );
    } else if( m_loadMode == render::PARTICLELOAD_EVENLY_DISTRIBUTE ) {
        particleInput = apply_fractional_particle_istream( particleInput, m_loadFraction, m_loadLimit, true );
    } else if( m_loadMode == render::PARTICLELOAD_EVENLY_DISTRIBUTE_IDS ) {
        if( particleInput->get_native_channel_map().has_channel( _T( "ID" ) ) )
            particleInput =
                apply_fractional_by_id_particle_istream( particleInput, m_loadFraction, _T( "ID" ), m_loadLimit );
        else
            particleInput = apply_fractional_particle_istream( particleInput, m_loadFraction, m_loadLimit, true );
    } else {
        FF_LOG( warning ) << "Unrecognized loading mode : " << m_loadMode << " at " << __FILE__ << ":" << __FUNCTION__
                          << "(" << __LINE__ << ")" << std::endl;
    }

    // Clamp current time
    const double rangeStart = m_rangeStart;
    const double rangeEnd = m_rangeEnd;
    m_currentTime = frantic::math::clamp( m_currentTime, rangeStart, rangeEnd );

    double roundedFrame = static_cast<double>( interpolationFrames.first );
    double timeOffset = ( ( m_currentTime + m_frameOffset ) - roundedFrame ) / m_framesPerSecond;
    double timeDerivative = m_timeDerivative;
    bool hasPosition = particleInput->get_channel_map().has_channel( _T( "Position" ) );
    bool hasVelocity = particleInput->get_channel_map().has_channel( _T( "Velocity" ) );
    static boost::array<frantic::tstring, 2> subframePushChannels = { _T( "Position" ), _T( "Velocity" ) };
    static boost::array<frantic::tstring, 1> velocityScaleChannels = { _T( "Velocity" ) };
    bool inRange = true;

    if( interpolationFrames.first < m_rangeStart || interpolationFrames.first > m_rangeEnd ) {
        inRange = false;
        timeOffset = 0.0;
        timeDerivative = 0.0;
    }

    if( inRange && hasPosition && fabs( timeOffset ) > 0.001f && !m_interpolateSubFrames )
        particleInput.reset( new apply_function_particle_istream<vector3f( const vector3f&, const vector3f& )>(
            particleInput, boost::bind( &ops::add_velocity_to_pos, _1, _2, (float)timeOffset ), _T( "Position" ),
            subframePushChannels ) );

    if( inRange && hasVelocity && fabs( timeDerivative - 1.f ) > 0.001f )
        particleInput.reset( new apply_function_particle_istream<vector3f( const vector3f& )>(
            particleInput, boost::bind( &ops::scale_vector, _1, (float)timeDerivative ), _T( "Velocity" ),
            velocityScaleChannels ) );

    return particleInput;
}

/**
 * Returns the given viewport render load particle mode as a render particle load mode
 *
 * @param  loadMode the viewport load mode to convert
 * @return the corresponding render load mode, or 0 if no corresponding value exists
 */
render::RENDER_LOAD_PARTICLE_MODE get_as_render_mode( viewport::RENDER_LOAD_PARTICLE_MODE loadMode ) {
    switch( loadMode ) {
    case viewport::PARTICLELOAD_EVENLY_DISTRIBUTE:
        return render::PARTICLELOAD_EVENLY_DISTRIBUTE;

    case viewport::PARTICLELOAD_FIRST_N:
        return render::PARTICLELOAD_FIRST_N;

    case viewport::PARTICLELOAD_EVENLY_DISTRIBUTE_IDS:
        return render::PARTICLELOAD_EVENLY_DISTRIBUTE_IDS;

    default:
        FF_LOG( warning ) << "Attempting to convert invalid viewport loading mode : " << loadMode;
        return (render::RENDER_LOAD_PARTICLE_MODE)0;
    }
}

/**
 * Calculates the reduced particle count that 'get_reduced_particle_stream' will give
 * 'currentTime' is measured in 'ticksPerFrame'
 *
 * @param  rawNumParticles size of the stream
 * @param  fraction the fraction to take from the stream
 * @param  limit the upper limit to take from the stream
 */
boost::int64_t get_fractional_particle_count( boost::int64_t rawNumParticles, double fraction, boost::int64_t limit ) {
    return math::clamp( static_cast<boost::int64_t>( rawNumParticles * fraction ), static_cast<boost::int64_t>( 0 ),
                        limit );
}

} // namespace krakatoa
