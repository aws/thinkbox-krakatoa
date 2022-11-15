// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include "krakatoa/sticky_channel_particle_istream.hpp"

stickychannel_particle_istream::stickychannel_particle_istream(
    boost::shared_ptr<frantic::particles::streams::particle_istream> pin,
    std::map<boost::int64_t, boost::shared_array<char>>& birthValues, const frantic::tstring& sourceChannel,
    const frantic::tstring& destChannel, const frantic::tstring& idChannel, size_t bufferSize )
    : m_delegate( pin )
    , m_birthValues( birthValues )
    , m_sourceChannelName( sourceChannel )
    , m_destinationChannelName( destChannel )
    , m_idChannelName( idChannel )
    , m_bufferedParticlesIndex( std::numeric_limits<size_t>::max() ) {
    FF_LOG( debug ) << "Adding birth channel evaluation to stream named \"" << m_delegate->name() << "\".\n";

    m_currentBufferSize = 0;
    m_particleIndex = 0;
    m_bufferedParticlesIndex = 0;

    // determine buffer size (normally it's user-inputted)
    m_maxBufferSize = std::min( (boost::int64_t)bufferSize, m_delegate->particle_count() );
    if( m_maxBufferSize == -1 )
        m_maxBufferSize = bufferSize;

    init_channel_map( m_delegate->get_channel_map() );
}

void stickychannel_particle_istream::init_channel_map( const frantic::channels::channel_map& inputChannelMap ) {
    // if a default particle was previously set, copy over the old default structure, otherwise, create a new default
    // particle
    if( m_defaultParticle ) {
        boost::shared_array<char> newDefaultParticle( new char[inputChannelMap.structure_size()] );
        frantic::channels::channel_map_adaptor oldToNewChannelMapAdaptor( inputChannelMap, m_channelMap );
        oldToNewChannelMapAdaptor.copy_structure( newDefaultParticle.get(), m_defaultParticle.get() );
        m_defaultParticle.swap( newDefaultParticle );
    } else {
        m_defaultParticle.reset( new char[inputChannelMap.structure_size()] );
        memset( m_defaultParticle.get(), 0, inputChannelMap.structure_size() );
    }

    // set our outgoing map to this requested map
    m_channelMap = inputChannelMap;

    // set our delegate to it's own native map
    m_delegate->set_channel_map( m_delegate->get_native_channel_map() );
    m_delegateChannelMap = m_delegate->get_channel_map();

    // Make sure required channels are there
    if( !m_delegateChannelMap.has_channel( m_idChannelName ) ) {
        throw std::runtime_error( "stickychannel_particle_istream::init_channel_map ID channel \"" +
                                  frantic::strings::to_string( m_idChannelName ) + "\" not found." );
    }
    if( !m_delegateChannelMap.has_channel( m_sourceChannelName ) ) {
        throw std::runtime_error( "stickychannel_particle_istream::init_channel_map source channel \"" +
                                  frantic::strings::to_string( m_sourceChannelName ) + "\" not found." );
    }
    size_t sourceArity;
    frantic::channels::data_type_t sourceType;
    m_delegateChannelMap.get_channel_definition( m_sourceChannelName, sourceType, sourceArity );

    // create native channel map for our stream. this is the delegate's native map, plus our new channels.
    m_nativeChannelMap = m_delegate->get_channel_map();
    if( !m_nativeChannelMap.has_channel( m_destinationChannelName ) ) {
        m_nativeChannelMap.append_channel( m_destinationChannelName, sourceArity, sourceType );
    } else {
        size_t destArity;
        frantic::channels::data_type_t destType;
        m_delegateChannelMap.get_channel_definition( m_destinationChannelName, destType, destArity );

        if( destArity != sourceArity || destType != sourceType ) {
            throw std::runtime_error( "stickychannel_particle_istream::init_channel_map channels \"" +
                                      frantic::strings::to_string( m_sourceChannelName ) + "\" and \"" +
                                      frantic::strings::to_string( m_destinationChannelName ) +
                                      "\" must have the same type and arity." );
        }
    }

    // make adaptor
    // delegate provides particles in their native form, so this adaptor switches them into our requested form
    m_cma.set( m_nativeChannelMap, m_delegateChannelMap );

    // create the buffer with the correct channel map, and size it to the right number of particles.
    m_bufferedParticles = frantic::particles::particle_array( m_nativeChannelMap );
    m_bufferedParticles.resize( m_maxBufferSize );
}

void stickychannel_particle_istream::set_channel_map( const frantic::channels::channel_map& particleChannelMap ) {
    if( m_particleIndex > 0 ) // this is an unfortunate consequence of pre-buffering particles.
        throw std::runtime_error(
            "stickychannel_particle_istream::set_channel_map can only be called prior to calling get_particle()." );
    init_channel_map( particleChannelMap );
}

void stickychannel_particle_istream::set_default_particle( char* rawParticleBuffer ) {
    memcpy( m_defaultParticle.get(), rawParticleBuffer, m_channelMap.structure_size() );
}

const frantic::channels::channel_map& stickychannel_particle_istream::get_channel_map() const { return m_channelMap; }

const frantic::channels::channel_map& stickychannel_particle_istream::get_native_channel_map() const {
    return m_nativeChannelMap;
}

bool stickychannel_particle_istream::get_particle( char* outParticleBuffer ) {

    // fill the buffer if need be
    // the first time though, these will be zero, and the buffer will be filled.
    // the next times though, it will only re-fill when it gets to the end
    if( m_bufferedParticlesIndex == m_currentBufferSize ) {
        m_bufferedParticlesIndex = 0;
        m_currentBufferSize = stickychannel_fill_particle_buffer();
        if( m_currentBufferSize == 0 ) {
            m_bufferedParticles.clear(); // deallocate our internal buffer (we don't need the memory any more).
            return false;                // return if the delegate stream is exhaused
        }
    }

    // get the next particle
    memcpy( outParticleBuffer, m_bufferedParticles[m_bufferedParticlesIndex], m_nativeChannelMap.structure_size() );

    ++m_bufferedParticlesIndex;
    ++m_particleIndex;
    return true;
}

bool stickychannel_particle_istream::get_particles( char* buffer, std::size_t& numParticles ) {
    // This function could be optimized, instead of just taking one by one from the stream. We have a particle_array
    // after all.
    size_t offset = 0;
    size_t particleSize = m_channelMap.structure_size();
    while( offset < numParticles * particleSize ) {
        if( !get_particle( buffer + offset ) ) {
            numParticles = offset / particleSize;
            return false;
        }
        offset += particleSize;
    }
    return true;
}

size_t stickychannel_particle_istream::stickychannel_fill_particle_buffer() {
    frantic::channels::channel_cvt_accessor<boost::int64_t> idAccSigned =
        m_delegate->get_channel_map().get_cvt_accessor<boost::int64_t>( m_idChannelName );

    size_t newBufferSize = m_maxBufferSize;
    boost::shared_array<char> particleBuffer( new char[m_delegateChannelMap.structure_size()] );

    for( size_t i = 0; i < m_maxBufferSize; ++i ) {
        if( !m_delegate->get_particle( particleBuffer.get() ) ) {
            newBufferSize = i;
            break;
        }

        boost::int64_t id = idAccSigned.get( particleBuffer.get() );

        char* srcPtr;
        std::map<boost::int64_t, boost::shared_array<char>>::iterator iter = m_birthValues.find( id );
        if( iter != m_birthValues.end() ) {
            // Value is already there so just grab it
            srcPtr = iter->second.get();
        } else {
            // Value is not in there.  Update the map
            frantic::channels::channel_general_accessor inAcc =
                m_delegate->get_channel_map().get_general_accessor( m_sourceChannelName );
            size_t srcSize = inAcc.primitive_size();

            boost::shared_array<char> birthValue( new char[srcSize] );
            char* initialPtr = inAcc.get_channel_data_pointer( particleBuffer.get() );
            memcpy( birthValue.get(), initialPtr, srcSize );

            m_birthValues[id] = birthValue;
            srcPtr = m_birthValues[id].get();
        }

        // convert native channel map particle to our current channel map. this will copy it into our buffer in the
        // correct spot.
        m_cma.copy_structure( m_bufferedParticles[i], particleBuffer.get(), m_defaultParticle.get() );

        frantic::channels::channel_general_accessor outAcc =
            m_bufferedParticles.get_channel_map().get_general_accessor( m_destinationChannelName );
        size_t size = outAcc.primitive_size();
        char* dstPtr = outAcc.get_channel_data_pointer( m_bufferedParticles[i] );
        memcpy( dstPtr, srcPtr, size );
    }

    return newBufferSize;
}
