// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>

#include <frantic/graphics/units.hpp>
#include <frantic/particles/particle_array.hpp>

class stickychannel_particle_istream : public frantic::particles::streams::particle_istream {
  private:
    // variables from delegate
    boost::shared_ptr<frantic::particles::streams::particle_istream> m_delegate;
    boost::int64_t m_particleIndex;

    frantic::tstring m_idChannelName;
    frantic::tstring m_sourceChannelName;
    frantic::tstring m_destinationChannelName;

    // our current channel map
    frantic::channels::channel_map m_channelMap;
    frantic::channels::channel_map_adaptor m_cma; // m_delegateChannelMap to m_channelMap

    // the native channel map
    frantic::channels::channel_map m_nativeChannelMap;
    frantic::channels::channel_map m_delegateChannelMap;

    // for the buffered particles
    frantic::particles::particle_array m_bufferedParticles;
    size_t m_maxBufferSize; // user set
    size_t m_currentBufferSize;
    size_t m_bufferedParticlesIndex;

    // our default particle
    boost::shared_array<char> m_defaultParticle;

    // our birth channel mapping
    std::map<boost::int64_t, boost::shared_array<char>>& m_birthValues;

  public:
    /**
     * Creates a stream from a delegate that sets a channel to be the result of the birth channel.
     * @param pin This incoming particle stream
     * @param birthValues Map for storing and retrieving the birth positions.  The key is determined by the ID channel
     * values.
     * @param sourceChannel Channel to get the birth value from
     * @param destChannel to set the birth value to
     * @param idChannel ID channel (use to store the current birth positions)
     * @param bufferSize Internally, this stream buffers chunks of particles in memory. This parameter determines the
     * number of particles to hold in memory at once. This is needed because Maya must process particles in batch. Small
     * buffer means low performance, but less memory usuage. Large buffers mean high performance, but more memory usage.
     */
    stickychannel_particle_istream( boost::shared_ptr<frantic::particles::streams::particle_istream> pin,
                                    std::map<boost::int64_t, boost::shared_array<char>>& birthValues,
                                    const frantic::tstring& sourceChannel = _T("Position"),
                                    const frantic::tstring& destChannel = _T("BirthPosition"),
                                    const frantic::tstring& idChannel = _T("ID"), size_t bufferSize = 100000 );

    void close() { m_delegate->close(); }

    // The stream can return its filename or other identifier for better error messages.
    frantic::tstring name() const { return m_delegate->name(); }

    // This is the size of the particle structure which will be loaded, in bytes.
    std::size_t particle_size() const { return m_channelMap.structure_size(); }

    // Returns the number of particles, or -1 if unknown
    boost::int64_t particle_count() const { return m_delegate->particle_count(); }
    boost::int64_t particle_index() const { return m_particleIndex; }
    boost::int64_t particle_count_left() const {
        boost::int64_t particleCount = m_delegate->particle_count();
        if( particleCount == -1 )
            return -1;
        return particleCount - m_particleIndex;
    }

    boost::int64_t particle_progress_count() const { return particle_count(); }
    boost::int64_t particle_progress_index() const { return particle_index(); }
    boost::int64_t particle_count_guess() const { return m_delegate->particle_count_guess(); }

    // This allows you to change the particle layout that's being loaded on the fly, in case it couldn't
    // be set correctly at creation time.
    void set_channel_map( const frantic::channels::channel_map& particleChannelMap );

    // This is the particle channel map which specifies the byte layout of the particle structure that is being used.
    const frantic::channels::channel_map& get_channel_map() const;

    // This is the particle channel map which specifies the byte layout of the input to this stream.
    // NOTE: This value is allowed to change after the following conditions:
    //    * set_channel_map() is called (for example, the empty_particle_istream equates the native map with the
    //    external map)
    const frantic::channels::channel_map& get_native_channel_map() const;

    /** This provides a default particle which should be used to fill in channels of the requested channel map
     *	which are not supplied by the native channel map.
     *	IMPORTANT: Make sure the buffer you pass in is at least as big as particle_size() bytes.
     */
    void set_default_particle( char* rawParticleBuffer );

    // This reads a particle into a buffer matching the channel_map.
    // It returns true if a particle was read, false otherwise.
    // IMPORTANT: Make sure the buffer you pass in is at least as big as particle_size() bytes.
    bool get_particle( char* rawParticleBuffer );

    // This reads a group of particles. Returns false if the end of the source
    // was reached during the read.
    bool get_particles( char* rawParticleBuffer, std::size_t& numParticles );

  private:
    void init_channel_map( const frantic::channels::channel_map& inputChannelMap );
    size_t stickychannel_fill_particle_buffer();
};