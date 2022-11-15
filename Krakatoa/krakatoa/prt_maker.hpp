// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#pragma warning( push, 3 )
#include <boost/random.hpp>
#pragma warning( pop )

#include <frantic/channels/channel_map_adaptor.hpp>
#include <frantic/particles/streams/particle_istream.hpp>

namespace krakatoa {

class particle_istream_base : public frantic::particles::streams::particle_istream {
  protected:
    boost::int64_t m_particleCount, m_particleIndex;
    boost::scoped_array<char> m_defaultParticle;

    frantic::channels::channel_map m_channelMap, m_nativeMap;

  public:
    particle_istream_base( boost::int64_t particleCount )
        : m_particleCount( particleCount )
        , m_particleIndex( -1 ) {}

    virtual ~particle_istream_base() { close(); }

    virtual void close() {}

    virtual frantic::tstring name() const = 0;

    virtual std::size_t particle_size() const { return m_channelMap.structure_size(); }

    virtual boost::int64_t particle_count() const { return m_particleCount; }

    virtual boost::int64_t particle_index() const { return m_particleIndex; }

    virtual boost::int64_t particle_count_left() const { return m_particleCount - m_particleIndex - 1; }

    virtual boost::int64_t particle_progress_count() const { return m_particleCount; }

    virtual boost::int64_t particle_progress_index() const { return m_particleIndex; }

    virtual void set_channel_map( const frantic::channels::channel_map& newMap ) {
        boost::scoped_array<char> newDefault( new char[newMap.structure_size()] );

        newMap.construct_structure( newDefault.get() );

        if( m_defaultParticle ) {
            frantic::channels::channel_map_adaptor tempAdaptor( newMap, m_channelMap );

            tempAdaptor.copy_structure( newDefault.get(), m_defaultParticle.get() );
        }

        m_defaultParticle.swap( newDefault );

        m_channelMap = newMap;
    }

    virtual const frantic::channels::channel_map& get_channel_map() const { return m_channelMap; }

    virtual const frantic::channels::channel_map& get_native_channel_map() const { return m_nativeMap; }

    virtual void set_default_particle( char* rawParticleBuffer ) {
        m_defaultParticle.reset( new char[m_channelMap.structure_size()] );
        m_channelMap.copy_structure( m_defaultParticle.get(), rawParticleBuffer );
    }

    virtual bool get_particle( char* rawParticleBuffer ) = 0;

    virtual bool get_particles( char* rawParticleBuffer, std::size_t& numParticles ) {
        for( std::size_t i = 0; i < numParticles; ++i, rawParticleBuffer += m_channelMap.structure_size() ) {
            if( !get_particle( rawParticleBuffer ) ) {
                numParticles = i;
                return false;
            }
        }

        return true;
    }
};

class simple_particle_istream : public particle_istream_base {
    frantic::channels::channel_cvt_accessor<float> m_randAccessor;

    boost::mt19937 m_rngEngine;
    boost::variate_generator<boost::mt19937&, boost::uniform_01<float>> m_rng;

  public:
    simple_particle_istream( const frantic::channels::channel_map& pcm, boost::int64_t count, unsigned long seed )
        : particle_istream_base( count )
        , m_rng( m_rngEngine, boost::uniform_01<float>() ) {
        m_rngEngine.seed( seed );

        m_nativeMap.define_channel<frantic::graphics::vector3f>( _T("Position") );
        m_nativeMap.define_channel<float>( _T("ColorScalar") );
        m_nativeMap.define_channel<float>( _T("RandomValue") );
        m_nativeMap.end_channel_definition();

        set_channel_map( pcm );
    }

    virtual ~simple_particle_istream() {}

    // The stream can return its filename or other identifier for better error messages.
    virtual frantic::tstring name() const { return _T("simple_particle_istream"); }

    virtual void set_channel_map( const frantic::channels::channel_map& channelMap ) {
        particle_istream_base::set_channel_map( channelMap );

        if( m_channelMap.has_channel( _T("RandomValue") ) )
            m_randAccessor = m_channelMap.get_cvt_accessor<float>( _T("RandomValue") );
        else
            m_randAccessor.reset();
    }

    virtual bool get_particle( char* rawParticleBuffer ) {
        if( ++m_particleIndex >= m_particleCount )
            return ( m_particleIndex = m_particleCount, false );

        m_channelMap.copy_structure( rawParticleBuffer, m_defaultParticle.get() );

        if( m_randAccessor.is_valid() )
            m_randAccessor.set( rawParticleBuffer, m_rng() );

        return true;
    }
};

class ifs_particle_istream : public particle_istream_base {
    frantic::channels::channel_accessor<frantic::graphics::vector3f> m_posAccessor;
    frantic::channels::channel_cvt_accessor<float> m_colorAccessor;
    frantic::channels::channel_cvt_accessor<float> m_randAccessor;

    std::vector<frantic::graphics::transform4f> m_affineTMs;
    std::vector<float> m_tmWeights;

    frantic::graphics::vector3f m_prevPos;
    float m_prevColor;

    boost::mt19937 m_rngEngine;
    boost::variate_generator<boost::mt19937&, boost::uniform_01<float>> m_rng;

  private:
    inline void apply( float randValue, frantic::graphics::vector3f& inoutPos, float& inoutColor ) const {
        std::size_t fnIndex = 0;

        for( std::size_t fnIndexEnd = m_tmWeights.size() - 1; fnIndex < fnIndexEnd; ++fnIndex ) {
            if( randValue <= m_tmWeights[fnIndex] )
                break;
        }

        inoutPos = m_affineTMs[fnIndex] * inoutPos;
        inoutColor =
            0.5f * ( inoutColor + ( static_cast<float>( fnIndex ) / static_cast<float>( m_tmWeights.size() - 1 ) ) );
    }

  public:
    ifs_particle_istream( const frantic::channels::channel_map& pcm, boost::int64_t count, unsigned long seed )
        : particle_istream_base( count )
        , m_rng( m_rngEngine, boost::uniform_01<float>() ) {
        m_rngEngine.seed( seed );

        m_nativeMap.define_channel<frantic::graphics::vector3f>( _T("Position") );
        m_nativeMap.define_channel<float>( _T("ColorScalar") );
        m_nativeMap.define_channel<float>( _T("RandomValue") );
        m_nativeMap.end_channel_definition();

        set_channel_map( pcm );
    }

    virtual ~ifs_particle_istream() {}

    void add_affine_transform( const frantic::graphics::transform4f& affineTM, float relativeWeight = 1.f ) {
        m_affineTMs.push_back( affineTM );
        m_tmWeights.push_back( relativeWeight );
    }

    // The stream can return its filename or other identifier for better error messages.
    virtual frantic::tstring name() const { return _T("ifs_particle_istream"); }

    virtual void set_channel_map( const frantic::channels::channel_map& channelMap ) {
        particle_istream_base::set_channel_map( channelMap );

        m_posAccessor = m_channelMap.get_accessor<frantic::graphics::vector3f>( _T("Position") );

        if( m_channelMap.has_channel( _T("ColorScalar") ) )
            m_colorAccessor = m_channelMap.get_cvt_accessor<float>( _T("ColorScalar") );

        if( m_channelMap.has_channel( _T("RandomValue") ) )
            m_randAccessor = m_channelMap.get_cvt_accessor<float>( _T("RandomValue") );
    }

    virtual bool get_particle( char* rawParticleBuffer ) {
        if( m_particleIndex < 0 ) {
            // Initialize the stream since this is the first particle
            float accum = 0.f;
            for( std::vector<float>::const_iterator it = m_tmWeights.begin(), itEnd = m_tmWeights.end(); it != itEnd;
                 ++it )
                accum += *it;

            float prev = 0.f;
            for( std::vector<float>::iterator it = m_tmWeights.begin(), itEnd = m_tmWeights.end(); it != itEnd; ++it )
                prev = *it = ( *it + prev ) / accum;

            m_prevPos.x = 2 * m_rng() - 1;
            m_prevPos.y = 2 * m_rng() - 1;
            m_prevPos.z = 2 * m_rng() - 1;
            m_prevColor = m_rng();

            for( std::size_t i = 0; i < 20; ++i )
                apply( m_rng(), m_prevPos, m_prevColor );

            m_particleIndex = 0;
        } else if( ++m_particleIndex >= m_particleCount )
            return ( m_particleIndex = m_particleCount, false );

        m_channelMap.copy_structure( rawParticleBuffer, m_defaultParticle.get() );

        float randVal = m_rng();

        apply( randVal, m_prevPos, m_prevColor );

        m_posAccessor.get( rawParticleBuffer ) = m_prevPos;

        if( m_colorAccessor.is_valid() )
            m_colorAccessor.set( rawParticleBuffer, m_prevColor );

        if( m_randAccessor.is_valid() )
            m_randAccessor.set( rawParticleBuffer, randVal );

        return true;
    }
};

} // namespace krakatoa
