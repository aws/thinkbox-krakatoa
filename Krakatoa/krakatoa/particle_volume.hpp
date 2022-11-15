// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/geometry/trimesh3.hpp>
#include <frantic/logging/progress_logger.hpp>
#include <frantic/particles/streams/particle_istream.hpp>
#include <frantic/volumetrics/levelset/rle_level_set.hpp>
#include <frantic/volumetrics/voxel_sampler_interface.hpp>

#pragma warning( push, 3 )
#include <boost/random.hpp>
#pragma warning( pop )

namespace krakatoa {

class particle_volume_voxel_sampler;

/**
 * Creates a levelset from a trimesh for use with the rle_levelset_particle_istream.
 *
 * @param theTriMesh Input triangle mesh
 * @param voxelSpacing Voxel length of returned levelset
 * @param progressLogger Progress logger
 * @return A pointer to the resulting levelset
 */
boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set>
get_particle_volume_levelset( const frantic::geometry::trimesh3& theTriMesh, float voxelSpacing,
                              frantic::logging::progress_logger& progressLogger );

/**
 * Creates a voxel_sampler_interface object for for use with the rle_levelset_particle_istream.
 *
 * @param subdivCount The number of subdivisions per voxel. Zero if subdividing is disabled.
 * @param doJitter Places particles non-centered within the subdivided voxels. Enables numPerVoxel, randomSeed,
 * randomCount, and jitteredWellDistributed parameters.
 * @param numPerVoxel How many particles to put in each subdivided voxel. Only used when jittering.
 * @param randomSeed Random seed used for all random numbers. Only used when jittering.
 * @param randomCount Random jittered positions are selected via a hash table from a pre-determined list of random
 * positions. The randomCount parameter determines the length of this pre-determined list. Only used when jittering.
 * @param jitterWellDistributed This parameter changes the algorithm that generates the random jitter positions. Setting
 * it to true uses a more-expensive poisson-sphere tiling algorithm. Only used when jittering.
 */
boost::shared_ptr<particle_volume_voxel_sampler>
get_particle_volume_voxel_sampler( unsigned int subdivCount, bool doJitter, int numPerVoxel, unsigned int randomSeed,
                                   size_t randomCount, bool jitterWellDistributed );

/**
 * Krakatoa implementation of a voxel volume sampler for particles.
 * This is used in MaxKrakatoaFumeFXObject, and by get_particle_volume_voxel_sampler.
 * TODO: refactor MaxKrakatoaFumeFXObject to use get_particle_volume_voxel_sampler.
 */
class particle_volume_voxel_sampler : public frantic::volumetrics::voxel_sampler_interface {
    frantic::graphics::vector3 m_currentVoxel;

    int m_numSubdivs;
    frantic::graphics::vector3 m_currentSubdiv;

    std::size_t m_numPerSubdiv;
    std::size_t m_current;

    // A factor that compensates for the number of samples placed in a voxel.
    float m_compensationFactor;

    // Picked rand48 since it seeds pretty quick compared to mt19937 (it only stores one 64bit int as state).
    boost::rand48 m_rndGen;

    std::vector<frantic::graphics::vector3f> m_samples;
    std::vector<std::size_t> m_shuffledIndices;

  private:
    /**
     * An implementation of the Fowler-Noll-Vo 32bit hash function (FNV-1A).
     * See http://en.wikipedia.org/wiki/Fowler-Noll-Vo_hash_function
     * This version will compute the continuation of a hash, assuming it has been started
     * by some other function. (ie. below)
     *
     * TODO: Try the MurmurHash hashing function, which is apparently faster.
     */
    inline static boost::uint32_t continue_hash( boost::uint32_t prevHash, boost::uint32_t nextVal ) {
        static const boost::uint32_t FNV_PRIME = 16777619;

        prevHash ^= ( nextVal & 0xFF );
        prevHash *= FNV_PRIME;
        prevHash ^= ( ( nextVal >> 8 ) & 0xFF );
        prevHash *= FNV_PRIME;
        prevHash ^= ( ( nextVal >> 16 ) & 0xFF );
        prevHash *= FNV_PRIME;
        prevHash ^= ( ( nextVal >> 24 ) & 0xFF );
        prevHash *= FNV_PRIME;

        return prevHash;
    }

    /**
     * An implementation of the Fowler-Noll-Vo 32bit hash function (FNV-1A).
     * See http://en.wikipedia.org/wiki/Fowler-Noll-Vo_hash_function
     *
     * TODO: Try the MurmurHash hashing function, which is apparently faster.
     */
    inline static boost::uint32_t hash( int x, int y, int z ) {
        static const boost::uint32_t FNV_PRIME = 16777619;
        static const boost::uint32_t FNV_OFFSET = 2166136261;

        boost::uint32_t hash = FNV_OFFSET;
        hash = continue_hash( hash, static_cast<boost::uint32_t>( x ) );
        hash = continue_hash( hash, static_cast<boost::uint32_t>( y ) );
        hash = continue_hash( hash, static_cast<boost::uint32_t>( z ) );

        return hash;
    }

    void shuffle_indices() {
        // Use the hash function to start the random number sequence.
        int hashInX = m_currentVoxel.x * m_numSubdivs + m_currentVoxel.x + m_currentSubdiv.x;
        int hashInY = m_currentVoxel.y * m_numSubdivs + m_currentVoxel.y + m_currentSubdiv.y;
        int hashInZ = m_currentVoxel.z * m_numSubdivs + m_currentVoxel.z + m_currentSubdiv.z;
        unsigned hashID = hash( hashInX, hashInY, hashInZ );

        if( m_numPerSubdiv == 1 ) {
            m_shuffledIndices.resize( 1 );
            m_shuffledIndices[0] = (std::size_t)hashID % m_samples.size();
        } else {
            if( m_numPerSubdiv > m_samples.size() )
                throw std::runtime_error( "multi_sample_generator::shuffle_indices() There were more particles "
                                          "requested than random sample points created." );

            m_shuffledIndices.resize( m_samples.size() );

            for( std::size_t i = 0; i < m_shuffledIndices.size(); ++i )
                m_shuffledIndices[i] = i;

            m_rndGen.seed( hashID );

            // Picks 'm_numPerVoxel' random (non-duplicate) indices
            for( std::size_t i = 0; i < m_numPerSubdiv; ++i ) {
                boost::uniform_smallint<> rndRange( (int)i, (int)( m_shuffledIndices.size() - 1 ) );
                boost::variate_generator<boost::rand48&, boost::uniform_smallint<>> rng( m_rndGen, rndRange );

                std::swap( m_shuffledIndices[i], m_shuffledIndices[rng()] );
            }
        }
    }

  public:
    particle_volume_voxel_sampler() {
        m_currentVoxel.x = m_currentVoxel.y = m_currentVoxel.z = std::numeric_limits<int>::min();

        set_subdivs( 0, 1 );
    }

    virtual ~particle_volume_voxel_sampler() {}

    void set_subdivs( int subdivs, int numPerSubdiv ) {
        m_numSubdivs = subdivs;
        m_numPerSubdiv = (std::size_t)numPerSubdiv;

        m_current = m_numPerSubdiv;
        m_currentSubdiv.x = m_currentSubdiv.y = m_currentSubdiv.z = m_numSubdivs;
        m_compensationFactor = 1.f / float( numPerSubdiv * ( subdivs + 1 ) * ( subdivs + 1 ) * ( subdivs + 1 ) );
    }

    void add_sample_position( const frantic::graphics::vector3f& p ) { m_samples.push_back( p ); }

    float get_compensation_factor() const { return m_compensationFactor; }

    virtual void update_for_voxel( const frantic::graphics::vector3& voxel ) {
        m_currentVoxel = voxel;
        m_current = (std::size_t)-1;
        m_currentSubdiv.x = m_currentSubdiv.y = m_currentSubdiv.z = 0;

        shuffle_indices();
    }

    virtual bool get_next_position( frantic::graphics::vector3f& outPosition, float& outCompensationFactor ) {
        if( ++m_current >= m_numPerSubdiv ) {
            if( m_currentSubdiv.z == m_numSubdivs ) {
                if( m_currentSubdiv.y == m_numSubdivs ) {
                    if( m_currentSubdiv.x == m_numSubdivs )
                        return false;
                    ++m_currentSubdiv.x;
                    m_currentSubdiv.y = 0;
                    m_currentSubdiv.z = 0;
                } else {
                    ++m_currentSubdiv.y;
                    m_currentSubdiv.z = 0;
                }
            } else
                ++m_currentSubdiv.z;
            m_current = 0;
            shuffle_indices();
        }

        frantic::graphics::vector3f samplePos = m_samples[m_shuffledIndices[m_current]];

        samplePos.x = ( (float)m_currentSubdiv.x + samplePos.x ) / (float)( m_numSubdivs + 1 );
        samplePos.y = ( (float)m_currentSubdiv.y + samplePos.y ) / (float)( m_numSubdivs + 1 );
        samplePos.z = ( (float)m_currentSubdiv.z + samplePos.z ) / (float)( m_numSubdivs + 1 );

        outPosition = samplePos;
        outCompensationFactor = m_compensationFactor;
        return true;
    }
};

} // namespace krakatoa
