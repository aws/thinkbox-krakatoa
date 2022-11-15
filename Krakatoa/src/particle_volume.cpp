// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include "krakatoa/particle_volume.hpp"

#include <frantic/geometry/trimesh3_named_channels.hpp>
#include <frantic/graphics/boundbox3.hpp>
#include <frantic/graphics/poisson_sphere.hpp>
#include <frantic/particles/streams/empty_particle_istream.hpp>
#include <frantic/particles/streams/rle_levelset_particle_istream.hpp>
#include <frantic/volumetrics/levelset/geometry_to_levelset.hpp>
#include <frantic/volumetrics/voxel_coord_system.hpp>

using namespace frantic::graphics;
using namespace frantic::geometry;
using namespace frantic::volumetrics::levelset;
using namespace frantic::particles::streams;

namespace krakatoa {

boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set>
get_particle_volume_levelset( const frantic::geometry::trimesh3& theTriMesh, float voxelSpacing,
                              frantic::logging::progress_logger& progressLogger ) {

    boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set> pLevelset;

    frantic::volumetrics::voxel_coord_system vcs( vector3f(), voxelSpacing );
    pLevelset.reset( new rle_level_set( vcs ) );

    std::vector<frantic::tstring> meshVertexChannels;
    theTriMesh.get_vertex_channel_names( meshVertexChannels );

    for( std::vector<frantic::tstring>::const_iterator it = meshVertexChannels.begin(),
                                                       itEnd = meshVertexChannels.end();
         it != itEnd; ++it ) {
        const_trimesh3_vertex_channel_general_accessor accessor = theTriMesh.get_vertex_channel_general_accessor( *it );
        pLevelset->add_channel( *it, accessor.arity(), accessor.data_type() );
    }
    progressLogger.update_progress( 10.0f );

    static const float SQRT3 = 1.7320508075688772935274463415059f; // sqrt 3
    convert_geometry_to_levelset( theTriMesh, -SQRT3, SQRT3, *pLevelset );

    // We want all the levelset values in the volume to be filled, so now we do that using extrapolation.
    rle_index_spec ris;
    ris.build_by_filling( pLevelset->get_rle_index_spec() );

    pLevelset->switch_rle_index_spec_with_swap( ris, _T("Populated") );
    pLevelset->duplicate_channel( _T("PopulatedChannelData"), _T("Populated") );
    progressLogger.update_progress( 35.0f );

    progressLogger.push_progress( 35.0f, 75.0f );
    pLevelset->reinitialize_signed_distance_from_populated(
        progressLogger, _T("Populated"), -std::numeric_limits<float>::max(), vcs.voxel_length() * SQRT3 );
    pLevelset->erase_channel( _T("Populated") );
    progressLogger.pop_progress();

    pLevelset->extrapolate_channels( _T("PopulatedChannelData") );
    pLevelset->erase_channel( _T("PopulatedChannelData") );
    progressLogger.update_progress( 100.0f );

    return pLevelset;
}

boost::shared_ptr<particle_volume_voxel_sampler>
get_particle_volume_voxel_sampler( unsigned int subdivCount, bool doJitter, int numPerVoxel, unsigned int randomSeed,
                                   size_t randomCount, bool jitterWellDistributed ) {
    std::vector<frantic::graphics::vector3f> poissonSamples;

    boost::shared_ptr<particle_volume_voxel_sampler> pSampleGen( new particle_volume_voxel_sampler );

    pSampleGen->set_subdivs( subdivCount, numPerVoxel );

    if( doJitter ) {
        boost::mt19937 gen( randomSeed );
        boost::uniform_01<float> range;
        boost::variate_generator<boost::mt19937, boost::uniform_01<float>> rng( gen, range );

        std::size_t requestedSamples = randomCount;
        if( (std::size_t)numPerVoxel > requestedSamples )
            throw std::runtime_error( "The number of samples per voxel is larger than the number of available random "
                                      "values. Please increase the number of potential random values." );

        if( jitterWellDistributed ) {
            float radius = std::pow( 4 * std::sqrt( 2.f ) * (float)requestedSamples / 0.6f, -1.f / 3.f );
            if( radius > 0.1f )
                radius = 0.1f;

            frantic::graphics::build_poisson_sphere_tiles( rng, &poissonSamples, 1, radius, 0.6f, 200000 );

            if( numPerVoxel > (int)poissonSamples.size() )
                throw std::runtime_error(
                    "The number of samples generated from the poisson sphere was less than the number requested per "
                    "voxel. Please increase the number of potential random values." );

            for( std::vector<frantic::graphics::vector3f>::iterator it = poissonSamples.begin(),
                                                                    itEnd = poissonSamples.end();
                 it != itEnd; ++it )
                pSampleGen->add_sample_position( *it );
        } else {
            for( std::size_t i = 0; i < requestedSamples; ++i )
                pSampleGen->add_sample_position( frantic::graphics::vector3f::from_random( rng ) );
        }
    } else {
        pSampleGen->add_sample_position( frantic::graphics::vector3f( 0.5f ) );
    }
    return pSampleGen;
}

} // namespace krakatoa
