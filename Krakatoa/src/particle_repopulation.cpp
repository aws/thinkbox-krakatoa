// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoa/particle_repopulation.hpp>

#include <frantic/channels/channel_propagation_policy.hpp>
#include <frantic/particles/particle_grid_tree.hpp>
#include <frantic/particles/streams/apply_function_particle_istream.hpp>
#include <frantic/particles/streams/rle_levelset_particle_istream.hpp>
#include <frantic/particles/streams/set_channel_particle_istream.hpp>
#include <frantic/volumetrics/implicitsurface/implicit_surface_to_rle_level_set.hpp>
#include <frantic/volumetrics/voxel_sampler_interface.hpp>
#include <krakatoa/particle_volume.hpp>

#include <boost/bind.hpp>
#include <tbb/task_scheduler_init.h>

using namespace frantic::particles::streams;

namespace krakatoa {

boost::shared_ptr<particle_istream>
create_particle_repopulation_istream( boost::shared_ptr<particle_istream> pin, float fillRadius, int fillRadiusSubdivs,
                                      int numParticlesPerSubdiv, float densityFalloffStart, unsigned randomSeed,
                                      frantic::logging::progress_logger& logger ) {

    // if it's an empty stream, or does not have a "Position" channel, we don't have to do anything.
    if( pin->particle_count() == 0 || !pin->get_native_channel_map().has_channel( _T("Position") ) )
        return pin;

    tbb::task_scheduler_init tbbInit;

    densityFalloffStart = frantic::math::clamp( densityFalloffStart, 0.0f, 1.0f ); // sanity check input.

    const float implicitThreshold = 0.3f;        // hard-coded
    const float effectRadiusScale = 1.5f;        // hard-coded
    const float voxelLengthToRadiusRatio = 0.5f; // hard-coded

    const float voxelLength =
        fillRadius * voxelLengthToRadiusRatio; // our voxels will be half as large as the particle's requested radius.
                                               // this is an arbitrary choice, but yields decent results.

    // Our outgoing stream must have the same channel map as the incoming stream.
    // So, save it here, as we are going to modify the incoming stream when creating the PGT/level set.
    channel_map originalPinChannelMap = pin->get_channel_map();

    // modify the Radius channel to include the voxelLength*0.3 shift (see above comments).
    if( !pin->get_native_channel_map().has_channel( _T("Radius") ) ) {
        pin.reset( new set_channel_particle_istream<float>( pin, _T("Radius"), fillRadius ) );
    } else {
        // limit the particle radius to the max
        static boost::array<frantic::tstring, 1> modifyRadiusAffectedChannels = { _T("Radius") };
        struct modify_radius {
            static float fn( float radius, float maxRadius ) { return std::min( radius, maxRadius ); }
        };
        pin.reset( new apply_function_particle_istream<float( float )>(
            pin, boost::bind( &modify_radius::fn, _1, fillRadius ), _T("Radius"), modifyRadiusAffectedChannels ) );
    }

    // Add a density of 1.0 if it's not there (could be done with default particle, but isn't).
    if( !pin->get_native_channel_map().has_channel( _T("Density") ) ) {
        pin.reset( new set_channel_particle_istream<half>( pin, _T("Density"), 1.0f ) );
    }

    // create a channel map that will be used for the particles that are being entered into the levelset.
    // this levelset will be almost exactly the same as the input stream's native map, except for a few exceptions.
    // The excpetions are that there are a few channels we don't want, and there are a few channels that we ALWAYS want.
    channel_map newChannelMap;
    const channel_map& nativeChannelMap = pin->get_native_channel_map();
    size_t channelCount = nativeChannelMap.channel_count();
    for( size_t i = 0; i < channelCount; ++i ) {
        frantic::strings::tstring name;
        frantic::channels::data_type_t dataType;
        size_t arity;
        nativeChannelMap.get_channel_definition( i, name, dataType, arity );
        if( !frantic::channels::is_channel_data_type_float(
                dataType ) ) // We don't want to put integer channels onto the levelset. The output from that is
                             // meaningless.
            continue;
        if( name == _T("SignedDistance") || name == _T("SignedDistanceGradient") ||
            name == _T("Normal") ) // SignedDistance, SignedDistanceGradient, Normal are reserved for the level set.
                                   // Leave them out of the input stream.
            continue;
        if( name == _T("Density" ) ) // Density *must* be defined as float32, because that's what the post-level set
                                     // creation is expecting. Density is added after this loop.
            continue;
        if( name == _T("Emission" ) ) { // Emission also has a strict type definition, because that's what the
                                        // post-level set creation is expecting. So defined it as such.
            newChannelMap.define_channel( _T("Emission"), 3, frantic::channels::data_type_float32 );
            continue;
        }

        if( dataType == frantic::channels::data_type_float16 )
            // Promote half to float since half is more prone to overflow
            newChannelMap.define_channel( name, arity, frantic::channels::data_type_float32 );
        else
            newChannelMap.define_channel( name, arity, dataType );
    }
    newChannelMap.define_channel( _T("Density"), 1,
                                  frantic::channels::data_type_float32 ); // must exist, and must be float32
    newChannelMap.end_channel_definition();

    // set our stream to use our brand-new channel map.
    pin->set_channel_map( newChannelMap );

    // Set up the particle grid tree. This is an intermediate in-memory tree of all the particles required by the level
    // set creation function. The voxel length of the PGT must be larger than the level set's voxel length, because
    // there is an interaction radius to consider.
    frantic::volumetrics::voxel_coord_system pgtVCS( vector3f( 0.0f ), fillRadius * effectRadiusScale );
    frantic::particles::particle_grid_tree pgt( newChannelMap, pgtVCS );
    pgt.insert_particles( pin );

    // Create a levelset from particle grid tree.
    frantic::volumetrics::voxel_coord_system levelSetVCS( vector3f( 0.0f ), voxelLength );
    boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set> levelSet(
        new frantic::volumetrics::levelset::rle_level_set );

    // Convert the PGT into a levelset.
    FF_LOG( progress ) << _T("Computing particle repopulation grid.") << std::endl;
    frantic::channels::channel_propagation_policy allChannels;
    if( densityFalloffStart == 0.0f )
        allChannels.add_channel( _T("Position") ); // exclude Position if we don't need it later.
    frantic::volumetrics::implicitsurface::metaball_convert_particles_to_level_set(
        pgt, allChannels, fillRadius, effectRadiusScale, implicitThreshold, levelSetVCS, *levelSet.get(), logger );

    // We're done with the intermediate tree format. So we can free its memory.
    pgt.clear();

    // Handle original position is used to compute the falloff from the center of the sphere.
    // We do not want it to be called "Position", since it's going to conflict with the newly created particles.
    // Instead we rename it "OriginalPosition", which will be used in a custom stream later in this function.
    if( densityFalloffStart > 0.0f ) {
        levelSet->duplicate_channel( _T("OriginalPosition"), _T("Position") );
        levelSet->erase_channel( _T("Position") );
    }

    // There will always be a Density channel in the level set because we explicitly added it before we began.
    frantic::volumetrics::levelset::rle_channel_accessor<float> densityAcc =
        levelSet->get_channel_accessor<float>( _T("Density") );
    frantic::volumetrics::levelset::rle_channel_accessor<vector3f> emissionAcc;
    if( levelSet->has_channel( _T("Emission") ) )
        emissionAcc = levelSet->get_channel_accessor<vector3f>( _T("Emission") );

    // Compute number of outgoing particles per voxel. This is based on user input.
    numParticlesPerSubdiv = std::max( 1, numParticlesPerSubdiv ); // input check: has to be at least one.
    fillRadiusSubdivs = std::max( 0, fillRadiusSubdivs );         // input check: cannot be negative.
    float numberOfParticlePerVoxel = numParticlesPerSubdiv * powf( fillRadiusSubdivs + 1.0f, 3.0f );

    // Modify the Density channel for the voxels.
    // The densities were simply extrapolated from the incoming particles, however, density is additive, so the more
    // original particles there were, the higher density in the level set must be. Sames goes for Emission. To do so, we
    // include the "signed distance" factor in the density channel. This is not really a signed distance, but a particle
    // contribution factor that we are using.
    size_t numVoxels = levelSet->size();
    for( size_t i = 0; i < numVoxels; ++i ) {
        const float METABALL_DENSITY_FACTOR =
            0.1f; // This is the approximate factor that modifies metaball surfaces signed distance multiplies. It is an
                  // attempt to ensure the incoming particle set density sum, and out going particle set's density sum
                  // are close to equal.

        float particleContributionSignedDist =
            ( *levelSet )[i]; // This can be very large because it hasn't been reinitialized, and is always negative
        float signedDistance = implicitThreshold - particleContributionSignedDist;

        float totalScaleFactor = signedDistance * METABALL_DENSITY_FACTOR / numberOfParticlePerVoxel;
        densityAcc[i] *= totalScaleFactor;
        if( emissionAcc.valid() )
            emissionAcc[i] *= totalScaleFactor;
    }

    // create voxel sampler (for istream)
    boost::shared_ptr<frantic::volumetrics::voxel_sampler_interface> voxelSampler =
        krakatoa::get_particle_volume_voxel_sampler( fillRadiusSubdivs, true, numParticlesPerSubdiv, randomSeed, 1024,
                                                     false );

    // create a particle stream from the level set.
    boost::shared_ptr<particle_istream> outputStream =
        boost::shared_ptr<particle_istream>( new rle_levelset_particle_istream(
            originalPinChannelMap, levelSet, voxelSampler, -std::numeric_limits<float>::max(), 0.0f, false ) );

    // apply optional density "falloff".
    // density falloff is being computed by using the blurred "OriginalPosition" channel with the current position
    // channel. this may produce brighter spots between interacting particles. hopefully it will be acceptable, because
    // the signed distances we are getting from the level set are not accurate (and we'd need a reinitialization to do
    // it that way).
    if( densityFalloffStart > 0.0f ) {
        static boost::array<frantic::tstring, 4> modifyDensityAffectedChannels = {
            _T("Position"), _T("OriginalPosition"), _T("Radius"), _T("Density") };
        struct modify_density {
            static float fn( vector3f position, vector3f originalPosition, float radius, float originalDensity,
                             float falloffStart ) {
                // just using a linear falloff. may need to change that yet.
                if( radius > 0.0f ) {
                    float dist =
                        ( position - originalPosition )
                            .get_magnitude(); // square root in here. this is slow. consider exponential falloff. also,
                                              // this is not an accurate distance to the "outside" of the radius of this
                                              // particle. just an approximation based on the "blurred"
                                              // "OriginalPosition" channel. we will get hot spots places.
                    if( dist < radius ) {
                        dist -= radius * ( 1.0f - falloffStart );
                        if( dist > 0.0f ) {
                            // dist is between zero and falloffStart*radius, zero=1, falloffStart*radius=0
                            return originalDensity *
                                   std::max( 0.0f, 1.0f - dist / ( falloffStart *
                                                                   radius ) ); // technically don't need the "max", but
                                                                               // it's just for floating point error
                        }
                        return originalDensity;
                    }
                }
                return 0.0f;
            }
        };
        outputStream.reset( new apply_function_particle_istream<float( vector3f, vector3f, float, float )>(
            outputStream, boost::bind( &modify_density::fn, _1, _2, _3, _4, densityFalloffStart ), _T("Density"),
            modifyDensityAffectedChannels ) );
    }

    return outputStream;
}

} // namespace krakatoa
