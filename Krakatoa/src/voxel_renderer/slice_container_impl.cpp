// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <frantic/particles/particle_array.hpp>

#include <krakatoa/voxel_renderer/sample.hpp>
#include <krakatoa/voxel_renderer/slice_container_impl.hpp>
#include <krakatoa/voxel_renderer/slice_coordsys.hpp>

using frantic::graphics::color3f;
using frantic::graphics2d::vector2f;

namespace krakatoa {
namespace voxel_renderer {

slice_container_impl::slice_container_impl() {}

slice_container_impl::~slice_container_impl() {}

void slice_container_impl::set_new_voxel_callback( new_voxel_callback_interface* newVoxelCallback ) {
    m_newVoxelCallback.reset( newVoxelCallback );
}

void slice_container_impl::clear() { m_data.clear(); }

void slice_container_impl::reset( const frantic::channels::channel_map& dataLayout ) {
    m_data.reset( dataLayout );

    m_copyBlocks.clear();

    for( std::size_t i = 0, iEnd = dataLayout.channel_count(); i < iEnd; ++i ) {
        if( dataLayout[i].data_type() != frantic::channels::data_type_float32 )
            throw std::runtime_error( "slice_container_impl::reset() - Channel #" +
                                      boost::lexical_cast<std::string>( i ) +
                                      " is not a float32 channel and not supported by the voxel renderer" );

        if( dataLayout[i].name() != _T("Density") && dataLayout[i].name() != _T("Emission") &&
            dataLayout[i].name() != _T( "BokehBlendInfluence" ) ) {
            if( !m_copyBlocks.empty() && m_copyBlocks.back().first + sizeof( float ) * m_copyBlocks.back().second ==
                                             (int)dataLayout[i].offset() ) {
                m_copyBlocks.back().second += (int)dataLayout[i].arity();
            } else {
                m_copyBlocks.push_back(
                    std::pair<int, int>( (int)dataLayout[i].offset(), (int)dataLayout[i].arity() ) );
            }
        }
    }

    m_densityAccessor = dataLayout.get_accessor<float>( _T("Density") );

    m_emissionAccessor.reset();
    if( dataLayout.has_channel( _T("Emission") ) )
        m_emissionAccessor = dataLayout.get_accessor<color3f>( _T("Emission") );

    m_bokehBlendInfluenceAccessor.reset();
    if( dataLayout.has_channel( _T( "BokehBlendInfluence" ) ) ) {
        m_bokehBlendInfluenceAccessor = dataLayout.get_accessor<float>( _T( "BokehBlendInfluence" ) );
    }
}

bool slice_container_impl::is_empty() const { return m_data.is_empty(); }

void slice_container_impl::construct_sample( frantic::channels::property_map& outSampleStruct ) const {
    outSampleStruct.clear();
    outSampleStruct.set_channel_map( m_data.get_channel_map() );
}

void slice_container_impl::construct_sample( sample& outSampleStruct ) const {
    outSampleStruct.reset( m_data.get_channel_map() );
}

void slice_container_impl::get_voxel_indices_for_write( int x, int y, unsigned width, int outIndices[] ) {
    frantic::graphics2d::boundrect2 newVoxelBounds;

    // TODO: Make this function return whether or not newVoxelBounds was filled.
    m_data.get_or_make_data_indices( x, y, width, outIndices, newVoxelBounds );

    if( m_newVoxelCallback && !newVoxelBounds.is_empty() )
        m_newVoxelCallback->on_new_voxel( newVoxelBounds );
}

void slice_container_impl::add_to_voxel_by_weight( int index, const frantic::channels::property_map& dataStruct,
                                                   float weight ) {
    float density = m_densityAccessor.get( dataStruct.get_raw_buffer() );
    float weightedDensity = weight * density;

    char* dest = m_data[index];
    const char* src = dataStruct.get_raw_buffer();

    std::vector<std::pair<int, int>>::const_iterator it = m_copyBlocks.begin();
    std::vector<std::pair<int, int>>::const_iterator itEnd = m_copyBlocks.end();

    for( ; it != itEnd; ++it ) {
        float* floatDest = reinterpret_cast<float*>( dest + it->first );
        const float* floatSrc = reinterpret_cast<const float*>( src + it->first );
        const float* floatSrcEnd = floatSrc + it->second;

        for( ; floatSrc != floatSrcEnd; ++floatSrc, ++floatDest )
            ( *floatDest ) += weightedDensity * ( *floatSrc );
    }

    // Density does not get multiplied by itself.
    m_densityAccessor.get( dest ) += weightedDensity;

    // Emission is not weighted by the Density channel.
    if( m_emissionAccessor.is_valid() )
        m_emissionAccessor.get( dest ) += weight * m_emissionAccessor.get( dataStruct.get_raw_buffer() );

    // BokehBlendInfluence is not weighted by the Density channel.
    if( m_bokehBlendInfluenceAccessor.is_valid() ) {
        m_bokehBlendInfluenceAccessor.get( dest ) =
            std::max( m_bokehBlendInfluenceAccessor.get( dest ),
                      m_bokehBlendInfluenceAccessor.get( dataStruct.get_raw_buffer() ) );
    }
}

bool slice_container_impl::get_sample( frantic::graphics2d::vector2f pos, sample& outSample ) const {
    const frantic::channels::channel_map& dataMap = m_data.get_channel_map();

#ifndef NDEBUG
    if( outSample.get_channel_map() != dataMap )
        throw std::runtime_error( "slice_container_impl::get_sample() - Incorrect channel map in sample object" );
#endif

    // Zero out the sample.
    outSample.clear();

    int radius = m_readFilter->get_radius();
    int totalSize = 4 * radius * radius;

    float floorX = floorf( pos.x );
    float alphaX; // = pos.x - floorX;
    int baseX = (int)floorX - radius + 1;

    float floorY = floorf( pos.y );
    float alphaY; // = pos.y - floorY;
    int baseY = (int)floorY - radius + 1;

    int* indices = (int*)alloca( sizeof( int ) * totalSize );
    float* weights = (float*)alloca( sizeof( float ) * totalSize );

    m_data.get_data_indices( baseX, baseY, 2 * radius, indices );

    bool result = false;
    for( int i = 0; i < totalSize; ++i, ++indices /*, ++weights*/ ) {
        if( *indices >= 0 ) {
            if( !result ) {
                alphaX = pos.x - floorX;
                alphaY = pos.y - floorY;
                m_readFilter->do_filter( vector2f( alphaX, alphaY ), weights );

                result = true;
            }

            const char* src = m_data[*indices];
            for( std::size_t j = 0, jEnd = dataMap.channel_count(); j < jEnd; ++j ) {
                const float* floatSrc = reinterpret_cast<const float*>( src + dataMap[j].offset() );
                float* floatDest =
                    reinterpret_cast<float*>( const_cast<char*>( outSample.get_raw_buffer() ) + dataMap[j].offset() );
                for( std::size_t k = 0, kEnd = dataMap[j].arity(); k < kEnd; ++k )
                    floatDest[k] += weights[i] * floatSrc[k];
            }
        }
    }

    return result;
}

void slice_container_impl::get_voxels_as_particles( frantic::particles::particle_array& outParticles,
                                                    const slice_coordsys& coordSys ) const {
    using frantic::graphics::vector3f;

    m_data.get_voxels_as_particles( outParticles );

    frantic::channels::channel_accessor<vector3f> posAccessor =
        outParticles.get_channel_map().get_accessor<vector3f>( _T("Position") );

    // TODO: Transform any other channels that need it. Most are stored as world-space channels anyways.

    for( frantic::particles::particle_array::iterator it = outParticles.begin(), itEnd = outParticles.end();
         it != itEnd; ++it ) {
        posAccessor.get( *it ) = coordSys.transform_to_world( posAccessor.get( *it ) );
    }
}

} // namespace voxel_renderer
} // namespace krakatoa
