// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#pragma warning( push, 3 )
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#include <tbb/task.h>

#include <boost/random.hpp>
#pragma warning( pop )
#include <boost/math/special_functions/fpclassify.hpp>

#include <frantic/channels/property_map.hpp>
#include <frantic/graphics/boundbox3f.hpp>
#include <frantic/graphics2d/boundrect2.hpp>
#include <frantic/particles/particle_utilities.hpp>

#include <krakatoa/voxel_renderer/particle_data_source.hpp>
#include <krakatoa/voxel_renderer/slice_container.hpp>
#include <krakatoa/voxel_renderer/slice_coordsys.hpp>

using frantic::graphics::color3f;
using frantic::graphics::vector3f;

#include <frantic/diagnostics/profiling_section.hpp>
frantic::diagnostics::profiling_section g_voxelIndexCreation( _T("Voxel:Splat:TreeCreation") );

namespace krakatoa {
namespace voxel_renderer {

particle_data_source::particle_data_source( particle_data_source::particle_type& particles, bool disableThreading ) {
    m_disableThreading = disableThreading;
    m_particles = &particles;
    m_curIt = m_endIt = particles.end();
    m_mblurTimeSeconds = 0;
}

particle_data_source::~particle_data_source() {}

const frantic::channels::channel_map& particle_data_source::get_channel_map() const {
    return m_particles->get_channel_map();
}

void particle_data_source::set_motion_blur_time( float mblurTimeIntervalCenterSeconds,
                                                 float mblurTimeIntervalWidthSeconds, int seed ) {
    if( m_particles->get_channel_map().has_channel( _T("MBlurTime") ) && mblurTimeIntervalWidthSeconds > 0 ) {
        boost::mt19937 generator( seed );
        boost::variate_generator<boost::mt19937&, boost::uniform_01<float>> rng( generator,
                                                                                 boost::uniform_01<float>() );

        frantic::channels::channel_cvt_accessor<float> timeAccessor =
            m_particles->get_channel_map().get_cvt_accessor<float>( _T("MBlurTime") );

        for( particle_type::iterator it = m_particles->begin(), itEnd = m_particles->end(); it != itEnd; ++it ) {
            float realTime = mblurTimeIntervalCenterSeconds + ( rng() - 0.5f ) * mblurTimeIntervalWidthSeconds;
            timeAccessor.set( *it, realTime );
        }

        m_mblurTimeSeconds = std::numeric_limits<float>::infinity();
    } else {
        // If we are using a single time for all particles, use the center of this range.
        m_mblurTimeSeconds = mblurTimeIntervalCenterSeconds;
    }
}

namespace {
class particle_compare {
    data_source::coordsys_ptr m_coordSys;

    frantic::channels::channel_accessor<vector3f> posAccessor;
    frantic::channels::channel_cvt_accessor<vector3f> velAccessor;
    frantic::channels::channel_cvt_accessor<float> timeAccessor;

  public:
    particle_compare() {}

    particle_compare( data_source::coordsys_ptr coordSys, const frantic::channels::channel_map& pcm, float defaultTime )
        : m_coordSys( coordSys )
        , velAccessor( vector3f( 0 ) )
        , timeAccessor( defaultTime ) {
        posAccessor = pcm.get_accessor<vector3f>( _T("Position") );

        if( pcm.has_channel( _T("Velocity") ) )
            velAccessor = pcm.get_cvt_accessor<vector3f>( _T("Velocity") );

        if( pcm.has_channel( _T("MBlurTime") ) )
            timeAccessor = pcm.get_cvt_accessor<float>( _T("MBlurTime") );
    }

    bool operator()( const char* lhs, const char* rhs ) const {
        float lhsZ =
            m_coordSys
                ->transform_from_world( posAccessor.get( lhs ) + timeAccessor.get( lhs ) * velAccessor.get( lhs ) )
                .z;
        float rhsZ =
            m_coordSys
                ->transform_from_world( posAccessor.get( rhs ) + timeAccessor.get( rhs ) * velAccessor.get( rhs ) )
                .z;
        return lhsZ > rhsZ;
    }

    float operator()( const char* p ) const {
        return m_coordSys->transform_from_world( posAccessor.get( p ) + timeAccessor.get( p ) * velAccessor.get( p ) )
            .z;
    }
};
} // namespace

std::pair<int, int> particle_data_source::reset( coordsys_ptr coordSys ) {
    m_coordSys = coordSys;

    frantic::graphics::vector3f dir =
        vector3f::normalize( coordSys->transform_vector_to_world( vector3f( 0, 0, -1 ) ) );
    particle_compare func( coordSys, m_particles->get_channel_map(), m_mblurTimeSeconds );
    // frantic::sort::threaded_sort( m_particles->begin(), m_particles->end(), func, m_disableThreading );

    frantic::sort::parallel_sort( m_particles->begin(), m_particles->end(), func, *m_progressLogger,
                                  m_disableThreading );

    m_curIt = m_particles->begin();
    m_endIt = m_particles->end();

    int nearSlice = 0, farSlice = 0;
    if( m_particles->size() > 0 ) {
        frantic::channels::channel_accessor<vector3f> posAccessor =
            m_particles->get_channel_map().get_accessor<vector3f>( _T("Position") );
        frantic::channels::channel_cvt_accessor<vector3f> velAccessor( vector3f( 0 ) );
        frantic::channels::channel_cvt_accessor<float> timeAccessor( m_mblurTimeSeconds );

        if( m_particles->get_channel_map().has_channel( _T("Velocity") ) )
            velAccessor = m_particles->get_channel_map().get_cvt_accessor<vector3f>( _T("Velocity") );

        if( m_particles->get_channel_map().has_channel( _T("MBlurTime") ) &&
            !boost::math::isfinite( m_mblurTimeSeconds ) )
            timeAccessor = m_particles->get_channel_map().get_cvt_accessor<float>( _T("MBlurTime") );

        vector3f pNear = posAccessor.get( *m_curIt ) + timeAccessor.get( *m_curIt ) * velAccessor.get( *m_curIt );
        vector3f pFar = posAccessor.get( *( m_endIt - 1 ) ) +
                        timeAccessor.get( *( m_endIt - 1 ) ) * velAccessor.get( *( m_endIt - 1 ) );

        nearSlice = (int)ceilf( coordSys->transform_from_world( pNear ).z ) + m_drawFilter->get_radius();
        farSlice = (int)floorf( coordSys->transform_from_world( pFar ).z ) - m_drawFilter->get_radius();
    }

    return std::make_pair( nearSlice, farSlice );
}

bool particle_data_source::do_step( data_ptr dataSlice ) {
    dataSlice->clear();

    if( m_curIt == m_endIt )
        return false;

    int filterRadius = m_drawFilter->get_radius();
    float filterRadiusF = (float)filterRadius;

    frantic::channels::channel_accessor<vector3f> posAccessor =
        m_particles->get_channel_map().get_accessor<vector3f>( _T("Position") );
    frantic::channels::channel_cvt_accessor<vector3f> velAccessor( vector3f( 0 ) );
    frantic::channels::channel_cvt_accessor<float> timeAccessor( m_mblurTimeSeconds );

    if( m_particles->get_channel_map().has_channel( _T("Velocity") ) )
        velAccessor = m_particles->get_channel_map().get_cvt_accessor<vector3f>( _T("Velocity") );

    if( m_particles->get_channel_map().has_channel( _T("MBlurTime") ) )
        timeAccessor = m_particles->get_channel_map().get_cvt_accessor<float>( _T("MBlurTime") );

    vector3f p = m_coordSys->transform_from_world( posAccessor.get( *m_curIt ) +
                                                   timeAccessor.get( *m_curIt ) * velAccessor.get( *m_curIt ) );
    while( p.z > filterRadiusF ) {
        if( ++m_curIt == m_endIt )
            return false;
        p = m_coordSys->transform_from_world( posAccessor.get( *m_curIt ) +
                                              timeAccessor.get( *m_curIt ) * velAccessor.get( *m_curIt ) );
    }

    particle_type::const_iterator it = m_curIt;

    frantic::channels::property_map pmap;
    dataSlice->construct_sample( pmap );

    frantic::channels::channel_map_adaptor adaptor( pmap.get_channel_map(), m_particles->get_channel_map() );

    int filterArea = 4 * filterRadius * filterRadius;

    int* indices = (int*)alloca( sizeof( int ) * filterArea );
    float* weights = (float*)alloca( sizeof( float ) * filterArea );

    while( p.z > -filterRadiusF ) {
        // Adapt the particle to the voxel format.
        adaptor.copy_structure( (char*)pmap.get_raw_buffer(), *it );

        // Calculate the voxels that are covered by this particle splat, and the offset of the center for the filter.
        float floorX = floorf( p.x );
        float alphaX = p.x - floorX;
        int x = (int)floorX - filterRadius + 1;

        float floorY = floorf( p.y );
        float alphaY = p.y - floorY;
        int y = (int)floorY - filterRadius + 1;

        // Compute the filter weights
        m_drawFilter->get_weights( vector3f( alphaX, alphaY, p.z ), weights );

        // g_voxelIndexCreation.enter();
        // Allocate and retrieve space for the voxels we are about to write to.
        dataSlice->get_voxel_indices_for_write( x, y, 2 * filterRadius, indices );
        // g_voxelIndexCreation.exit();

        for( int i = 0; i < filterArea; ++i ) {
            if( weights[i] > 0 )
                dataSlice->add_to_voxel_by_weight( indices[i], pmap, weights[i] );
        }

        if( ++it == m_endIt )
            break;
        p = m_coordSys->transform_from_world( posAccessor.get( *it ) +
                                              timeAccessor.get( *it ) * velAccessor.get( *it ) );
    }

    return true;
}

} // namespace voxel_renderer
} // namespace krakatoa
