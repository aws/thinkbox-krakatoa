// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/channels/channel_map.hpp>

// This makes us need to include tbbmalloc.dll with produced binaries. That's not ideal.
//#include <tbb/scalable_allocator.h>

namespace krakatoa {
namespace voxel_renderer {

class sample {
    const frantic::channels::channel_map* m_pMap;
    char* m_data;

  public:
    sample()
        : m_pMap( NULL )
        , m_data( NULL ) {}

    ~sample() {
        if( m_data )
            delete m_data;
        // scalable_free( m_data );
    }

    void clear() { memset( m_data, 0, m_pMap->structure_size() ); }

    void reset( const frantic::channels::channel_map& pcm ) {
        if( m_data )
            delete m_data;
        // scalable_free( m_data );
        m_pMap = &pcm;
        m_data = new char[m_pMap->structure_size()];
        // m_data = (char*)scalable_malloc( m_pMap->structure_size() );
    }

    const frantic::channels::channel_map& get_channel_map() const { return *m_pMap; }

    char* get_raw_buffer() { return m_data; }

    const char* get_raw_buffer() const { return m_data; }

    bool has_property( const frantic::tstring& propertyName ) const { return m_pMap->has_channel( propertyName ); }

    template <class T>
    T& get( const frantic::tstring& propertyName ) {
        return m_pMap->get_accessor<T>( propertyName ).get( m_data );
    }
};

} // namespace voxel_renderer
} // namespace krakatoa
