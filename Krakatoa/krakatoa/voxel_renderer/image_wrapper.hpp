// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace krakatoa {
namespace voxel_renderer {

template <class PixelType>
class image_wrapper {
    frantic::graphics2d::size2 m_size;
    PixelType* m_pData;
    bool m_ownsData;

  private:
    void reset( const frantic::graphics2d::size2 imageSize ) {
        if( m_ownsData )
            delete m_pData;
        m_size = imageSize;
        m_ownsData = true;
        m_pData = new PixelType[imageSize.get_area()];

        // for( int i = 0, iEnd = imageSize.get_area(); i < iEnd; ++i )
        //	m_pData[i] = PixelType();
    }

  public:
    image_wrapper( const frantic::graphics2d::size2 imageSize )
        : m_ownsData( false )
        , m_pData( NULL ) {
        reset( imageSize );
    }

    template <class ImageType>
    explicit image_wrapper( ImageType& fb )
        : m_size( fb.size() )
        , m_pData( &fb.data()[0] )
        , m_ownsData( false ) {}

    ~image_wrapper() {
        if( m_ownsData )
            delete m_pData;
    }

    void swap( image_wrapper& rhs ) {
        std::swap( m_size, rhs.m_size );
        std::swap( m_pData, rhs.m_pData );
        std::swap( m_ownsData, rhs.m_ownsData );
    }

    frantic::graphics2d::size2 size() const { return m_size; }
    int width() const { return m_size.xsize; }
    int height() const { return m_size.ysize; }

    void set_pixel( int x, int y, const PixelType& p ) { m_pData[x + m_size.xsize * y] = p; }

    const PixelType& get_pixel( int x, int y ) const { return m_pData[x + m_size.xsize * y]; }

    void blend_over( int x, int y, const PixelType& p ) { m_pData[x + m_size.xsize * y].blend_over( p ); }

    void blend_under( int x, int y, const PixelType& p ) { m_pData[x + m_size.xsize * y].blend_under( p ); }

    void blend_over( const image_wrapper& rhs ) {
        for( int i = 0, iEnd = m_size.get_area(); i < iEnd; ++i )
            m_pData[i].blend_over( rhs.m_pData[i] );
    }

    void blend_under( const image_wrapper& rhs ) {
        for( int i = 0, iEnd = m_size.get_area(); i < iEnd; ++i )
            m_pData[i].blend_under( rhs.m_pData[i] );
    }
};

} // namespace voxel_renderer
} // namespace krakatoa
