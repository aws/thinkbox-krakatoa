// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <boost/lexical_cast.hpp>
#include <krakatoa/splat_renderer/filter2f.hpp>

namespace krakatoa {
namespace splat_renderer {

class nearest_neighbour_filter2f : public filter2f {
  public:
    virtual int get_width() const { return 1; }

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) {
        outPixel.x = (int)std::floor( screenPt.x );
        outPixel.y = (int)std::floor( screenPt.y );
        outWeights[0] = 1;
    }
};

class bilinear_filter2f : public filter2f {
  public:
    virtual int get_width() const { return 2; }

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) {
        float floorX = std::floor( screenPt.x - 0.5f );
        float alphaX = screenPt.x - 0.5f - floorX;
        outPixel.x = (int)floorX;

        float floorY = std::floor( screenPt.y - 0.5f );
        float alphaY = screenPt.y - 0.5f - floorY;
        outPixel.y = (int)floorY;

        float xy = alphaX * alphaY;

        outWeights[0] = 1 - alphaX - alphaY + xy; //(1-offset.x) * (1-offset.y)
        outWeights[1] = alphaX - xy;              // offset.x * (1-offset.y)
        outWeights[2] = alphaY - xy;              //(1-offset.x) * offset.y
        outWeights[3] = xy;                       // offset.x * offset.y
    }
};

class bilinear2_filter2f : public filter2f {
    int m_radius;

  public:
    bilinear2_filter2f( int radius )
        : m_radius( radius ) {}

    virtual int get_width() const { return 2 * m_radius; }

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) {
        float floorX = std::floor( screenPt.x - 0.5f );
        float alphaX = screenPt.x - 0.5f - floorX;
        outPixel.x = (int)floorX - m_radius + 1;

        float floorY = std::floor( screenPt.y - 0.5f );
        float alphaY = screenPt.y - 0.5f - floorY;
        outPixel.y = (int)floorY - m_radius + 1;

        float* weights = outWeights;

        // Compensate to ensure total weight == 1.0
        // int(-r,r){r - |x-a|}dx = int(-r,0){r+x-a}dx  +  int(0,r){r-x+a}dx
        //                        = (rx+xx/2-ax)|-r,0   +  (rx-xx/2+ax)|0,r
        //                        = rr-rr/2-ar          +  rr-rr/2+ar
        //                        = rr
        float denom = static_cast<float>( m_radius * m_radius );
        denom *= denom;

        for( int y = -m_radius + 1; y <= m_radius; ++y ) {
            float weightY = ( static_cast<float>( m_radius ) - fabsf( (float)y - alphaY ) );

            for( int x = -m_radius + 1; x <= m_radius; ++x, ++weights ) {
                float weightXY = ( static_cast<float>( m_radius ) - fabsf( (float)x - alphaX ) ) * weightY;

                *weights = weightXY / denom;
            }
        }
    }
};

class bicubic_filter2f : public filter2f {
  public:
    virtual int get_width() const { return 3; }

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) {
        float floorX = std::floor( screenPt.x );
        float alphaX = screenPt.x - floorX;
        outPixel.x = (int)floorX - 1;

        float floorY = std::floor( screenPt.y );
        float alphaY = screenPt.y - floorY;
        outPixel.y = (int)floorY - 1;

        float x0 = 0.5f * ( 1.f - alphaX ) * ( 1.f - alphaX );
        float x2 = 0.5f * alphaX * alphaX;
        float x1 = x0 + x2 + 2 * alphaX * ( 1.f - alphaX );

        float y0 = 0.5f * ( 1.f - alphaY ) * ( 1.f - alphaY );
        float y2 = 0.5f * alphaY * alphaY;
        float y1 = y0 + y2 + 2 * alphaY * ( 1.f - alphaY );

        outWeights[0] = x0 * y0;
        outWeights[1] = x1 * y0;
        outWeights[2] = x2 * y0;
        outWeights[3] = x0 * y1;
        outWeights[4] = x1 * y1;
        outWeights[5] = x2 * y1;
        outWeights[6] = x0 * y2;
        outWeights[7] = x1 * y2;
        outWeights[8] = x2 * y2;
    }
};

class bicubic2_filter2f : public filter2f {
  public:
    virtual int get_width() const { return 4; }

    virtual void do_filter( frantic::graphics2d::vector2f screenPt, frantic::graphics2d::vector2& outPixel,
                            float outWeights[] ) {
        float floorX = std::floor( screenPt.x - 0.5f );
        float alphaX = screenPt.x - 0.5f - floorX;
        outPixel.x = (int)floorX - 1;

        float floorY = std::floor( screenPt.y - 0.5f );
        float alphaY = screenPt.y - 0.5f - floorY;
        outPixel.y = (int)floorY - 1;

        struct impl {
            inline static float filter1( float x ) {
                float xx = x * x;
                return ( 3 * x * xx - 6 * xx + 4 ) / 6.f;
            }

            inline static float filter2( float x ) {
                float xx = x * x;
                return ( -x * xx + 6 * xx - 12 * x + 8 ) / 6.f;
            }
        };

        float xWeights[] = { impl::filter2( 2.f - alphaX ), impl::filter1( 1.f - alphaX ), impl::filter1( alphaX ),
                             impl::filter2( 1.f + alphaX ) };
        float yWeights[] = { impl::filter2( 2.f - alphaY ), impl::filter1( 1.f - alphaY ), impl::filter1( alphaY ),
                             impl::filter2( 1.f + alphaY ) };

        outWeights[0] = xWeights[0] * yWeights[0];
        outWeights[1] = xWeights[1] * yWeights[0];
        outWeights[2] = xWeights[2] * yWeights[0];
        outWeights[3] = xWeights[3] * yWeights[0];

        outWeights[4] = xWeights[0] * yWeights[1];
        outWeights[5] = xWeights[1] * yWeights[1];
        outWeights[6] = xWeights[2] * yWeights[1];
        outWeights[7] = xWeights[3] * yWeights[1];

        outWeights[8] = xWeights[0] * yWeights[2];
        outWeights[9] = xWeights[1] * yWeights[2];
        outWeights[10] = xWeights[2] * yWeights[2];
        outWeights[11] = xWeights[3] * yWeights[2];

        outWeights[12] = xWeights[0] * yWeights[3];
        outWeights[13] = xWeights[1] * yWeights[3];
        outWeights[14] = xWeights[2] * yWeights[3];
        outWeights[15] = xWeights[3] * yWeights[3];
    }
};

filter2f_ptr filter2f::create_instance( const frantic::tstring& filter ) {
    filter2f_ptr result;
    if( filter == _T("Nearest Neighbor") ) {
        result.reset( new nearest_neighbour_filter2f );
    } else if( filter == _T("Bilinear") ) {
        result.reset( new bilinear_filter2f );
    } else if( filter == _T("Bicubic") ) {
        // result.reset( new bicubic_filter2f );
        result.reset( new bicubic2_filter2f );
    } else if( filter.substr( 0, 8 ) == _T("Bilinear") ) {
        int radius = boost::lexical_cast<int>( filter.substr( 8 ) );
        if( radius <= 0 )
            throw std::runtime_error( "Invalid radius for Bilinear filter: " +
                                      frantic::strings::to_string( filter.substr( 8 ) ) );
        else if( radius == 1 )
            result.reset( new bilinear_filter2f );
        else
            result.reset( new bilinear2_filter2f( radius ) );
    } else {
        throw std::runtime_error(
            "splat_renderer::filter2f::create_instance() Attempted to set invalid draw point filter, \"" +
            frantic::strings::to_string( filter ) + "\"." );
    }

    return result;
}

} // namespace splat_renderer
} // namespace krakatoa
