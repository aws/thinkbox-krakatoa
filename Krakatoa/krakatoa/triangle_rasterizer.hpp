// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/camera.hpp>
#include <frantic/graphics/plane3f.hpp>
#include <frantic/graphics/vector3f.hpp>

#include <frantic/graphics2d/framebuffer.hpp>

namespace krakatoa {

struct clipped_triangle {
    frantic::graphics::vector3f verts[3];
    frantic::graphics2d::vector2f screenVerts[3];

    void init() {}

    void reset() {}

    void swap_verts( int v0, int v1 ) {
        std::swap( verts[v0], verts[v1] );
        std::swap( screenVerts[v0], screenVerts[v1] );
    }
};

/**
 * A container class for the primary function raster_triangle() that will scanline rasterize a triangle. This is
 * implemented as a class instead of a free function in order to only have to compute various properties once when
 * rasterizing multiple triangles.
 *
 * @tparam PixelShaderType the type abstracts away the actual image being rasterized to. It is required to implement
 * these functions: int get_buffer_width(); int get_buffer_width(); void new_scan( int y, float leftAlpha, float
 * rightAlpha ); void evaluate( int x, float alpha ); These functions will be called by
 * triangle_rasterizer::raster_triangle(). new_scan() is called once per scanline. evaluate() is called once per pixel.
 *
 *                         PixelShaderType is required to implement these types:
 *                          triangle_type <- Must be interface consistent or a sub-class of clipped_triangle.
 */
template <class PixelShaderType>
class triangle_rasterizer {
    const frantic::graphics::camera<float>&
        m_camera; // NOTE: This reference means your camera better be defined in a longer lasting scope!!!!

    float m_pixelScale;
    frantic::graphics::plane3f m_clipPlanes[5]; // TODO: support far clipping.
    frantic::graphics::transform4f m_worldToCameraTM;

    typedef typename PixelShaderType::triangle_type triangle_type;

    // Storage for the initial triangle and others possibly created by clipping to the camera frustrum. As a back of the
    // envelop calculation I am assuming at most 2^5 = 32 triangles can be generated.
    triangle_type m_triangles[32];

  private:
    int clip_triangle_to_frustrum();

    void do_raster_triangle( PixelShaderType& pxShader, triangle_type& tri );

    void do_raster_triangle_piece( PixelShaderType& pxShader, triangle_type& tri, int startScan, int endScan );

    triangle_rasterizer& operator=( const triangle_rasterizer& rhs );

  public:
    triangle_rasterizer( const frantic::graphics::camera<float>& renderCamera, float time = 0.5f );

    void set_object_to_world_transform( const frantic::graphics::transform4f& xform );

    void set_pixel_scale( float pixelScale );

    void raster_triangle( PixelShaderType& pxShader, const frantic::graphics::vector3f& p0,
                          const frantic::graphics::vector3f& p1, const frantic::graphics::vector3f& p2 );
};

/**
 * BEGIN IMPLEMENTATION
 */
template <class PixelShaderType>
triangle_rasterizer<PixelShaderType>::triangle_rasterizer( const frantic::graphics::camera<float>& renderCam,
                                                           float time )
    : m_camera( renderCam )
    , m_pixelScale( 1.f ) {
    m_worldToCameraTM = renderCam.world_transform_inverse( time );

    frantic::graphics::plane3f clipPlanes[5];
    m_clipPlanes[0] = frantic::graphics::plane3f( 0, 0, 1, renderCam.near_distance() );

    if( renderCam.projection_mode() != frantic::graphics::projection_mode::orthographic ) {
        float halfHorizontalFOV = 0.5f * renderCam.horizontal_fov();
        float sinHalfHorizontalFOV = sin( halfHorizontalFOV );
        float cosHalfHorizontalFOV = cos( halfHorizontalFOV );

        float halfVerticalFOV = 0.5f * renderCam.vertical_fov();
        float sinHalfVerticalFOV = sin( halfVerticalFOV );
        float cosHalfVerticalFOV = cos( halfVerticalFOV );

        m_clipPlanes[1] = frantic::graphics::plane3f( cosHalfHorizontalFOV, 0, sinHalfHorizontalFOV, 1e-3f );
        m_clipPlanes[2] = frantic::graphics::plane3f( -cosHalfHorizontalFOV, 0, sinHalfHorizontalFOV, 1e-3f );
        m_clipPlanes[3] = frantic::graphics::plane3f( 0, cosHalfVerticalFOV, sinHalfVerticalFOV, 1e-3f );
        m_clipPlanes[4] = frantic::graphics::plane3f( 0, -cosHalfVerticalFOV, sinHalfVerticalFOV, 1e-3f );
    } else {
        float aspect = (float)renderCam.get_output_size().ysize / (float)renderCam.get_output_size().xsize;
        float orthoHalfWidth = 0.5f * renderCam.orthographic_width() + 1e-3f;
        float orthoHeightHalf = 0.5f * renderCam.orthographic_width() * aspect + 1e-3f;

        m_clipPlanes[1] = frantic::graphics::plane3f( 1, 0, 0, orthoHalfWidth );
        m_clipPlanes[2] = frantic::graphics::plane3f( -1, 0, 0, orthoHalfWidth );
        m_clipPlanes[3] = frantic::graphics::plane3f( 0, 1, 0, orthoHeightHalf );
        m_clipPlanes[4] = frantic::graphics::plane3f( 0, -1, 0, orthoHeightHalf );
    }
}

template <class PixelShaderType>
void triangle_rasterizer<PixelShaderType>::set_object_to_world_transform(
    const frantic::graphics::transform4f& xform ) {
    m_worldToCameraTM = m_worldToCameraTM * xform;
}

template <class PixelShaderType>
void triangle_rasterizer<PixelShaderType>::set_pixel_scale( float pixelScale ) {
    m_pixelScale = pixelScale;
}

template <class PixelShaderType>
void triangle_rasterizer<PixelShaderType>::raster_triangle( PixelShaderType& pxShader,
                                                            const frantic::graphics::vector3f& p0,
                                                            const frantic::graphics::vector3f& p1,
                                                            const frantic::graphics::vector3f& p2 ) {
    bool isValid;

    m_triangles[0].reset();
    m_triangles[0].verts[0] = m_worldToCameraTM * p0;
    m_triangles[0].verts[1] = m_worldToCameraTM * p1;
    m_triangles[0].verts[2] = m_worldToCameraTM * p2;

    int numTriangles = clip_triangle_to_frustrum();

    triangle_type* pCurTriangle = m_triangles;
    for( int k = 0; k < numTriangles; ++k, ++pCurTriangle ) {
        pCurTriangle->screenVerts[0] =
            m_pixelScale * m_camera.from_cameraspace_position( pCurTriangle->verts[0], ( isValid = true ) );
        pCurTriangle->screenVerts[1] =
            m_pixelScale * m_camera.from_cameraspace_position( pCurTriangle->verts[1], ( isValid = true ) );
        pCurTriangle->screenVerts[2] =
            m_pixelScale * m_camera.from_cameraspace_position( pCurTriangle->verts[2], ( isValid = true ) );

        pCurTriangle->init();

        if( pCurTriangle->screenVerts[1].y < pCurTriangle->screenVerts[0].y &&
            pCurTriangle->screenVerts[1].y <= pCurTriangle->screenVerts[2].y )
            pCurTriangle->swap_verts( 0, 1 );
        else if( pCurTriangle->screenVerts[2].y < pCurTriangle->screenVerts[0].y &&
                 pCurTriangle->screenVerts[2].y <= pCurTriangle->screenVerts[1].y )
            pCurTriangle->swap_verts( 0, 2 );
        if( pCurTriangle->screenVerts[2].y < pCurTriangle->screenVerts[1].y )
            pCurTriangle->swap_verts( 1, 2 );

        do_raster_triangle( pxShader, *pCurTriangle );
    }
}

template <class PixelShaderType>
int triangle_rasterizer<PixelShaderType>::clip_triangle_to_frustrum() {
    using frantic::graphics::vector3f;

    int numTriangles = 1;

    for( int i = 0; i < 5; ++i ) {
        triangle_type* pTri = m_triangles;
        triangle_type* pOutTri = m_triangles + numTriangles;

        while( pTri != pOutTri ) {
            float kVert0 = m_clipPlanes[i].get_signed_distance_to_plane( pTri->verts[0] );
            float kVert1 = m_clipPlanes[i].get_signed_distance_to_plane( pTri->verts[1] );
            float kVert2 = m_clipPlanes[i].get_signed_distance_to_plane( pTri->verts[2] );

            if( kVert0 > 0 ) {
                if( kVert1 > 0 ) {
                    if( kVert2 > 0 ) {
                        // All verts are outside clipping plane, so cull this triangle. TODO: This can cause double
                        // processing of newly created triangles.
                        --numTriangles;
                        *pTri = *( --pOutTri );
                    } else {
                        // Verts 0 and 1 are outside so clip both to plane
                        float t0 = ( m_clipPlanes[i].constant() - kVert2 ) / ( kVert0 - kVert2 );
                        float t1 = ( m_clipPlanes[i].constant() - kVert2 ) / ( kVert1 - kVert2 );
                        pTri->verts[0] = pTri->verts[2] + t0 * ( pTri->verts[0] - pTri->verts[2] );
                        pTri->verts[1] = pTri->verts[2] + t1 * ( pTri->verts[1] - pTri->verts[2] );
                        // pTri->baryCoords[0] = pTri->baryCoords[2] + t0 * ( pTri->baryCoords[0] - pTri->baryCoords[2]
                        // ); pTri->baryCoords[1] = pTri->baryCoords[2] + t1 * ( pTri->baryCoords[1] -
                        // pTri->baryCoords[2] );
                        ++pTri;
                    }
                } else if( kVert2 > 0 ) {
                    // Verts 0 and Verts 2 are outside so clip both to plane
                    float t0 = ( m_clipPlanes[i].constant() - kVert1 ) / ( kVert0 - kVert1 );
                    float t2 = ( m_clipPlanes[i].constant() - kVert1 ) / ( kVert2 - kVert1 );
                    pTri->verts[0] = pTri->verts[1] + t0 * ( pTri->verts[0] - pTri->verts[1] );
                    pTri->verts[2] = pTri->verts[1] + t2 * ( pTri->verts[2] - pTri->verts[1] );
                    // pTri->baryCoords[0] = pTri->baryCoords[1] + t0 * ( pTri->baryCoords[0] - pTri->baryCoords[1] );
                    // pTri->baryCoords[2] = pTri->baryCoords[1] + t2 * ( pTri->baryCoords[2] - pTri->baryCoords[1] );
                    ++pTri;
                } else {
                    // Vert 0 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal
                    // edge to prevent sliver triangles.
                    float t1 = ( m_clipPlanes[i].constant() - kVert1 ) / ( kVert0 - kVert1 );
                    float t2 = ( m_clipPlanes[i].constant() - kVert2 ) / ( kVert0 - kVert2 );

                    vector3f e01 = ( pTri->verts[0] - pTri->verts[1] );
                    vector3f e20 = ( pTri->verts[0] - pTri->verts[2] );
                    vector3f p01 = pTri->verts[1] + t1 * e01;
                    vector3f p20 = pTri->verts[2] + t2 * e20;

                    // vector3f bc01 = pTri->baryCoords[1] + t1 * (pTri->baryCoords[0] - pTri->baryCoords[1]);
                    // vector3f bc20 = pTri->baryCoords[2] + t2 * (pTri->baryCoords[0] - pTri->baryCoords[2]);

                    // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make
                    // better triangles.
                    if( e01.get_magnitude_squared() >= e20.get_magnitude_squared() ) {
                        // p01, p1, p2 and p01, p2, p20
                        pOutTri->verts[0] = pTri->verts[0] = p01;
                        pOutTri->verts[1] = pTri->verts[2];
                        pOutTri->verts[2] = p20;
                        // pOutTri->baryCoords[0] = pTri->baryCoords[0] = bc01;
                        // pOutTri->baryCoords[1] = pTri->baryCoords[2];
                        // pOutTri->baryCoords[2] = bc20;
                    } else {
                        // p20, p1, p2 and p20, p01, p1
                        pOutTri->verts[0] = pTri->verts[0] = p20;
                        pOutTri->verts[1] = p01;
                        pOutTri->verts[2] = pTri->verts[1];
                        // pOutTri->baryCoords[0] = pTri->baryCoords[0] = bc20;
                        // pOutTri->baryCoords[1] = bc01;
                        // pOutTri->baryCoords[2] = pTri->baryCoords[1];
                    }
                    ++numTriangles;
                    ++pOutTri;
                    ++pTri;
                }
            } else if( kVert1 > 0 ) {
                if( kVert2 > 0 ) {
                    // Verts 1 and 2 are outside so move them to the plane
                    float t1 = ( m_clipPlanes[i].constant() - kVert0 ) / ( kVert1 - kVert0 );
                    float t2 = ( m_clipPlanes[i].constant() - kVert0 ) / ( kVert2 - kVert0 );
                    pTri->verts[1] = pTri->verts[0] + t1 * ( pTri->verts[1] - pTri->verts[0] );
                    pTri->verts[2] = pTri->verts[0] + t2 * ( pTri->verts[2] - pTri->verts[0] );
                    // pTri->baryCoords[1] = pTri->baryCoords[0] + t1 * ( pTri->baryCoords[1] - pTri->baryCoords[0] );
                    // pTri->baryCoords[2] = pTri->baryCoords[0] + t2 * ( pTri->baryCoords[2] - pTri->baryCoords[0] );
                    ++pTri;
                } else {
                    // Vert 1 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal
                    // edge to prevent sliver triangles.
                    float t0 = ( m_clipPlanes[i].constant() - kVert0 ) / ( kVert1 - kVert0 );
                    float t2 = ( m_clipPlanes[i].constant() - kVert2 ) / ( kVert1 - kVert2 );

                    vector3f e01 = ( pTri->verts[1] - pTri->verts[0] );
                    vector3f e12 = ( pTri->verts[1] - pTri->verts[2] );
                    vector3f p01 = pTri->verts[0] + t0 * e01;
                    vector3f p12 = pTri->verts[2] + t2 * e12;

                    // vector3f bc01 = pTri->baryCoords[0] + t0 * (pTri->baryCoords[1] - pTri->baryCoords[0]);
                    // vector3f bc12 = pTri->baryCoords[2] + t2 * (pTri->baryCoords[1] - pTri->baryCoords[2]);

                    // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make
                    // better triangles.
                    if( e01.get_magnitude_squared() >= e12.get_magnitude_squared() ) {
                        // p0, p01, p2 and p01, p12, p2
                        pOutTri->verts[0] = pTri->verts[1] = p01;
                        pOutTri->verts[1] = p12;
                        pOutTri->verts[2] = pTri->verts[2];
                        // pOutTri->baryCoords[0] = pTri->baryCoords[1] = bc01;
                        // pOutTri->baryCoords[1] = bc12;
                        // pOutTri->baryCoords[2] = pTri->baryCoords[2];
                    } else {
                        // p0, p12, p2 and p0, p01, p12
                        pOutTri->verts[0] = pTri->verts[0];
                        pOutTri->verts[1] = p01;
                        pOutTri->verts[2] = pTri->verts[1] = p12;
                        // pOutTri->baryCoords[0] = pTri->baryCoords[0];
                        // pOutTri->baryCoords[1] = bc01;
                        // pOutTri->baryCoords[2] = pTri->baryCoords[1] = bc12;
                    }
                    ++numTriangles;
                    ++pOutTri;
                    ++pTri;
                }
            } else if( kVert2 > 0 ) {
                // Vert 2 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal edge
                // to prevent sliver triangles.
                float t0 = ( m_clipPlanes[i].constant() - kVert0 ) / ( kVert2 - kVert0 );
                float t1 = ( m_clipPlanes[i].constant() - kVert1 ) / ( kVert2 - kVert1 );

                vector3f e20 = ( pTri->verts[2] - pTri->verts[0] );
                vector3f e12 = ( pTri->verts[2] - pTri->verts[1] );
                vector3f p20 = pTri->verts[0] + t0 * e20;
                vector3f p12 = pTri->verts[1] + t1 * e12;

                // vector3f bc20 = pTri->baryCoords[0] + t0 * (pTri->baryCoords[2] - pTri->baryCoords[0]);
                // vector3f bc12 = pTri->baryCoords[1] + t1 * (pTri->baryCoords[2] - pTri->baryCoords[1]);

                // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make better
                // triangles.
                if( e20.get_magnitude_squared() >= e12.get_magnitude_squared() ) {
                    // p0, p1, p20 and p20, p1, p12
                    pOutTri->verts[0] = pTri->verts[2] = p20;
                    pOutTri->verts[1] = pTri->verts[1];
                    pOutTri->verts[2] = p12;
                    // pOutTri->baryCoords[0] = pTri->baryCoords[2] = bc20;
                    // pOutTri->baryCoords[1] = pTri->baryCoords[1];
                    // pOutTri->baryCoords[2] = bc12;
                } else {
                    // p0, p1, p12 and p0, p12, p20
                    pOutTri->verts[0] = pTri->verts[0];
                    pOutTri->verts[1] = pTri->verts[2] = p12;
                    pOutTri->verts[2] = p20;
                    // pOutTri->baryCoords[0] = pTri->baryCoords[0];
                    // pOutTri->baryCoords[1] = pTri->baryCoords[2] = bc12;
                    // pOutTri->baryCoords[2] = bc20;
                }
                ++numTriangles;
                ++pOutTri;
                ++pTri;
            } else {
                ++pTri;
            }
        }
    }

    return numTriangles;
}

template <class PixelShaderType>
void triangle_rasterizer<PixelShaderType>::do_raster_triangle( PixelShaderType& pxShader, triangle_type& tri ) {
    using namespace frantic::graphics;

    int height = pxShader.get_buffer_height();

    int pixelStart = (int)std::ceil( tri.screenVerts[0].y );
    int pixelMiddle = (int)std::ceil( tri.screenVerts[1].y );
    int pixelEnd = (int)std::ceil( tri.screenVerts[2].y );

    if( pixelStart >= height || pixelEnd <= 0 )
        return;

    // do_raster_triangle_piece() uses an open y interval, so this sequence of clamps will include a horizontal edge
    // along the bottom of a triangle, but not along the top. This is a good thing because then two connected triangles
    // that share a horizontal edge will only write to the "shared" scanline once.
    pixelStart = frantic::math::clamp( pixelStart, 0, height - 1 );
    pixelMiddle = frantic::math::clamp( pixelMiddle, 0, height - 1 );
    pixelEnd = frantic::math::clamp( pixelEnd, 0, height );

    // Determine if the cross product of the edge has negative, positive or zero in the z direction. We want a negative
    // z orientation.
    float zCross1 = ( tri.screenVerts[1].x - tri.screenVerts[0].x ) * ( tri.screenVerts[2].y - tri.screenVerts[0].y );
    float zCross2 = ( tri.screenVerts[1].y - tri.screenVerts[0].y ) * ( tri.screenVerts[2].x - tri.screenVerts[0].x );

    if( zCross1 == zCross2 ) // This means the projection made a degenerate sliver, so we draw nothing.
        return;
    if( zCross1 > zCross2 ) {
        tri.swap_verts( 1, 2 );
    }

    do_raster_triangle_piece( pxShader, tri, pixelStart, pixelMiddle );
    if( pixelMiddle != pixelEnd ) {
        if( zCross1 > zCross2 ) {
            tri.swap_verts( 0, 1 );
        } else {
            tri.swap_verts( 0, 2 );
        }

        do_raster_triangle_piece( pxShader, tri, pixelMiddle, pixelEnd );
    }
}

template <class PixelShaderType>
void triangle_rasterizer<PixelShaderType>::do_raster_triangle_piece( PixelShaderType& pxShader, triangle_type& tri,
                                                                     int startScan, int endScan ) {
    int width = pxShader.get_buffer_width();

    float leftHeight = tri.screenVerts[1].y - tri.screenVerts[0].y;
    float leftWidth = tri.screenVerts[1].x - tri.screenVerts[0].x;
    float rightHeight = tri.screenVerts[2].y - tri.screenVerts[0].y;
    float rightWidth = tri.screenVerts[2].x - tri.screenVerts[0].x;

    for( int pixelY = startScan; pixelY < endScan; ++pixelY ) {
        float yOffset = (float)pixelY - tri.screenVerts[0].y;
        float leftYInterp = yOffset / leftHeight;
        float rightYInterp = yOffset / rightHeight;

        pxShader.new_scan( pixelY, leftYInterp, rightYInterp );

        float fLeftX = tri.screenVerts[0].x + leftYInterp * leftWidth;
        float fRightX = tri.screenVerts[0].x + rightYInterp * rightWidth;
        float fWidth = fRightX - fLeftX;

        // This is using an open interval on X, so that adjacent triangles that share a vertical edge (exactly on a
        // pixel coordinate) will only draw on the shared edge once. ie. horizontal edges on the bottom and vertical
        // edges on the left are drawn.
        int leftX = (int)std::ceil( fLeftX );
        int rightX = (int)std::ceil( fRightX );
        if( leftX < 0 )
            leftX = 0;
        if( rightX > width )
            rightX = width;

        for( int pixelX = leftX; pixelX < rightX; ++pixelX ) {
            float xInterp = ( (float)pixelX - fLeftX ) / fWidth;

            pxShader.evaluate( pixelX, xInterp );
        } // for(int pixelX = leftX; pixelX <= rightX; ++pixelX)
    }     // for(int pixelY = startScan; pixelY < endScan; ++pixelY)
}

} // namespace krakatoa
