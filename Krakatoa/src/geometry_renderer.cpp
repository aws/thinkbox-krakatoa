// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <boost/bind.hpp>
#include <krakatoa/geometry_renderer.hpp>
#include <krakatoa/scene_context.hpp>

#include <frantic/graphics/cubeface.hpp>
#include <frantic/graphics/plane3f.hpp>
#include <frantic/graphics/vector4f.hpp>

#pragma warning( push, 3 )
#include <boost/function.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#pragma warning( pop )

using namespace frantic::geometry;
using namespace frantic::graphics;
using namespace frantic::graphics2d;
using frantic::rendering::framebuffer_cubeface;

typedef boost::function<float( const raytrace_intersection& )> opacity_shader;

namespace krakatoa {

//
//
// Implementation for base matte_interface
//
//

void matte_interface::generate_depth_map( const frantic::graphics::camera<float>& renderCam, float mblurTime,
                                          frantic::graphics2d::framebuffer<float>& outDepthImg, bool forLight ) {
    using namespace frantic::graphics;
    using namespace frantic::graphics2d;

    if( !m_context )
        throw std::runtime_error( "matte_interface::generate_depth_map() - No scene context available" );

    const krakatoa::scene_context::matte_collection& matteCollection = m_context->get_matte_objects();

    // Check that the framebuffer and camera have coherent sizes.
    size2 camSize = renderCam.get_output_size();
    size2 imgSize = outDepthImg.size();

    if( ( imgSize.xsize % camSize.xsize ) != 0 )
        throw std::runtime_error( "matte_interface::generate_depth_map() - The requested image width is not a valid "
                                  "supersample of the camera" );
    if( ( imgSize.ysize % camSize.ysize ) != 0 )
        throw std::runtime_error( "matte_interface::generate_depth_map() - The requested image height is not a valid "
                                  "supersample of the camera" );

    int xSuperSample = imgSize.xsize / camSize.xsize;
    int ySuperSample = imgSize.ysize / camSize.ysize;
    if( xSuperSample != ySuperSample )
        throw std::runtime_error(
            "matte_interface::generate_depth_map() - The requested supersampling rate is not uniform" );

    if( renderCam.projection_mode() == projection_mode::perspective ||
        renderCam.projection_mode() == projection_mode::orthographic ) {
        FF_LOG( debug ) << _T("Rasterizing depth map") << std::endl;

        // maybe the matte_interface should hold copies of the raster_depth_map_renderer and raytrace_depth_map_renderer
        // objects, so only "rebuild" is necc.

        raster_depth_map_renderer rasterRenderer;
        for( int i = 0, iEnd = (int)matteCollection.size(); i < iEnd; ++i ) {
            if( ( forLight && matteCollection.get( i )->is_visible_to_lights() ) ||
                ( !forLight && matteCollection.get( i )->is_visible_to_cameras() ) ) {
                matteCollection.get( i )->set_time( mblurTime );
                rasterRenderer.add_primitive( matteCollection.get( i ) );
            }
        }

        if( frantic::logging::is_logging_stats() ) {
            std::size_t numFaces = 0;
            for( std::size_t i = 0, iEnd = matteCollection.size(); i < iEnd; ++i )
                numFaces += matteCollection.get( (int)i )->get_triangle_count();
            frantic::logging::stats << _T("Rendering ") << numFaces << _T(" triangles from ") << matteCollection.size()
                                    << _T(" meshes") << std::endl;
        }

        rasterRenderer.rasterize_depthmap( renderCam, outDepthImg, m_numThreads );
    } else {
        FF_LOG( debug ) << _T("Raytracing depth map") << std::endl;

        raytrace_depth_map_renderer raytraceRenderer;
        for( int i = 0, iEnd = (int)matteCollection.size(); i < iEnd; ++i ) {
            matteCollection.get( i )->set_time( mblurTime );
            raytraceRenderer.add_primitive( matteCollection.get( i ) );
        }

        if( frantic::logging::is_logging_stats() ) {
            std::size_t numFaces = 0;
            for( std::size_t i = 0, iEnd = matteCollection.size(); i < iEnd; ++i )
                numFaces += matteCollection.get( (int)i )->get_triangle_count();
            frantic::logging::stats << _T("Rendering ") << numFaces << _T(" triangles from ") << matteCollection.size()
                                    << _T(" meshes") << std::endl;
        }

        raytraceRenderer.raytrace_depthmap( renderCam, outDepthImg, forLight );
    }
}

//
//
// raster_depth_map_renderer: Perform rasters rendering of a collection of matte_primitive objects.
//
//

bool raster_depth_map_renderer::run_next_task( const frantic::graphics::camera<float>& renderCamera,
                                               frantic::graphics2d::framebuffer<float>& renderBuffer ) {
    krakatoa::matte_primitive* pPrimitive;
    std::size_t rangeStart;
    std::size_t rangeEnd;

    {
        boost::mutex::scoped_lock theLock( m_mutex );
        if( m_curPrimitive == m_primitives.size() )
            return false;
        pPrimitive = m_primitives[m_curPrimitive].get();
        rangeStart = m_curFace;
        rangeEnd = m_curFace + 5000;

        if( rangeEnd >= pPrimitive->get_triangle_count() ) {
            rangeEnd = pPrimitive->get_triangle_count();

            ++m_curPrimitive;
            m_curFace = 0;
        } else
            m_curFace = rangeEnd;
    }

    pPrimitive->raster_depthmap( renderCamera, rangeStart, rangeEnd, renderBuffer );
    return true;
}

void raster_depth_map_renderer::rasterize_depthmap( const frantic::graphics::camera<float>& renderCamera,
                                                    frantic::graphics2d::framebuffer<float>& renderBuffer,
                                                    int numThreads ) {
    if( numThreads <= 1 || ( m_primitives.size() == 1 && m_primitives[0]->get_triangle_count() <= 5000 ) ) {
        renderBuffer.fill( std::numeric_limits<float>::max() );
        for( std::size_t i = 0, iEnd = m_primitives.size(); i < iEnd; ++i )
            m_primitives[i]->raster_depthmap( renderCamera, renderBuffer );
    } else {
        boost::scoped_array<thread_type> threads( new thread_type[numThreads] );
        boost::thread_group threadGroup;

        for( int i = 1; i < numThreads; ++i ) {
            threads[i].renderBuffer.set_size( renderBuffer.size() );
            threads[i].renderBuffer.fill( std::numeric_limits<float>::max() );
            threads[i].renderCamera = &renderCamera;

            threadGroup.create_thread( boost::bind( &thread_type::evaluate, &threads[i], boost::ref( *this ) ) );
        }

        renderBuffer.fill( std::numeric_limits<float>::max() );
        renderBuffer.swap( threads[0].renderBuffer );
        threads[0].renderCamera = &renderCamera;
        threads[0].evaluate( *this );
        renderBuffer.swap( threads[0].renderBuffer );

        threadGroup.join_all();

        // Merge the buffers back together taking the minimum values.
        for( int i = 1; i < numThreads; ++i ) {
            for( int y = 0; y < renderBuffer.height(); ++y ) {
                for( int x = 0; x < renderBuffer.width(); ++x ) {
                    float oldZ = renderBuffer.get_pixel( x, y );
                    float newZ = threads[i].renderBuffer.get_pixel( x, y );
                    if( newZ < oldZ )
                        renderBuffer.set_pixel( x, y, newZ );
                }
            }
        }
    }
}

//
//
// raytrace_depth_map_renderer: Performs the raytracing of a collection of matte_primitive objects.
//
//

frantic::diagnostics::profiling_section g_psRebuildBasic( _T("Rebuilding Geometry") );
frantic::diagnostics::profiling_section g_psRebuildInterp( _T("Interpolating Geometry") );

void raytrace_depth_map_renderer::rebuild( float motionSegmentTime ) {
    m_srcKDTree.reset();
    m_cachedSegmentTime = motionSegmentTime;

    for( std::size_t i = 0, iEnd = m_primitives.size(); i < iEnd; ++i )
        m_primitives[i]->set_time( motionSegmentTime );
}

void raytrace_depth_map_renderer::raytrace_depthmap( const frantic::graphics::camera<float>& renderCam,
                                                     frantic::graphics2d::framebuffer<float>& renderBuffer,
                                                     bool forLight ) {
    frantic::diagnostics::profiling_section psTrees( _T("Building kd-trees") );
    frantic::diagnostics::profiling_section psRaytrace( _T("Raytracing geometry") );

    psTrees.enter();
    // Build a list of objects which are visible to this camera
    std::vector<boost::shared_ptr<krakatoa::matte_primitive>> activeObjects;

    if( !m_srcKDTree ) {
        for( std::size_t i = 0; i < m_primitives.size(); ++i )
            m_primitives[i]->build_kdtree();
    }
    for( std::size_t i = 0; i < m_primitives.size(); ++i ) {
        if( !forLight ) {
            if( m_primitives[i]->is_visible_to_cameras() )
                activeObjects.push_back( m_primitives[i] );
        } else {
            if( m_primitives[i]->is_visible_to_lights() )
                activeObjects.push_back( m_primitives[i] );
        }
    }
    m_srcKDTree.reset( new frantic::geometry::primitive_kdtree<krakatoa::matte_primitive>( activeObjects ) );
    psTrees.exit();

    frantic::graphics2d::size2 imgSize = renderBuffer.size();

    float pixelScale = float( imgSize.xsize / renderCam.get_output_size().xsize );
    vector3f viewDir = renderCam.view_direction( m_cachedSegmentTime );
    frantic::geometry::raytrace_intersection ri;

    renderBuffer.fill( std::numeric_limits<float>::max() );

    psRaytrace.enter();
    for( int y = 0; y < imgSize.ysize; ++y ) {
        for( int x = 0; x < imgSize.xsize; ++x ) {
            vector2f pixel( float( x ) / pixelScale, float( y ) / pixelScale );

            bool isValid = true;
            ray3f r = renderCam.get_worldspace_ray( pixel, m_cachedSegmentTime, isValid );

            if( m_srcKDTree->intersect_ray( r, 0, std::numeric_limits<float>::max(), ri ) ) {
                float dist = float( vector3f::dot( viewDir, r.direction() ) * ri.distance );
                renderBuffer.set_pixel( x, y, dist );
            }
        }
    }
    psRaytrace.exit();

    FF_LOG( debug ) << psTrees << std::endl << psRaytrace << std::endl;
}

//
//
// matte_primitive implementation (including helper functions).
// This primitive base object handles raytracing and rasterization.
//
//

namespace detail {

struct clipped_triangle {
    vector3f verts[3];
    vector3f baryCoords[3];
    vector2f screenVerts[3];
    float depth[3];

    void reset_barycoords() {
        baryCoords[0].set( 1, 0, 0 );
        baryCoords[1].set( 0, 1, 0 );
        baryCoords[2].set( 0, 0, 1 );
    }

    void swap_verts( int v0, int v1 ) {
        std::swap( verts[v0], verts[v1] );
        std::swap( baryCoords[v0], baryCoords[v1] );
        std::swap( screenVerts[v0], screenVerts[v1] );
        std::swap( depth[v0], depth[v1] );
    }
};

/**
 * This function will rasterize part of a triangle for the given scanline range.
 * @param shader The opacity shader for this triangle.
 * @param zBuffer The target buffer to draw to.
 * @param faceIndex The index of the face.
 * @param verts The camera-space vertex positions, in same order as the original face.
 * @param projVerts The screen-space vertices, with X and Y in pixels, and Z as camera-space depth. They are ordered
 * such that projVerts[0] is the "middle" vertex, projVerts[1] is on the left and projVerts[2] is on the right. In this
 * case, middle is the vertex with edges connected to projVerts[1] and projVerts[2]. IMPORTANT: If isPerspective is
 * true, then projVerts[i].z must be reciprocal depth at vertex i.
 * @param vertexOrder Given element i of projVerts, vertexOrder[i] is the mapping back to the original face ordering.
 * @param startScan The inclusive scanline to start rendering at.
 * @param endScan The exclusive scanline to stop rendering at.
 * @param isPerspective Set this to true if rendering with a perspective projection.
 */
void render_triangle_piece( const opacity_shader& shader, framebuffer<float>& zBuffer, int faceIndex,
                            clipped_triangle& theTri, int startScan, int endScan, bool isPerspective ) {
    float leftHeight = theTri.screenVerts[1].y - theTri.screenVerts[0].y;
    float leftWidth = theTri.screenVerts[1].x - theTri.screenVerts[0].x;
    float rightHeight = theTri.screenVerts[2].y - theTri.screenVerts[0].y;
    float rightWidth = theTri.screenVerts[2].x - theTri.screenVerts[0].x;

    for( int pixelY = startScan; pixelY < endScan; ++pixelY ) {
        float yOffset = (float)pixelY - theTri.screenVerts[0].y;
        float leftYInterp = yOffset / leftHeight;
        float rightYInterp = yOffset / rightHeight;

        // This is the interpolated depth values on the left and right edges.
        float zLeft = theTri.depth[0] + leftYInterp * ( theTri.depth[1] - theTri.depth[0] );
        float zRight = theTri.depth[0] + rightYInterp * ( theTri.depth[2] - theTri.depth[0] );

        float fLeftX = theTri.screenVerts[0].x + leftYInterp * leftWidth;
        float fRightX = theTri.screenVerts[0].x + rightYInterp * rightWidth;
        float fWidth = fRightX - fLeftX;

        int leftX = (int)std::ceil( fLeftX );
        int rightX = (int)std::floor( fRightX );
        if( leftX < 0 )
            leftX = 0;
        if( rightX >= zBuffer.width() )
            rightX = zBuffer.width() - 1;

        for( int pixelX = leftX; pixelX <= rightX; ++pixelX ) {
            float xInterp = ( (float)pixelX - fLeftX ) / fWidth;
            float z = zLeft + xInterp * ( zRight - zLeft );
            float bufferZ = zBuffer.get_pixel( pixelX, pixelY );

            if( isPerspective )
                z = 1.f / z;

            if( z >= 0 && z < bufferZ ) {
                if( shader ) {
                    float gamma = xInterp * rightYInterp;
                    float beta = ( 1.f - xInterp ) * leftYInterp;

                    if( isPerspective ) {
                        // We need to perspective correct the barycentric coordinates. z is the real depth of the pixel,
                        // projVerts[i].z is repicrocal depth at vertex i. Refer to
                        // file:///R:/Research/Papers/Rendering/Low%20%282002%29%20Perspective-Correct%20Interpolation.pdf
                        // for more details.
                        gamma *= z * theTri.depth[2];
                        beta *= z * theTri.depth[1];
                    }

                    float alpha = 1.f - beta - gamma;

                    raytrace_intersection ri;
                    ri.faceIndex = faceIndex;
                    ri.barycentricCoords =
                        alpha * theTri.baryCoords[0] + beta * theTri.baryCoords[1] + gamma * theTri.baryCoords[2];

                    ri.position = alpha * theTri.verts[0] + beta * theTri.verts[1] + gamma * theTri.verts[2];

                    if( shader( ri ) > 0.f )
                        zBuffer.set_pixel( pixelX, pixelY, z );
                } else {
                    zBuffer.set_pixel( pixelX, pixelY, z );
                }
            }
        } // for(int pixelX = leftX; pixelX <= rightX; ++pixelX)
    }     // for(int pixelY = pixelStart; pixelY < pixelMiddle; ++pixelY)
}

// void render_triangle_depth( const opacity_shader& shader, framebuffer<float>& zBuffer, int faceIndex, vector3f
// verts[], vector3f projVerts[], int vertexOrder[], bool isPerspective ){
void render_triangle_depth( const opacity_shader& shader, framebuffer<float>& zBuffer, int faceIndex,
                            clipped_triangle& theTri, bool isPerspective ) {

    // Determine where the two halves of the triangle are.
    int pixelStart = frantic::math::clamp( (int)std::ceil( theTri.screenVerts[0].y ), 0, zBuffer.height() - 1 );
    int pixelMiddle = frantic::math::clamp( (int)std::ceil( theTri.screenVerts[1].y ), 0, zBuffer.height() );
    int pixelEnd = frantic::math::clamp( (int)std::ceil( theTri.screenVerts[2].y ), 0, zBuffer.height() - 1 ) + 1;

    // Determine if the cross product of the edge has negative, positive or zero in the z direction. We want a negative
    // z orientation.
    float zCross1 =
        ( theTri.screenVerts[1].x - theTri.screenVerts[0].x ) * ( theTri.screenVerts[2].y - theTri.screenVerts[0].y );
    float zCross2 =
        ( theTri.screenVerts[1].y - theTri.screenVerts[0].y ) * ( theTri.screenVerts[2].x - theTri.screenVerts[0].x );

    if( zCross1 == zCross2 ) // This means the projection made a degenerate sliver, so we draw nothing.
        return;
    if( zCross1 > zCross2 ) {
        theTri.swap_verts( 1, 2 );
    }

    if( isPerspective ) {
        // Invert the depth values to handle the perspective projection.
        theTri.depth[0] = 1.f / theTri.depth[0];
        theTri.depth[1] = 1.f / theTri.depth[1];
        theTri.depth[2] = 1.f / theTri.depth[2];
    }

    render_triangle_piece( shader, zBuffer, faceIndex, theTri, pixelStart, pixelMiddle, isPerspective );
    if( pixelMiddle >= pixelEnd )
        return;

    if( zCross1 > zCross2 ) {
        theTri.swap_verts( 0, 1 );
    } else {
        theTri.swap_verts( 0, 2 );
    }

    render_triangle_piece( shader, zBuffer, faceIndex, theTri, pixelMiddle, pixelEnd, isPerspective );
}

// pVerts must be large enought to store at least 2 ^ numClipPlanes triangles.
int clip_triangle_to_frustrum( int numClipPlanes, const frantic::graphics::plane3f clipPlanes[],
                               clipped_triangle inoutCSVerts[] ) {
    int numTriangles = 1;

    for( int i = 0; i < numClipPlanes; ++i ) {
        clipped_triangle* pTri = inoutCSVerts;
        clipped_triangle* pOutTri = inoutCSVerts + numTriangles;

        while( pTri != pOutTri ) {
            float kVert0 = clipPlanes[i].get_signed_distance_to_plane( pTri->verts[0] );
            float kVert1 = clipPlanes[i].get_signed_distance_to_plane( pTri->verts[1] );
            float kVert2 = clipPlanes[i].get_signed_distance_to_plane( pTri->verts[2] );

            if( kVert0 > 0 ) {
                if( kVert1 > 0 ) {
                    if( kVert2 > 0 ) {
                        // All verts are outside clipping plane, so cull this triangle. TODO: This can cause double
                        // processing of newly created triangles.
                        --numTriangles;
                        *pTri = *( --pOutTri );
                    } else {
                        // Verts 0 and 1 are outside so clip both to plane
                        float t0 = ( clipPlanes[i].constant() - kVert2 ) / ( kVert0 - kVert2 );
                        float t1 = ( clipPlanes[i].constant() - kVert2 ) / ( kVert1 - kVert2 );
                        pTri->verts[0] = pTri->verts[2] + t0 * ( pTri->verts[0] - pTri->verts[2] );
                        pTri->verts[1] = pTri->verts[2] + t1 * ( pTri->verts[1] - pTri->verts[2] );
                        pTri->baryCoords[0] = pTri->baryCoords[2] + t0 * ( pTri->baryCoords[0] - pTri->baryCoords[2] );
                        pTri->baryCoords[1] = pTri->baryCoords[2] + t1 * ( pTri->baryCoords[1] - pTri->baryCoords[2] );
                        ++pTri;
                    }
                } else if( kVert2 > 0 ) {
                    // Verts 0 and Verts 2 are outside so clip both to plane
                    float t0 = ( clipPlanes[i].constant() - kVert1 ) / ( kVert0 - kVert1 );
                    float t2 = ( clipPlanes[i].constant() - kVert1 ) / ( kVert2 - kVert1 );
                    pTri->verts[0] = pTri->verts[1] + t0 * ( pTri->verts[0] - pTri->verts[1] );
                    pTri->verts[2] = pTri->verts[1] + t2 * ( pTri->verts[2] - pTri->verts[1] );
                    pTri->baryCoords[0] = pTri->baryCoords[1] + t0 * ( pTri->baryCoords[0] - pTri->baryCoords[1] );
                    pTri->baryCoords[2] = pTri->baryCoords[1] + t2 * ( pTri->baryCoords[2] - pTri->baryCoords[1] );
                    ++pTri;
                } else {
                    // Vert 0 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal
                    // edge to prevent sliver triangles.
                    float t1 = ( clipPlanes[i].constant() - kVert1 ) / ( kVert0 - kVert1 );
                    float t2 = ( clipPlanes[i].constant() - kVert2 ) / ( kVert0 - kVert2 );

                    vector3f e01 = ( pTri->verts[0] - pTri->verts[1] );
                    vector3f e20 = ( pTri->verts[0] - pTri->verts[2] );
                    vector3f p01 = pTri->verts[1] + t1 * e01;
                    vector3f p20 = pTri->verts[2] + t2 * e20;

                    vector3f bc01 = pTri->baryCoords[1] + t1 * ( pTri->baryCoords[0] - pTri->baryCoords[1] );
                    vector3f bc20 = pTri->baryCoords[2] + t2 * ( pTri->baryCoords[0] - pTri->baryCoords[2] );

                    // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make
                    // better triangles.
                    if( e01.get_magnitude_squared() >= e20.get_magnitude_squared() ) {
                        // p01, p1, p2 and p01, p2, p20
                        pOutTri->verts[0] = pTri->verts[0] = p01;
                        pOutTri->verts[1] = pTri->verts[2];
                        pOutTri->verts[2] = p20;
                        pOutTri->baryCoords[0] = pTri->baryCoords[0] = bc01;
                        pOutTri->baryCoords[1] = pTri->baryCoords[2];
                        pOutTri->baryCoords[2] = bc20;
                    } else {
                        // p20, p1, p2 and p20, p01, p1
                        pOutTri->verts[0] = pTri->verts[0] = p20;
                        pOutTri->verts[1] = p01;
                        pOutTri->verts[2] = pTri->verts[1];
                        pOutTri->baryCoords[0] = pTri->baryCoords[0] = bc20;
                        pOutTri->baryCoords[1] = bc01;
                        pOutTri->baryCoords[2] = pTri->baryCoords[1];
                    }
                    ++numTriangles;
                    ++pOutTri;
                    ++pTri;
                }
            } else if( kVert1 > 0 ) {
                if( kVert2 > 0 ) {
                    // Verts 1 and 2 are outside so move them to the plane
                    float t1 = ( clipPlanes[i].constant() - kVert0 ) / ( kVert1 - kVert0 );
                    float t2 = ( clipPlanes[i].constant() - kVert0 ) / ( kVert2 - kVert0 );
                    pTri->verts[1] = pTri->verts[0] + t1 * ( pTri->verts[1] - pTri->verts[0] );
                    pTri->verts[2] = pTri->verts[0] + t2 * ( pTri->verts[2] - pTri->verts[0] );
                    pTri->baryCoords[1] = pTri->baryCoords[0] + t1 * ( pTri->baryCoords[1] - pTri->baryCoords[0] );
                    pTri->baryCoords[2] = pTri->baryCoords[0] + t2 * ( pTri->baryCoords[2] - pTri->baryCoords[0] );
                    ++pTri;
                } else {
                    // Vert 1 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal
                    // edge to prevent sliver triangles.
                    float t0 = ( clipPlanes[i].constant() - kVert0 ) / ( kVert1 - kVert0 );
                    float t2 = ( clipPlanes[i].constant() - kVert2 ) / ( kVert1 - kVert2 );

                    vector3f e01 = ( pTri->verts[1] - pTri->verts[0] );
                    vector3f e12 = ( pTri->verts[1] - pTri->verts[2] );
                    vector3f p01 = pTri->verts[0] + t0 * e01;
                    vector3f p12 = pTri->verts[2] + t2 * e12;

                    vector3f bc01 = pTri->baryCoords[0] + t0 * ( pTri->baryCoords[1] - pTri->baryCoords[0] );
                    vector3f bc12 = pTri->baryCoords[2] + t2 * ( pTri->baryCoords[1] - pTri->baryCoords[2] );

                    // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make
                    // better triangles.
                    if( e01.get_magnitude_squared() >= e12.get_magnitude_squared() ) {
                        // p0, p01, p2 and p01, p12, p2
                        pOutTri->verts[0] = pTri->verts[1] = p01;
                        pOutTri->verts[1] = p12;
                        pOutTri->verts[2] = pTri->verts[2];
                        pOutTri->baryCoords[0] = pTri->baryCoords[1] = bc01;
                        pOutTri->baryCoords[1] = bc12;
                        pOutTri->baryCoords[2] = pTri->baryCoords[2];
                    } else {
                        // p0, p12, p2 and p0, p01, p12
                        pOutTri->verts[0] = pTri->verts[0];
                        pOutTri->verts[1] = p01;
                        pOutTri->verts[2] = pTri->verts[1] = p12;
                        pOutTri->baryCoords[0] = pTri->baryCoords[0];
                        pOutTri->baryCoords[1] = bc01;
                        pOutTri->baryCoords[2] = pTri->baryCoords[1] = bc12;
                    }
                    ++numTriangles;
                    ++pOutTri;
                    ++pTri;
                }
            } else if( kVert2 > 0 ) {
                // Vert 2 is outside the plane, so we create a quad and triangulate it, minimizing the new diagonal edge
                // to prevent sliver triangles.
                float t0 = ( clipPlanes[i].constant() - kVert0 ) / ( kVert2 - kVert0 );
                float t1 = ( clipPlanes[i].constant() - kVert1 ) / ( kVert2 - kVert1 );

                vector3f e20 = ( pTri->verts[2] - pTri->verts[0] );
                vector3f e12 = ( pTri->verts[2] - pTri->verts[1] );
                vector3f p20 = pTri->verts[0] + t0 * e20;
                vector3f p12 = pTri->verts[1] + t1 * e12;

                vector3f bc20 = pTri->baryCoords[0] + t0 * ( pTri->baryCoords[2] - pTri->baryCoords[0] );
                vector3f bc12 = pTri->baryCoords[1] + t1 * ( pTri->baryCoords[2] - pTri->baryCoords[1] );

                // Pick the larger edge to have the diagonal on it. This should minimize the diagonal and make better
                // triangles.
                if( e20.get_magnitude_squared() >= e12.get_magnitude_squared() ) {
                    // p0, p1, p20 and p20, p1, p12
                    pOutTri->verts[0] = pTri->verts[2] = p20;
                    pOutTri->verts[1] = pTri->verts[1];
                    pOutTri->verts[2] = p12;
                    pOutTri->baryCoords[0] = pTri->baryCoords[2] = bc20;
                    pOutTri->baryCoords[1] = pTri->baryCoords[1];
                    pOutTri->baryCoords[2] = bc12;
                } else {
                    // p0, p1, p12 and p0, p12, p20
                    pOutTri->verts[0] = pTri->verts[0];
                    pOutTri->verts[1] = pTri->verts[2] = p12;
                    pOutTri->verts[2] = p20;
                    pOutTri->baryCoords[0] = pTri->baryCoords[0];
                    pOutTri->baryCoords[1] = pTri->baryCoords[2] = bc12;
                    pOutTri->baryCoords[2] = bc20;
                }
                ++numTriangles;
                ++pOutTri;
                ++pTri;
            } else {
                ++pTri;
            }
        }

        // FF_LOG(debug) << "After clipping to plane " << i << ": " << clipPlanes[i].str() << " there are " <<
        // numTriangles << " triangles" << std::endl; for( int j = 0; j < numTriangles; ++j ) 	FF_LOG(debug) <<
        //"\tTriangle " << j << ": " << inoutCSVerts[j].verts[0] << ", " << inoutCSVerts[j].verts[1] << ", " <<
        //inoutCSVerts[j].verts[2] << std::endl;
    }

    return numTriangles;
}

} // namespace detail

void matte_primitive::raster_depthmap( const frantic::graphics::camera<float>& renderCam, std::size_t rangeStart,
                                       std::size_t rangeEnd,
                                       frantic::graphics2d::framebuffer<float>& renderBuffer ) const {
    bool isPerspective = ( renderCam.projection_mode() != frantic::graphics::projection_mode::orthographic );
    float pixelScale = float( renderBuffer.size().xsize / renderCam.get_output_size().xsize );

    boost::function<float( const frantic::geometry::raytrace_intersection& )> opacityShader =
        boost::bind( &matte_primitive::get_opacity, this, _1 );

    transform4f tm = renderCam.world_transform_inverse( m_currentTime ) * get_transform();

    float nearPlane = renderCam.near_distance();
    vector2f subpixelOffset = renderCam.get_subpixel_offset();

    float xsize = (float)renderCam.get_output_size().xsize * 0.5f + 1e-3f;
    float ysize = (float)renderCam.get_output_size().ysize * 0.5f + 1e-3f;
    subpixelOffset = pixelScale * vector2f( subpixelOffset.x / xsize, subpixelOffset.y / ysize );

    float halfHorizontalFOV = 0.5f * renderCam.horizontal_fov();
    float sinHalfHorizontalFOV = sin( halfHorizontalFOV );
    float cosHalfHorizontalFOV = cos( halfHorizontalFOV );

    float halfVerticalFOV = 0.5f * renderCam.vertical_fov();
    float sinHalfVerticalFOV = sin( halfVerticalFOV );
    float cosHalfVerticalFOV = cos( halfVerticalFOV );

    // TODO: These are perspective camera planes while we may want to handle orthographic cameras too.
    frantic::graphics::plane3f clipPlanes[5];
    clipPlanes[0] = frantic::graphics::plane3f( 0, 0, 1, nearPlane );

    if( isPerspective ) {
        clipPlanes[1] =
            frantic::graphics::plane3f( cosHalfHorizontalFOV + subpixelOffset.x, 0, sinHalfHorizontalFOV, 1e-3f );
        clipPlanes[2] =
            frantic::graphics::plane3f( -cosHalfHorizontalFOV + subpixelOffset.x, 0, sinHalfHorizontalFOV, 1e-3f );
        clipPlanes[3] =
            frantic::graphics::plane3f( 0, cosHalfVerticalFOV + subpixelOffset.y, sinHalfVerticalFOV, 1e-3f );
        clipPlanes[4] =
            frantic::graphics::plane3f( 0, -cosHalfVerticalFOV + subpixelOffset.y, sinHalfVerticalFOV, 1e-3f );
    } else {
        float aspect = (float)renderCam.get_output_size().ysize / (float)renderCam.get_output_size().xsize;
        float orthoHalfWidth = 0.5f * renderCam.orthographic_width() + 1e-3f;
        float orthoHeightHalf = 0.5f * renderCam.orthographic_width() * aspect + 1e-3f;
        clipPlanes[1] = frantic::graphics::plane3f( 1, 0, 0, orthoHalfWidth );
        clipPlanes[2] = frantic::graphics::plane3f( -1, 0, 0, orthoHalfWidth );
        clipPlanes[3] = frantic::graphics::plane3f( 0, 1, 0, orthoHeightHalf );
        clipPlanes[4] = frantic::graphics::plane3f( 0, -1, 0, orthoHeightHalf );
    }

    // Storage for the initial triangle and others possibly created by clipping to the camera frustrum.
    // As a back of the envelop calculation I am assuming at most 2^5 = 32 triangles can be generated.
    detail::clipped_triangle csTriangles[32];

    for( std::size_t i = rangeStart; i < rangeEnd; ++i ) {
        bool isValid = true;

        this->get_triangle( i, csTriangles[0].verts );

        csTriangles[0].reset_barycoords();
        csTriangles[0].verts[0] = tm * csTriangles[0].verts[0];
        csTriangles[0].verts[1] = tm * csTriangles[0].verts[1];
        csTriangles[0].verts[2] = tm * csTriangles[0].verts[2];

        // TODO: The expected case is that most triangles are on-screen. It might make sense to only clip to the
        // near-plane and then project the resulting
        //       vertices (at most 6) onto the screen. If any verts are off-screen, we can then apply the other clipping
        //       planes. Measure, measure, measure!
        int numTriangles = clip_triangle_to_frustrum( 5, clipPlanes, csTriangles );

        detail::clipped_triangle* pCurTriangle = csTriangles;
        for( int k = 0; k < numTriangles; ++k, ++pCurTriangle ) {
            pCurTriangle->screenVerts[0] =
                pixelScale * renderCam.from_cameraspace_position( pCurTriangle->verts[0], ( isValid = true ) );
            pCurTriangle->screenVerts[1] =
                pixelScale * renderCam.from_cameraspace_position( pCurTriangle->verts[1], ( isValid = true ) );
            pCurTriangle->screenVerts[2] =
                pixelScale * renderCam.from_cameraspace_position( pCurTriangle->verts[2], ( isValid = true ) );

            pCurTriangle->depth[0] = -pCurTriangle->verts[0].z;
            pCurTriangle->depth[1] = -pCurTriangle->verts[1].z;
            pCurTriangle->depth[2] = -pCurTriangle->verts[2].z;

            if( pCurTriangle->screenVerts[1].y < pCurTriangle->screenVerts[0].y &&
                pCurTriangle->screenVerts[1].y <= pCurTriangle->screenVerts[2].y )
                pCurTriangle->swap_verts( 0, 1 );
            else if( pCurTriangle->screenVerts[2].y < pCurTriangle->screenVerts[0].y &&
                     pCurTriangle->screenVerts[2].y <= pCurTriangle->screenVerts[1].y )
                pCurTriangle->swap_verts( 0, 2 );
            if( pCurTriangle->screenVerts[2].y < pCurTriangle->screenVerts[1].y )
                pCurTriangle->swap_verts( 1, 2 );

            render_triangle_depth( opacityShader, renderBuffer, (int)i, *pCurTriangle, isPerspective );
        }
    }
}

void matte_primitive::build_kdtree() {
    if( !m_pKDTree ) {

        // store a copy of the current mesh as m_pKDTreeTrimesh, and make a kdtree of it (the kd-tree doesn't own the
        // mesh, we do)
        get_mesh_copy( m_pKDTreeTrimesh );
        m_pKDTree.reset( new frantic::geometry::trimesh3_kdtree( m_pKDTreeTrimesh ) );
        m_kdTreeTM = this->get_transform();
        m_kdTreeTMInverse = m_kdTreeTM.to_inverse();
    }
}

frantic::graphics::boundbox3f matte_primitive::get_bounds() const {
    if( !m_pKDTree )
        return frantic::graphics::boundbox3f();
    return m_pKDTree->get_mesh().compute_bound_box();
}

bool matte_primitive::intersect_ray( const frantic::graphics::ray3f& ray, double tMin, double tMax,
                                     frantic::geometry::raytrace_intersection& outIntersection ) const {
    if( !m_pKDTree )
        return false;

    ray3f r = m_kdTreeTMInverse * ray;
    while( m_pKDTree->intersect_ray( r, tMin, tMax, outIntersection ) ) {
        outIntersection.position = m_kdTreeTM * outIntersection.position;
        outIntersection.geometricNormal =
            m_kdTreeTMInverse.transpose_transform_no_translation( outIntersection.geometricNormal );

        if( this->get_opacity( outIntersection ) > 1e-5f )
            return true;
        tMin = outIntersection.distance + 1e-5;
    }
    return false;
}

} // namespace krakatoa
