// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/intrusive_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <frantic/geometry/primitive_kdtree.hpp>
#include <frantic/geometry/trimesh3_kdtree.hpp>
#include <frantic/graphics/camera.hpp>
#include <frantic/graphics/motion_blurred_transform.hpp>
#include <frantic/graphics2d/framebuffer.hpp>
#include <frantic/rendering/framebuffer_cubeface.hpp>
#pragma warning( push, 3 )
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#pragma warning( pop )

namespace krakatoa {

// forward declarations
class scene_context;
typedef boost::intrusive_ptr<scene_context> scene_context_ptr;
class matte_primitive;

/**
 * Implementation for base matte_interface
 * This function passes its calls to either
 * a) raster_depth_map_renderer
 * b) raytrace_depth_map_renderer
 * Both of which rely on the matte_primitive object to do the actual work
 */
class matte_interface {
  public:
    typedef boost::shared_ptr<matte_interface> ptr_type;

  protected:
    const scene_context_ptr m_context;
    int m_numThreads;
    matte_interface& operator=( const matte_interface& ) { return *this; } // disabled

  public:
    matte_interface( const scene_context_ptr context )
        : m_context( context ) {
        // initialize the number of threads. let boost decide the number of threads based on the hardware cores.
        m_numThreads = boost::thread::hardware_concurrency();
    }

    virtual ~matte_interface() {}

    /*
     * Sets the number of threads for the geometry renderer to use. -1 defaults to one thread per logical core.
     */
    void set_num_threads( int numThreads ) {
        // set threads to 1, or use boost again to decide.
        m_numThreads = ( numThreads == -1 ) ? boost::thread::hardware_concurrency() : numThreads;
    }

    virtual void generate_depth_map( const frantic::graphics::camera<float>& camera, float mblurTime,
                                     frantic::graphics2d::framebuffer<float>& outDepthImg, bool forLight );
};

typedef matte_interface::ptr_type matte_interface_ptr;

/**
 * Class used by the matte_interface to perform raster rendering of a collection of matte_primitive objects.
 * It holds a list of matte_primitive objects to be rendered, and calls their rasterizing functions to do the work.
 */
class raster_depth_map_renderer {
    std::vector<boost::shared_ptr<krakatoa::matte_primitive>> m_primitives;
    std::size_t m_curPrimitive;
    std::size_t m_curFace;
    boost::mutex m_mutex;

    // inner class for multi-threading
    struct thread_type {
        frantic::graphics2d::framebuffer<float> renderBuffer;
        const frantic::graphics::camera<float>* renderCamera;

        void evaluate( raster_depth_map_renderer& task ) {
            try {
                // Grab each successive task until there are no more.
                while( task.run_next_task( *renderCamera, renderBuffer ) ) {
                }
            } catch( const std::exception& ) {
                // Can't really do anything like this from a different thread. Need to use boost::exception_ptr to
                // transfer across threads. FF_LOG(error) << e.what() << std::endl;
            } catch( ... ) {
                // Can't really do anything like this from a different thread. Need to use boost::exception_ptr to
                // transfer across threads. FF_LOG(error) << "Unknown exception in: " << __FILE__ << " line: " << __LINE
                // __ << std::endl;
            }
        }
    };

    // multi-threading function
    bool run_next_task( const frantic::graphics::camera<float>& renderCamera,
                        frantic::graphics2d::framebuffer<float>& renderBuffer );

  public:
    raster_depth_map_renderer()
        : m_curPrimitive( 0 )
        , m_curFace( 0 ) {}

    void add_primitive( boost::shared_ptr<krakatoa::matte_primitive> pPrim ) { m_primitives.push_back( pPrim ); }

    void rasterize_depthmap( const frantic::graphics::camera<float>& renderCamera,
                             frantic::graphics2d::framebuffer<float>& renderBuffer, int numThreads );
};

/**
 * Class used by the matte_interface to perform the raytracing of a collection of matte_primitive objects.
 * It holds a list of matte_primitive objects to be raytraced, and calls their raytracing funtions to do the work.
 */
class raytrace_depth_map_renderer {

    std::vector<boost::shared_ptr<krakatoa::matte_primitive>> m_primitives;
    boost::scoped_ptr<frantic::geometry::primitive_kdtree<krakatoa::matte_primitive>> m_srcKDTree;

    float m_cachedSegmentTime;

  public:
    /**
     * Creates a new raytrace_depth_map_renderer for the specified motion interval.
     * @param motionInterval The motion interval to render over.
     * @param deformingMBlur If true, the geometry will be re-acquired from the nodes
     *                       during each call to rebuild().
     * @param numMBlurSegments If 'deformingMBlur' is true, this specified the number of distinct geometry samples to
     * interpolate between. If this value is 0, then a new sample will be generated on each call to rebuild() and no
     * interpolation is needed.
     */
    raytrace_depth_map_renderer()
        : m_cachedSegmentTime( 0.0f ) {}

    /**
     * Adds a primitive to the list of geometry to be renderered.
     * @param pPrim An abstract primitive object.
     */
    inline void add_primitive( boost::shared_ptr<krakatoa::matte_primitive> pPrim ) { m_primitives.push_back( pPrim ); }

    /**
     * Updates the kd-tree (and underlying primitives) to the specified motionIntervalTime.
     * Will re-create the primitive nodes if m_deformingMBlur is true.
     * @param motionSegmentTime The time inside the interval [0,1] of the motion segment.
     */
    void rebuild( float motionSegmentTime );

    /**
     * Will render a depthmap of the geometry collection using raytraces. It will build the kd-tree
     * if it is not already built.
     * @param renderCam The camera from which to render the depthmap.
     * @param renderBuffer The buffer to render the depthmap into.
     * @param forLight True if this render is for a shadow depthmap, false if it is a camera depthmap.
     */
    void raytrace_depthmap( const frantic::graphics::camera<float>& renderCam,
                            frantic::graphics2d::framebuffer<float>& renderBuffer, bool forLight );
};

/**
 * Generic implementation for matte_primitive.
 * This primitive object does all the work for raytracing and rasterization and is called by either:
 * a) raster_depth_map_renderer
 * b) raytrace_depth_map_renderer
 */
class matte_primitive {
  public:
    typedef boost::shared_ptr<matte_primitive> ptr_type;

  protected:
    float m_currentTime;

    // objects used by the raytracer. they are not used when raster rendering (as build_kdtree is never called)
    frantic::geometry::trimesh3 m_pKDTreeTrimesh;
    boost::scoped_ptr<frantic::geometry::trimesh3_kdtree> m_pKDTree;
    frantic::graphics::transform4f m_kdTreeTM;
    frantic::graphics::transform4f m_kdTreeTMInverse;

    matte_primitive()
        : m_currentTime( 0.5f ) {}

  public:
    float get_time() const { return m_currentTime; }

    /**
     * Sets the time that the primitive is being evaluated at. This must be called before inspecting the
     * primtives properties and geometry.
     * @note When creating a base class the override implementation must call this function.
     * @param motionSegmentTime The new [0,1] time to evaluate the primitive at.
     */
    virtual void set_time( float motionSegmentTime ) {
        // for raytracer implementation
        if( m_currentTime != motionSegmentTime )
            m_pKDTree.reset();
        m_currentTime = motionSegmentTime;
    }

    /**
     * Returns the number of triangles the primitive has at the current time.
     */
    virtual std::size_t get_triangle_count() const = 0;

    /**
     * Gets the specific triangle from the primitive at the current time. This is the suggested way to extract triangles
     * from a primitive because it doesn't rely on any particular storage mechanism.
     * @param index The triangle index in the range [ 0,get_triangle_count() ).
     * @param outVertices A 3-element array for storing the vertices of the triangle.
     */
    virtual void get_triangle( std::size_t index, frantic::graphics::vector3f outVertices[] ) const = 0;

    /**
     * Evaluates the primitive's opacity at the given surface point.
     * @TODO change this AWAY from a raytrace_intersection object, and use possibly a face index + barycentric coord.
     * @param ri The surface point being shaded. Created from a raytrace or raster operation.
     * @result A [0,1] opacity value, 0 being completely transparent, 1 being completely opaque.
     */
    virtual float get_opacity( const frantic::geometry::raytrace_intersection& ri ) const = 0;

    /**
     * Returns a transform at the current time.
     * @result A transform valid for the current motion segment time as specified
     *         by the last call to set_time().
     */
    virtual frantic::graphics::transform4f get_transform() const = 0;

    /**
     * Returns a copy mesh at the current time. This will always allocate a new trimesh regardless of native
     * representation. For that reason, it is recommended to use get_triangle() if possible instead of this routine.
     * This is currently only used for the raytracer version of the renderer, and should never be called otherwise.
     * @result A reference to a cached mesh valid for the current motion segment time as specified
     *         by the last call to set_time().
     */
    virtual void get_mesh_copy( frantic::geometry::trimesh3& outMesh ) const = 0;

    /**
     * Returns true if this primitive is visible when rendering from the perspective of a light.
     */
    virtual bool is_visible_to_lights() const { return true; }

    /**
     * Returns true if this primitive is visible when rendering from the perspective of a camera.
     */
    virtual bool is_visible_to_cameras() const { return true; }

    /**
     * Will render a depthmap of this mesh using a z-buffer rasterization algorthim, at the cached
     * motion-segment time.
     *
     * @note Calling this function with a camera that has a non-linear projection type (ie. Spherical, Cubic distortion,
     * etc.) will result in undefined behaviour.
     *
     * @param renderCam The camera from which to render the depthmap.
     * @param rangeStart The index of the start of the face range to render.
     * @param rangeEnd The index of one past the last face in the range to render.
     * @param renderBuffer The buffer to render the depthmap into.
     */
    void raster_depthmap( const frantic::graphics::camera<float>& renderCam, std::size_t rangeStart,
                          std::size_t rangeEnd, frantic::graphics2d::framebuffer<float>& renderBuffer ) const;

    /**
     * @overload
     */
    void raster_depthmap( const frantic::graphics::camera<float>& renderCam,
                          frantic::graphics2d::framebuffer<float>& renderBuffer ) const {
        raster_depthmap( renderCam, 0, get_triangle_count(), renderBuffer );
    }

    /**
     * Builds the trimesh3_kdtree of the mesh cached by this MattePrimitive
     */
    void build_kdtree();

    /**
     * Returns the bounding box of the geometry in world space, using the currently cached transform.
     * This is a required function for being used in the primitive_kdtree.
     */
    frantic::graphics::boundbox3f get_bounds() const;

    /**
     * Will return the first intersection of the ray with the underlying MattePrimitive geometry.
     * The ray will be transformed into the MattePrimitive's space using the cached motion segment time.
     * This is a required function for being using in the primitive_kdtree.
     *
     * @param ray The ray to intersect with the geometry
     * @param tMin The minimum accepted distance along the ray to an intersection
     * @param tMax The maxmum accepted distance along the ray to an intersection
     * @param outIntersection Stores the results of the ray intersection. Only valid if true is returned.
     */
    bool intersect_ray( const frantic::graphics::ray3f& ray, double tMin, double tMax,
                        frantic::geometry::raytrace_intersection& outIntersection ) const;
};

typedef matte_primitive::ptr_type matte_primitive_ptr;

} // namespace krakatoa
