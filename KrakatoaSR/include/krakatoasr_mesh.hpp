// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_MESH__
#define __KRAKATOASR_MESH__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * Triangle mesh.
 * This is a helper class for the renderer. Triangle meshes are used for matte objects and holdout masks in the
 * renderer. Note that an alternate way to specify matte objects and holdout maskes is to use pre-rendered deep images.
 */
class CLSEXPORT triangle_mesh {
  private:
    triangle_mesh_params* m_data;

  public:
    /**
     * Creates an empty mesh.
     * When using the default constructor, the mesh can be constructed by using the "set_num_vertices",
     * "set_vertex_position" calls, etc.
     */
    triangle_mesh();

    ~triangle_mesh();
    triangle_mesh( const triangle_mesh& t );
    triangle_mesh& operator=( const triangle_mesh& t );

    const triangle_mesh_params* get_data() const;
    triangle_mesh_params* get_data();

    /**
     * Sets whether or not this mesh is visible to the camera.
     * If this is set to true, this mesh will be used as a holdout mask.
     * @param visibleToCamera Boolean flag.
     */
    void set_visible_to_camera( bool visibleToCamera );

    /**
     * Sets whether or not this mesh is visible to the lights in the scene.
     * If this is set to true, this mesh will be used as a matte object for the scene lights.
     * @param visibleToLights Boolean flag.
     */
    void set_visible_to_lights( bool visibleToLights );

    /**
     * Sets the shutter time that this mesh is represented at.
     * When the renderer retrieves the mesh, it is offset from this inputted time to the desired shutter time based on
     * the mesh's vertex velocities.
     * @param shutterTime The shutter time in seconds that this mesh is represented at. Defaults to 0.0.
     */
    void set_shutter_time( float shutterTime );

    /**
     * Sets the shutter time that this mesh is represented at.
     * This is typically only used when doing multi-segment meshes that have changing topology within the renderer's
     *shutter begin and end times. If the renderer requests a mesh outside of these begin and end times, it will produce
     *an empty mesh.
     *
     * Example of a two-segement motion blurred mesh. Both mesh1 and mesh2 would be added to the renderer using
     *"add_mesh":
     * @code
     *		triangle_mesh mesh1, mesh2;
     *		triangle_mesh::load_from_file( mesh1, "segment1.xmesh" ); //segment1.xmesh is a mesh with velocities at
     *shutter time -0.05, and is valid from shutter times -0.05 to 0. triangle_mesh::load_from_file( mesh2,
     *"segment2.xmesh" ); //segment2.xmesh is a mesh with velocities at shutter time 0, and is valid from shutter times
     *0 to 0.05. mesh1.set_shutter_time( -0.05 ); mesh1.set_shutter_validity( -0.05, 0.0 ); mesh2.set_shutter_time( 0.0
     *); mesh2.set_shutter_validity( 0.0, 0.05 );
     * @endcode
     *
     * @param shutterTimeBegin The shutter time in seconds that represents the starting validity range of this mesh.
     *Defaults to -infinity.
     * @param shutterTimeEnd The shutter time in seconds that represents the ending validity range of this mesh.
     *Defaults to +infinity.
     */
    void set_shutter_validity( float shutterTimeBegin, float shutterTimeEnd );

    /**
     * Sets the number of vertices in this mesh.
     * This function is used when constructing a mesh manually. It must be called before setting vertex data.
     * @param numVertices The number of vertices in this mesh. Defaults to 0.
     */
    void set_num_vertices( int numVertices );

    /**
     * Sets a single vertex position.
     * @param vertexIndex The vertex index. Must be between 0 and "numVertices" (as set by set_num_vertices).
     * @param x The "x" component of the vertex position.
     * @param y The "y" component of the vertex position.
     * @param z The "z" component of the vertex position.
     */
    void set_vertex_position( int vertexIndex, float x, float y, float z );

    /**
     * Sets all the vertex positions with an array.
     * @param vertexPositionData A float array of length "numVertices" * 3 representing vertex positions. The array
     * order goes as follows: x0,y0,z0,x1,y1,z1,...
     */
    void set_vertex_position_data( const float* vertexPositionData );

    /**
     * Sets a single vertex velocity in units per second.
     * @param vertexIndex The vertex index. Must be between 0 and "numVertices" (as set by set_num_vertices).
     * @param vx The "x" component of the vertex velocity in units per second.
     * @param vy The "y" component of the vertex velocity in units per second.
     * @param vz The "z" component of the vertex velocity in units per second.
     */
    void set_vertex_velocity( int vertexIndex, float vx, float vy, float vz );

    /**
     * Sets all the vertex velocities in units per second with an array.
     * @param vertexVelocityData A float array of length "numVertices" * 3 representing vertex velocities in units per
     * second. The array order goes as follows: vx0,vy0,vz0,vx1,vy1,vz1,...
     */
    void set_vertex_velocity_data( const float* vertexVelocityData );

    /**
     * Sets the number of triangles in this mesh.
     * This function is used when constructing a mesh manually. It must be called before setting face data.
     * @param numTriangleFaces The number of faces in this mesh. Defaults to 0.
     */
    void set_num_triangle_faces( int numTriangleFaces );

    /**
     * Sets a single face's vertices.
     * @param faceIndex The face index. Must be between 0 and "numTriangleFaces" (as set by set_num_triangle_faces).
     * @param vert1 The first vertex that makes up this triangle.
     * @param vert2 The second vertex that makes up this triangle.
     * @param vert3 The third vertex that makes up this triangle.
     */
    void set_face( int faceIndex, int vert1, int vert2, int vert3 );

    /**
     * Sets all the faces with an array.
     * @param triangleFaceIndices An integer array of length "numTriangleFaces" * 3 representing face vertices. The
     * array order goes as follows: face0vert1,face0vert2,face0vert3,face1vert1,face1vert2,face1vert3,...
     */
    void set_face_data( const int* triangleFaceIndices );

    /**
     * Sets this mesh to an empty mesh.
     */
    void clear();

  public:
    /**
     * Static function that sets a mesh's data based on an "obj" or "xmesh" mesh file.
     * Note that "obj" files do not include velocity, so to have an animated mesh, use the
     * load_from_files_with_velocities function instead.
     * @param outTriangleMesh The mesh object to populate.
     * @param filename The "obj" or "xmesh" filename.
     */
    static void load_from_file( triangle_mesh& outTriangleMesh, const char* filename );

    /**
     * Static function that sets a mesh's data based on a pair of "obj" or "xmesh" mesh file.
     * This function produces a single mesh and populates its vertex velocity based on the change in vertex positions
     * between the two mesh files. Note that the two meshes passed in must have the same number of vertices.
     * @param outTriangleMesh The mesh object to populate.
     * @param filenameBaseMesh An "obj" or "xmesh" filename. This file will be used to generate the mesh's vertices and
     * faces.
     * @param filenameOffsetMesh An "obj" or "xmesh" filename. This file will be used to generate the mesh's vertex
     * velocities. It must have the same number of vertices as the base mesh.
     * @param timeBetweenBaseAndOffset The time in seconds between the base mesh and the offset mesh.
     */
    static void load_from_files_with_velocities( triangle_mesh& outTriangleMesh, const char* filenameBaseMesh,
                                                 const char* filenameOffsetMesh, float timeBetweenBaseAndOffset );

  private:
    void initialize_velocity();
};

} // namespace krakatoasr

#endif
