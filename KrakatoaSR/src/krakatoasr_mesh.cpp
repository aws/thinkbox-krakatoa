// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_mesh.hpp>

#include <krakatoasr_renderer/params.hpp>

#include <frantic/geometry/trimesh3_file_io.hpp>

using namespace frantic::graphics;
using namespace frantic::geometry;

namespace krakatoasr {

triangle_mesh::triangle_mesh() {
    m_data = new triangle_mesh_params;

    // set defaults
    m_data->visibleToCamera = true;
    m_data->visibleToLights = true;
    m_data->shutterTime = 0.0f;
    m_data->shutterValidityBegin = -std::numeric_limits<float>::max();
    m_data->shutterValidityEnd = std::numeric_limits<float>::max();
    m_data->mesh.reset( new trimesh3 );
}

triangle_mesh::~triangle_mesh() { delete m_data; }

triangle_mesh::triangle_mesh( const triangle_mesh& t ) {
    m_data = new triangle_mesh_params;
    *this = t;
}

triangle_mesh& triangle_mesh::operator=( const triangle_mesh& t ) {
    *m_data = *t.m_data;
    return *this;
}

const triangle_mesh_params* triangle_mesh::get_data() const { return m_data; }

triangle_mesh_params* triangle_mesh::get_data() { return m_data; }

void triangle_mesh::set_visible_to_camera( bool visibleToCamera ) { m_data->visibleToCamera = visibleToCamera; }

void triangle_mesh::set_visible_to_lights( bool visibleToLights ) { m_data->visibleToLights = visibleToLights; }

void triangle_mesh::set_shutter_time( float shutterTime ) { m_data->shutterTime = shutterTime; }

void triangle_mesh::set_shutter_validity( float shutterTimeBegin, float shutterTimeEnd ) {
    m_data->shutterValidityBegin = shutterTimeBegin;
    m_data->shutterValidityEnd = shutterTimeEnd;
}

void triangle_mesh::set_num_vertices( int numVertices ) { m_data->mesh->set_vertex_count( numVertices ); }

void triangle_mesh::set_vertex_position( int vertexIndex, float x, float y, float z ) {
    m_data->mesh->get_vertex( vertexIndex ).set( x, y, z );
}

void triangle_mesh::set_vertex_position_data( const float* vertexPositionData ) {
    std::vector<vector3f>& vertVector = m_data->mesh->vertices_ref();
    size_t vertCount = vertVector.size();
    for( size_t i = 0; i < vertCount; ++i )
        vertVector[i].set( vertexPositionData[i * 3], vertexPositionData[i * 3 + 1], vertexPositionData[i * 3 + 2] );
}

void triangle_mesh::set_vertex_velocity( int vertexIndex, float vx, float vy, float vz ) {
    initialize_velocity();
    m_data->velocityAcc[vertexIndex].set( vx, vy, vz );
}

void triangle_mesh::set_vertex_velocity_data( const float* vertexVelocityData ) {
    initialize_velocity();
    size_t vertCount = m_data->mesh->vertex_count();
    for( int i = 0; i < vertCount; ++i )
        m_data->velocityAcc[i].set( vertexVelocityData[i * 3], vertexVelocityData[i * 3 + 1],
                                    vertexVelocityData[i * 3 + 2] );
}

void triangle_mesh::set_num_triangle_faces( int numTriangleFaces ) { m_data->mesh->set_face_count( numTriangleFaces ); }

void triangle_mesh::set_face( int faceIndex, int vert1, int vert2, int vert3 ) {
    m_data->mesh->get_face( faceIndex ).set( vert1, vert2, vert3 );
}

void triangle_mesh::set_face_data( const int* triangleFaceIndices ) {
    std::vector<vector3>& faceVector = m_data->mesh->faces_ref();
    size_t faceCount = faceVector.size();
    for( size_t i = 0; i < faceCount; ++i )
        faceVector[i].set( triangleFaceIndices[i * 3], triangleFaceIndices[i * 3 + 1], triangleFaceIndices[i * 3 + 2] );
}

void triangle_mesh::clear() { m_data->mesh->clear(); }

void triangle_mesh::load_from_file( triangle_mesh& outTriangleMesh, const char* filename ) {
    // load mesh from file
    boost::shared_ptr<trimesh3> newMesh( new trimesh3 );
    load_mesh_file( frantic::strings::to_tstring( filename ), *newMesh.get() );

    // set velocity accessor
    if( newMesh->has_vertex_channel( _T( "Velocity" ) ) )
        outTriangleMesh.m_data->velocityAcc = newMesh->get_vertex_channel_accessor<vector3f>( _T( "Velocity" ) );
    else
        outTriangleMesh.m_data->velocityAcc = trimesh3_vertex_channel_accessor<vector3f>(); // reset velocity accessor

    // assign output mesh
    outTriangleMesh.m_data->mesh = newMesh;
}

void triangle_mesh::load_from_files_with_velocities( triangle_mesh& outTriangleMesh, const char* filenameBaseMesh,
                                                     const char* filenameOffsetMesh, float timeBetweenBaseAndOffset ) {
    if( timeBetweenBaseAndOffset == 0.0f )
        throw std::runtime_error(
            "Attempted to generate velocities from two mesh files. However, the time difference specified between the "
            "meshes was zero. Please specify a non-zero time between meshes." );

    // load both meshes from file
    boost::shared_ptr<trimesh3> newMesh( new trimesh3 );
    trimesh3 offsetMesh;
    load_mesh_file( frantic::strings::to_tstring( filenameBaseMesh ), *newMesh.get() );
    load_mesh_file( frantic::strings::to_tstring( filenameOffsetMesh ), offsetMesh );
    size_t numVerts = newMesh->vertex_count();
    if( numVerts != offsetMesh.vertex_count() )
        throw std::runtime_error( "Attempted to generate velocities from two mesh files. However, the two meshes have "
                                  "differing topology. Meshes must have the same number of vertices.\n" +
                                  std::string( filenameBaseMesh ) + " has " +
                                  boost::lexical_cast<std::string>( numVerts ) + " verticies.\n" +
                                  std::string( filenameOffsetMesh ) + " has " +
                                  boost::lexical_cast<std::string>( offsetMesh.vertex_count() ) + " verticies." );

    // assign vertex velocity
    if( !newMesh->has_vertex_channel( _T( "Velocity" ) ) )
        newMesh->add_vertex_channel<vector3f>( _T( "Velocity" ) );
    trimesh3_vertex_channel_accessor<vector3f> velAcc =
        newMesh->get_vertex_channel_accessor<vector3f>( _T( "Velocity" ) );
    for( size_t i = 0; i < numVerts; ++i )
        velAcc[i] = ( offsetMesh.get_vertex( i ) - newMesh->get_vertex( i ) ) / timeBetweenBaseAndOffset;

    // assign output mesh
    outTriangleMesh.m_data->mesh = newMesh;
    outTriangleMesh.m_data->velocityAcc = velAcc;
}

void triangle_mesh::initialize_velocity() {
    // initialize velocity accessor
    if( !m_data->velocityAcc.valid() ) {

        // add velocity channel and zero if it's not there
        if( !m_data->mesh->has_vertex_channel( _T( "Velocity" ) ) ) {
            m_data->mesh->add_vertex_channel<vector3f>( _T( "Velocity" ) );
            trimesh3_vertex_channel_accessor<vector3f> acc =
                m_data->mesh->get_vertex_channel_accessor<vector3f>( _T( "Velocity" ) );
            size_t vertCount = m_data->mesh->vertex_count();
            for( int i = 0; i < vertCount; ++i )
                acc[i].set( 0.0f );
        }
        m_data->velocityAcc = m_data->mesh->get_vertex_channel_accessor<vector3f>( _T( "Velocity" ) );
    }
}

} // namespace krakatoasr
