// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 14
-Writing out a PRT file.
-Saving particles from multiple PRT files into a single PRT file.
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // this example create 3 particle streams from a mesh and then saves them all into a new prt

        // To start we create the renderer object
        krakatoasr::krakatoa_renderer renderer;
        // create the triangle mesh object
        krakatoasr::triangle_mesh mesh;
        // set the mesh by loading in the file boxmesh.obj, the first parameter is which mesh object you want to fill
        mesh.load_from_file( mesh, "boxmesh.obj" );
        // create the particle stream from the mesh with voxel spacing of 0.1, 1 subdivision, jittered particles and 2
        // particles per jitter
        krakatoasr::particle_stream stream_one =
            krakatoasr::particle_stream::create_from_mesh_volume( mesh, .25, 1, true, 2 );
        // add the particle stream to the renderer
        renderer.add_particle_stream( stream_one );

        // create the particle stream from the mesh with voxel spacing of 0.1, 1 subdivision, jittered particles and 2
        // particles per jitter
        krakatoasr::particle_stream stream_two =
            krakatoasr::particle_stream::create_from_mesh_volume( mesh, .25, 1, true, 2 );
        stream_two.set_transform( krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 3, 0, 0, 1 ) );
        // add the particle stream to the renderer
        renderer.add_particle_stream( stream_two );

        // create the particle stream from the mesh with voxel spacing of 0.1, 1 subdivision, jittered particles and 2
        // particles per jitter
        krakatoasr::particle_stream stream_three =
            krakatoasr::particle_stream::create_from_mesh_volume( mesh, .25, 1, true, 2 );
        stream_three.set_transform( krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -3, 0, 0, 1 ) );
        // add the particle stream to the renderer
        renderer.add_particle_stream( stream_three );
        // tell the renderer we want to save the output to a prt file
        renderer.save_output_prt( "example14.prt", false );
        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
