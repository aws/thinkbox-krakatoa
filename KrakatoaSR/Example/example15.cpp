// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 15
-Writing out a PRT file.
-Saving out particles generated from "Particle Multiplication".
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // In this example we are creating a particle stream from a mesh we are then adding a particle multiplication to
        // that particle stream to multiply the particles into the millions

        krakatoasr::set_global_logging_level( krakatoasr::LOG_DEBUG );
        // To start we create the renderer object
        krakatoasr::krakatoa_renderer renderer;
        // create the triangle mesh object
        // create the particle stream from the mesh with voxel spacing of 0.1, 1 subdivision, jittered particles and 2
        // particles per jitter
        //  Breaks the code somewhere not sure where
        krakatoasr::triangle_mesh mesh;
        // set the mesh by loading in the file boxmesh.obj, the first parameter is which mesh object you want to fill
        krakatoasr::triangle_mesh::load_from_file( mesh, "boxmesh.obj" );
        // create the particle stream from the mesh with voxel spacing of 0.1, 1 subdivision, jittered particles and 2
        // particles per jitter
        krakatoasr::particle_stream stream =
            krakatoasr::particle_stream::create_from_mesh_volume( mesh, .1, 1, true, 1 );
        // add the particle multiplication to the particle stream we can have thousands more
        // krakatoasr::add_particle_multiplication( stream, 3.0f, 2,1);
        // add the particle stream to the renderer
        renderer.add_particle_stream( stream );

        // tell krakatoa that you want to save the results to a prt
        renderer.save_output_prt( "example15.prt", false );
        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
