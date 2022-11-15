// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 4
-Manually creates a small mesh (setting verts/faces manually)
-Use that mesh for occlusion
-Create fractals (self-illuminating)
-exr output
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // in this example we are creating a 1X1X1 square mesh centered around the origin which we are using for
        // occlusion with the same Fractals that were generated in example 1

        // To start we create the renderer object
        krakatoasr::krakatoa_renderer renderer;
        // We then set the render resolution. Since this is the default it is not necessary however is normally
        // included.
        renderer.set_render_resolution( 640, 480 );
        // set up a float array that includes all of the vertexes for the points in the ordering
        // vert1x,vert1y,vert1z,vert2,vert2y...
        float meshVerts[24] = { 0.5, 0.5,  0.5, 0.5, 0.5,  -0.5, -0.5, 0.5,  -0.5, -0.5, 0.5,  0.5,
                                0.5, -0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, 0.5 };
        // Set up all of the triangle faces of the mesh to create the box
        // faces are set up in the order tri1a, tri1b, tri1c, tri2a, tri2b...
        int meshFaces[36] = { // top
                              0, 1, 2, 0, 3, 2,
                              // bottom
                              6, 5, 4, 6, 7, 4,
                              // front
                              0, 3, 7, 0, 7, 4,
                              // left
                              2, 3, 6, 3, 6, 7,
                              // right
                              1, 0, 5, 1, 4, 5,
                              // back
                              2, 1, 6, 1, 6, 5 };
        // create a triangle mesh object
        krakatoasr::triangle_mesh mesh;
        // set the number of vertices that are going to be included in the mesh, this must be completed before the
        // vertices are added to the mesh
        mesh.set_num_vertices( 8 );
        // add the vertex data this takes a float array of size 3x where x is the number of vertices
        mesh.set_vertex_position_data( meshVerts );
        // set the number of faces that are going to be included in the mesh, this must be completed before the face
        // data is added to the mesh
        mesh.set_num_triangle_faces( 12 );
        // add the face data to the mesh this takes a float array of size 3x where x is the number of faces
        mesh.set_face_data( meshFaces );
        // add the mesh to the renderer with a set transform, for this example we are moving it 2 units towards the
        // camera so it occludes more of the fractals
        renderer.add_mesh( &mesh, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 2, 1 ) );

        // here we are creating a transform that we will be applying to the camera to move it 10 units in the z
        // direction
        krakatoasr::animated_transform cameraTransform =
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 10, 1 );
        // we now apply the transform to the camera
        renderer.set_camera_tm( cameraTransform );

        // Here we are setting the density of the particles in the final render.
        // after this change the end density will be 9*10^-1
        renderer.set_density_per_particle( 9 );
        // change the exponent on the final render particle density
        // after this change the end density will be 9*10^-3
        renderer.set_density_exponent( -3 );
        // we are now creating the parameters that will be used for the fractals
        krakatoasr::fractal_parameters fractalParams;
        // we are setting these parameters from a random generator using 5 affine transformations and 4 different colors
        // being used
        fractalParams.set_from_random( 5, 3, 46 );
        // we now create the particle stream from the fractal parameters from last step with a total particle count of
        // 5,000,000
        krakatoasr::particle_stream particleStream =
            krakatoasr::particle_stream::create_from_fractals( 5000000, fractalParams );
        // we now add the particle stream to the renderer
        renderer.add_particle_stream( particleStream );
        // since we are using emission instead of lights we must tell the renderer that.
        renderer.use_emission( true );
        // for the emission we want to lower the strength so that all of the pixels are do not have there color washed
        // out in this example I am using 1*10^-2
        renderer.set_emission_strength( 1.0 );
        renderer.set_emission_strength_exponent( -2 );
        // we now create a file saver to let krakatoa know where we want our results to go
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example04.exr" );
        // add the file saver to the renderer
        renderer.set_render_save_callback( &fileSaver );
        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
