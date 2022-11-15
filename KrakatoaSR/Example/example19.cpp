// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 19
-Adds two teapot.prt to the scene side-by-side.
-Uses "add_particle_multiplication" to ONE of the teapot to turn it into several million of particles.
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // This example creates 2 particles streams from the same prt file and then adds a particle multiplication to
        // one of them.

        // To start we create the renderer object
        krakatoasr::krakatoa_renderer renderer;
        // We then set the render resolution. Since this is the default it is not necessary however is normally
        // included.
        renderer.set_render_resolution( 640, 480 );

        // here we are creating a transform that we will be applying to the camera to move it 10 units in the z
        // direction
        krakatoasr::animated_transform cameraTransform =
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 10, 1 );
        // we now apply the transform to the camera
        renderer.set_camera_tm( cameraTransform );

        // here we are creating a particle stream using the prt file smallTeapot.prt
        krakatoasr::particle_stream particleStream1 =
            krakatoasr::particle_stream::create_from_file( "smallTeapot.prt" );
        // we are now moving the translating the particle stream down 1 unit so that it fits better in the end render
        // we are also moving it to the right so that we can see both teapots
        particleStream1.set_transform(
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 3, -1, 0, 1 ) );
        // we now add the particle stream to the renderer
        renderer.add_particle_stream( particleStream1 );

        // Here we are creating a second particle stream in the same way as the first
        krakatoasr::particle_stream particleStream2 =
            krakatoasr::particle_stream::create_from_file( "smallTeapot.prt" );
        // we are now moving the translating the particle stream down 1 unit so that it fits better in the end render
        // we are also moving it to the left 3 units so that we can see both teapots
        particleStream2.set_transform(
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, -3, -1, 0, 1 ) );
        // add the particle multiplication to this stream, filling a radius of 0.5 with 3 subdivisions for each and
        // putting 1 particle in each subdivision krakatoasr::add_particle_multiplication( particleStream2, .5f, 3, 1);
        // we now turn on emission so that the particles will self illuminate
        renderer.add_particle_stream( particleStream2 );
        renderer.use_emission( true );
        // we are now turning down the emission strength so that the particles are not washed out to much
        renderer.set_emission_strength( 5.0 );
        renderer.set_emission_strength_exponent( -3 );
        // create a file saver object so that we can save the results
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example21.exr" );
        // Add the file saver to the renderer
        renderer.set_render_save_callback( &fileSaver );
        // Begin the Render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
