// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.
Note, the examples are somewhat disorganized at the moment.

EXAMPLE 2
-Uses teapot.prt particles
-Uses 3 different light types
-Sets the shader
-exr output
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // In this example we are taking a prt file of a teapot, which has an emission channel, and adding it to a scene
        // and having the particles self illuminate

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
        krakatoasr::particle_stream particleStream = krakatoasr::particle_stream::create_from_file( "smallTeapot.prt" );
        // we are now moving the translating the particle stream down 1 unit so that it fits better in the end render
        particleStream.set_transform(
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, -1, 0, 1 ) );
        // we now add the particle stream to the renderer
        renderer.add_particle_stream( particleStream );
        // we now turn on emission so that the particles will self illuminate
        renderer.use_emission( true );
        // we are now turning down the emission strength so that the particles are not washed out to much
        renderer.set_emission_strength( 5.0 );
        renderer.set_emission_strength_exponent( -3 );
        // create a file saver object so that we can save the results
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example02.exr" );
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
