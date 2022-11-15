// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 7
-Uses an animated moving_teapot.prt (self-illuminated)
-Set the motion blur shutter
-exr output
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // In this example we are taking a prt file that has a velocity channel and applying motion blur to it

        // create a renderer object that will do the majority of the work
        krakatoasr::krakatoa_renderer renderer;
        // Set the resolution of the final render. Since this the default resolution it does not have to be called.
        renderer.set_render_resolution( 640, 480 );
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

        // we are enabling motion blur to simulate motion
        renderer.enable_motion_blur( true );
        // for the motion blur we are blurring it from time -1 to 1 taking 10 samples between that time and we are using
        // jittered motion blur
        renderer.set_motion_blur( 2.4, 2.6, 10, true );

        // here we are creating a particle stream using the prt file smallTeapot.prt
        krakatoasr::particle_stream particleStream =
            krakatoasr::particle_stream::create_from_file( "moving_teapot.prt" );
        // we are now moving the translating the particle stream down 1 unit so that it fits better in the end render
        particleStream.set_transform(
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, -1, 0, 1 ) );
        // we now add the particle stream to the renderer
        renderer.add_particle_stream( particleStream );

        // since we are using emission instead of lights we must tell the renderer that.
        renderer.use_emission( true );
        // for the emission we want to lower the strength so that all of the pixels are do not have there color washed
        // out in this example I am using 1*10^-2
        renderer.set_emission_strength( 1.0 );
        renderer.set_emission_strength_exponent( -2 );
        // we now create a file saver to let krakatoa know where we want our results to go
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example07.exr" );
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
