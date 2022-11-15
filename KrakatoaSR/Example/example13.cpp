// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 13
-Creates a "frame buffer" update display for displaying semi-rendered images to show progress while the renderer is
working. -This example produces a 20x20 image, and prints out the 20x20 as a grid to standard out.
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>
#include <string>
#include <vector>

// this is a custom frame buffer class that prints out the image using "." wherever the alpha channel is 0 and "#"
// everywhere else
class my_frame_buffer_interface : public krakatoasr::frame_buffer_interface {
  public:
    virtual void set_frame_buffer( int width, int height, const krakatoasr::frame_buffer_pixel_data* data ) {
        int count = 0;
        // since krakatoa stores the pixels starting at the bottom left corner we go in the revers order as normal
        for( int y = height - 1; y >= 0; --y ) {
            for( int x = 0; x < width; ++x ) {
                // check the alpha channel and print out the appropriate character
                float alpha =
                    ( data[x + y * width].r_alpha + data[x + y * width].g_alpha + data[x + y * width].b_alpha ) / 3.0f;
                if( alpha == 0 ) {
                    printf( "." );
                } else {
                    printf( "#" );
                }
            }
            // print out a newline
            printf( "\n" );
        }
    }
};

int main( void ) {
    try {
        // This example creates a frame buffer interface which is added to a render of fractals the interface that we
        // created prints out
        //"." wherever the alpha channel is 0 and "#"every where else

        // create a renderer object that will do the majority of the work
        krakatoasr::krakatoa_renderer renderer;
        // Set the resolution of the final render. we are making it smaller this time beacause of the way we areprinting
        // it out multiple times during the render
        renderer.set_render_resolution( 20, 20 );

        // here we are creating a transform that we will be applying to the camera to move it 10 units in the z
        // direction
        krakatoasr::animated_transform cameraTransform =
            krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 10, 1 );
        // we now apply the transform to the camera
        renderer.set_camera_tm( cameraTransform );
        // Here we are setting the density of the particles in the final render.
        // after this change the end density will be 9*10^-1
        renderer.set_density_per_particle( 5 );
        // change the exponent on the final render particle density
        // after this change the end density will be 9*10^-3
        renderer.set_density_exponent( -6 );
        // we are now creating the parameters that will be used for the fractals
        krakatoasr::fractal_parameters fractalParams;
        // we are setting these parameters from a random generator using 5 affine transformations and 4 different colors
        // being used
        fractalParams.set_from_random( 5, 3, 46 );
        // we now create the particle stream from the fractal parameters from last step with a total particle count of
        // 5,000,000
        krakatoasr::particle_stream particleStream =
            krakatoasr::particle_stream::create_from_fractals( 50000000, fractalParams );
        // we now add the particle stream to the renderer
        renderer.add_particle_stream( particleStream );

        renderer.use_emission( true );
        // for the emission we want to lower the strength so that all of the pixels are do not have there color washed
        // out in this example I am using 1*10^-2
        renderer.set_emission_strength( 7.0 );
        renderer.set_emission_strength_exponent( -6 );

        // create and instance of the new frame buffer interface and add it to the render
        my_frame_buffer_interface frame_buffer = my_frame_buffer_interface();
        renderer.set_frame_buffer_update( &frame_buffer );

        // create a file saver object so that we can save the results
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example13.exr" );
        // Add the file saver to the renderer
        renderer.set_render_save_callback( &fileSaver );

        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
