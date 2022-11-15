// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 12
-Generates fractals
-Uses a custom user-defined render saving code (user-created render_save_interface class)
-Render save code creates a text file that uses "." or "#" to draw the image
*/

#include <fstream>
#include <iostream>
#include <krakatoasr_renderer.hpp>
#include <sstream>
#include <string>

// this is a custom stream of particles which currently creates a square of particles
class text_image_saver : public krakatoasr::render_save_interface {
  private:
    std::string filename;

  public:
    text_image_saver( char* outputFile ) { filename = outputFile; }

    virtual void save_render_data( int width, int height, int imageCount, const krakatoasr::output_type_t* listOfTypes,
                                   const krakatoasr::frame_buffer_pixel_data* const* listOfImages ) {
        for( size_t i = 0; i < imageCount; ++i ) {
            krakatoasr::output_type_t imageType = listOfTypes[i];

            const krakatoasr::frame_buffer_pixel_data* imageData = listOfImages[i];
            if( imageType == krakatoasr::OUTPUT_RGBA ) {
                std::stringstream results;
                // skip the saving if the filename's empty
                if( !( filename == "" ) ) {
                    for( int y = height - 1; y >= 0; --y ) {
                        for( int x = 0; x < width; ++x ) {
                            const krakatoasr::frame_buffer_pixel_data& col = imageData[x + y * width];
                            float alpha = ( col.r_alpha + col.g_alpha + col.b_alpha ) / 3.0f;
                            if( alpha == 0 ) {
                                results << ".";
                            } else {
                                results << "#";
                            }
                        }
                        results << "\n";
                    }
                    std::ofstream myfile;
                    myfile.open( filename );
                    myfile << results.str();
                    myfile.close();
                }
            }
        }
    }
};

int main( void ) {
    try {
        // create a renderer object that will do the majority of the work
        krakatoasr::krakatoa_renderer renderer;
        // Set the resolution of the final render. we are making it smaller this time beacause of the way we are saving
        // the render
        renderer.set_render_resolution( 133, 100 );

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

        text_image_saver fileSaver = text_image_saver( "example12.txt" );
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
