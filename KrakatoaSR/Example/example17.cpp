// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 17
-Writing out a PRT file.
-Reducing a PRT file size by only saving "Position" and "Velocity" channels.
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // in this example we are saving a prt file that includes a velocity channel out to another prt with only the
        // channels we choose

        // create the renderer object
        krakatoasr::krakatoa_renderer renderer;
        // add the prt file to the renderer
        krakatoasr::particle_stream myStream = krakatoasr::particle_stream::create_from_file( "moving_teapot.prt" );
        renderer.add_particle_stream( myStream );
        // tell the renderer that we are going to save the results to a prt file, and that we are not going to use the
        // default channels
        renderer.save_output_prt( "example17.prt", false, false ); // set useDefaultChannels to false
        // append the channels that we would like onto the output file
        // in this case we are getting the position and velocity channels
        renderer.append_output_prt_channel( "Position", krakatoasr::DATA_TYPE_FLOAT32, 3 );
        renderer.append_output_prt_channel( "Velocity", krakatoasr::DATA_TYPE_FLOAT16, 3 );
        // Start up the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
