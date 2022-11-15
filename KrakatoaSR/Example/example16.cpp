// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 16
-Writing out a PRT file.
-Writing out a particle set with a computed "Lighting" channel.
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // in this example we are taking a particle stream adding multiple lights to shine on it and then saving it out
        // to a prt this way the particles have a built in lighting channel

        krakatoasr::krakatoa_renderer renderer;
        // add several lights to r
        // Create a direct light to illuminate the particles originaly starts at the origin
        krakatoasr::direct_light dLight = krakatoasr::direct_light();
        // change the flux of the light, normal is 12,12,12 which is white light we are using yellow light
        dLight.set_flux( 20, 20, 0 );
        // add the light and specify that we want it 4 units from the origin
        renderer.add_light( &dLight, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 4, 1 ) );

        // Create a direct light to illuminate the particles originaly starts at the origin
        krakatoasr::point_light pLight = krakatoasr::point_light();
        // change the flux of the light, normal is 12,12,12 which is white light
        dLight.set_flux( 10, 20, 20 );
        // add the light and specify that we want it 5 units above the teapot
        renderer.add_light( &dLight, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 5, 0, 1 ) );
        // add the teapot to the renderer
        krakatoasr::particle_stream myStream = krakatoasr::particle_stream::create_from_file( "smallTeapot.prt" );
        renderer.add_particle_stream( myStream );

        // tell the renderer we want to save the output to a prt file
        renderer.save_output_prt( "example16.prt", false );
        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
