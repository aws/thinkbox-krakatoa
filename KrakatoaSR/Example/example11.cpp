// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 11
-Generates fractals
-Uses a mesh boxmesh.obj
-Render elements: occluded, z depth
-exr output as multi-channel exr
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
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
        // create the triangle mesh object
        krakatoasr::triangle_mesh mesh;
        // set the mesh by loading in the file boxmesh.obj, the first parameter is which mesh object you want to fill
        mesh.load_from_file( mesh, "boxmesh.obj" );
        renderer.add_mesh( &mesh, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 2, 1 ) );

        // we are now creating the parameters that will be used for the fractals
        krakatoasr::fractal_parameters fractalParams;
        // we are setting these parameters from a random generator using 5 affine transformations and 4 different colors
        // being used
        fractalParams.set_from_random( 5, 3, 46 );
        // we now create the particle stream from the fractal parameters from last step with a total particle count of
        // 5,000,000
        krakatoasr::particle_stream particleStream =
            krakatoasr::particle_stream::create_from_fractals( 5000000, fractalParams );
        // add the particle stream to the renderer
        renderer.add_particle_stream( particleStream );

        renderer.enable_occluded_rgba_render( true );
        renderer.enable_z_depth_render( true );

        // Create a direct light to illuminate the particles originaly starts at the origin
        krakatoasr::direct_light dLight = krakatoasr::direct_light();
        // change the flux of the light, normal is 12,12,12 which is white light we are using yellow light
        dLight.set_flux( 20, 20, 0 );
        // add the light and specify that we want it 4 units from the origin
        renderer.add_light( &dLight, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 4, 1 ) );

        // we now create a file saver to let krakatoa know where we want our results to go
        krakatoasr::multi_channel_exr_file_saver multichannelFileSaver =
            krakatoasr::multi_channel_exr_file_saver( "example11.exr" );
        // add the file saver to the renderer
        renderer.set_render_save_callback( &multichannelFileSaver );
        // and finally start the render
        renderer.render();
    } catch( std::exception& e ) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
