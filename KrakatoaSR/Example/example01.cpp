// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.
Note, the examples are somewhat disorganized at the moment.

EXAMPLE 1
-Set all the parameters of r to default values
-Uses set_global_logging_level to debug
-Creates fractals (self-illuminating)
-exr output
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>

int main( void ) {
    try {
        // This example first explicitly sets all of the renderers settings to default.
        // It then creates a scene with the camera 10 units form the origin and a fractal located at the origin
        // The fractal is illuminated by its emission channel

        // Sets the level logging level of the renderer to determine what messages to show.
        // There is no default for this example we are using Debug which will show all messages
        krakatoasr::set_global_logging_level( krakatoasr::LOG_DEBUG );
        // The first block of code will be going through all of the different calls of the krakatoasr renderer setting
        // them to the defaults and giving some information about them since these are all being set to there default
        // values they are not required unless you are changing the values

        // create a renderer object that will do the majority of the work
        krakatoasr::krakatoa_renderer renderer;
        // if this setting is true then krakatoa will throw an error when no valid license is found
        // when it is false it will instead watermark the end render
        renderer.set_error_on_missing_license( false );
        // this will set the background color to the given rgb values
        renderer.set_background_color( 0, 0, 0 );
        // this is the coefficient of the density as expressed in scientific notation, which determines the size of the
        // particles
        renderer.set_density_per_particle( 5.0 );
        // this is the exponent power of 10 for expressing the density in scientific notation
        renderer.set_density_exponent( -1 );
        // this is the coefficient of the lighting density as expressed in scientific notation
        renderer.set_lighting_density_per_particle( 5.0 );
        // this is the exponent power of 10 for expressing the lighting density in scientific notation
        renderer.set_lighting_density_exponent( -1 );
        // if the use_emission property is set to true then all particles will self illuminate
        renderer.use_emission( false );
        // the emission strength effect the amount of light created from the particles
        renderer.set_emission_strength( 5.0 );
        // the emission strength exponent is used with the emission strength to define the emission of each particle
        renderer.set_emission_strength_exponent( -1 );
        // when use absorption color is set to true the user can define per-particle absorption values if the particles
        // have an absorption channel
        renderer.use_absorption_color( false );
        // when set to particle,each particle will be rendered as pixel-sized points
        // when set to voxel, each particle will be encoded onto a voxel grid which iwll be shaded
        renderer.set_rendering_method( krakatoasr::METHOD_PARTICLE );
        // this controls the number of neighboring voxels to filter over when shading with 0 meaning none
        renderer.set_voxel_filter_radius( 1 );
        // the radius of the voxels
        renderer.set_voxel_size( 0.5 );
        // this determines how particles are rendered in particle mode when they fall partially into more than one
        // integral pixel 	Nearest - The full value of the particle is placed into the nearest pixel of the output image.
        //   Bilinear - Uses a simple linear equation to split the value of the particle into the affected pixels of the
        //   output image. Bicubic - Uses a cubic filtering function to average the value of the particle into
        //   surrounding pixels of the output image
        // the last parameter determines the filter size for a bilinear filter
        renderer.set_draw_point_filter( krakatoasr::FILTER_BILINEAR, 1 );
        // this method determines if additive mode is being used when it is the color channel of each particle is copied
        // onto its emission channel there color and absorption channels are then set to black
        renderer.set_additive_mode( false );
        // the attenuation lookup filter determines how particle self-shadowing attenuation is evaluated from the
        // attenuation buffer in Particle Mode
        renderer.set_attenuation_lookup_filter( krakatoasr::FILTER_BICUBIC, 1 );
        // this will increase the number of subdivisions that are created during the lighting pass to gett a better
        // understanding of the shadows
        renderer.set_matte_renderer_supersampling( 1 );
        // sets a deep image or multilayered file to be used as a holdout mask
        renderer.set_deep_matte_filename( "" );
        // enables z-depth output in the render
        renderer.enable_z_depth_render( false );
        // enables a normal vector image output in the renderer
        renderer.enable_normal_render( false );
        // enables a velocity vector image output in the render
        renderer.enable_velocity_render( false );
        // enables an occluded RBGA image output in the renderer.  This is analogous to a two layer deep image
        renderer.enable_occluded_rgba_render( false );
        // set the transform that is applied to the camera before the render
        renderer.set_camera_tm( krakatoasr::animated_transform() );
        // set the type of camera that is being used for the render
        // Perspective and orthographic are currently the only types available
        renderer.set_camera_type( krakatoasr::CAMERA_PERSPECTIVE );
        // the horizontal field of view for the camera in radians
        renderer.set_camera_perspective_fov( 1.5708 );
        // set the camera's clipping planes only particles that are inside the clipping plane will be drawn
        renderer.set_camera_clipping( 0.001, 1e+10f );
        // sets and offset which is turned into a trnasform on the screenspace. Units in pixels
        renderer.set_screen_offset( 0, 0 );
        // the resolution of the final render
        renderer.set_render_resolution( 640, 480 );
        // the aspect ratio of the pixels in the render
        renderer.set_pixel_aspect_ratio( 1.0f );
        // when this is true motion blur will be used according to the motion blur properties
        renderer.enable_motion_blur( false );
        // this set the initial time, end time, number of samples taken and if the motion blur will be jittered
        renderer.set_motion_blur( 0, 0, 2, false );
        // this makes it so that depth of field will be used to blur particles
        renderer.enable_depth_of_field( false );
        // the aperture size of the camera, the focal length, focal distance and sample rate, for which the depth of
        // field is created
        renderer.set_depth_of_field( 1e30, 30.0, 100, 0.1 );
        // when this a non blank file name the renderer will create a prt of the scene so that it can be imported into
        // other projects
        renderer.save_output_prt( "", false, true );
        // End of Default Parameters

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
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example1.exr" );
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
