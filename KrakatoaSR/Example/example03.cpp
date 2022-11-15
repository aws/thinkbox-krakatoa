// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/*
Krakatoa SR API example file.

EXAMPLE 3
-Load a custom user-defined particle stream
-Uses one direct light
-exr output
*/

#include <iostream>
#include <krakatoasr_renderer.hpp>
#include <random>

// this is a custom stream of particles which currently creates a square of particles
class my_stream : public krakatoasr::particle_stream_interface {
    krakatoasr::channel_data m_position;
    krakatoasr::channel_data m_velocity;
    krakatoasr::channel_data m_density;
    krakatoasr::channel_data m_color;
    krakatoasr::INT64 m_currentParticle;
    krakatoasr::INT64 m_particleCount;

  public:
    my_stream( krakatoasr::INT64 particleCount = 0 ) {
        // the channel names used have special meaning to krakatoa.  They are case-sensitive, and generally
        // must have the correct data type (i.e. some kind of float) and arity.
        m_position = append_channel( "Position", krakatoasr::DATA_TYPE_FLOAT32, 3 );
        m_velocity = append_channel( "Velocity", krakatoasr::DATA_TYPE_FLOAT32, 3 );
        m_density = append_channel( "Density", krakatoasr::DATA_TYPE_FLOAT32, 1 );
        m_color = append_channel( "Color", krakatoasr::DATA_TYPE_FLOAT32, 3 );
        m_currentParticle = 0;
        m_particleCount = particleCount;
    }
    krakatoasr::INT64 particle_count() const { return m_particleCount; }
    // the main working method of the interface which creates each particle
    // returns true if the particle was created succesfully
    bool get_next_particle( void* particleData ) {
        // check if we have any more particles to emit
        if( m_currentParticle < m_particleCount ) {
            // the particles will be in a 1X1X1 square centered on the origin
            float myPosition[3] = { ( (float)rand() / RAND_MAX ) - 0.5, ( (float)rand() / RAND_MAX ) - 0.5,
                                    ( (float)rand() / RAND_MAX ) - 0.5 };
            // the particles will not be moving
            float myVelocity[3] = { 0.0f, 0.0f, 0.0f };
            // the particles will be white, however because of the light we are using they will appear yellow
            float myColor[3] = { 1, 1, 1 };
            // the particles have a density of 1
            float myDensity = 1.0f;
            // add the particles to stream
            set_channel_value( m_position, particleData, myPosition );
            set_channel_value( m_velocity, particleData, myVelocity );
            set_channel_value( m_color, particleData, myColor );
            set_channel_value( m_density, particleData, &myDensity );
            ++m_currentParticle;
            return true;
        }
        return false;
    }

    void close() {}
};

int main( void ) {
    try {
        // In this example we are creating a custom stream of particles which is creating a box  filled with particles
        // we are then adding a light to the scene to illuminate the particles

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

        // Create a direct light to illuminate the particles originaly starts at the origin
        krakatoasr::direct_light dLight = krakatoasr::direct_light();
        // change the flux of the light, normal is 12,12,12 which is white light we are using yellow light
        dLight.set_flux( 20, 20, 0 );
        // add the light and specify that we want it 4 units from the origin
        renderer.add_light( &dLight, krakatoasr::animated_transform( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 4, 1 ) );
        // create the particle stream interface with 1,000,000 particles
        my_stream particleStreamInterface( 1000000 );
        // create a particle stream from the interface
        krakatoasr::particle_stream particleStream =
            krakatoasr::particle_stream::create_from_particle_stream_interface( &particleStreamInterface );
        // add the particles to the stream
        renderer.add_particle_stream( particleStream );

        // create a file saver object so that we can save the results
        krakatoasr::file_saver fileSaver = krakatoasr::file_saver( "example03.exr" );
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
