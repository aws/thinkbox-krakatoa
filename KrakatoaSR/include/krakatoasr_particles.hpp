// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_PARTICLES__
#define __KRAKATOASR_PARTICLES__

#include <krakatoasr_datatypes.hpp>
#include <krakatoasr_mesh.hpp>
#include <krakatoasr_transformation.hpp>

namespace krakatoasr {

//
//
// Particle streams
//
//

/**
 * Class that defines a particle stream for the renderer.
 * All streams provide particles that are at shutter time zero. They are offset to the desired shutter time based on
 * their "Velocity" channel. To create a stream object, users must use one of the static factory functions contained
 * within this class.
 */
class CLSEXPORT particle_stream {
  private:
    particle_stream_data* m_data;

  public:
    particle_stream();
    ~particle_stream();
    particle_stream( const particle_stream& t );
    particle_stream& operator=( const particle_stream& t );
    const particle_stream_data* get_data() const;
    particle_stream_data* get_data();

    /**
     * Sets a transformation matrix on the particles.
     * Typically used to move particles which are in object-space into world-space.
     * This matrix transforms all coordinate channels such as "Position" and "Velocity".
     * @param tm The matrix. Defaults to the identity matrix.
     */
    void set_transform( const animated_transform& tm );

  public:
    /**
     * Static factory function to create a particle stream from a "prt", "bin", or "csv" file.
     * @param filename The "prt", "bin", or "csv" file name.
     * @return The resulting particle_stream object which can be added to the renderer.
     */
    static particle_stream create_from_file( const char* filename );

    /**
     * Static factory function to create a particle stream by filling in particles within a mesh's volume.
     * This function may take a while to compute.
     * @param mesh The source triangle mesh.
     * @param voxelSpacing Controls the resolution of the voxel field. A smaller number will produce a higher resolution
     * levelset. Careful: A very small voxelSpacing may take a very long time to compute.
     * @param subdivCount Controls the number of particles that are seeded in each voxel. This number subdivides the
     * voxel into smaller voxels. Careful: Increasing this number will exponentially increase the number of voxels in
     * your render.
     * @param particleJitteringEnabled If true, particles will be placed randomly within their voxel subdivision.
     * @param numJitteredParticlesPerVoxel When jittering is enabled, this parameter controls how many particles are
     * placed within each voxel subdivision.
     * @param randomJitterSeed When jittering is enabled, this parameter controls the seed for the random generator.
     * @param wellDistributedJittering When jittering is enabled and this parameter is true, the random positions will
     * be seeded so that the distances between samples are uniform and avoid accidental clumping of particles. Setting
     * this parameter false will increase performance.
     * @param enableShell When true, only a "shell" around the mesh surface will be filled with particles.
     * @param shellStart When "enableShell" is true, this parameter controls the distance from the mesh surface to start
     * the "shell" effect.
     * @param shellThickness When "enableShell" is true, this parameter controls the thickness of the "shell" effect.
     * @param cancelCheckCallback A user cancel callback. This is provided because this function can take a long time to
     * compute. Passing in NULL means there is no callback. Normally the user will pass the same object as is given to
     * the krakatoa_renderer::set_cancel_render_callback function.
     * @param wasCancelled When "cancelCheckCallback" is non-null, this must point to a boolean. The boolean data is set
     * to true if the function was cancelled prematurely. Or false if the function completed successfully.
     * @return The resulting particle_stream object which can be added to the renderer.
     */
    static particle_stream
    create_from_mesh_volume( const triangle_mesh& mesh,                    // source mesh
                             float voxelSpacing, unsigned int subdivCount, // voxel seeding parameters
                             bool particleJitteringEnabled = false, int numJitteredParticlesPerVoxel = 1,
                             int randomJitterSeed = 42, bool wellDistributedJittering = false, // jittered parameters
                             bool enableShell = false, float shellStart = 0.0f,
                             float shellThickness = 1e10f, // shell parameter
                             cancel_render_interface* cancelCheckCallback = 0,
                             bool* wasCancelled = 0 ); // cancel progress callback parameters, NULL by default.

    /**
     * Static factory function to crate a particle stream by placing particles on the surface of a mesh.
     * @param mesh The source triangle mesh. Note: You don't have to keep this triangle_mesh object in memory, however,
     * Krakatoa will only make a soft-copy of its underlying data, so any modifications to the mesh after this call will
     * be reflected in the render.
     * @param particleCount Controls the total number of particles that will be seeded on the mesh surface. This
     * parameter is only used when useParticleSpacing is false.
     * @param particleSpacing Controls the total number of particle by using the surface collection's total area divided
     * by the average spacing. This parameter is only used when useParticleSpacing is true.
     * @param useParticleSpacing A switch parameter that controls how particles are seeded. When false, "particleCount"
     * is used, when true, "particleSpacing" is used.
     * @param randomSeed This parameter controls the seed for the random generator.
     */
    static particle_stream create_from_surface( const triangle_mesh& mesh, // source mesh
                                                INT64 particleCount = 0, float particleSpacing = 0.0f,
                                                bool useParticleSpacing = false, // voxel seeding parameters
                                                int randomSeed = 42 );

    /**
     * Static factory function to create a particle stream that produces particles in fractal patterns based on a set of
     * parameters.
     * @param particleCount How many particles to produce.
     * @param fractalParams The object that holds all the parameters needed to create the fractals. See the
     * "fractal_parameters" documentation for how to set these parameters.
     * @return The resulting particle_stream object which can be added to the renderer.
     */
    static particle_stream create_from_fractals( INT64 particleCount, const fractal_parameters& fractalParams );

    /**
     * Static factory function to create a particle stream from a custom implementation of particle_stream_interface.
     * @param userStream An object of a sub-class of particle_stream_interface which will provide custom particles to
     * the renderer. Only one particle_stream can be created per userStream instance.
     * @return The resulting particle_stream object which can be added to the renderer.
     */
    static particle_stream create_from_particle_stream_interface( particle_stream_interface* userStream );
};

//
//
// Modification functions for existing particle streams
//
//

/**
 * Adds a scale modification operator to a channel of an existing stream.
 * @param stream The particle stream to modify.
 * @param channelName The name of the channel to modify.
 * @param scaleValue The amount to scale the channel by.
 */
FCNEXPORT void channelop_scale( particle_stream& stream, const char* channelName, float scaleValue );

/**
 * Copys an existing channel to another channel name for an existing stream. It will add the destination channel or
 * overwrite it if it exists.
 * @param stream The particle stream to modify.
 * @param sourceChannelName The destination channel.
 * @param destChannelName The source channel.
 */
FCNEXPORT void channelop_copy( particle_stream& stream, const char* destChannelName, const char* sourceChannelName );

/**
 * Sets all value in a given channel to the provided value for an existing stream. Channel will be added if necessary.
 * NOTE: Normally the user would NOT use this function, and instead use: "channelop_set_float", "channelop_set_vector",
 * or "channelop_set_integer"
 * @param stream The particle stream to modify.
 * @param channelName The name of the channel to modify.
 * @param data_type_t The data type of "data".
 * @param arity The arity of "data".
 * @param data The raw memory buffer containing the data.
 */
FCNEXPORT void channelop_set( particle_stream& stream, const char* channelName, data_type_t dataType, int arity,
                              const void* data );
/// Sets all values in a floating-point channel.
inline void channelop_set_float( particle_stream& stream, const char* channelName, double value ) {
    channelop_set( stream, channelName, DATA_TYPE_FLOAT64, 1, &value );
}
/// Sets all values in a floating-point vector channel.
inline void channelop_set_vector( particle_stream& stream, const char* channelName, double vectorValue1,
                                  double vectorValue2, double vectorValue3 ) {
    double value[3] = { vectorValue1, vectorValue2, vectorValue3 };
    channelop_set( stream, channelName, DATA_TYPE_FLOAT64, 3, value );
}
/// Sets all values in an integer channel.
inline void channelop_set_integer( particle_stream& stream, const char* channelName, INT64 value ) {
    channelop_set( stream, channelName, DATA_TYPE_INT64, 1, &value );
}

/**
 * For computing the vector magnitude of a float[3] channel.
 * @param The particle stream to modify.
 * @param destChannelName The name of the channel that will hold the vector magnitude values. This channel will be
 * created if it does not already exist.
 * @param destChannelArity Defines the arity of the destination channel. This can be either 1, if you would like a float
 * channel created, or 3 if you would like a float[3] channel created. In the case of 3, all x,y,z components of the
 * resulting channel will be identical. They will contain the magnitude of the source channel.
 * @parma sourceChannelName The name of the channel that is the source vector for computation. This channel must be of
 * type float[3].
 */
FCNEXPORT void create_vector_magnitude_channel( particle_stream& stream, const char* destChannelName,
                                                int destChannelArity, const char* sourceChannelName );

/**
 * Modifies an existing particle stream.
 * It will multiply the particle count by filling particles within a radius around the original particle set.
 * This function has the potential to take a long time to compute because it will actually exhaust the original particle
 * stream, and replace it with a new stream with the multiplied particles.
 * @param stream This is the stream that will be muliplied. After this call, the stream will contain more particles.
 * @param fillRadius The radius around each particle to seed new particles.
 * @param fillRadiusSubdivs The number of seeding subdivisions. Increasing this number will exponentially increase
 * output particle count.
 * @param numParticlesPerSubdiv The number of particles placed in each subdivision. Increasing this number will linearly
 * increase output particle count.
 * @param densityFalloffStart A value between zero and one that defines the start (between the center and outer radius)
 * of the linear falloff of density. zero being no falloff, one being a linear falloff from the center of the original
 * particle to the outer radius.
 * @param progressLogger Logs the current progress that the particle repopulation is making. Pass in NULL to skip
 * progress logging.
 * @param cancelCheck An interface object to cancel the render early. Pass in NULL to skip cancel checking.
 */
FCNEXPORT void add_particle_repopulation( particle_stream& stream, float fillRadius, int fillRadiusSubdivs,
                                          int numParticlesPerSubdiv, float densityFalloffStart = 0.0f,
                                          unsigned randomSeed = 42, progress_logger_interface* progressLogger = 0,
                                          cancel_render_interface* cancelCheck = 0 );

/**
 * Moves the particle's Positions in time.
 * To use this function, your particle stream must have a "Velocity" channel. This function will push the particle
 * positions in time base based on their velocity.
 * @param stream The stream to modify
 * @param timeOffset The time in seconds to move the particles by.
 */
FCNEXPORT void add_time_offset( particle_stream& stream, float timeOffset );

/**
 * Reduces the number of particles in a stream.
 * The modified stream will contain the provided fraction of the original particles after the call.
 * @param stream The stream to modified. After the call the stream will have fewer particles.
 * @param fraction The fraction of particles that should remain in the stream. Must be between 0 and 1.
 */
FCNEXPORT void add_fractional_particle_stream( particle_stream& stream, float fraction );

//
//
// Class for creating fractal-style particles
//
//

/**
 * The class that holds all the parameters needed to create particle streams of fractals.
 * Fractals are generated from two or more "affine transforms". The affine transforms can also be generated randomly
 * based on a random seed. Fractals can be difficult
 */
class CLSEXPORT fractal_parameters {
  private:
    fractal_parameters_data* m_data;

  public:
    fractal_parameters();
    virtual ~fractal_parameters();
    fractal_parameters( const fractal_parameters& c );
    fractal_parameters& operator=( const fractal_parameters& c );
    const fractal_parameters_data* get_data() const;
    fractal_parameters_data* get_data();

    /**
     * Adds an affine transform to the spline generator.
     * This function must be called at least twice, since there must be a minimum two affine transforms to create
     * fractal particles. An alternative to calling this function is to let Krakatoa create a random set of affine
     * transforms based on a random seed using "set_from_random".
     * @param positionX, positionY, positionZ Defines the "position" element of the affine transform.
     * @param rotationX, rotationY, rotationZ, rotationW Defines the "rotation" quaternion for the affine transform.
     * @param skewOrientationX, skewOrientationY, skewOrientationZ, skewOrientationW Defines the "skew orientation"
     * quaternion for the affine transform.
     * @param skewAngle Defines a "skew angle" in degrees.
     * @param weight Value between zero and one. It affects the overall contribution of the affine transform to the
     * fractal particles.
     */
    void append_affine_transform( float positionX, float positionY, float positionZ, float rotationX, float rotationY,
                                  float rotationZ, float rotationW, float scaleX, float scaleY, float scaleZ,
                                  float skewOrientationX, float skewOrientationY, float skewOrientationZ,
                                  float skewOrientationW, float skewAngle, float weight = 1.0f );

    /**
     * Sets a value at a specific location along the fractal to interpolate color values.
     * Fractal particles also generate a "Color" and "Emission" channel, this allows the user to set which colors are
     * generated. Calling this function two or more times at different positions will produce gradients of colors. If
     * you imagine the fractal as a stright line of particles, the "position" is how far along that line this color
     * sample is applied. Adding more samples will produce linear gradients between the two samples.
     * @param r,g,b The color value at this position.
     * @param position A value between zero and one. Representing the position to apply this color along the fractal.
     */
    void append_color_gradient( float r, float g, float b, float position );

    /**
     * Sets the parameters to be a set of random affine transformations, and random color values.
     * This is an alternative to setting each affine transformation manually.
     * It is useful because fractals are very difficult to get a specific look by entering manual parameters.
     * @param affineTMCount This defines the number of randomly generated affine transforms to create (the equivalent to
     * calling "append_affine_transform" this many times). Must be two or more.
     * @param colorGradientCount The defines the number of randomly generated colors to apply evenly along the fractal
     * length.
     * @param randomSeed This is a random seed that drives both the affine transform creation and the color values.
     */
    void set_from_random( int affineTMCount, int colorGradientCount, int randomSeed );
};

//
//
// Class to create user-defined custom particle streams
//
//

/**
 * Helper struct for particle_stream_interface. See the particle_stream_interface documentation.
 * Used by the class's get_channel_value and set_channel_value.
 */
struct CLSEXPORT channel_data {
    int byteOffsetInParticle;
    int channelByteSize;
};

/**
 * Stream-style interface class for direct import of particles.
 * Users must create a sub-class and override all the pure virutal member functions.
 *
 * Example sub-class. This stream places 100 particles randomly within a 10x10x10 cube:
 * @code
 *	class my_stream : public krakatoasr::particle_stream_interface {
 *		krakatoasr::channel_data m_position;
 *		krakatoasr::channel_data m_velocity;
 *		krakatoasr::channel_data m_density;
 *		int m_count;
 *	public:
 *		my_stream() {
 *			m_position = append_channel( "Position", krakatoasr::DATA_TYPE_FLOAT32, 3 );
 *			m_velocity = append_channel( "Velocity",krakatoasr::DATA_TYPE_FLOAT32, 3 );
 *			m_density = append_channel( "Density", krakatoasr::DATA_TYPE_FLOAT32, 1 );
 *			m_count = 0;
 *		}
 *		virtual krakatoasr::INT64 particle_count() const {
 *			return 100;
 *		}
 *		virtual bool get_particle( void* particleData ) {
 *			//set these values to your own custom particle values
 *			float myPosition[3] = { rand() * 10.0f / RAND_MAX, rand() * 10.0f / RAND_MAX, rand() * 10.0f /
 *RAND_MAX }; float myVelocity[3] = { 0.0f, 0.0f, 0.0f }; float myDensity = 1.0f; set_channel_value( m_position,
 *particleData, myPosition ); set_channel_value( m_velocity, particleData, myVelocity ); set_channel_value( m_density,
 *particleData, &myDensity );
 *			++m_count;
 *			return ( m_count <= 100 );
 *		}
 *		virtual void close() {}
 *	};
 * @endcode
 */
class CLSEXPORT particle_stream_interface {
  private:
    particle_stream_interface_data* m_data;

  public:
    particle_stream_interface();
    virtual ~particle_stream_interface();
    particle_stream_interface( const particle_stream_interface& c );
    particle_stream_interface& operator=( const particle_stream_interface& c );
    const particle_stream_interface_data* get_data() const;
    particle_stream_interface_data* get_data();

    /**
     * Adds a channel to the stream.
     * This is typically called at construction time to define which channels will be available in the stream.
     * Note that all streams are required to have a "Position" channel. All other channels are optional.
     * All the channels must be defined prior to adding a stream object to the renderer. See example usage.
     * @param name The name of the channel. Channel names in Krakatoa have special meaning. The renderer uses the
     * following special channel names: "Position", "Color", "Density", "Lighting", "Velocity", "MBlurTime",
     * "Absorption", "Emission", "Normal", "Tangent", "Eccentricity", "SpecularPower", "SpecularLevel", "DiffuseLevel".
     * @param dataType Specifies the data type of the channel.
     * @param arity Specifies the arity of the channel. For example, "Position" needs an arity of 3 (xyz), and "Density"
     * needs an arity of 1.
     * @return A channel_data object. This object can be used by get_channel_value and set_channel_value during
     * get_next_particle. See example usage.
     */
    channel_data append_channel( const char* name, data_type_t dataType, int arity );

  public:
    /**
     * Defines how many particles this stream will provide. See example usage.
     * @return The number of particles in the stream, or -1 if unknown.
     */
    virtual INT64 particle_count() const = 0;

    /**
     * Gets the next particle in the stream.
     * The user must use the given memory buffer to set all the channels. This buffer contains all the channels
     * requested by "append_channel". Channels values for the current particle can be set by using set_channel_value.
     * See example usage.
     * @param particleData A buffer of memory provided by the system. It is the user's job to populate this buffer with
     * channel data. See example usage.
     * @return Whether or not there are any particles left in the stream.
     */
    virtual bool get_next_particle( void* particleData ) = 0;

    /**
     * Called by the system once the stream is exhaused.
     * This is a useful function for closing files, or performing cleanup as needed.
     */
    virtual void close() = 0;

  protected:
    /**
     * Sets a specific channel value within a particle buffer. See example usage.
     * @param channelData The "accessor" object. This object was provided by "append_channel".
     * @param particleData The buffer of data for the current particle.
     * @param inValue A pointer to the channel's new value being set. It should point to data that is of the channel's
     * own data type. See example usage.
     */
    void set_channel_value( const channel_data& channelData, void* particleData, const void* inValue ) const;

    /**
     * Strips out a specific channel value from a particle buffer. Its use is very similar to "set_channel_value".
     * @param channelData The "accessor" object. This object was provided by "append_channel".
     * @param particleData The buffer of data for the current particle.
     * @param outValue A pointer to the channel value that was stripped out from particleData. Once retrieved, it should
     * be cast as a pointer to the channel's own data type.
     */
    void get_channel_value( const channel_data& channelData, const void* particleData, void* outValue ) const;
};

} // namespace krakatoasr

#endif
