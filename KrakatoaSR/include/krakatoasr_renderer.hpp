// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_RENDERER__
#define __KRAKATOASR_RENDERER__

#include <cstdlib>

#include <krakatoasr_datatypes.hpp>
#include <krakatoasr_light.hpp>
#include <krakatoasr_mesh.hpp>
#include <krakatoasr_particles.hpp>
#include <krakatoasr_progress.hpp>
#include <krakatoasr_render_saving.hpp>
#include <krakatoasr_shader.hpp>
#include <krakatoasr_transformation.hpp>

namespace krakatoasr {

/**
 * This is the main Krakatoa renderer class.
 * It is used to collect all the parameters, lights, particles, meshes, etc. and produce a render.
 */
class CLSEXPORT krakatoa_renderer {
  private:
    krakatoa_renderer_params* m_data;

  public:
    krakatoa_renderer();
    ~krakatoa_renderer();
    krakatoa_renderer( const krakatoa_renderer& t );
    krakatoa_renderer& operator=( const krakatoa_renderer& t );
    const krakatoa_renderer_params* get_params() const;
    krakatoa_renderer_params* get_params();

    //
    // general rendering options
    //

    /**
     * @param errorOnMissingLicense When set to true, the renderer will thow an error when no license is found. When set
     * to false, the renderer will produce a watermarked image. Defaults to false.
     */
    void set_error_on_missing_license( bool errorOnMissingLicense );
    /**
     * @param r,g,b Defines the background color used in the output image. Defaults to [0,0,0].
     */
    void set_background_color( float r, float g, float b );
    /**
     * The first of two settings required to define the rendered density of each particle.
     * This value is the coefficient of the density as expressed in scientific notation. Smaller values make the
     * particles appear less dense, while larger values increase the rendered density.
     * @param densityPerParticle Defines the density coefficient of the particles during the render pass. Defaults
     * to 5.0.
     */
    void set_density_per_particle( float densityPerParticle );
    /**
     * The second of two settings required to define the rendered density of each particle.
     * This value is the exponent of the power of 10, where the density is expressed in scientific notation.
     * This provides a quick way to make large density adjustments.
     * @param densityExponent Defines the density exponent of the particles during the render pass. Defaults to -1.0.
     */
    void set_density_exponent( int densityExponent );
    /**
     * The first of two settings required to define the lighting pass density of each particle. Typically this is the
     * same as render-pass density. This value is the coefficient of the density as expressed in scientific notation.
     * There are two sets of density controls "Lighting Pass Density" and "Final Pass Density".
     * The Lighting Pass performs the calculation of attenuation maps used for particle shadowing. The Final Pass draws
     * the particles or voxels into the frame buffer.
     * @param lightingDensityPerParticle Defines the density coefficient of the particles during the lighting pass.
     * Defaults to 5.0.
     */
    void set_lighting_density_per_particle( float lightingDensityPerParticle );
    /**
     * The second of two settings required to define the lighting pass density of each particle. Typically this is the
     * same as render-pass density exponent. This value is the exponent of the power of 10, where the density is
     * expressed in scientific notation. This provides a quick way to make large density adjustments.
     * @param lightingDensityExponent Defines the density exponent of the particles during the lighting pass. Defaults
     * to -1.0.
     */
    void set_lighting_density_exponent( int lightingDensityExponent );
    /**
     * Sets whether or not the particles will "emit" light without any light source.
     * The amount of light emitted will be set by the "set_emission_strength" and "set_emission_strength_exponent"
     * functions. If an "Emission" channel exists within the particles, the renderer will use this channel as a
     * multiplier of emission strength.
     * @param useEmission Defines whether or not the particles will emit light. Defaults to false.
     */
    void use_emission( bool useEmission );
    /**
     * The first of two settings required to define the emission of each particle. Typically this is the same as
     * render-pass density. This value is the coefficient of the emission as expressed in scientific notation.
     * @param emissionStrength Defines the emission coefficient of the particles. Defaults to 5.0.
     */
    void set_emission_strength( float emissionStrength );
    /**
     * The second of two settings required to define the emission of each particle. Typically this is the same as
     * render-pass density exponent. This value is the exponent of the power of 10, where the emission is expressed in
     * scientific notation. This provides a quick way to make large emission adjustments.
     * @param emissionStrengthExponent Defines the emission exponent of the particles. Defaults to -1.0.
     */
    void set_emission_strength_exponent( int emissionStrengthExponent );
    /**
     * The "Absorption" channel of particles can control how much light a particle will absorb.
     * When using the "Absorption" channel, the user can define per-particle absorption values.
     * When not using the "Absorption" channel, all particles will have uniform absorbtion value of 1.0.
     * @param useAbsorptionColor Defines non-uniform per-particle absorption values. Particles must have an "Absorption"
     * channel. Defaults to false.
     */
    void use_absorption_color( bool useAbsorptionColor );
    /**
     * Sets Krakatoa's rendering mode.
     * When set to "particle", every particle will be rendered as a pixel-sized point.
     * When set to "voxel", every particle will be encoded onto a voxel grid and the grid will be shaded.
     * @param method The rendering method to use. Defaults to krakatoasr::METHOD_PARTICLE.
     */
    void set_rendering_method( rendering_method_t method );
    /**
     * Controls the number of neighbor voxels to filter over when shading a voxel.
     * A value of 1 means no filtering. Higher values produce fuzzier results at cost of render time.
     * Only used in "voxel" rendering mode.
     * @param radius The filter's radius. Defaults to 1.
     */
    void set_voxel_filter_radius( int radius );
    /**
     * Sets the global the resolution of the voxel grid.
     * Only used in "voxel" rendering mode.
     * @param voxelSize The size of the voxels. Defaults to 0.5.
     */
    void set_voxel_size( float voxelSize );
    /**
     * The Draw Point Filter determines how particles are rendered in Particle Mode when they fall partially into more
     * than one integral pixel in the image plane. Used during the render pass. Options are: Nearest - The full value of
     * the particle is placed into the nearest pixel of the output image. Bilinear - Uses a simple linear equation to
     * split the value of the particle into the affected pixels of the output image. Bicubic - Uses a cubic filtering
     * function to average the value of the particle into surrounding pixels of the output image
     * @param filter The filter style to use. Defaults to krakatoasr::FILTER_BILINEAR
     * @param filterSize The filter size. Only used for the "Bilinear" filter. Example: A value of 1 affects the nearest
     * 4 pixels. Defaults to 1.
     */
    void set_draw_point_filter( filter_t filter, int filterSize = 1 );
    /**
     * The Attenuation lookup filter determines how particle self-shadowing attenuation is evaluated from the
     * attenuation buffer in Particle Mode. Used during the lighting pass. Options are: Nearest - The full value of the
     * particle is placed into the nearest pixel of the output image. Bilinear - Uses a simple linear equation to split
     * the value of the particle into the affected pixels of the output image. Bicubic - Uses a cubic filtering function
     * to average the value of the particle into surrounding pixels of the output image
     * @param filter The filter style to use. Defaults to krakatoasr::FILTER_BICUBIC
     * @param filterSize The filter size. Only used for the "Bilinear" filter. Example: A value of 1 affects the nearest
     * 4 pixels. Defaults to 1.
     */
    void set_attenuation_lookup_filter( filter_t filter, int filterSize = 1 );
    /**
     * This special override forces additive rendering by overwriting certain particle channels under the hood.
     * The following operations will be performed internally when this option is checked:
     *   The Color Channel will be copied into the Emission Channel.
     *   The Color and Absorption channels will be set to black.
     * @param additiveMode Whether or not to use this mode. Defaults to false
     */
    void set_additive_mode( bool additiveMode );
    /**
     * Defines the size of the holdout matte render should be rendered at.
     * This setting is NOT used to anti-alias the edge of the matte cutout.
     * Use this setting to match what the geometry renderer is using for subdivisions.
     * @param rayDivisions The number of subdivisions of the matte render. 1 will produce a render at the render
     * resolution. 2 will produce a render twice the resolution. Defaults to 1.
     */
    void set_matte_renderer_supersampling( int subDivisions );
    /**
     * Sets a multi-layered "exr" file to be used as a holdout mask.
     * This can be used in place of rendering meshes to produce holdouts.
     * @param filename The multi-layered "exr" file. Defaults to ""
     */
    void set_deep_matte_filename( const char* filename );

    //
    // shader options
    //

    /**
     * Sets the phase function Krakatoa will use to render the particles.
     * This function makes a copy of the shader object.
     * @param shader A shader object. Must be a subclass of krakatoasr::shader provided by this API.
     */
    void set_shader( const shader* shader );

    /**
     * Enables multi-shader mode. If NULL is passed in for any parameter, default values will be used for that shader.
     * A particular particle will be shaded
     * @param isotropic An isotropic shader
     * @param phong A phong shader
     * @param henyey a henyey greenstein shader
     * @param schlick a Schlick shader
     * @param kajiya a Kajiya Kay shader
     * @param marschner a Marschner shader
     */
    void set_enable_multi_shader_mode( const shader_isotropic* isotropic = NULL, const shader_phong* phong = NULL,
                                       const shader_henyey_greenstein* henyey = NULL,
                                       const shader_schlick* schlick = NULL, const shader_kajiya_kay* kajiya = NULL,
                                       const shader_marschner* marschner = NULL );

    //
    // render elements options
    //

    /**
     * Enables a Z-depth image output in the renderer.
     * Defaults to false
     */
    void enable_z_depth_render( bool enable );
    /**
     * Enables a normal vector image output in the renderer.
     * Defaults to False
     */
    void enable_normal_render( bool enable );
    /**
     * Enables a velocity vector image output in the renderer.
     * Defaults to false
     */
    void enable_velocity_render( bool enable );
    /**
     * Enables an "occluded RBGA" image output in the renderer.
     * The reason this is used is for compositing Krakatoa images that have matte objects with the geometry renders of
     * those matte objects. This render layer provides a image of what is "behind" the occluded/semi-occluded. It can be
     * used to during compositing to be placed "behind" the geometry, and the normal RBGA can be used to composite
     * "over" the geometry render. It is analogous to a two layer "deep image", and is meant to solve the simple
     * compositing cases of when deep image outputs are desired, but not provided by the geometry renderer. Defaults to
     * false
     */
    void enable_occluded_rgba_render( bool enable );

    //
    // camera options
    //

    /**
     * Adds a new camera based on the values in krakatoa_render_params.
     * @param reset Whether to call reset_camera immediately after this method.
     */
    void push_camera( bool reset = true );
    /**
     * Resets all camera member variables, except for the vector
     */
    void reset_camera();
    /**
     * Add a new camera with the given parameters.
     */
    void add_camera( const animated_transform& tm = animated_transform(),
                     camera_type_t type = krakatoasr::CAMERA_PERSPECTIVE, float fov = 1.5708,
                     float orthographicWidth = 30.0f, float nearClipping = 0.001f, float farClipping = 1e+10f,
                     float horizontalOffset = 0.0f, float verticalOffset = 0.0f, int xResolution = 640,
                     int yResolution = 480, float pixelAspectRatio = 1.0f,
                     render_save_interface* renderSaveCallback = NULL );
    /**
     * Removes all added cameras.
     */
    void clear_cameras();
    /**
     * Sets the scene camera position.
     * @param tm The world-space to camera-space transformation matrix. Defaults to the identity matrix.
     */
    void set_camera_tm( const animated_transform& tm );
    /**
     * Sets the camera model.
     * There are currently two camera options: Perspective and Orthographic camera models.
     * @param type The camera type. Defaults to krakatoasr::CAMERA_PERSPECTIVE.
     */
    void set_camera_type( camera_type_t type );
    /**
     * Sets the camera's field of view.
     * Only applicable when using Perspective-type camera.
     * @param horizonalFov The horizonal field of view in radians. Defaults to 1.5708 (90 degrees).
     */
    void set_camera_perspective_fov( float horizonalFov );
    /**
     * Sets the cameras width when set to orthographic
     * @param orthographicWidth the width of the camera. Defaults to 30.0f
     */
    void set_camera_orthographic_width( float orthographicWidth );
    /**
     * Sets the camera's clipping planes
     * Particles outside these clipping planes will not be drawn.
     * @param nearClipping The near camera clipping plane. Defaults to 0.001.
     * @param farClipping The far camera clipping plane. Defaults to 1e+10.
     */
    void set_camera_clipping( float nearClipping, float farClipping );
    /**
     * Sets a screen space x,y "film offset" to the camera.
     * @param horizontalOffset Screen space horizontal offset in pixels. Defaults to 0.0f.
     * @param verticalOffset Screen space vertical offset in pixels. Defaults to 0.0f.
     */
    void set_screen_offset( float horizontalOffset, float verticalOffset );

    //
    // render output
    //

    /**
     * Sets the resolution of the output rendered image.
     * @param xResolution,yResolution Defaults to [640,480].
     */
    void set_render_resolution( int xResolution, int yResolution );
    /**
     * Sets the aspect ratio of the pixels for the final rendered image.
     * @param ratio The width-to-height ratio of the output image pixels. Defaults to 1.0.
     */
    void set_pixel_aspect_ratio( float ratio );

    //
    // motion blur and depth of field
    //

    /**
     * Enable motion blur in the scene.
     * If this option is enabled, the user must also set the shutterBegin and shutterEnd by using "set_motion_blur".
     * @param enableMotionBlur Whether or not to use motion blur. Defaults to false
     */
    void enable_motion_blur( bool enableMotionBlur );
    /**
     * Sets up the motion blur parameters.
     * This function is only applicable when motion blur has been enabled using "enable_motion_blur".
     * @param shutterBegin The time in seconds of the beginning of the motion blur segement. Zero being the render time.
     * Defaults to 0.
     * @param shutterEnd The time in seconds of the end of the motion blur segment. Zero being the render time. Defaults
     * to 0.
     * @param numSamples The number of motion blur samples to use for motion blur. Defaults to 2.
     * @param jitteredMotionBlur Whether or not to place particles randomly between motion blur samples, rather than
     * exactly at the sample time. Useful to reduce "stepping" in the motion blur. Defaults to false.
     */
    void set_motion_blur( float shutterBegin, float shutterEnd, int numSamples, bool jitteredMotionBlur );
    /**
     * Enables depth of field rendering of the particles.
     * If this option is enabled, the user may also want to set the depth of field parameters using
     * "set_depth_of_field".
     * @param enableDepthOfField Whether or not to use depth of field. Defaults to false
     */
    void enable_depth_of_field( bool enableDepthOfField );

    /**
     * Enables camera blur in rendering of the particles.
     * @param enableCameraBlur Whether or not to use camera blur. Defaults to true
     */
    void enable_camera_blur( bool enableCameraBlur );

    /**
     * Sets up the depth of field parameters.
     * This function is only applicable when depth of field has been enabled using "enable_depth_of_field".
     * @param fStop Standard f-stop/aperture size camera parameter. Defaults to 1e30 (very large). The default is often
     * acceptable.
     * @param focalLength Standard focal length camera parameter. Defaults to 30.0.
     * @param focalDistance Standard focal distance camera parameter. Defaults to 100.0.
     * @param sampleRate Controls the "quality" of the depth of field effect. This value scales the amount of samples
     * used per particle. 0.1 is fairly poor quality, and 1.0 is fairly high quality but will be slower. Defaults to
     * 0.1.
     */
    void set_depth_of_field( float fStop, float focalLength, float focalDistance, float sampleRate );

    /**
     * Controls the use of adaptive motion blur.
     * Adaptive motion blur automatically adjusts the number of motion blur samples that will be used
     * based on the velocity of the fastest moving particle in screen space.
     * Motion blur itself must be enabled to use adaptive motion blur.
     * @param enable enable adaptive motion blur if true, disables otherwise.
     */
    void enable_adaptive_motion_blur( bool enable = true );

    /**
     * Set a hard cap on the minimum number of samples adaptive motion blur can use.
     * @param minSamples the cap.
     */
    void set_adaptive_motion_blur_min_samples( int minSamples );

    /**
     * Set a hard cap on the maximum number of samples adaptive motion blur can use.
     * @param maxSamples the cap.
     */
    void set_adaptive_motion_blur_max_samples( int maxSamples );

    /**
     * Set a coefficient for the number of motion blur samples to be multiplied against.
     * This still respects the minimum and maximum number of samples.
     * @param smoothness the coefficient.
     */
    void set_adaptive_motion_blur_smoothness( float smoothness );

    /**
     * Set an exponent which can be used to control the curve for which the number of samples changes
     * with respect to maximum screen space particle velocity.
     * @param exponent the exponent
     */
    void set_adaptive_motion_blur_exponent( float exponent );

    /**
     * Sets a texture for which the alpha channel will be used to control the probability
     * of a sample being drawn in a particular location within a bokeh.
     * This allows the shape of the bokeh to be controlled.
     * @param texture The texture to use for probabilities. The pixel buffer must be width*width long,
     *                    and width must be a power of 2.
     */
    void set_bokeh_shape_map( const texture_data& texture );

    /**
     * Sets a texture, the RGB data of which is used to blend the color of samples drawn for a bokeh.
     * @param texture The texture to use for the blending. The pixel buffer must be width*width long,
     *                    and width must be a power of 2.
     */
    void set_bokeh_blend_map( const texture_data& texture );

    /**
     * Sets a value used to control how strongly the original color of a bokeh sample is blended using the blend map.
     * Note that this is overriden if the particles have a "BokehBlendInfluence" channel.
     * @param influence The infleunce value [0.0f, 1.0f]
     */
    void set_bokeh_blend_influence( float influence );

    /**
     * Squeezes bokehs to simulate an anamorphic lens.
     * @param squeeze The amount to squeeze the bokeh. 1.0f means no squeeze,
     *                  values less than 1.0f will result in a verticle squeeze,
     *                  values greater than 1.0f will result in a horizontal squeeze.
     *                  values will be clamped to the range [0.1f, 10.0f].
     */
    void set_anamorphic_squeeze( float squeeze );

    /**
     * Sets a coefficient to multiply the resolution of each bokeh by before selecting an appropriate mipmap level.
     * This can be used to reduce banding artifacts that sometimes appear when using a bokeh blend map.
     * The default is 1 (no scaling). The higher this is set to, the noisier the colors of in-focus particles will be.
     * @param scale The scale to multiply by.
     */
    void set_bokeh_blend_mipmap_scale( int scale );

    /**
     * Sets if the "BokehBlendInfluence" channel should be allocated or not.
     * @param allocate If true, the "BokehBlendInfluence" channel will be allocated.
     */
    void set_allocate_bokeh_blend_influence( bool allocate );

    //
    // light options
    //

    /**
     * Adds a light to the scene.
     * This function makes a copy of the light.
     * @param lt A light object. Must be a subclass of krakatoasr::light provided by this API.
     * @param tm The world-space transformation matrix of this light.
     */
    void add_light( const light* lt, const animated_transform& tm );
    /**
     * Removes all added lights added with "add_light" from the renderer.
     */
    void remove_all_lights();

    //
    // holdout/matte mesh options
    //

    /**
     * Adds a mesh to the renderer.
     * This function does not make a copy of the mesh. The user ensure the mesh object has not been deleted before
     * render time. If it has, the renderer will fail unexpectedly. The same mesh can be added multiple times with
     * different transformation matrices.
     * @param mesh The mesh object to add to the renderer.
     * @param tm The world space transformation matrix of the mesh.
     */
    void add_mesh( const triangle_mesh* mesh, const animated_transform& tm );
    /**
     * Removes all meshes added with "add_mesh" from the renderer.
     */
    void remove_all_meshes();

    //
    // add particles to a renderer
    //

    /**
     * Adds a particle stream to the renderer.
     * Particle streams can only be added once to the renderer.
     * Note that all exhaused stream objects will be automatically removed after "render" is called.
     * Example of adding a "PRT" file to the renderer:
     *     krakatoasr::krakatoa_renderer renderer;
     *     particle_stream myStream = krakatoasr::particle_stream::create_from_file( "my_file.prt" );
     *     renderer.add_particle_stream( myStream );
     * @param stream The stream object to add.
     */
    void add_particle_stream( particle_stream stream );
    /**
     * Removes all stream objects from the renderer.
     * Note that all exhaused stream objects will be automatically removed after "render" is called.
     */
    void remove_all_particle_streams();

    //
    // prt output files
    //

    /**
     * Saves a PRT of all the scene particles in the renderer.
     * The purpose of this function is generally:
     *   - 1) To combine multiple particle sources to a single PRT file.
     *   - 2) To save out particle "Volumes" or particles generated though "Particle Repopulation", or modified using
     * "Channel Operations".
     *   - 3) To "reduce" the filesize of the PRT files by removing channels, or reducing the bitsize of existing
     * channels.
     *   - 4) To "bake" light illumination into the "Lighting" channel. Can be useful at later renders if the "Lighting"
     * channel is copied into the "Emission" channel for self-illumniated particles. To do this, set computeLighting to
     * true.
     *
     * Example Usage:
     * @code
     *     // Example 1: Saving particles from multiple PRT files into a single PRT file:
     *     krakatoasr::krakatoa_renderer r;
     *     krakatoasr::particle_stream myStream1 = krakatoasr::particle_stream::create_from_file(
     * "my_particles_1of3.prt" ); krakatoasr::particle_stream myStream2 = krakatoasr::particle_stream::create_from_file(
     * "my_particles_2of3.prt" ); krakatoasr::particle_stream myStream3 = krakatoasr::particle_stream::create_from_file(
     * "my_particles_3of3.prt" ); r.add_particle_stream( myStream1 ); r.add_particle_stream( myStream2 );
     *     r.add_particle_stream( myStream3 );
     *     r.save_output_prt( "my_particles_combined.prt", false );
     *     r.render();
     *
     *     // Example 2: Saving out particles generated from "Particle Repopulation":
     *     krakatoasr::krakatoa_renderer r;
     *     krakatoasr::particle_stream myStream = krakatoasr::particle_stream::create_from_file(
     * "my_small_particle_set.prt" ); krakatoasr::add_particle_repopulation( myStream, 5.0f, 4, 1 ); //fills particles
     * within 5 units of the original particles r.add_particle_stream( myStream ); r.save_output_prt(
     * "my_multiplied_particles.prt", false ); r.render();
     *
     *     // Example 3: Writing out a particle set with a computed "Lighting" channel:
     *     krakatoasr::krakatoa_renderer r;
     *     //add several lights to r
     *     krakatoasr::particle_stream myStream = krakatoasr::particle_stream::create_from_file( "my_particles.prt" );
     *     r.add_particle_stream( myStream );
     *     r.save_output_prt( "my_lit_particles.prt", true );
     *     r.render();
     *
     *     // Example 4: Reducing a PRT file size by only saving "Position" and "Velocity" channels.
     *     krakatoasr::krakatoa_renderer r;
     *     krakatoasr::particle_stream myStream = krakatoasr::particle_stream::create_from_file(
     * "my_particles_with_lots_of_channels.prt" ); r.add_particle_stream( myStream ); r.save_output_prt(
     * "my_particles_with_few_channels.prt", false, false ); //set useDefaultChannels to false
     *     r.append_output_prt_channel( "Position", krakatoasr::DATA_TYPE_FLOAT32, 3 );
     *     r.append_output_prt_channel( "Velocity", krakatoasr::DATA_TYPE_FLOAT16, 3 );
     *     r.render();
     * @endcode
     *
     * @param prtFilename The filename to be saved. Will be saved in ".prt" format. Passing an empty string disables PRT
     * saving.
     * @param computeLightingChannel If this is true, the particles will have a "Lighting" channel with post-render
     * lighting data.
     * @param useDefaultChannels When this is true, the renderer will choose which channels to write based on which ones
     * the renderer will use. If this is false, the user must make calls to "append_output_prt_channel" to specify which
     * channels to save.
     * @param unitLength The unit length (in meters) of the scene.  Set this to zero for unspecified.  This affects how
     * the particles are loaded when they are reopened in another CAD or compositing program.  This only affects the
     * file metadata for PRT files.
     * @param coordinateSystem The coordinate system of the scene.  This affects how the particles are loaded when they
     * are reopened in another CAD or compositing program.  This only affects the file metadata for PRT files.
     * @param fps The frame rate (in frames per second) of the scene.  Set this to zero for unspecified.  This affects
     * how motion blur is calculated when the particles are reopened in another CAD or compositing program.  This only
     * affects the file metadata for PRT files.
     */
    void save_output_prt( const char* prtFilename, bool computeLightingChannel, bool useDefaultChannels = true,
                          float unitLength = 0.0f,
                          coordinate_system_type_t coordinateSystem = COORDINATE_SYSTEM_UNSPECIFIED, int fps = 0 );
    /**
     * Overrides the channels that are to be written by save_output_prt().
     * Only used when save_output_prt is called with useDefaultChannels = false. See save_output_prt examples.
     * @param channelName The channel name to save. The renderer uses the following special channels: "Position",
     * "Color", "Density", "Lighting", "Velocity", "MBlurTime", "Absorption", "Emission", "Normal", "Tangent",
     * "Eccentricity", "SpecularPower", "SpecularLevel", "DiffuseLevel"
     * @param channelDataType Specify the desired data type of the channel.
     * @param channelArity The arity of the channel. For example, Position is 3 (x,y,z), Density is 1 (single float).
     */
    void append_output_prt_channel( const char* channelName, data_type_t channelDataType, int channelArity );

    //
    // callbacks
    //

    /**
     * Sets a custom progress logger for the renderer.
     * Useful for tracking the progress during a "render" call. See "progress_logger_interface" class.
     * @param progressLoggerUpdater Progress logging object. Passing NULL disables progress logging. Defaults to the
     * static krakatoasr::get_console_progress_logger(), which logs progress to standard out.
     */
    void set_progress_logger_update( progress_logger_interface* progressLoggerUpdater );

    /**
     * Sets a custom render frame updater for the renderer.
     * Useful for getting visual feedback of the rendering particles during the render. See "frame_buffer_interface"
     * class.
     * @param frameBufferUpdater Frame buffer updater object. Passing NULL disables this option. Defaults to NULL.
     */
    void set_frame_buffer_update( frame_buffer_interface* frameBufferUpdater );

    /**
     * Sets a render cancel checking object, which can be used to stop the render before completion.
     * @param cancelRenderCheck cancel render interface object.  Passing NULL disables this check, Default is NULL.
     */
    void set_cancel_render_callback( cancel_render_interface* cancelRenderCheck );

    /**
     * Sets a render image saver. Needed to save the rendered image!
     * Normally the user would use a built-in saver (see example below). The user can also write their own saver class
     * to do custom image saving.
     *
     * Example of how to save the render as a multi-channel EXR:
     * @code
     *     krakatoasr::krakatoa_renderer r;
     *     krakatoasr::multi_channel_exr_file_saver exrFileSaver( "my_output_image.exr" );
     *     r.set_render_save_callback( &exrFileSaver );
     * @endcode
     *
     * Example of how to save the render as a TIFF file:
     * @code
     *     krakatoasr::krakatoa_renderer r;
     *     krakatoasr::file_saver fileSaver( "my_output_image.tiff" );
     *     r.set_render_save_callback( &fileSaver );
     * @endcode
     *
     * @param renderSaveCallback The callback object that will do the rendered image saving.
     */
    void set_render_save_callback( render_save_interface* renderSaveCallback );

    //
    //
    // Performance
    //
    //

    void set_number_of_threads( int numThreads );

    void set_frame_buffer_available_memory_fraction( float fraction );

    //
    // launch the render
    //

    /**
     * Launches the render.
     * After the renderer is fully set up, call this function to launch the final render.
     * Once the render has completed, the krakatoa_renderer object will still be valid, however all particle streams
     * will be exhaused and removed. Note: All error reporting is done via throwing exceptions. Therefore, it is very
     * important to catch any std::exceptions that this function might throw.
     *
     * For example, this will print out any errors and cancel messages:
     * @code
     *     krakatoasr::krakatoa_renderer r;
     *     try {
     *         //...
     *         bool success = r.render();
     *         if( !success )
     *             std::cout << "The Krakatoa render was cancelled." << std::endl;
     *     } catch( std::exception& e ) {
     *         std::cout << "The Krakatoa render reported the following error: " << e.what() << std::endl;
     *     }
     * @endcode
     *
     * @return This function will always return true except it wil return false if the renderer was cancelled (using a
     * cancel_render_interface object).
     */
    bool render();

    //
    // reset
    //

    /**
     * Clear the renderer, removes all particles, meshes, and sets all parameters to default values.
     * This call puts the krakatoa_renderer object back in its original state on construction.
     */
    void reset_renderer();
};

} // namespace krakatoasr

#endif
