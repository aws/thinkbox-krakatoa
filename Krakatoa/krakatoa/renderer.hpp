// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
/**
 * @file renderer.hpp
 *
 * Contains the main interface for creating images with Krakatoa
 */

#pragma once

#include <krakatoa/atmosphere_interface.hpp>
#include <krakatoa/particle_render_element_interface.hpp>
#include <krakatoa/scene_context.hpp>
#include <krakatoa/shaders.hpp>
#if defined( KRAKATOA_RT_CALLBACKS )
#include <krakatoa/mblur_data.hpp>
#endif
#include <krakatoa/lighting_data.hpp>

#include <frantic/graphics/alpha3f.hpp>
#include <frantic/graphics/color3f.hpp>
#include <frantic/graphics/color_with_alpha.hpp>
#include <frantic/graphics/vector3f.hpp>
#include <frantic/logging/render_progress_logger.hpp>
#include <frantic/rendering/environment_map_provider.hpp>
#include <frantic/rendering/lights/lightinterface.hpp>

#include <boost/function.hpp>
#include <boost/optional.hpp>

/**
 * @def KRAKATOA_PARTICLE_CONTAINER_TYPE
 * The appropriate container to use depends on platform. On 32 bit, memory fragmentation means a single large contiguous
 * container like particle_array will not be effective so we use a particle_deque. On 64 bit, this is not an issue and
 * the performance is much better with a particle_array
 */
#include <limits>

#if defined( _WIN64 ) || ( __WORDSIZE == 64 )
#include <frantic/particles/particle_array.hpp>
#define KRAKATOA_PARTICLE_CONTAINER_TYPE frantic::particles::particle_array
#else
#include <frantic/particles/particle_deque.hpp>
#define KRAKATOA_PARTICLE_CONTAINER_TYPE frantic::particles::particle_deque
#endif

namespace krakatoa {

/**
 * This is the main interface for interacting with Krakatoa. It also contains the vast majority of typedefs for the
 * various pluggable components of the rendering process, as well as the basic types like color, alpha, etc.
 */
class renderer {
  public:
    typedef frantic::graphics::color3f color_type; /**All colors are stored in a 32bit float triple*/
    typedef frantic::graphics::alpha3f
        alpha_type; /**All alphas are stored in a 32bit float triple that corresponds to thhe channels of color_type*/
    typedef frantic::graphics::color_with_alpha<color_type, alpha_type>
        pixel_type; /**A pixel has a color and alpha per color component*/

    typedef frantic::graphics2d::framebuffer<pixel_type>
        image_type; /**The image type that the renderer will operate on*/

    typedef boost::function<void( image_type& )>
        watermark_fn; /**Type used to apply a watermark to images before display and finishing a render*/

    typedef KRAKATOA_PARTICLE_CONTAINER_TYPE particle_container_type; /**Type used to hold particles*/

    typedef krakatoa::krakatoa_shader
        shader_type; /**Type of shader object used for calculating reflected light at a point*/
    typedef boost::shared_ptr<shader_type> shader_ptr_type;

    typedef frantic::rendering::environment_map_provider<color_type>
        environment_type; /**Type used for sampling the environment*/
    typedef boost::shared_ptr<environment_type> environment_ptr_type;

    typedef frantic::logging::render_progress_logger
        progress_type; /**Type used for reporting progress and updated images during a render*/
    typedef boost::shared_ptr<progress_type> progress_ptr_type;

    typedef render_element_interface render_element_type; /**Type of render elements accepted by add_render_element()*/
    typedef boost::shared_ptr<render_element_type> render_element_ptr_type;

    typedef boost::shared_ptr<renderer> ptr_type; /**Pointer to a renderer*/

    /**
     * Flags for determining if we are rendering with the normal Scattering/Absorption model of density, or an Additive
     * one.
     */
    struct mode_type {
        enum enum_t {
            normal,  // In normal mode, Scattering Color, Absorption and Emission have their standard meaning.
            additive // In additive mode, the Color channel is used as emission and density is assumed to be 0.
        };
    };

  protected:
    // Scalar affecting Density channel values when computing from the camera's perspective.
    float m_cameraDensityScale;

    // Scalar affecting Emission channel values when computing from the camera's perspective.
    float m_cameraEmissionScale;

    // Scalar affecting Density channel values when computing from the light's perspective.
    float m_lightDensityScale;

    // Integer number of threads to use, when it cannot automatically be chosen.
    bool m_disableThreading;

    // If true the sorting phase will be threaded.
    bool m_doThreadedSorting;

    // Renderer mode, for toggling between additive and regular rendering.
    mode_type::enum_t m_renderMode;

    // Plug in component for having an ambient atmosphere that affects light travelling through the scene
    atmosphere_interface_ptr m_atmosphere;

    // Plug in component for having an "environment" (ie. sphere of infinite radius emitting light) that offers simple
    // reflections
    environment_ptr_type m_environment;

    // Plug in component for rendering matte depthmaps.
    matte_interface_ptr m_matteSampler;

    // Deep image object used for additonal matting.
    boost::shared_ptr<frantic::rendering::singleface_atten_loader> m_deepMatte;

    // Particles that are being renderered.
    particle_container_type* m_particles; // TODO: Make this share ownership perhaps?

    // Plug in component that is used to report progress and show partially completed images
    progress_ptr_type m_progress;

    // Plug in component that desribes scene properties like time, the active camera, geometry objects, etc.
    scene_context_ptr m_sceneContext;

    // Plug in component that determines how light scatters at a given position.
    shader_ptr_type m_shader;

    // If `m_useMixedShaders` is true, all shaders will be initialized into m_shaders, and the one to use will be
    // determined by the "PhaseFunction" channel.
    bool m_useMixedShaders;
    std::vector<shader_ptr_type> m_shaders;

    // Plug in component for modifying the rendered image before display, and before the render completes.
    watermark_fn m_watermarkFn;

    // Duration in seconds to collect motion samples over.
    float m_mblurDuration;

    /**
     * Affects the relative position of the motion blur interval relative to the time specified in m_sceneContext.
     * Accepted values are in [-1,1]. -1 means the motiion blur interval ends at the time in m_sceneContext, 0 (the
     * default) means that the time in m_sceneContext is at the middle of the motion blur interval, and 1 makes the time
     * in m_sceneContext at the start of the motion blur interval.
     */
    float m_mblurBias;

    // The number of discrete time samples to use when drawing motion blur. 0 means motion blur is disabled.
    int m_mblurSamples;

    // True if randomization should be used on particles to improve motion blur sampling. Requires an extra 16bit float
    // per-particle "MBlurTime" that the renderer will populate internally.
    bool m_mblurJittered;

    // If true, Krakatoa will attempt to determine how many motion blur samples will need to be drawn based on the
    // velocities of each particle.
    bool m_adaptiveMblur;

    // If m_adaptiveMblur is enabled, this will be the minimum number of samples.
    int m_adaptiveMBlurSampleLowerBound;

    // If m_adaptiveMblur is enabled, this will be the maximum number of samples.
    int m_adaptiveMBlurSampleUpperBound;

    // The number of samples recommended by adaptive motion blur will be multiplied by this prior to clamping within the
    // bounds.
    float m_adaptiveMBlurSmoothness;

    // The number of samples recommended by adaptive motion blur will be taken to this power after multiplication with
    // smoothness but before clamping.
    float m_adaptiveMBlurExponent;

    // Specifies a multiplier on the inernal resolution of the matte depthmaps. Can allow higher quality matte holdouts.
    int m_matteSuperSampling;

    // True if depth of field is allowed. If true, the camera in m_sceneContext must have appropraite DOF settings or
    // this will be ignored.
    bool m_dofEnabled;

    // Multiplier affecting the quality of depth of field. Larger values make better DOF but are more expensive.
    float m_dofSampleRate;

    // Color to be drawn as the background of the image.
    pixel_type m_backgroundColor;

    // If true, the background will be 'm_backgroundColor' even if an environment is present.
    bool m_defaultBackground;

    // If an environment is present, this controls if particles reflect it.
    bool m_enableReflections;

    // If per-particle reflection is enabled, this controls the power of it
    float m_reflectionStrength;

    // The maximum number of threads that the renderer could reasonably be allowed to use
    // If this is -1, it will use the size of the TBB thread pool
    int m_renderingThreadLimit;

    // Percentage of available physical memory Krakatoa is allowed to use for frame buffers
    float m_frameBufferAvailableMemoryFraction;

    // If false, this will ignore the "Emission" channel
    bool m_useEmissionChannel;

    // If false, this will ignore the "Absorption" channel
    bool m_useAbsorptionChannel;

    // An optional mask of probabilities for calculating Bokeh offsets
    boost::optional<frantic::graphics2d::image_channel<float>> m_bokehMask;

    // An optional map of colors to blend the the bokeh samples with
    boost::optional<std::vector<frantic::graphics2d::image_channel<frantic::graphics::color3f>>> m_bokehBlendMap;

    // Amount to blend the samples with m_bokehBlend
    boost::optional<float> m_bokehBlendAmount;

    // Amount to do the anamorphic squeeze
    boost::optional<float> m_anamorphicSqueeze;

    // Callback to do something with individual mblur passes before they're composited
#if defined( KRAKATOA_RT_CALLBACKS )
    boost::optional<boost::function<void( const mblur_data& )>> m_mblurPassCallback;
#endif
    boost::optional<boost::function<void( double, frantic::graphics2d::framebuffer<float>&, int )>> m_matteGenerator;
    boost::optional<boost::function<frantic::graphics::color3f( const krakatoa::lighting_data& )>> m_lightingCallback;
    int m_mipmapResolutionCoefficient;

    bool m_saveOccludedZDepth;

  public:
    renderer();

    virtual ~renderer() {}

    /**
     * @param theAtmosphere A new atmosphere_interface object to use when rendering
     */
    virtual void set_atmosphere( atmosphere_interface_ptr theAtmosphere );

    /**
     * @param theEnvironment A new environment object to use when rendering
     * @parma reflectionStrength The amount of light from the environment that reflects off particles. If 0, no
     * reflection occurs.
     */
    virtual void set_environment( environment_ptr_type theEnvironment, float reflectionStrength = 1.f );

    /**
     * @param matteSampler A new matte_interface object to use for creating matte depth maps while rendering.
     * @param superSampling A multiplier on the resolution of matte depthmaps created, relative to the size of the
     *                      camera's resolution. For example, if superSampling=2 for a camera resoltion of 640x480
     *                      the matte depth map will have resolution 1280x960.
     */
    virtual void set_matte_sampler( matte_interface_ptr matteSampler, int superSampling = 1 );

    /**
     * @param deepMatteMap The deep image object that will provide additional matting of the final image.
     */
    virtual void set_deep_matte_map( boost::shared_ptr<frantic::rendering::singleface_atten_loader> deepMatteMap );

    /**
     * @param theParticles Sets the particles to render.
     */
    virtual void set_particles( particle_container_type* theParticles );

    /**
     * @param theShader A new shader object to use during rendering
     */
    virtual void set_shader( shader_ptr_type theShader );

    /**
     * @param theContext A new scene_context to use during rendering. Contains information about the current time,
     * active camera, matte geometry in the scene, and active lights in the scene.
     */
    virtual void set_scene_context( scene_context_ptr theContext );

    /**
     * @param prog A new object to use for recording progress and partial images during render
     */
    virtual void set_progress_logger( progress_ptr_type prog );

    /**
     * @param theWatermak A new watermarking function to be applied before displaying any partial images, and once
     * before finishing the render
     */
    virtual void set_watermark( const watermark_fn& theWatermark );

    /**
     * @param cameraScale Sets the scalar affecting the Density channel from the camera's perspective.
     * @param lightScale Sets the scalar affecting the Density channel from a light's perspective.
     */
    virtual void set_density_scale( float cameraScale, float lightScale = 1.f );

    /**
     * @param emissionScale Sets the scalar affecting the Emission channel.
     */
    virtual void set_emission_scale( float emissionScale );

    /**
     * @param disableThreading Specifies if threading is to be disabled.
     */
    virtual void disable_threading( bool disableThrading );

    /**
     * @param disableThreadingForSort Specifies if threading is to be disabled for sorting.
     */
    virtual void disable_threading_for_sort( bool disableThreadingForSort );

    /**
     * @param renderMode The rendering mode to use.
     */
    virtual void set_render_mode( mode_type::enum_t renderMode );

    /**
     * @param duration The motion blur duration in seconds. The amount of time the camera shutter is open.
     * @param bias The motion blur interval bias [-1,1]. Linearly shifts the interval relative to
     * m_sceneContext->get_time();
     */
    virtual void set_motion_blur_interval( float duration, float bias = 0.f );

    /**
     * @param numSamples The number of motion blur samples to render. A value of 0 disables motion blur sampling.
     */
    virtual void set_motion_blur_samples( int numSamples );

    /**
     * @param isJittered If true, motion will be sampled stochastically at random positions within a motion interval.
     * This effect is only supported by some parts of the renderer. It will add a slight amount of noise to the image in
     * return for better motion blur.
     */
    virtual void set_motion_blur_jittered( bool isJittered = true );

    /**
     * @param enable if true Krakatoa will use adaptive motion blur where it will automatically determine how many
     * motion blur passes should be used
     */
    virtual void set_enable_adaptive_motion_blur( bool enable );

    /**
     * @param numSamples The minimum number of motion blur samples to render.
     */
    virtual void set_adaptive_motion_blur_min_samples( int numSamples );

    /**
     * @param numSamples The maximum number of motion blur samples to render.
     */
    virtual void set_adaptive_motion_blur_max_samples( int numSamples );

    /**
     * @param smoothness the percentage [0, inf) of the adaptively chosen motion blur sample number to be used.
     */
    virtual void set_adaptive_motion_blur_smoothness( float smoothness );

    /**
     * @param exponent the exponent adaptive motion blur samples should be taken to after multiplication by smoothness.
     */
    virtual void set_adaptive_motion_blur_exponent( float exponent );

    /**
     * @param enabled Allows depth of field rendering, if supported by the camera in m_sceneContext
     * @param sampleRate Controls the quality of depth of field rendering. Higher means better.
     */
    virtual void set_depth_of_field_enabled( bool enabled = true, float sampleRate = 0.1f );

    /**
     * @param backgroundColor Specifies the color of the background when rendering.
     */
    virtual void set_background_color( const pixel_type& backgroundColor, bool useEnvironBackground = true );

    /**
     * @param Adds a new render element to be filled in during a render. Render elements are additional images that are
     * drawn in addition to the main image created by a call to render. Typical uses are to record specific parts of a
     * shader, or additional data stored in a particle.
     */
    virtual void add_render_element( krakatoa::render_element_interface_ptr renderElement ) = 0;

    /**
     * Called to precompute lighting on the particles, if supported by the renderer.
     */
    virtual void precompute_lighting() = 0;

    /**
     * This is the main rendering function. Will use the state specified by all calls to set_XXXX() to render an image
     * of the described particles and scene.
     * @note You must call at least set_particles() before calling this.
     * @param outImage The image to draw the particles to.
     */
    virtual void render( image_type& outImage ) = 0;

    /**
     * Sets the rendering thread limit so Krakatoa doesn't try to use more threads than there are cores available
     * Normally this will be -1, indicating the size of the TBB thread pool which is either created in Krakatoa SR, or
     *Krakatoa MX.
     * @note this is strictly the total number of threads Krakatoa could possibly use based on what is pysically
     *available the renderer itself may decide to only use a fraction of this if it cannot fit frame buffers for each
     *thread in memory this is the thread cap before the memory optimization in get_max_threads
     * @param threadlimit the maximum number of threads Krakatoa might be able to use. If -1 is passed in, it will use
     *the default size of the tbb threads pool.
     */
    virtual void set_rendering_thread_limit( int threadLimit ) { m_renderingThreadLimit = threadLimit; }

    /**
     * Sets the fraction of available physical memory Krakatoa is allowed to use for frame buffers
     * This should be significantly less than 100% since other things might be using memory, plus we need contiguous
     * memory for each buffer
     */
    virtual void set_fram_buffer_available_memory_fraction( float availableMemoryFraction ) {
        m_frameBufferAvailableMemoryFraction = availableMemoryFraction;
    }

    /**
     * @param Sets if the renderer should use the "Emission" channel
     */
    virtual void set_use_emission( bool useEmission );

    /**
     * @param Sets if the renderer should use the "Absorption" channel
     */
    virtual void set_use_absorption( bool useAbsorption );

    virtual void set_bokeh_mask( const frantic::graphics2d::image_channel<float>& bokehMask );

    virtual void
    set_bokeh_blend_map( const frantic::graphics2d::image_channel<frantic::graphics::color3f>& bokehBlendMap );

    virtual void set_bokeh_blend_amount( float amount );

    virtual void set_anamorphic_squeeze( float squeeze );

#if defined( KRAKATOA_RT_CALLBACKS )
    virtual void set_mblur_pass_callback( const boost::function<void( const mblur_data& )>& callback );
#endif

    virtual void set_matte_generator(
        const boost::function<void( double, frantic::graphics2d::framebuffer<float>&, int )> matteGenerator );

    virtual void set_lighting_callback(
        const boost::function<frantic::graphics::color3f( const krakatoa::lighting_data& )> callback );

    void set_mipmap_resolution_coefficient( int coefficient );

    void set_save_occluded_ZDepth( bool save );

    void set_use_mixed_shaders( bool useMixedShaders );

    void add_shader( shader_ptr_type shader );

  protected:
    virtual std::size_t get_num_output_images();

    const boost::optional<frantic::graphics2d::image_channel<float>>& bokeh_mask() const;

    boost::optional<const frantic::graphics2d::image_channel<frantic::graphics::color3f>&>
    bokeh_blend_map( std::size_t resolution ) const;

    boost::optional<float> bokeh_blend_amount() const;

    boost::optional<float> anamorphic_squeeze() const;

    virtual int get_adaptive_motion_blur_passes() const;

  private:
    // Disable copying and assignment
    renderer( const renderer& );
    renderer& operator=( const renderer& );

    virtual int get_point_recommended_motion_blur_passes( const frantic::graphics::vector3f& minPoint,
                                                          const frantic::graphics::vector3f& maxPoint,
                                                          bool& valid ) const;
};

/**
 * Any references to a renderer subclass should be through pointers of this type.
 */
typedef renderer::ptr_type renderer_ptr;

} // namespace krakatoa
