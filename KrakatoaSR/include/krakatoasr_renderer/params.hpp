// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/function.hpp>
#include <boost/optional.hpp>

#include <krakatoasr_datatypes.hpp>
#include <krakatoasr_particles.hpp>
#include <krakatoasr_shader.hpp>
#include <krakatoasr_transformation.hpp>

#include <krakatoa/shader_functions.hpp>
#if defined( KRAKATOA_RT_CALLBACKS )
#include <krakatoa/mblur_data.hpp>
#endif
#include <krakatoa/lighting_data.hpp>

#include <frantic/geometry/trimesh3.hpp>
#include <frantic/graphics/camera.hpp>
#include <frantic/graphics/transform4f.hpp>

#include <frantic/graphics2d/vector2f.hpp>
#include <frantic/particles/streams/particle_istream.hpp>
namespace krakatoasr {

struct texture_data {
    int width;
    frame_buffer_pixel_data* pixels;

    texture_data()
        : width( 0 )
        , pixels( NULL ) {}
    texture_data( int pWidth, frame_buffer_pixel_data* pPixels )
        : width( pWidth )
        , pixels( pPixels ) {}
};

struct animated_transform_params {
    bool unanimated;
    std::map<float, frantic::graphics::transform4f> tm;
};

struct triangle_mesh_params {
    // scene parameters
    bool visibleToCamera;
    bool visibleToLights;
    float shutterTime;
    float shutterValidityBegin;
    float shutterValidityEnd;

    // mesh data
    boost::shared_ptr<frantic::geometry::trimesh3> mesh;
    frantic::geometry::trimesh3_vertex_channel_accessor<frantic::graphics::vector3f> velocityAcc;
};

#if 0
struct particle_channel_map_params  {
	frantic::channels::channel_map cm;
};
#endif

struct shader_params {
    krakatoa::shader_params params;
};

struct light_params {
    // light type
    std::string lightType;

    // for direct, spot & point
    std::string name;
    frantic::graphics::color3f flux;
    int decayExponent;
    bool enableShadows;
    float shadowDensity;
    int shadowMapWidth;

    bool useNearAtten;
    bool useFarAtten;
    float nearAttenuationStart;
    float nearAttenuationEnd;
    float farAttenuationStart;
    float farAttenuationEnd;

    // for direct & spot
    light_shape_t lightShape;
    float lightAspect;

    // for direct lights
    float innerRectRadius;
    float outerRectRadius;

    // for spot lights
    float innerConeAngle;
    float outerConeAngle;

    // for saving attenuation maps
    std::string attenuationMapSavingPath;
    int attenuationMapDepthSamples;
    float attenuationMapSampleSpacing;
    bool attenuationMapExponentialSampleSpacing;

    // for single-face attenuation map saving
    std::string attenMapLoadingPath;

    float decayRadius;
};

void reset_light_params( light_params& light );

struct fractal_parameters_data {
    // affine tm data
    std::vector<frantic::graphics::vector3f> position;
    std::vector<frantic::graphics::quat4f> rotation;
    std::vector<frantic::graphics::vector3f> scale;
    std::vector<frantic::graphics::quat4f> skewOrientation;
    std::vector<float> skewAngle;
    std::vector<float> weight;

    // color gradient data
    std::vector<frantic::graphics::vector3f> colors;
    std::vector<float> colorPositions;
};
void set_default_random_fractal_parameters( int affineTMCount, int colorGradientCount, int randomSeed,
                                            fractal_parameters_data& outParams );

struct particle_stream_interface_data {
    frantic::channels::channel_map channelMap;
    bool hasBeenAddedToStream;
};

struct particle_stream_data {
    boost::shared_ptr<frantic::particles::streams::particle_istream> stream;
    animated_transform tm; // tm is stored separately because transformation depends on shutterBegin and shutterEnd,
                           // which is only known at render time.
};

struct custom_channel_exr_file_saver_data {
    std::string r;
    std::string g;
    std::string b;
    krakatoasr::exr_bit_depth_t bitDepth;
};

struct multi_channel_exr_file_saver_data {
    std::string filename;
    krakatoasr::exr_compression_t exrCompressionType;
    std::string r;
    std::string g;
    std::string b;
    std::string a;
    krakatoasr::exr_bit_depth_t rgbaBitDepth;
    std::string z;
    krakatoasr::exr_bit_depth_t zBitDepth;
    std::string normalx;
    std::string normaly;
    std::string normalz;
    krakatoasr::exr_bit_depth_t normalBitDepth;
    std::string velocityx;
    std::string velocityy;
    std::string velocityz;
    krakatoasr::exr_bit_depth_t velocityBitDepth;
    std::string occludedr;
    std::string occludedg;
    std::string occludedb;
    std::string occludeda;
    krakatoasr::exr_bit_depth_t occludedBitDepth;
    std::string emissionr;
    std::string emissiong;
    std::string emissionb;
    krakatoasr::exr_bit_depth_t emissionBitDepth;
    std::string specularr;
    std::string specularg;
    std::string specularb;
    krakatoasr::exr_bit_depth_t specularBitDepth;
    std::map<std::string, custom_channel_exr_file_saver_data> customChannels;
    std::vector<std::string> customChannelNames;
    bool tiled;
    int tileWidth;
    int tileHeight;
};

struct file_saver_data {
    std::string rgbaFile;
    std::string zFile;
    std::string normalFile;
    std::string velocityFile;
    std::string rgbaOccludedFile;
};

struct particle_output_channel {
    std::string name;
    data_type_t type;
    int arity;
};

struct krakatoa_camera {
    animated_transform tm;
    camera_type_t type;
    float fov;
    float orthographicWidth;
    float nearClipping;
    float farClipping;
    frantic::graphics2d::vector2f screenOffset;

    frantic::graphics2d::size2 renderResolution;
    float pixelAspectRatio;

    render_save_interface* renderSaveCallback;
};

struct krakatoa_renderer_params {

    // general rendering options
    bool errorOnMissingLicense;
    frantic::graphics::color3f backgroundColor;
    float densityPerParticle;
    int densityExponent;
    // bool overrideLightingDensity; //just make sure lightingDensityPerParticle == densityPerParticle, same with
    // lightingDensityExponent == densityExponent
    float lightingDensityPerParticle;
    int lightingDensityExponent;
    bool useEmissionColor;
    // bool overrideEmissionStrength; //just make sure emissionStrength==densityPerParticle and
    // emissionStrengthExponent==densityExponent
    float emissionStrength;
    int emissionStrengthExponent;
    bool useAbsorptionColor;
    rendering_method_t renderingMethod;
    int voxelFilterRadius;
    float voxelSize;
    std::string drawPointFilter;
    bool additiveMode;
    std::string attenuationLookupFilter;
    int matteSuperSampling;
    std::string deepMatteFilename;
    bool lightingOnly;

    // shader options
    krakatoa::shader_params shaderParams;

    // render elements options
    bool enableZDepthElement;
    bool enableNormalElement;
    bool enableVelocityElement;
    bool enableOccludedRgbaElement;
    bool enableEmissionElement;
    bool enableSpecularElement;
    bool enableSpecular2Element;
    std::vector<particle_output_channel> customChannelElements;

    // camera options
    std::vector<krakatoa_camera> cameras;
    animated_transform cameraTm;
    camera_type_t cameraType;
    float cameraFov;
    float cameraOrthographicWidth;
    float cameraNearClipping;
    float cameraFarClipping;
    frantic::graphics2d::vector2f screenOffset;

    // render output
    frantic::graphics2d::size2 renderResolution;
    float pixelAspectRatio;

    // motion blur and depth of field
    bool enableMotionBlur;
    float shutterBegin; // TODO: keep in mind, when using these parameters, first check "enableMotionBlur"
    float shutterEnd;
    int numMotionBlurSamples;
    bool useJitteredMotionBlur;
    bool enableDof;
    bool disableCameraBlur;
    float dofFStop;
    float dofFocalLength;
    float dofFocalDistance;
    float dofSampleRate;

    // Adaptive Motion Blur
    bool enableAdaptiveMotionBlur;
    int adaptiveMotionBlurMinSamples;
    int adaptiveMotionBlurMaxSamples;
    float adaptiveMotionBlurSmoothness;
    float adaptiveMotionBlurExponent;

    // Bokeh Control
    bool useBokehShapeMap;
    texture_data bokehShapeMap;
    bool useBokehBlendMap;
    float bokehBlendInfluence;
    texture_data bokehBlendMap;
    bool useAnamorphicSqueeze;
    float anamorphicSqueeze;
    int bokehBlendMipmapScale;
    bool allocateBokehBlendInfluenceChannel;

    // prt output saving
    std::string particleOutputFilename;
    bool particleOutputComputeLighting;
    bool particleOutputUseDefaultChannels;
    std::vector<particle_output_channel> particleOutputChannels;
    float particleOutputMetadataUnits;
    coordinate_system_type_t particleOutputMetadataCoordSys;
    int particleOutputMetadataFramerate;

    // scene data. (note that animated transforms are stored separately since the values we get from them depend on
    // shutter time, and shutter time can be set after objects are added)
    std::vector<std::pair<light_params, animated_transform>> lights;
    std::vector<std::pair<const triangle_mesh_params*, animated_transform>> meshes;
    std::vector<particle_stream> particles;

    // initial matte depth buffer (currently only used in KMY and KC4D, and no public access from KSR API)
    std::vector<float> initialMatteDepthBuffer;
    frantic::graphics2d::size2 initialMatteDepthBufferSize;

    // callbacks
    progress_logger_interface* progressLoggerUpdater;
    frame_buffer_interface* frameBufferUpdater;
    cancel_render_interface* cancelRenderCheck;
    render_save_interface* renderSaveCallback;

    // private callbacks for raytracer integration
#if defined( KRAKATOA_RT_CALLBACKS )
    boost::optional<boost::function<void( const krakatoa::mblur_data& )>> mblurPassCallback;
    boost::optional<boost::function<void( double, frantic::graphics2d::framebuffer<float>&, int )>> matteGenerator;
    boost::optional<boost::function<frantic::graphics::color3f( const krakatoa::lighting_data& )>> lightingCallback;
#endif

    // performance
    int numThreads; //-1 for default.
    float frameBufferAvailableMemoryFraction;

    // Reflections (For raytracer integration only, this should not be exposed)
    bool doReflection;
    bool allocateReflectionStrength;

    bool saveOccludedZDepth;

    bool useMultiShaderMode;
    std::vector<krakatoa::shader_params> shaders;
};

void reset_krakatoa_renderer_params( krakatoa_renderer_params& renderer );

} // namespace krakatoasr
