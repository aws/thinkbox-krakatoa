// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_renderer/params.hpp>

#include <krakatoasr_progress.hpp>

#include <boost/filesystem.hpp>
#include <frantic/files/files.hpp>
#include <frantic/geometry/trimesh3.hpp>
#include <frantic/graphics/camera.hpp>
#include <frantic/graphics/transform4f.hpp>
#include <frantic/rendering/lights/directlight.hpp>
#include <frantic/rendering/lights/pointlight.hpp>
#include <frantic/rendering/lights/spotlight.hpp>
#include <krakatoa/shader_functions.hpp>

#include <boost/random.hpp>

using namespace frantic::graphics;
using namespace frantic::geometry;

namespace krakatoasr {

void reset_light_params( light_params& light ) {

    light.lightType = "unknown light type"; // no real default

    // provides defaults for all the light parameters
    light.name = "light";
    light.flux.set( 12.5f, 12.5f, 12.5f );
    light.decayExponent = 0;
    light.enableShadows = true;
    light.shadowDensity = 1.0f;
    light.shadowMapWidth = 512;

    light.useNearAtten = false;
    light.useFarAtten = false;
    light.nearAttenuationStart = 0.0f;
    light.nearAttenuationEnd = 40.0f;
    light.farAttenuationStart = 80.0f;
    light.farAttenuationEnd = 200.0f;

    light.lightShape = krakatoasr::SHAPE_ROUND;
    light.lightAspect = 1.0f;

    light.innerRectRadius = 43.0f;
    light.outerRectRadius = 45.0f;

    light.innerConeAngle = 43.0f;
    light.outerConeAngle = 45.0f;

    light.attenuationMapSavingPath = "";
    light.attenuationMapDepthSamples = 1;
    light.attenuationMapSampleSpacing = 1.0f;
    light.attenuationMapExponentialSampleSpacing = false;

    light.attenMapLoadingPath = "";

    light.decayRadius = 0.f;
}

void set_default_random_fractal_parameters( int affineTMCount, int colorGradientCount, int randomSeed,
                                            fractal_parameters_data& outParams ) {
    if( affineTMCount < 2 )
        throw std::runtime_error(
            "There must be two or more affine transformation matrices to create particle fractals." );
    if( colorGradientCount < 1 )
        throw std::runtime_error( "There must be one or more color gradients to create particle fractals." );

    boost::mt19937 gen( randomSeed );
    boost::uniform_01<float> range;
    boost::variate_generator<boost::mt19937, boost::uniform_01<float>> rng( gen, range );

    outParams.position.resize( affineTMCount );
    outParams.rotation.resize( affineTMCount );
    outParams.scale.resize( affineTMCount );
    outParams.skewOrientation.resize( affineTMCount );
    outParams.skewAngle.resize( affineTMCount );
    outParams.weight.resize( affineTMCount );

    // generate a random TM
    // This code was a copy of the maxscript generation code from Krakatoa MX.
    for( int i = 0; i < affineTMCount; ++i ) {
        outParams.position[i] = vector3f::from_random( rng ) * 2.0f - vector3f( 1.0f );
        outParams.rotation[i] = frantic::graphics::quat4f::from_angle_axis(
            rng() * 2.0f * (float)M_PI, vector3f::normalize( vector3f::from_random_in_sphere( rng ) ) );
        outParams.scale[i] = vector3f::from_random( rng );
        outParams.skewOrientation[i] = frantic::graphics::quat4f::from_angle_axis(
            rng() * 2.0f * (float)M_PI, vector3f::normalize( vector3f::from_random_in_sphere( rng ) ) );
        outParams.skewAngle[i] = rng() * 360.0f;
        outParams.weight[i] = 1.0f;
    }

    outParams.colors.resize( colorGradientCount );
    outParams.colorPositions.resize( colorGradientCount );

    // set random Color gradient
    if( colorGradientCount == 1 ) {
        outParams.colorPositions[0] = 0.0f;
        outParams.colors[0] = vector3f::from_random( rng );
    } else {
        for( int i = 0; i < colorGradientCount; ++i ) {
            outParams.colorPositions[i] = (float)i / ( colorGradientCount - 1 );
            outParams.colors[i] = vector3f::from_random( rng );
        }
    }
}

void reset_krakatoa_renderer_params( krakatoa_renderer_params& params ) {

    // general rendering options
    params.errorOnMissingLicense = false;
    params.backgroundColor.set( 0.0f, 0.0f, 0.0f );
    params.densityPerParticle = 5.0f;
    params.densityExponent = -1;
    params.lightingDensityPerParticle = 5.0f;
    params.lightingDensityExponent = -1;
    params.useEmissionColor = false;
    params.emissionStrength = 5.0f;       // use 1.0 when overriding the emission strength
    params.emissionStrengthExponent = -1; // use 0 when overriding the emission strength
    params.useAbsorptionColor = false;
    params.renderingMethod = METHOD_PARTICLE;
    params.voxelFilterRadius = 1;
    params.voxelSize = 0.5f;
    params.drawPointFilter = "Bilinear";
    params.additiveMode = false;
    params.attenuationLookupFilter = "Bicubic";
    params.matteSuperSampling = 1;
    params.deepMatteFilename = "";
    params.lightingOnly = false;

    // shader options
    params.shaderParams = krakatoa::shader_params();

    // render elements options
    params.enableZDepthElement = false;
    params.enableNormalElement = false;
    params.enableVelocityElement = false;
    params.enableOccludedRgbaElement = false;
    params.enableEmissionElement = false;
    params.enableSpecularElement = false;
    params.enableSpecular2Element = false;

    // camera options
    params.cameraTm = animated_transform();
    params.cameraType = CAMERA_PERSPECTIVE;
    params.cameraFov = 90.0f / 180.0f * (float)M_PI;
    params.cameraOrthographicWidth = 30.0f;
    params.cameraNearClipping = 0.001f;
    params.cameraFarClipping = 1e+10f;
    params.screenOffset.set( 0.0f, 0.0f );

    // render output
    params.renderResolution = frantic::graphics2d::size2( 640, 480 );
    params.pixelAspectRatio = 1.0f;

    // motion blur and depth of field
    params.enableMotionBlur = false;
    params.shutterBegin = 0.0f;
    params.shutterEnd = 0.0f;
    params.numMotionBlurSamples = 2;
    params.useJitteredMotionBlur = false;
    params.enableDof = false;
    params.disableCameraBlur = false;
    params.dofFStop = 1e30f;
    params.dofFocalLength = 30.0f;
    params.dofFocalDistance = 100.0f;
    params.dofSampleRate = 0.1f;
    params.enableAdaptiveMotionBlur = false;
    params.adaptiveMotionBlurMinSamples = 2;
    params.adaptiveMotionBlurMaxSamples = 1024;
    params.adaptiveMotionBlurSmoothness = 1.0f;
    params.adaptiveMotionBlurExponent = 1.0f;
    params.useBokehShapeMap = false;
    params.useBokehBlendMap = false;
    params.bokehBlendInfluence = 1.0f;
    params.useAnamorphicSqueeze = false;
    params.anamorphicSqueeze = 1.0f;
    params.bokehBlendMipmapScale = 1;
    params.allocateBokehBlendInfluenceChannel = false;

    // prt output saving
    params.particleOutputFilename = "";
    params.particleOutputComputeLighting = false;
    params.particleOutputUseDefaultChannels = true;
    params.particleOutputChannels.clear();
    params.particleOutputMetadataFramerate = 0;
    params.particleOutputMetadataUnits = 0;
    params.particleOutputMetadataCoordSys = COORDINATE_SYSTEM_UNSPECIFIED;

    // scene data
    params.lights.clear();
    params.meshes.clear();
    params.particles.clear();

    // initial matte depth buffer
    params.initialMatteDepthBuffer.clear();
    params.initialMatteDepthBufferSize = frantic::graphics2d::size2( 0, 0 );

    // callbacks
    params.progressLoggerUpdater = get_console_progress_logger();
    params.frameBufferUpdater = NULL;
    params.renderSaveCallback = NULL;
    params.cancelRenderCheck = NULL;

    // performance
    params.numThreads = -1;
    params.frameBufferAvailableMemoryFraction = 0.75f;

    params.doReflection = false;
    params.allocateReflectionStrength = false;

    params.saveOccludedZDepth = false;

    params.useMultiShaderMode = false;
}

} // namespace krakatoasr
