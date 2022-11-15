// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_DATATYPES__
#define __KRAKATOASR_DATATYPES__

#if defined( KSR_STATIC )
#define FCNEXPORT
#define CLSEXPORT
#elif defined( _WIN32 )
#define FCNEXPORT extern "C" __declspec( dllexport )
#define CLSEXPORT __declspec( dllexport )
#elif HAVE_VISIBILITY
#define FCNEXPORT __attribute__( ( __visibility__( "default" ) ) )
#define CLSEXPORT __attribute__( ( __visibility__( "default" ) ) )
#else
#define FCNEXPORT
#define CLSEXPORT
#endif

/**
 * Krakatoa SR
 */
namespace krakatoasr {

#if defined( WIN32 )
typedef __int64 INT64;
#elif defined( __APPLE__ )
typedef long long INT64;
#else
typedef long long INT64;
#endif

enum logging_level_t { LOG_NONE = 0, LOG_ERRORS, LOG_WARNINGS, LOG_PROGRESS, LOG_STATS, LOG_DEBUG, LOG_CUSTOM = -1 };

enum output_type_t {
    OUTPUT_RGBA,
    OUTPUT_Z,
    OUTPUT_NORMAL,
    OUTPUT_VELOCITY,
    OUTPUT_RGBA_OCCLUDED,
    OUTPUT_EMISSION,
    OUTPUT_SPECULAR,
    OUTPUT_SPECULAR2,
    OUTPUT_CUSTOM
};

enum rendering_method_t { METHOD_PARTICLE, METHOD_VOXEL };

enum filter_t {
    FILTER_BILINEAR, // when using bilinear, also specify a filter "size" (default to 1)
    FILTER_BICUBIC,
    FILTER_NEAREST_NEIGHBOR
};

enum camera_type_t { CAMERA_PERSPECTIVE, CAMERA_ORTHOGRAPHIC };

enum light_shape_t { SHAPE_SQUARE, SHAPE_ROUND };

enum data_type_t {
    DATA_TYPE_INVALID,
    DATA_TYPE_INT8,
    DATA_TYPE_INT16,
    DATA_TYPE_INT32,
    DATA_TYPE_INT64,
    DATA_TYPE_UINT8,
    DATA_TYPE_UINT16,
    DATA_TYPE_UINT32,
    DATA_TYPE_UINT64,
    DATA_TYPE_FLOAT16,
    DATA_TYPE_FLOAT32,
    DATA_TYPE_FLOAT64
};

enum coordinate_system_type_t {
    COORDINATE_SYSTEM_UNSPECIFIED,
    COORDINATE_SYSTEM_RIGHT_HANDED_YUP,
    COORDINATE_SYSTEM_RIGHT_HANDED_ZUP,
    COORDINATE_SYSTEM_LEFT_HANDED_YUP,
    COORDINATE_SYSTEM_LEFT_HANDED_ZUP,
    COORDINATE_SYSTEM_INVALID
};

enum exr_compression_t { // this has to exact matches Imf::Compression
    COMPRESSION_NONE,    // no compression
    COMPRESSION_RLE,     // run length encoding
    COMPRESSION_ZIPS,    // zlib compression, one scan line at a time
    COMPRESSION_ZIP,     // zlib compression, in blocks of 16 scan lines
    COMPRESSION_PIZ,     // piz-based wavelet compression
    COMPRESSION_PXR24,   // lossy 24-bit float compression
    COMPRESSION_B44,     // lossy 4-by-4 pixel block compression, fixed compression rate
    COMPRESSION_B44A,    // lossy 4-by-4 pixel block compression, flat fields are compressed more
};

enum exr_bit_depth_t {
    BIT_DEPTH_UINT,  // using usigned int
    BIT_DEPTH_HALF,  // using 16 bits float
    BIT_DEPTH_FLOAT, // using 32 bits float
};

struct animated_transform_params;
struct triangle_mesh_params;
struct shader_params;
struct light_params;
struct particle_stream_data;
struct particle_stream_interface_data;
struct fractal_parameters_data;
struct multi_channel_exr_file_saver_data;
struct file_saver_data;
struct krakatoa_renderer_params;
struct texture_data;

class progress_logger_interface;
class frame_buffer_interface;
class cancel_render_interface;
class render_save_interface;
class fractal_parameters;
class particle_stream_interface;

/**
 * Data layout used by frame_buffer_interface and render_save_interface.
 */
struct frame_buffer_pixel_data {
    float r;
    float g;
    float b;
    float r_alpha;
    float g_alpha;
    float b_alpha;
};

} // namespace krakatoasr

#endif
