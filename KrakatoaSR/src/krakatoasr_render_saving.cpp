// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_render_saving.hpp>

#include <krakatoasr_renderer/image_writer.hpp>
#include <krakatoasr_renderer/params.hpp>

namespace krakatoasr {

multi_channel_exr_file_saver::multi_channel_exr_file_saver( const char* exrFilename ) {
    m_data = new multi_channel_exr_file_saver_data;
    m_data->filename = exrFilename;
    m_data->exrCompressionType = krakatoasr::COMPRESSION_ZIP;

    m_data->r = "R";
    m_data->g = "G";
    m_data->b = "B";
    m_data->a = "A";
    m_data->rgbaBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->z = "Z";
    m_data->zBitDepth = krakatoasr::BIT_DEPTH_FLOAT;

    m_data->normalx = "normal.X";
    m_data->normaly = "normal.Y";
    m_data->normalz = "normal.Z";
    m_data->normalBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->velocityx = "velocity.X";
    m_data->velocityy = "velocity.Y";
    m_data->velocityz = "velocity.Z";
    m_data->velocityBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->occludedr = "occluded.R";
    m_data->occludedg = "occluded.G";
    m_data->occludedb = "occluded.B";
    m_data->occludeda = "occluded.A";
    m_data->occludedBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->emissionr = "emission.R";
    m_data->emissiong = "emission.G";
    m_data->emissionb = "emission.B";
    m_data->emissionBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->specularr = "specular.R";
    m_data->specularg = "specular.G";
    m_data->specularb = "specular.B";
    m_data->specularBitDepth = krakatoasr::BIT_DEPTH_HALF;

    m_data->tiled = false;
    m_data->tileWidth = 256;
    m_data->tileHeight = 256;
}

multi_channel_exr_file_saver::~multi_channel_exr_file_saver() { delete m_data; }

multi_channel_exr_file_saver::multi_channel_exr_file_saver( const multi_channel_exr_file_saver& t ) {
    m_data = new multi_channel_exr_file_saver_data;
    *this = t;
}

multi_channel_exr_file_saver& multi_channel_exr_file_saver::operator=( const multi_channel_exr_file_saver& t ) {
    *m_data = *t.m_data;
    return *this;
}

void multi_channel_exr_file_saver::save_render_data( int width, int height, int imageCount,
                                                     const output_type_t* listOfTypes,
                                                     const frame_buffer_pixel_data* const* listOfImages ) {
    if( m_data->tiled ) {
        krakatoasr::save_tiled_multi_channel_exr_file( *m_data, width, height, imageCount, listOfTypes, listOfImages,
                                                       m_data->tileWidth, m_data->tileHeight,
                                                       m_data->customChannelNames );
    } else {
        krakatoasr::save_multi_channel_exr_file( *m_data, width, height, imageCount, listOfTypes, listOfImages );
    }
}

void multi_channel_exr_file_saver::set_channel_name_rgba( const char* r, const char* g, const char* b, const char* a,
                                                          const krakatoasr::exr_bit_depth_t d ) {
    m_data->r = r;
    m_data->g = g;
    m_data->b = b;
    m_data->a = a;
    m_data->rgbaBitDepth = d;
}

void multi_channel_exr_file_saver::set_channel_name_z( const char* z, const krakatoasr::exr_bit_depth_t d ) {
    m_data->z = z;
    m_data->zBitDepth = d;
}

void multi_channel_exr_file_saver::set_channel_name_normal( const char* x, const char* y, const char* z,
                                                            const krakatoasr::exr_bit_depth_t d ) {
    m_data->normalx = x;
    m_data->normaly = y;
    m_data->normalz = z;
    m_data->normalBitDepth = d;
}

void multi_channel_exr_file_saver::set_channel_name_velocity( const char* x, const char* y, const char* z,
                                                              const krakatoasr::exr_bit_depth_t d ) {
    m_data->velocityx = x;
    m_data->velocityy = y;
    m_data->velocityz = z;
    m_data->velocityBitDepth = d;
}

void multi_channel_exr_file_saver::set_channel_name_rgba_occluded( const char* r, const char* g, const char* b,
                                                                   const char* a,
                                                                   const krakatoasr::exr_bit_depth_t d ) {
    m_data->occludedr = r;
    m_data->occludedg = g;
    m_data->occludedb = b;
    m_data->occludeda = a;
    m_data->occludedBitDepth = d;
}

void multi_channel_exr_file_saver::set_exr_compression_type( krakatoasr::exr_compression_t t ) {
    m_data->exrCompressionType = t;
}

void multi_channel_exr_file_saver::set_tiled( bool tiled ) { m_data->tiled = tiled; }

void multi_channel_exr_file_saver::set_tile_width( int width ) { m_data->tileWidth = std::max( 1, width ); }

void multi_channel_exr_file_saver::set_tile_height( int height ) { m_data->tileHeight = std::max( 1, height ); }

multi_channel_exr_file_saver_data* multi_channel_exr_file_saver::get_data() { return m_data; }

file_saver::file_saver( const char* rgbaFile, const char* zFile, const char* normalFile, const char* velocityFile,
                        const char* rgbaOccludedFile ) {
    m_data = new file_saver_data;
    m_data->rgbaFile = rgbaFile;
    m_data->zFile = zFile;
    m_data->normalFile = normalFile;
    m_data->velocityFile = velocityFile;
    m_data->rgbaOccludedFile = rgbaOccludedFile;
}

file_saver::~file_saver() { delete m_data; }

file_saver::file_saver( const file_saver& t ) {
    m_data = new file_saver_data;
    *this = t;
}

file_saver& file_saver::operator=( const file_saver& t ) {
    *m_data = *t.m_data;
    return *this;
}

void file_saver::save_render_data( int width, int height, int imageCount, const output_type_t* listOfTypes,
                                   const frame_buffer_pixel_data* const* listOfImages ) {
    krakatoasr::save_image_file( *m_data, width, height, imageCount, listOfTypes, listOfImages );
}

} // namespace krakatoasr
