// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_renderer/image_writer.hpp>

#include <frantic/files/files.hpp>
#include <frantic/logging/logging_level.hpp>

#include <OpenImageIO/imageio.h>

#pragma warning( push, 3 )
#pragma warning( disable : 4996 )
#include <ImfChannelList.h>
#include <ImfHeader.h>
#include <ImfInputFile.h>
#include <ImfOutputFile.h>
#pragma warning( pop )

#include <iomanip>

using namespace frantic::graphics;

namespace krakatoasr {

inline void convert_to_exr_datatype( float /*lowerBound*/, float /*upperBound*/, float inData, float& outData ) {
    outData = inData;
}
inline void convert_to_exr_datatype( float /*lowerBound*/, float /*upperBound*/, float inData, half& outData ) {
    outData = (half)inData;
}
inline void convert_to_exr_datatype( float lowerBound, float upperBound, float inData, boost::uint32_t& outData ) {
    if( upperBound <= lowerBound )
        throw std::runtime_error( "convert_to_exr_datatype: The range for upperBound and lowerBound is invalid." );
    inData = std::max( lowerBound, inData );
    inData = std::min( upperBound, inData );
    float outDataAsFloat = ( inData - lowerBound ) / ( upperBound - lowerBound );
    outData = (boost::uint32_t)std::min( outDataAsFloat, (float)std::numeric_limits<boost::uint32_t>::max() );
}

// -----------------------------------
// save_multi_channel_exr_file_helper
// @params component: used to specify which data in frame_buffer_pixel_data to be selected, and insert it to exrData.
//		      (0 for r; 1 for g; 2 for b; 3 for alpha; )
// @params ...
// -----------------------------------
template <typename T>
void save_multi_channel_exr_file_helper( const std::string& channelName, int width, int height,
                                         const frame_buffer_pixel_data* imageData, Imf::Header& exrHeader,
                                         Imf::FrameBuffer& exrData, const Imf::PixelType pixelType, int component,
                                         std::list<std::vector<char>>& exrBuffers, float lowerBound = 0.0,
                                         float upperBound = 1.0 ) {
    // add a new buffer element to our list
    exrBuffers.push_back( std::vector<char>() );
    std::vector<char>& buffer = exrBuffers.back();

    // size the new buffer
    buffer.resize( width * height * sizeof( T ) );
    T* rawBuffer = (T*)&buffer[0];

    // assign values to the new buffer
    int index = 0;
    for( int y = 0; y < height; ++y ) {
        for( int x = 0; x < width; ++x ) {
            const frame_buffer_pixel_data& pixel = imageData[x + ( height - y - 1 ) * width];

            float pixelValue = .0f;
            if( component == 0 )
                pixelValue = pixel.r;
            else if( component == 1 )
                pixelValue = pixel.g;
            else if( component == 2 )
                pixelValue = pixel.b;
            else
                pixelValue = ( pixel.r_alpha + pixel.g_alpha + pixel.b_alpha ) / 3.0f;

            // convert to the correct data type, and set in RawBuffer
            convert_to_exr_datatype( lowerBound, upperBound, pixelValue, rawBuffer[index] );
            ++index;
        }
    }
    exrHeader.channels().insert( channelName.c_str(), Imf::Channel( pixelType ) );
    exrData.insert( channelName.c_str(),
                    Imf::Slice( pixelType, (char*)&rawBuffer[0], sizeof( T ), width * sizeof( T ) ) );
}

void save_multi_channel_exr_file( const multi_channel_exr_file_saver_data& saverData, int width, int height,
                                  int imageCount, const output_type_t* listOfTypes,
                                  const frame_buffer_pixel_data* const* listOfImages ) {

    // create Header and channels for OutputFile (exr only);
    // the constants in exrheader constructor are default values except width, height and exrCompressionType;
    // Imf::Compression and krakatoasr::exr_compression_t are enum types, in order to use the casting properly, we have
    // to make sure their associated integers exactly matches.
    Imf::Header exrHeader( width, height, 1.0f, Imath::V2f( 0, 0 ), 1.0f, Imf::INCREASING_Y,
                           Imf::Compression( saverData.exrCompressionType ) );
    Imf::FrameBuffer exrData;

    // these hold the memory used until the exr file is written.
    std::list<std::vector<char>> exrBuffers;

    // exr may have multiple channels. build up all the channels, add them to "exrHeader" and "exrData"
    for( int i = 0; i < imageCount; ++i ) {

        output_type_t type = listOfTypes[i];
        const frame_buffer_pixel_data* imageData = listOfImages[i];

        if( type == OUTPUT_RGBA ) {
            if( !saverData.r.empty() && !saverData.g.empty() && !saverData.b.empty() && !saverData.a.empty() ) {
                if( saverData.rgbaBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.r, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.g, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.b, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.a, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 3, exrBuffers );
                } else if( saverData.rgbaBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.r, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.g, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.b, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.a, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 3, exrBuffers );
                } else if( saverData.rgbaBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.r, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.g, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.b, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.a, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 3, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"RGBA\"." );
                }
            }
        } else if( type == OUTPUT_RGBA_OCCLUDED ) {
            if( !saverData.occludedr.empty() && !saverData.occludedg.empty() && !saverData.occludedb.empty() &&
                !saverData.occludeda.empty() ) {
                if( saverData.occludedBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.occludedr, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludedg, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludedb, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludeda, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 3, exrBuffers );
                } else if( saverData.occludedBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.occludedr, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludedg, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludedb, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludeda, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 3, exrBuffers );
                } else if( saverData.occludedBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedr, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedg, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedb, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludeda, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 3, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Occluded RGBA\"." );
                }
            }
        } else if( type == OUTPUT_NORMAL ) {
            if( !saverData.normalx.empty() && !saverData.normaly.empty() && !saverData.normalz.empty() ) {
                if( saverData.normalBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.normalx, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.normaly, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.normalz, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.normalBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.normalx, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.normaly, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.normalz, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.normalBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normalx, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         -1.0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normaly, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         -1.0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normalz, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         -1.0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Normal\"." );
                }
            }
        } else if( type == OUTPUT_VELOCITY ) {
            if( !saverData.velocityx.empty() && !saverData.velocityy.empty() && !saverData.velocityz.empty() ) {
                if( saverData.velocityBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.velocityx, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.velocityy, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.velocityz, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.velocityBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.velocityx, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.velocityy, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.velocityz, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.velocityBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityx, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityy, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityz, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Velocity\"." );
                }
            }
        } else if( type == OUTPUT_Z ) {
            if( !saverData.z.empty() ) {
                // only need the red channel
                if( saverData.zBitDepth == BIT_DEPTH_HALF )
                    save_multi_channel_exr_file_helper<half>( saverData.z, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 0, exrBuffers );
                else if( saverData.zBitDepth == BIT_DEPTH_FLOAT )
                    save_multi_channel_exr_file_helper<float>( saverData.z, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                else if( saverData.zBitDepth == BIT_DEPTH_UINT )
                    throw std::runtime_error( "\"Z Depth\" channel was set to be stored as integers in the EXR file. Z "
                                              "depth values must be stored as floats." );
                else
                    throw std::runtime_error( "Unknown EXR bit depth for \"Z Depth\"." );
            }
        } else {
            // shouldn't happen given our code
            throw std::runtime_error(
                "Multi-layer EXR saving encountered an unknown output type. Could not save EXR." );
        }
    }

    // write multi-channel exr
    Imf::OutputFile out( saverData.filename.c_str(), exrHeader );
    out.setFrameBuffer( exrData );
    out.writePixels( height );
    FF_LOG( stats ) << "Image file written: " << frantic::strings::to_tstring( saverData.filename ) << std::endl;
}

void save_tiled_multi_channel_exr_file( const multi_channel_exr_file_saver_data& saverData, int width, int height,
                                        int imageCount, const output_type_t* listOfTypes,
                                        const frame_buffer_pixel_data* const* listOfImages, int tileWidth,
                                        int tileHeight, const std::vector<std::string>& listOfCustomChannelNames ) {

    // create Header and channels for OutputFile (exr only);
    // the constants in exrheader constructor are default values except width, height and exrCompressionType;
    // Imf::Compression and krakatoasr::exr_compression_t are enum types, in order to use the casting properly, we have
    // to make sure their associated integers exactly matches.
    Imf::Header exrHeader( width, height, 1.0f, Imath::V2f( 0, 0 ), 1.0f, Imf::INCREASING_Y,
                           Imf::Compression( saverData.exrCompressionType ) );
    Imf::FrameBuffer exrData;

    // these hold the memory used until the exr file is written.
    std::list<std::vector<char>> exrBuffers;

    std::vector<std::string>::const_iterator currentCustomChannel = listOfCustomChannelNames.begin();

    // exr may have multiple channels. build up all the channels, add them to "exrHeader" and "exrData"
    for( int i = 0; i < imageCount; ++i ) {

        output_type_t type = listOfTypes[i];
        const frame_buffer_pixel_data* imageData = listOfImages[i];

        if( type == OUTPUT_RGBA ) {
            if( !saverData.r.empty() && !saverData.g.empty() && !saverData.b.empty() && !saverData.a.empty() ) {
                if( saverData.rgbaBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.r, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.g, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.b, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.a, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 3, exrBuffers );
                } else if( saverData.rgbaBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.r, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.g, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.b, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.a, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 3, exrBuffers );
                } else if( saverData.rgbaBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.r, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.g, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.b, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.a, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 3, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"RGBA\"." );
                }
            }
        } else if( type == OUTPUT_RGBA_OCCLUDED ) {
            if( !saverData.occludedr.empty() && !saverData.occludedg.empty() && !saverData.occludedb.empty() &&
                !saverData.occludeda.empty() ) {
                if( saverData.occludedBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.occludedr, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludedg, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludedb, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.occludeda, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 3, exrBuffers );
                } else if( saverData.occludedBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.occludedr, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludedg, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludedb, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.occludeda, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 3, exrBuffers );
                } else if( saverData.occludedBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedr, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedg, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludedb, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.occludeda, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 3, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Occluded RGBA\"." );
                }
            }
        } else if( type == OUTPUT_NORMAL ) {
            if( !saverData.normalx.empty() && !saverData.normaly.empty() && !saverData.normalz.empty() ) {
                if( saverData.normalBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.normalx, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.normaly, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.normalz, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.normalBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.normalx, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.normaly, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.normalz, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.normalBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normalx, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         -1.0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normaly, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         -1.0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.normalz, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         -1.0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Normal\"." );
                }
            }
        } else if( type == OUTPUT_VELOCITY ) {
            if( !saverData.velocityx.empty() && !saverData.velocityy.empty() && !saverData.velocityz.empty() ) {
                if( saverData.velocityBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.velocityx, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.velocityy, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.velocityz, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.velocityBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.velocityx, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.velocityy, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.velocityz, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.velocityBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityx, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityy, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.velocityz, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Velocity\"." );
                }
            }
        } else if( type == OUTPUT_Z ) {
            if( !saverData.z.empty() ) {
                // only need the red channel
                if( saverData.zBitDepth == BIT_DEPTH_HALF )
                    save_multi_channel_exr_file_helper<half>( saverData.z, width, height, imageData, exrHeader, exrData,
                                                              Imf::HALF, 0, exrBuffers );
                else if( saverData.zBitDepth == BIT_DEPTH_FLOAT )
                    save_multi_channel_exr_file_helper<float>( saverData.z, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                else if( saverData.zBitDepth == BIT_DEPTH_UINT )
                    throw std::runtime_error( "\"Z Depth\" channel was set to be stored as integers in the EXR file. Z "
                                              "depth values must be stored as floats." );
                else
                    throw std::runtime_error( "Unknown EXR bit depth for \"Z Depth\"." );
            }
        } else if( type == OUTPUT_EMISSION ) {
            if( !saverData.emissionr.empty() && !saverData.emissiong.empty() && !saverData.emissionb.empty() ) {
                if( saverData.emissionBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.emissionr, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.emissiong, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.emissionb, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.emissionBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.emissionr, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.emissiong, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.emissionb, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.emissionBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.emissionr, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.emissiong, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.emissionb, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Emission RGB\"." );
                }
            }
        } else if( type == OUTPUT_SPECULAR || type == OUTPUT_SPECULAR2 ) {
            if( !saverData.specularr.empty() && !saverData.specularg.empty() && !saverData.specularb.empty() ) {
                if( saverData.specularBitDepth == BIT_DEPTH_HALF ) {
                    save_multi_channel_exr_file_helper<half>( saverData.specularr, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.specularg, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<half>( saverData.specularb, width, height, imageData, exrHeader,
                                                              exrData, Imf::HALF, 2, exrBuffers );
                } else if( saverData.specularBitDepth == BIT_DEPTH_FLOAT ) {
                    save_multi_channel_exr_file_helper<float>( saverData.specularr, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 0, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.specularg, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 1, exrBuffers );
                    save_multi_channel_exr_file_helper<float>( saverData.specularb, width, height, imageData, exrHeader,
                                                               exrData, Imf::FLOAT, 2, exrBuffers );
                } else if( saverData.specularBitDepth == BIT_DEPTH_UINT ) {
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.specularr, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 0, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.specularg, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 1, exrBuffers,
                                                                         .0f, 1.0f );
                    save_multi_channel_exr_file_helper<boost::uint32_t>( saverData.specularb, width, height, imageData,
                                                                         exrHeader, exrData, Imf::UINT, 2, exrBuffers,
                                                                         .0f, 1.0f );
                } else {
                    // you should never get here.
                    throw std::runtime_error( "Unknown EXR bit depth for \"Specular RGB\"." );
                }
            }
        } else if( type == OUTPUT_CUSTOM ) {
            if( currentCustomChannel != listOfCustomChannelNames.end() ) {
                std::map<std::string, custom_channel_exr_file_saver_data>::const_iterator findIt =
                    saverData.customChannels.find( *currentCustomChannel );
                if( findIt != saverData.customChannels.end() ) {
                    const custom_channel_exr_file_saver_data& channelData = findIt->second;
                    if( !channelData.r.empty() && !channelData.g.empty() && !channelData.b.empty() ) {
                        if( saverData.specularBitDepth == BIT_DEPTH_HALF ) {
                            save_multi_channel_exr_file_helper<half>( channelData.r, width, height, imageData,
                                                                      exrHeader, exrData, Imf::HALF, 0, exrBuffers );
                            save_multi_channel_exr_file_helper<half>( channelData.g, width, height, imageData,
                                                                      exrHeader, exrData, Imf::HALF, 1, exrBuffers );
                            save_multi_channel_exr_file_helper<half>( channelData.b, width, height, imageData,
                                                                      exrHeader, exrData, Imf::HALF, 2, exrBuffers );
                        } else if( saverData.specularBitDepth == BIT_DEPTH_FLOAT ) {
                            save_multi_channel_exr_file_helper<float>( channelData.r, width, height, imageData,
                                                                       exrHeader, exrData, Imf::FLOAT, 0, exrBuffers );
                            save_multi_channel_exr_file_helper<float>( channelData.g, width, height, imageData,
                                                                       exrHeader, exrData, Imf::FLOAT, 1, exrBuffers );
                            save_multi_channel_exr_file_helper<float>( channelData.b, width, height, imageData,
                                                                       exrHeader, exrData, Imf::FLOAT, 2, exrBuffers );
                        } else if( saverData.specularBitDepth == BIT_DEPTH_UINT ) {
                            save_multi_channel_exr_file_helper<boost::uint32_t>( channelData.r, width, height,
                                                                                 imageData, exrHeader, exrData,
                                                                                 Imf::UINT, 0, exrBuffers, .0f, 1.0f );
                            save_multi_channel_exr_file_helper<boost::uint32_t>( channelData.g, width, height,
                                                                                 imageData, exrHeader, exrData,
                                                                                 Imf::UINT, 1, exrBuffers, .0f, 1.0f );
                            save_multi_channel_exr_file_helper<boost::uint32_t>( channelData.b, width, height,
                                                                                 imageData, exrHeader, exrData,
                                                                                 Imf::UINT, 2, exrBuffers, .0f, 1.0f );
                        } else {
                            // you should never get here.
                            throw std::runtime_error( "Unknown EXR bit depth for \"Custom Channel RGB\"." );
                        }
                    }
                }

                ++currentCustomChannel;
            }
        } else {
            // shouldn't happen given our code
            throw std::runtime_error(
                "Multi-layer EXR saving encountered an unknown output type. Could not save EXR." );
        }
    }

    exrHeader.setTileDescription( Imf::TileDescription( tileWidth, tileHeight, Imf::ONE_LEVEL ) );

    // write multi-channel exr
    Imf::TiledOutputFile out( saverData.filename.c_str(), exrHeader );
    out.setFrameBuffer( exrData );
    for( int tileY = 0; tileY < out.numYTiles(); ++tileY ) {
        for( int tileX = 0; tileX < out.numXTiles(); ++tileX ) {
            out.writeTile( tileX, tileY );
        }
    }
    FF_LOG( stats ) << "Tiled Image file written: " << frantic::strings::to_tstring( saverData.filename ) << std::endl;
}

void save_image_file( const file_saver_data& saverData, int width, int height, int imageCount,
                      const output_type_t* listOfTypes, const frame_buffer_pixel_data* const* listOfImages ) {

    for( size_t i = 0; i < imageCount; ++i ) {

        output_type_t imageType = listOfTypes[i];
        const frame_buffer_pixel_data* imageData = listOfImages[i];

        // decide filename based on "type"
        std::string filename = "";
        if( imageType == OUTPUT_RGBA )
            filename = saverData.rgbaFile;
        else if( imageType == OUTPUT_Z )
            filename = saverData.zFile;
        else if( imageType == OUTPUT_NORMAL )
            filename = saverData.normalFile;
        else if( imageType == OUTPUT_VELOCITY )
            filename = saverData.velocityFile;
        else if( imageType == OUTPUT_RGBA_OCCLUDED )
            filename = saverData.rgbaOccludedFile;

        // skip the saving if the filename's empty
        if( !filename.empty() ) {

            std::string fileExtension = frantic::strings::to_lower( frantic::files::extension_from_path( filename ) );
            if( fileExtension == ".exr" ) {
                // we don't handle exr's in the function.
                // set up a default RBGA single channel exr file, and call "save_multi_channel_exr_file" on it (with a
                // single channel).
                output_type_t rgbaType = OUTPUT_RGBA;
                multi_channel_exr_file_saver_data exrSaverData;
                exrSaverData.filename = filename;

                exrSaverData.r = "R";
                exrSaverData.g = "G";
                exrSaverData.b = "B";
                exrSaverData.a = "A";

                exrSaverData.exrCompressionType = krakatoasr::COMPRESSION_ZIP;

                // use default bit depth, since the default bit depth is BIT_DEPTH_HALF, we do not have to worry about
                // normalizing floating point number to integer in which have different domain as predefined . (such as
                // normal channel)
                exrSaverData.rgbaBitDepth = krakatoasr::BIT_DEPTH_HALF;

                save_multi_channel_exr_file( exrSaverData, width, height, 1, &rgbaType, &imageData );
            } else {
                std::unique_ptr<OIIO::ImageOutput> out = OIIO::ImageOutput::create( filename );
                if( !out ) {
                    const std::string oiioError = OIIO::geterror();
                    std::stringstream ss;
                    ss << "Failed to create image output file " << filename << ": " << oiioError;
                    throw std::runtime_error( ss.str() );
                } else if( out->has_error() ) {
                    const std::string oiioError = out->geterror();
                    std::stringstream ss;
                    ss << "Failed to create image output file " << filename << ": " << oiioError;
                    throw std::runtime_error( ss.str() );
                }
                const int channelCount = 4; // RGBA
                OIIO::ImageSpec imageSpecification( width, height, channelCount, OIIO::TypeDesc::FLOAT );
                const bool openResult = out->open( filename, imageSpecification );
                if( !openResult ) {
                    const std::string oiioError = out->geterror();
                    std::stringstream ss;
                    ss << "Failed to open image output file " << filename << ": " << oiioError;
                    throw std::runtime_error( ss.str() );
                }
                std::vector<float> scanline( static_cast<std::size_t>( width * channelCount ), 0.f );
                for( int y = 0; y < height; ++y ) {
                    for( int x = 0; x < width; ++x ) {
                        const int pixelStart = x * channelCount;

                        const frame_buffer_pixel_data& col = imageData[x + ( height - y - 1 ) * width];
                        const float alpha = ( col.r_alpha + col.g_alpha + col.b_alpha ) / 3.f;
                        scanline[pixelStart] = frantic::math::clamp( col.r, 0.f, 1.f );
                        scanline[pixelStart + 1] = frantic::math::clamp( col.g, 0.f, 1.f );
                        scanline[pixelStart + 2] = frantic::math::clamp( col.b, 0.f, 1.f );
                        scanline[pixelStart + 3] = frantic::math::clamp( alpha, 0.f, 1.f );
                    }
                    out->write_scanline( y, 0, OIIO::TypeDesc::FLOAT, scanline.data() );
                }

                out->close();

                FF_LOG( stats ) << frantic::tstring( _T( "Image file written: " ) )
                                << frantic::strings::to_tstring( filename ) << std::endl;
            }
        }
    }
}

void read_from_OpenEXR( const std::string& filename, std::vector<float>& outBuffer, frantic::graphics2d::size2& outSize,
                        const std::string& depthFormat ) {
    Imf::InputFile file( filename.c_str() );
    Imath::Box2i dw = file.header().dataWindow();

    outSize.xsize = dw.max.x - dw.min.x + 1;
    outSize.ysize = dw.max.y - dw.min.y + 1;
    outBuffer.resize( outSize.get_area() );
    Imf::FrameBuffer frameBuffer;

    // Read the appropriate channel
    std::string desiredChannel;
    std::string channel;
    for( Imf::ChannelList::ConstIterator i = file.header().channels().begin(); i != file.header().channels().end();
         ++i ) {
        channel = i.name();
        if( depthFormat == "cameraSpace" && channel.size() >= 10 && channel.substr( 0, 10 ) == "CAMZ:depth" ) {
            desiredChannel = channel;
            break;
        }
        if( depthFormat == "normalized" && channel.size() >= 18 && channel.substr( 0, 18 ) == "CAMZ:depthRemapped" ) {
            desiredChannel = channel;
            break;
        }
        if( channel.find( "Depth" ) < channel.length() ) {
            desiredChannel = channel;
            break;
        }
    }

    // Make sure to read data if the situation doesn't match any of the cases defined above
    //(likely to be for type Inverted Normalized but not necessary)
    if( desiredChannel.empty() )
        desiredChannel = "R";

    frameBuffer.insert( desiredChannel,         // name
                        Imf::Slice( Imf::FLOAT, // type (must be fully qualified to disambiguate from windows.h)
                                    (char*)( &outBuffer[outSize.ysize * outSize.xsize - outSize.xsize] - // base
                                             dw.min.x - dw.min.y * outSize.xsize ),
                                    sizeof( float ) * 1,              // xStride
                                    sizeof( float ) * -outSize.xsize, // yStride
                                    1, 1,                             // x/y sampling
                                    0.0 ) );                          // fillValue

    file.setFrameBuffer( frameBuffer );
    file.readPixels( dw.min.y, dw.max.y );
}

void read_non_exr( const std::string& filename, std::vector<float>& outBuffer, frantic::graphics2d::size2& outSize ) {
    std::unique_ptr<OIIO::ImageInput> in = OIIO::ImageInput::open( filename );
    if( !in ) {
        const std::string oiioError = OIIO::geterror();
        std::stringstream ss;
        ss << "Failed to open input file " << filename << ": " << oiioError;
        throw std::runtime_error( ss.str() );
    } else if( in->has_error() ) {
        const std::string oiioError = in->geterror();
        std::stringstream ss;
        ss << "Failed to open input file " << filename << ": " << oiioError;
        throw std::runtime_error( ss.str() );
    }

    const OIIO::ImageSpec& specification = in->spec();
    outSize = frantic::graphics2d::size2( specification.width, specification.height );
    outBuffer.resize( outSize.area() * specification.nchannels );
    in->read_image( OIIO::TypeDesc::FLOAT, outBuffer.data() );
    in->close();
}

void load_depth_map_file( const std::string& filename, std::vector<float>& outBuffer,
                          frantic::graphics2d::size2& outSize, const std::string& depthFormat, float zNear,
                          float zFar ) {
    std::string ext = frantic::files::extension_from_path( filename );

    if( ext == ".exr" )
        read_from_OpenEXR( filename, outBuffer, outSize, depthFormat );
    else
        read_non_exr( filename, outBuffer, outSize );

    // Transform the data stored in the file to get the actual distance from camera
    float range = zFar - zNear;
    float temp;
    for( int i = 0; i < outSize.area(); ++i ) {
        temp = outBuffer[i];
        if( temp == 0 ) {
            outBuffer[i] = std::numeric_limits<float>::infinity();
        } else {
            if( depthFormat == "normalized" )
                outBuffer[i] = temp * range + zNear;
            else if( depthFormat == "invertedNormalized" )
                outBuffer[i] = ( 1.f - temp ) * range + zNear;
        }
    }
}

} // namespace krakatoasr
