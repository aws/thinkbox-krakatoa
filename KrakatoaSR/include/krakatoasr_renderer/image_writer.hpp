// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoasr_renderer/params.hpp>

namespace krakatoasr {
/// saves a list of images to a multi-channel exr file.
void save_multi_channel_exr_file( const multi_channel_exr_file_saver_data& saverData, int width, int height,
                                  int imageCount, const output_type_t* listOfTypes,
                                  const frame_buffer_pixel_data* const* listOfImages );
void save_tiled_multi_channel_exr_file( const multi_channel_exr_file_saver_data& saverData, int width, int height,
                                        int imageCount, const output_type_t* listOfTypes,
                                        const frame_buffer_pixel_data* const* listOfImages, int tileWidth,
                                        int tileHeight, const std::vector<std::string>& listOfCustomChannelNames );
/// saves a list of images to separate rbga image files.
void save_image_file( const file_saver_data& saverData, int width, int height, int imageCount,
                      const output_type_t* listOfTypes, const frame_buffer_pixel_data* const* listOfImages );
/// loads a depth map file
///  @param depthFormat Can be "cameraSpace", "normalized", or "invertedNormalized" that represents the data in the file
///  that is being loaded.
void load_depth_map_file( const std::string& filename, std::vector<float>& outBuffer,
                          frantic::graphics2d::size2& outSize, const std::string& depthFormat, float zNear = 0.0f,
                          float zFar = 0.0f );
} // namespace krakatoasr
