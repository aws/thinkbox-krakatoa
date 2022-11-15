// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdlib>

namespace krakatoa {

/**
 *	Gets the total number of threads that Krakatoa should be allowed to use for particle rendering.
 *	Since we must allocated a separate frame buffer for each thread, the size of a single frame buffer is calculated
 *	and then the number of frame buffers that can fit into a fraction of total system memory (minus the size of the
 *particle buffer) is returned as the number of threads that can be used.
 *	@param	width					the width of the frame buffer
 *	@param	height					the height of the frame buffer
 *	@param	numOutputImages			the number of output images (channels)
 *	@param	particleSizeInMemory	the total number of bytes the particle buffer takes up in memory
 *	@param	pixelSize				the number of bytes an individual pixel takes up in memory for one
 *output image
 *	@param	threadCap				the maximum number of threads that can be used in the render. -1
 *will set this to the size of the TBB thread pool
 *	@return							the optimal number of threads to be used in the render
 */
std::size_t get_max_threads( std::size_t width, std::size_t height, std::size_t numOutputImages,
                             std::size_t particleSizeInMemory, std::size_t pixelSize, float availPhysMemoryFraction,
                             int threadCap );

// gets the total physical memory size in bytes.
std::size_t get_total_physical_memory();

// Gets the available physical memory size in bytes.
// On OS X the returned value is an estimate.
std::size_t get_available_physical_memory();
} // namespace krakatoa