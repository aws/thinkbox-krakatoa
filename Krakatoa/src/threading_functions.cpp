// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include "krakatoa/threading_functions.hpp"

#include <frantic/logging/logging_level.hpp>

#include <boost/algorithm/clamp.hpp>
#include <boost/thread.hpp>

#ifdef WIN32
#include <windows.h>
#elif __APPLE__
#include <sys/types.h>
#include <sys/sysctl.h>
#elif __linux
#include <unistd.h>
#include <sys/types.h>
#include <sys/param.h>
#endif

std::size_t krakatoa::get_max_threads( std::size_t width, std::size_t height, std::size_t numOutputImages,
                                       std::size_t particleSizeInMemory, std::size_t pixelSize,
                                       float availPhysMemoryFraction, int threadCap ) {

    std::size_t totalPhysicalMemory = krakatoa::get_total_physical_memory();
    std::size_t availablePhysicalMemory = krakatoa::get_available_physical_memory();

    // Avoid an unused parameter warning since we only need this information in OS X.
#if !defined( __APPLE__ )
    (void)particleSizeInMemory;
#endif

#if defined( _WIN32 )
    // The available memory isn't necessarily sufficiently contiguous to allocate the frame buffers,
    // so we'll assume only a supplied percentage of this can be used.
    // Note that the particles have already been allocated at this point so we can ignore the amount of memory they'll
    // use.
    availablePhysicalMemory *= availPhysMemoryFraction;

#elif defined( __linux )
    // The same is true for Linux as for Windows.
    availablePhysicalMemory *= availPhysMemoryFraction;

#elif defined( __APPLE__ )
    // On OS X availablePhysicalMemory is an estimated amount since Apple does not make this information available.
    // The estimate is based on total physical memory, and doesn't account for the amount of memory the
    // particles will take. Therefore, we should do so here.
    availablePhysicalMemory -= particleSizeInMemory;
#endif

    // Calculate the total size in memory of a frame buffer for a single thread.
    const std::size_t bytesPerThread = numOutputImages * height * width * pixelSize;

    // Determine the number of threads that can be used based on the amount of memory we're allowed to used.
    // and the size of a frame buffer for a given thread.
    const std::size_t heuristicThreads =
        std::max( availablePhysicalMemory / bytesPerThread, static_cast<std::size_t>( 1 ) );

    // Get the number of logical threads supported by hardware.
    const std::size_t numLogicalThreads = static_cast<std::size_t>( boost::thread::hardware_concurrency() );

    // If the supplied thread cap is less than one we ignore it.
    const std::size_t actualThreadCap =
        threadCap < 1 ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>( threadCap );

    // We want to use the smaller of logical threads, heuristic threads, and the supplied thread cap.
    // We'll also explicitly ensure one thread is used in-case something went wrong somewhere.
    const std::size_t numThreads = boost::algorithm::clamp( std::min( numLogicalThreads, heuristicThreads ),
                                                            static_cast<std::size_t>( 1 ), actualThreadCap );

    FF_LOG( stats ) << "Capping number of threads automatically at " << numThreads << " because..." << std::endl;
    FF_LOG( stats ) << "\tNumber of frame buffers per thread: " << numOutputImages << std::endl;
    FF_LOG( stats ) << "\tBytes per frame buffer:" << height * width * pixelSize << std::endl;
    FF_LOG( stats ) << "\tTotal Pysical Memory Detected: " << totalPhysicalMemory << std::endl;
    FF_LOG( stats ) << "\tAvailable memory for frame buffers: " << availablePhysicalMemory << std::endl;
    FF_LOG( stats ) << "\tHeuristic thread cap: " << heuristicThreads << std::endl;
    FF_LOG( stats ) << "\tEffective user thread cap: " << actualThreadCap << std::endl;
    FF_LOG( stats ) << "\tNumber of logical threads detected " << numLogicalThreads << std::endl;

    return numThreads;
}

std::size_t krakatoa::get_total_physical_memory() {

    std::size_t totalPhysicalMemory = 0;

// Get the total physical memory of the system.
// There is no portable way to do this so we must use OS specific methods.
#if defined( _WIN32 )
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof( statex );
    GlobalMemoryStatusEx( &statex );
    totalPhysicalMemory = static_cast<std::size_t>( statex.ullTotalPhys );

#elif defined( __linux )
    // Linux has the same capabilities as Windows here, so we'll do the same thing.
    const std::size_t pageSize = static_cast<std::size_t>( sysconf( _SC_PAGESIZE ) );
    totalPhysicalMemory = static_cast<std::size_t>( sysconf( _SC_PHYS_PAGES ) ) * pageSize;

#elif defined( __APPLE__ )
    // As you might expect, Apple doesn't allow us to get all the information we need to do the job.
    // Namely, there's no way to query the amount of available physical memory.
    // Because of this, we need to use a slightly different heuristic.
    // For each common amount of total physical memory, we'll assume a certain amount of this can be used.
    int mib[] = { CTL_HW, HW_MEMSIZE };
    int64_t size = 0;
    std::size_t len = sizeof( size );
    if( sysctl( mib, 2, &size, &len, NULL, 0 ) == 0 ) {
        totalPhysicalMemory = static_cast<std::size_t>( size );
    }
#endif

    return totalPhysicalMemory;
}

std::size_t krakatoa::get_available_physical_memory() {

    std::size_t totalPhysicalMemory = get_total_physical_memory();
    std::size_t availablePhysicalMemory = 0;

#if defined( _WIN32 )
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof( statex );
    GlobalMemoryStatusEx( &statex );
    availablePhysicalMemory = static_cast<std::size_t>( statex.ullAvailPhys );

#elif defined( __linux )
    const std::size_t pageSize = static_cast<std::size_t>( sysconf( _SC_PAGESIZE ) );
    availablePhysicalMemory = static_cast<std::size_t>( sysconf( _SC_AVPHYS_PAGES ) ) * pageSize;

#elif defined( __APPLE__ )
    const std::size_t totalPhysicalInGigabytes = ( ( totalPhysicalMemory / 1024 ) / 1024 ) / 1024;
    if( totalPhysicalInGigabytes < 9 ) {
        availablePhysicalMemory = totalPhysicalMemory / 4;
    } else if( totalPhysicalInGigabytes < 17 ) {
        availablePhysicalMemory = totalPhysicalMemory / 2;
    } else if( totalPhysicalInGigabytes < 33 ) {
        availablePhysicalMemory = ( totalPhysicalMemory * 3 ) / 4;
    } else {
        availablePhysicalMemory = ( totalPhysicalMemory * 7 ) / 8;
    }
#endif

    return availablePhysicalMemory;
}