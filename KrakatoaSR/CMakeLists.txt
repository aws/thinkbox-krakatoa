# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required( VERSION 3.15 FATAL_ERROR )

project( KrakatoaSR )

find_package( thinkboxcmlibrary REQUIRED )
include( PrecompiledHeader )
include( ThinkboxCMLibrary )

option( LIBRARY_TYPE "Either SHARED or STATIC" SHARED )
option( BUILD_EXAMPLES "Build the KrakatoaSR examples" OFF )

add_library( krakatoasr ${LIBRARY_TYPE} )

set_property( TARGET krakatoasr PROPERTY CXX_STANDARD 17 )

target_include_directories( krakatoasr PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

file( GLOB_RECURSE H_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	"include/*.h"
	"include/*.hpp"
)

file( GLOB_RECURSE CXX_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	"src/*.cpp"
)

target_sources( krakatoasr PRIVATE
    stdafx.cpp
    stdafx.h
    ${H_FILES}
    ${CXX_FILES}
)

if( ${LIBRARY_TYPE} STREQUAL "STATIC" )
    target_compile_definitions( krakatoasr PUBLIC KSR_STATIC )
else()
    # The Conan version of Boost was built with this, and it changes the library names.
    # As a result, we need to set this to tell Boost to look for the right libraries to
    # link against.
    target_compile_definitions( krakatoasr PUBLIC BOOST_AUTO_LINK_SYSTEM )
    if( APPLE )
        set( PLATFORM_LINK_FLAGS "-Wl,-exported_symbols_list ${PROJECT_SOURCE_DIR}/KrakatoaSR.exp" )
        target_link_libraries( krakatoasr "-framework CoreServices" )
        target_link_libraries( krakatoasr "-framework CoreFoundation" )
        target_link_libraries( krakatoasr "-framework IOKit" )
    elseif( LINUX )
        set( PLATFORM_LINK_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/KrakatoaSR.map -s" )
    endif()
endif()

find_package( thinkboxlibrary REQUIRED )
find_package( krakatoa REQUIRED )
find_package( Boost REQUIRED )
find_package( OpenEXR REQUIRED )
find_package( ZLIB REQUIRED )
find_package( TBB REQUIRED )
find_package( xxHash REQUIRED )
find_package( OpenImageIO REQUIRED )

target_include_directories( krakatoasr PUBLIC ${thinkboxlibrary_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${krakatoa_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${Boost_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${OpenEXR_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${ZLIB_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${TBB_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${xxHash_INCLUDE_DIRS} )
target_include_directories( krakatoasr PUBLIC ${OpenImageIO_INCLUDE_DIRS} )

target_link_libraries( krakatoasr PUBLIC thinkboxlibrary::thinkboxlibrary )
target_link_libraries( krakatoasr PUBLIC krakatoa::krakatoa )
target_link_libraries( krakatoasr PUBLIC Boost::Boost )
target_link_libraries( krakatoasr PUBLIC OpenEXR::OpenEXR )
target_link_libraries( krakatoasr PUBLIC ZLIB::ZLIB )
target_link_libraries( krakatoasr PUBLIC TBB::tbb )
target_link_libraries( krakatoasr PUBLIC xxHash::xxHash )
target_link_libraries( krakatoasr PUBLIC OpenImageIO::OpenImageIO )

frantic_common_platform_setup( krakatoasr )
frantic_default_source_groups( krakatoasr HEADERDIR include SOURCEDIR src )

add_precompiled_header( krakatoasr stdafx.h SOURCE_CXX stdafx.cpp )

# Disable optimization for the RelWithDebInfo configuration on Windows.
# This allows breakpoints to be hit reliably when debugging in Visual Studio.
if( WIN32 )
	target_compile_options( krakatoasr PRIVATE "$<$<CONFIG:Release>:/O2>$<$<CONFIG:RelWithDebInfo>:/Od>" )
endif()

if( BUILD_EXAMPLES )
    add_subdirectory( Example )
endif()

install( DIRECTORY include
         DESTINATION "include/.."
         FILES_MATCHING PATTERN "*.hpp"
)
install( TARGETS krakatoasr
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib
)
