# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required( VERSION 3.15 FATAL_ERROR )

project( Krakatoa )

find_package( thinkboxcmlibrary REQUIRED )
include( PrecompiledHeader )
include( ThinkboxCMLibrary )

add_library( krakatoa STATIC )

set_property( TARGET krakatoa PROPERTY CXX_STANDARD 17 )

target_include_directories( krakatoa PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
)

file( GLOB_RECURSE H_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	"krakatoa/*.h"
	"krakatoa/*.hpp"
)

file( GLOB_RECURSE CXX_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	"src/*.cpp"
)

target_sources( krakatoa PRIVATE
	stdafx.cpp
	stdafx.h
	${H_FILES}
	${CXX_FILES}
)

# The Conan version of Boost was built with this, and it changes the library names.
# As a result, we need to set this to tell Boost to look for the right libraries to
# link against.
target_compile_definitions( krakatoa PUBLIC BOOST_AUTO_LINK_SYSTEM )

set_target_properties( krakatoa PROPERTIES PROJECT_LABEL "Krakatoa" )

find_package( thinkboxlibrary REQUIRED )
find_package( Boost REQUIRED )
find_package( OpenEXR REQUIRED )
find_package( tinyxml2 REQUIRED )
find_package( ZLIB REQUIRED )
find_package( TBB REQUIRED )

target_include_directories( krakatoa PUBLIC ${thinkboxlibrary_INCLUDE_DIRS} )
target_include_directories( krakatoa PUBLIC ${Boost_INCLUDE_DIRS} )
target_include_directories( krakatoa PUBLIC ${OpenEXR_INCLUDE_DIRS} )
target_include_directories( krakatoa PUBLIC ${tinyxml2_INCLUDE_DIRS} )
target_include_directories( krakatoa PUBLIC ${ZLIB_INCLUDE_DIRS} )
target_include_directories( krakatoa PUBLIC ${TBB_INCLUDE_DIRS} )

target_link_libraries( krakatoa INTERFACE thinkboxlibrary::thinkboxlibrary )
target_link_libraries( krakatoa INTERFACE Boost::Boost )
target_link_libraries( krakatoa INTERFACE OpenEXR::OpenEXR )
target_link_libraries( krakatoa INTERFACE tinyxml2::tinyxml2 )
target_link_libraries( krakatoa INTERFACE ZLIB::ZLIB )
target_link_libraries( krakatoa INTERFACE TBB::tbb )

frantic_default_source_groups( krakatoa HEADERDIR krakatoa SOURCEDIR src )
frantic_common_platform_setup( krakatoa )

add_precompiled_header( krakatoa stdafx.h SOURCE_CXX stdafx.cpp )

# Disable optimization for the RelWithDebInfo configuration on Windows.
# This allows breakpoints to be hit reliably when debugging in Visual Studio.
if( WIN32 )
	target_compile_options( krakatoa PRIVATE "$<$<CONFIG:Release>:/O2>$<$<CONFIG:RelWithDebInfo>:/Od>" )
endif()

install( DIRECTORY krakatoa
		 DESTINATION include
		 FILES_MATCHING PATTERN "*.hpp"
)
install( TARGETS krakatoa
		 RUNTIME DESTINATION bin
		 LIBRARY DESTINATION lib
		 ARCHIVE DESTINATION lib
)
