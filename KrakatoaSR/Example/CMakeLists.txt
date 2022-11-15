# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required( VERSION 3.15 FATAL_ERROR )

project( KrakatoaSRExamples )

foreach( EXAMPLE_INDEX RANGE 1 19 )
    set( TARGET_NAME "example${EXAMPLE_INDEX}" )
    if( EXAMPLE_INDEX LESS 10 )
        set( TARGET_NAME "example0${EXAMPLE_INDEX}" )
    endif()
    
    add_executable( ${TARGET_NAME} "${TARGET_NAME}.cpp" )
    target_link_libraries( ${TARGET_NAME} krakatoasr )
    set_target_properties( ${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${KrakatoaSR_BINARY_DIR} )
endforeach()
