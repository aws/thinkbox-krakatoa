// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/camera_manager.hpp>

namespace krakatoa {

camera_manager::camera_manager()
    : m_cameras( 1 ) {}

camera_manager::camera_manager( const frantic::graphics::camera<float>& camera ) { m_cameras.push_back( camera ); }

camera_manager::camera_manager( const std::vector<frantic::graphics::camera<float>>& cameraList ) {
    m_cameras = cameraList;
}

frantic::graphics::camera<float>& camera_manager::get_current_camera() { return m_cameras.at( 0 ); }

const frantic::graphics::camera<float>& camera_manager::get_current_camera() const { return m_cameras.at( 0 ); }

} // namespace krakatoa
