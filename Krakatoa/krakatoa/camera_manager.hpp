// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/camera.hpp>

#include <boost/shared_ptr.hpp>

namespace krakatoa {

class camera_manager {
  public:
    typedef boost::shared_ptr<camera_manager> ptr_type;

  private:
    std::vector<frantic::graphics::camera<float>> m_cameras;

  public:
    camera_manager();
    camera_manager( const frantic::graphics::camera<float>& camera );
    camera_manager( const std::vector<frantic::graphics::camera<float>>& cameraList );

    frantic::graphics::camera<float>& get_current_camera();
    const frantic::graphics::camera<float>& get_current_camera() const;
};

typedef camera_manager::ptr_type camera_manager_ptr;

} // namespace krakatoa
