// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/voxel_renderer/slice_coordsys.hpp>

namespace krakatoa {
namespace voxel_renderer {

class slice_coordsys_impl : public slice_coordsys {
    frantic::graphics::vector3f m_sliceToWorldAxes[3];
    float m_voxelSize;
    int m_currentSlice;

    bool m_isForward;

  public:
    slice_coordsys_impl();

    virtual ~slice_coordsys_impl();

    virtual int reset( const frantic::graphics::transform4f& cameraTM, float voxelSize );

    virtual std::pair<int, int> reset( const frantic::graphics::transform4f& cameraTM,
                                       const frantic::graphics::transform4f& lightTM, float voxelSize );

    virtual bool is_forward() const;

    virtual void set_slice_index( int index );

    virtual int get_slice_index() const;

    virtual std::pair<float, float> intersect_ray_with_slice( const frantic::graphics::ray3f& ray ) const;

    virtual frantic::graphics::vector3f transform_from_world( frantic::graphics::vector3f p ) const;

    virtual frantic::graphics::vector3f transform_to_world( frantic::graphics::vector3f p ) const;

    virtual frantic::graphics::vector3f transform_vector_from_world( frantic::graphics::vector3f p ) const;

    virtual frantic::graphics::vector3f transform_vector_to_world( frantic::graphics::vector3f p ) const;
};

} // namespace voxel_renderer
} // namespace krakatoa
