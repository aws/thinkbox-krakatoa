// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/scoped_ptr.hpp>

#include <frantic/channels/property_map.hpp>
#include <frantic/graphics2d/boundrect2.hpp>
#include <frantic/volumetrics/tree_voxel_field2d.hpp>

#include <krakatoa/voxel_renderer/slice_container.hpp>

namespace krakatoa {
namespace voxel_renderer {

class slice_container_impl : public slice_container {
    frantic::volumetrics::tree_voxel_field2d<5> m_data;
    frantic::channels::channel_accessor<float> m_densityAccessor;
    frantic::channels::channel_accessor<frantic::graphics::color3f> m_emissionAccessor;
    frantic::channels::channel_accessor<float> m_bokehBlendInfluenceAccessor;

    boost::scoped_ptr<new_voxel_callback_interface> m_newVoxelCallback;

    std::vector<std::pair<int, int>> m_copyBlocks;

  public:
    slice_container_impl();

    virtual ~slice_container_impl();

    virtual void set_new_voxel_callback( new_voxel_callback_interface* newVoxelCallback );

    virtual void clear();

    virtual void reset( const frantic::channels::channel_map& dataLayout );

    virtual bool is_empty() const;

    virtual void construct_sample( frantic::channels::property_map& outSampleStruct ) const;

    virtual void construct_sample( sample& outSampleStruct ) const;

    virtual void get_voxel_indices_for_write( int x, int y, unsigned width, int outIndices[] );

    virtual void add_to_voxel_by_weight( int index, const frantic::channels::property_map& dataStruct, float weight );

    virtual bool get_sample( frantic::graphics2d::vector2f pos, sample& outSample ) const;

    virtual void get_voxels_as_particles( frantic::particles::particle_array& outParticles,
                                          const slice_coordsys& coordSys ) const;
};

} // namespace voxel_renderer
} // namespace krakatoa
