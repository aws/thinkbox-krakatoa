// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/scoped_ptr.hpp>

#include <krakatoa/voxel_renderer/filter2f.hpp>

namespace frantic {
namespace channels {
class property_map;
}
namespace graphics2d {
class boundrect2;
}
namespace particles {
class particle_array;
}
} // namespace frantic

namespace krakatoa {
namespace voxel_renderer {

class slice_coordsys;
class new_voxel_callback_interface;
class sample;

/**
 * Abstract class defining the interface for storing a 2D slice of a voxer field. Also provides a mechanism for
 * sampling the slice using a 2D filter.
 */
class slice_container {
  protected:
    boost::scoped_ptr<filter2f> m_readFilter;

  public:
    /**
     * Virtual destructor to allow polymorphic deletion
     */
    virtual ~slice_container() {}

    /**
     * Sets a new filter for use when calling get_sample().
     * @param filter A pointer to a new filter to assign. Takes ownership of this object.
     */
    void set_read_filter( filter2f* filter ) { m_readFilter.reset( filter ); }

    /**
     * Sets a new callback object that is fired each time a voxel is written to for the first time.
     * @param newVoxelCallback The callback object to use. Takes ownership of this object.
     */
    virtual void set_new_voxel_callback( new_voxel_callback_interface* newVoxelCallback ) = 0;

    /**
     * Clears the container of all existing data.
     */
    virtual void clear() = 0;

    /**
     * Resets this object to store the specified data. Clears any existing data from before this call.
     */
    virtual void reset( const frantic::channels::channel_map& dataLayout ) = 0;

    /**
     * @return True iff there are no defined voxels in this container. False otherwise.
     */
    virtual bool is_empty() const = 0;

    /**
     * Sets the channel map of the specified property_map and clears the storage. Initializes this property_map for
     * use with add_to_voxel_by_weight().
     * @deprecated
     * @param outSampleStruct The property_map to initialize.
     */
    virtual void construct_sample( frantic::channels::property_map& outSampleStruct ) const = 0;

    /**
     * Sets the channel map for the sample and allocates new storage to hold the results of get_sample(). This must
     * be called for a sample object before it can be passed to get_sample().
     * @param outSampleStruct The sample object to initialize.
     */
    virtual void construct_sample( sample& outSampleStruct ) const = 0;

    /**
     * Fills the passed in array with integer indices of the voxels in the square with corners (x, y) and (x+width,
     * y+width). If no voxel is defined for a specified coordinate, one will be created.
     * @param x The x-coordinate of the box of indices to fill.
     * @param y The y-coordinate of the box of indices to fill.
     * @param width The width of the box of indices to fill.
     * @param outIndices Array to fill with indices. Must have at least width*width elements.
     */
    virtual void get_voxel_indices_for_write( int x, int y, unsigned width, int outIndices[] ) = 0;

    /**
     * Does a weighted sum of the channels of the voxel with the specified index and the supplied property_map. The
     * result is stored back into the container.
     * @param index The index of the voxel to add the sample to. Retrieved via get_voxel_indices_for_write().
     * @param dataStruct The property_map holding the data to add to the specified voxel. Must have been initialized by
     * construct_sample().
     * @param weight The scalar weight to apply to each channel before adding to the specified voxel.
     */
    virtual void add_to_voxel_by_weight( int index, const frantic::channels::property_map& dataStruct,
                                         float weight ) = 0;

    /**
     * Fills a sample object with the data from the voxel field slice at the specified coordinate. The data will have
     * been filtered by m_readFilter.
     * @param pos The 2D voxel coordinate to sample from.
     * @param outSample The sample object to store the results of filtering the voxel field slice at the specified
     * coordinate. Must have been initialized by construct_sample().
     * @return True iff some voxels in the filter's radius were filled with data.
     */
    virtual bool get_sample( frantic::graphics2d::vector2f pos, sample& outSample ) const = 0;

    /**
     * Fills the specified particle array with a particle for each voxel in the field slice.
     * @param outParticles The particle array to fill with particles.
     * @param coordSys The slice_coordsys used to transform the voxel data into worldspace.
     */
    virtual void get_voxels_as_particles( frantic::particles::particle_array& outParticles,
                                          const slice_coordsys& coordSys ) const = 0;
};

/**
 * Abstract class that defines the interface for a callback to be fired by slice_container when a voxel is written to
 * for the first time.
 */
class new_voxel_callback_interface {
  public:
    virtual ~new_voxel_callback_interface() {}

    /**
     * Callback function that is invoked after a slice_container writes to a group of voxels for the first time.
     * @param bounds The bounding rectangle of the voxels written to.
     */
    virtual void on_new_voxel( const frantic::graphics2d::boundrect2& bounds ) = 0;
};

} // namespace voxel_renderer
} // namespace krakatoa
