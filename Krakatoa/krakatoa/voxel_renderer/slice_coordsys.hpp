// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/ray3f.hpp>
#include <frantic/graphics/transform4f.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * Abstract class that defines the interface of a coordinate system for a slice of a voxel field.
 */
class slice_coordsys {
  public:
    /**
     * Virtual destructor to support polymorphic deletion
     */
    virtual ~slice_coordsys() {}

    /**
     * Resets the coordsys to be aligned with the specified camera, and having the specified voxel size.
     * @param cameraTM The transformation matrix of the camera object to align the slice_coordsys with.
     * @param voxelSize The spacing between voxels in world units.
     * @return The slice coordinate of the camera.
     */
    virtual int reset( const frantic::graphics::transform4f& cameraTM, float voxelSize ) = 0;

    /**
     * Resets the coordsys to be aligned such that a slice maximizes the angle between rays from a camera with
     * transformation matrix cameraTM and and a light with the specified TM.
     * @param cameraTM The transformation matrix of the camera object to align the slice_coordsys with.
     * @param lightTM The transformation matrix of the light object to align the slice_coordsys with.
     * @param voxelSize The spacing between voxels in world units.
     * @return The slice coordinates of the camera and light respectively.
     */
    virtual std::pair<int, int> reset( const frantic::graphics::transform4f& cameraTM,
                                       const frantic::graphics::transform4f& lightTM, float voxelSize ) = 0;

    /**
     * @return True iff the slice_coordsys marches away from the camera with each increase to set_slice_index().
     */
    virtual bool is_forward() const = 0;

    /**
     * @param index The slice coordinate to use when transforming to worldspace and vice-versa.
     */
    virtual void set_slice_index( int index ) = 0;

    /**
     * @return The current slice coordinate
     */
    virtual int get_slice_index() const = 0;

    /**
     * Intersects a world-space ray with the current slice.
     * @param ray The world-space ray to intersect with the slice.
     * @return The ray's t-value of the intersection, and the t-value of the ray's intersection with the previous slice.
     */
    virtual std::pair<float, float> intersect_ray_with_slice( const frantic::graphics::ray3f& ray ) const = 0;

    /**
     * @return The given point in worldspace, transformed into the slice_coordsys's space. An output Z-coordinate of 0
     * is exactly on the slice plane.
     */
    virtual frantic::graphics::vector3f transform_from_world( frantic::graphics::vector3f p ) const = 0;

    /**
     * @return The given point in slice_coordsys's space, transformed into worldspace. An input Z-coordinate of 0 is
     * exactly on the slice plane.
     */
    virtual frantic::graphics::vector3f transform_to_world( frantic::graphics::vector3f p ) const = 0;

    /**
     * @return The given direction vector in worldspace, transformed into the slice_coordsys's space.
     */
    virtual frantic::graphics::vector3f transform_vector_from_world( frantic::graphics::vector3f p ) const = 0;

    /**
     * @return The given direction vector in slice_coordsys's space, transformed into worldspace.
     */
    virtual frantic::graphics::vector3f transform_vector_to_world( frantic::graphics::vector3f p ) const = 0;
};

} // namespace voxel_renderer
} // namespace krakatoa
