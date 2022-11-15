// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoa/renderer.hpp>
#include <krakatoa/voxel_renderer/filter3f.hpp>

namespace krakatoa {
namespace voxel_renderer {

// HACK: Compatibility for now.
namespace renderer_mode {
typedef krakatoa::renderer::mode_type::enum_t mode_type;
}

/**
 * A pure virtual class that creates the interface for clients using a krakatoa voxel renderer.
 */
class voxel_renderer : public renderer {
  public:
    typedef boost::shared_ptr<voxel_renderer> ptr_type;

  protected:
    boost::shared_ptr<filter3f> m_particleFilter;

    // Spacing between voxel samples in world units.
    float m_voxelSize;

    // Current time in [0,1] within the motion blur sample period.
    float m_mblurTime;

    // Width of current sample range for motion [0,1]. Used with jittered motion blur to give a range of times to
    // particles.
    float m_mblurRange;

    // Flag for enabling depth of field approximation.
    bool m_doDOF;

  public:
    /**
     * Factory function for creating a concrete instance of this interface class.
     * @return a new instance of a class implementing this interface.
     */
    static voxel_renderer* create_instance();

    /**
     * Default constructor. Sets defaults for various POD members.
     */
    voxel_renderer() {
        m_voxelSize = 1.f;
        m_mblurTime = 0.5f;
        m_mblurRange = 0;
        m_doDOF = false;
    }

    /**
     * Virtual destructor to enable polymorphic deletion.
     */
    virtual ~voxel_renderer() {}

    void set_particle_filter( boost::shared_ptr<filter3f> theFilter ) { m_particleFilter = theFilter; }

    /**
     * @param voxelSize The new size to use for spacing between voxels.
     */
    void set_voxel_size( float voxelSize ) { m_voxelSize = voxelSize; }

    /**
     * @param renderElement Currently ignored.
     */
    virtual void add_render_element( render_element_interface_ptr renderElement ) = 0;

    /**
     * Does nothing
     */
    virtual void precompute_lighting() = 0;

    /**
     * This is the main rendering function. Will use the state specified by all calls to set_XXXX() to render an image
     * of the described particles and scene.
     * @note You must call at least set_particles() before calling this.
     * @param outImage The image to draw the particles to.
     */
    virtual void render( renderer::image_type& outImage ) = 0;
};

typedef voxel_renderer::ptr_type voxel_renderer_ptr;

} // namespace voxel_renderer
} // namespace krakatoa
