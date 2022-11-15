// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once
#if defined( OPENVDB_AVAILABLE )

// Clang and GCC need these functions to be defined before the main OpenVDB header is included.
#include <krakatoa/openvdb_overloads.hpp>

#include <krakatoa/raytrace_renderer/raytrace_renderer.hpp>
#include <krakatoa/splat_renderer/splat_lighting.hpp>

#include <frantic/graphics/color3f.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/RayIntersector.h>

namespace krakatoa {
namespace raytrace_renderer {

namespace detail {
class openvdb_renderer_impl_interface;

template <bool UseEmission, bool UseAbsorption>
class openvdb_renderer_impl;
} // namespace detail

/**
 * Two-stage ray-tracer that builds an OpenVDB grid from the lighting information from the splatting stage, then uses
 * the grid to perform ray-marching.
 *
 * More efficient than `raytrace_impl` when the number of rays to be traced and the number of voxels are large; performs
 * comparably when both values are small.
 */
class openvdb_renderer : public raytrace_renderer {
    // We want to give our implementation access to protected member variables of `renderer`.
    friend detail::openvdb_renderer_impl<true, true>;
    friend detail::openvdb_renderer_impl<true, false>;
    friend detail::openvdb_renderer_impl<false, true>;
    friend detail::openvdb_renderer_impl<false, false>;

  public:
    // Default OpenVDB grid type for given value type
    template <typename T>
    struct grid_of_t {
        typedef openvdb::Grid<typename openvdb::tree::Tree4<T, 5, 4, 3>::Type> type;
    };

    openvdb_renderer();

    virtual void add_render_element( krakatoa::render_element_interface_ptr renderElement ) override;

    /**
     * Compute the shadows. Should be called before `initialize`.
     */
    virtual void precompute_lighting() override;

    /**
     * Build the OpenVDB grid.
     */
    virtual void initialize() override;

    virtual void set_use_emission( bool useEmission ) override {
        if( m_initialized )
            throw std::runtime_error(
                "openvdb_renderer does not support enabling / disabling emission after it has been initialized!" );
        m_useEmissionChannel = useEmission;
    }

    virtual void set_use_absorption( bool useAbsorption ) override {
        if( m_initialized )
            throw std::runtime_error(
                "openvdb_renderer does not support enabling / disabling absorption after it has been initialized!" );
        m_useAbsorptionChannel = useAbsorption;
    }

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha ) override;

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha, float& nearestDepth, float alphaThreshold = 1e-4f ) override;

    virtual void raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1,
                                   alpha_type& accumAlpha ) override;

    virtual void render( image_type& outImage ) override;

    /**
     * Get a copy of the OpenVDB grid for debug purposes, or nullptr if the grid has not yet been initialized. The value
     * type is (density, lighting, emission, absorption).
     *
     * The type of this grid may not match that of the underlying grid, depending on the arguments supplied to
     * `set_use_emission` and `set_use_absorption`. The values of the emission and absorption components of the tuple
     * will be zeroed out if they are not present.
     */
    virtual grid_of_t<boost::tuple<float, frantic::graphics::color3f, frantic::graphics::color3f,
                                   frantic::graphics::color3f>>::type::ConstPtr
    grid() const;

    virtual ~openvdb_renderer();

  private:
    // In charge of doing ray marching
    boost::scoped_ptr<detail::openvdb_renderer_impl_interface> m_pImpl;

    // Whether or not `m_pImpl` has been initialized.
    bool m_initialized;

    // List of render elements to pass to the lighting engine
    std::vector<krakatoa::render_element_interface_ptr> m_renderElements;
};

} // namespace raytrace_renderer
} // namespace krakatoa

#endif