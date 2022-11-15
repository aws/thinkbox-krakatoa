// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <krakatoa/voxel_renderer/data_source.hpp>
#include <krakatoa/voxel_renderer/filter3f.hpp>

#include <krakatoa/renderer.hpp>

namespace krakatoa {
namespace voxel_renderer {

/**
 * Implementation of the data_source interface that filters particles onto a voxel field.
 */
class particle_data_source : public data_source {
  public:
    typedef renderer::particle_container_type particle_type;

    particle_type* m_particles;
    particle_type::const_iterator m_curIt, m_endIt;

    boost::shared_ptr<filter3f> m_drawFilter;

    coordsys_ptr m_coordSys;

    bool m_disableThreading;

    float m_mblurTimeSeconds;

  public:
    particle_data_source( particle_type& particles, bool disableThreading );

    void set_draw_filter( filter3f* drawFilter ) { m_drawFilter.reset( drawFilter ); }

    void set_draw_filter( boost::shared_ptr<filter3f> drawFilter ) { m_drawFilter = drawFilter; }

    virtual ~particle_data_source();

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual void set_motion_blur_time( float mblurTimeIntervalCenterSeconds, float mblurTimeIntervalWidthSeconds,
                                       int seed = 42 );

    virtual std::pair<int, int> reset( coordsys_ptr coordSys );

    virtual bool do_step( data_ptr dataSlice );
};

} // namespace voxel_renderer
} // namespace krakatoa
