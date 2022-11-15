// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/smart_ptr.hpp>

namespace frantic {
namespace channels {
class channel_map;
}

namespace logging {
class progress_logger;
}
} // namespace frantic

namespace krakatoa {
namespace voxel_renderer {

class slice_coordsys;
class slice_container;

/**
 * Abstract class defining the interface for producing a slice of a voxel field.
 */
class data_source {
  public:
    typedef boost::shared_ptr<const slice_coordsys> coordsys_ptr;
    typedef boost::shared_ptr<slice_container> data_ptr;
    typedef boost::shared_ptr<frantic::logging::progress_logger> progress_logger_ptr;

  protected:
    progress_logger_ptr m_progressLogger;

  public:
    virtual void set_progress_logger( progress_logger_ptr progressLogger ) { m_progressLogger = progressLogger; }

    /**
     * @return A reference to the channels available from this object.
     */
    virtual const frantic::channels::channel_map& get_channel_map() const = 0;

    /**
     * Resets the data source, a provides a new coordinate system in which to produce data.
     * @return The min and max slice coordinates for the data contained in this object.
     */
    virtual std::pair<int, int> reset( coordsys_ptr coordSys ) = 0;

    /**
     * Fills the supplied container with the data for the current slice of the slice_coordsys provided to the
     * last call to reset().
     * @param dataSlice A container to fill with the data stored in this object.
     * @return False iff no data was written to the container. True otherwise.
     */
    virtual bool do_step( data_ptr dataSlice ) = 0;
};

} // namespace voxel_renderer
} // namespace krakatoa
