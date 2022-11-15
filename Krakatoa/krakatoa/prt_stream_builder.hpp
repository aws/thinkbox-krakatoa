// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once


#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>
#include <frantic/channels/channel_map.hpp>
#include <frantic/graphics/units.hpp>
#include <frantic/particles/streams/particle_istream.hpp>

namespace krakatoa {

namespace render {
enum RENDER_LOAD_PARTICLE_MODE {
    PARTICLELOAD_EVENLY_DISTRIBUTE = 1,
    PARTICLELOAD_FIRST_N,
    PARTICLELOAD_EVENLY_DISTRIBUTE_IDS,
};
}

namespace viewport {
enum RENDER_LOAD_PARTICLE_MODE {
    PARTICLELOAD_USE_RENDER_SETTING = 1,
    PARTICLELOAD_EVENLY_DISTRIBUTE,
    PARTICLELOAD_FIRST_N,
    PARTICLELOAD_EVENLY_DISTRIBUTE_IDS
};
}

namespace clamp_mode {
enum clamp_mode_enum { hold = 1, blank };
}

namespace enabled_mode {
enum enabled_mode_enum {
    viewport = ( 1 << 0 ),
    render = ( 1 << 1 ),
    all = ( 1 << 0 ) | ( 1 << 1 ),
};
}

// converts viewport load mode to render load mode
render::RENDER_LOAD_PARTICLE_MODE get_as_render_mode( viewport::RENDER_LOAD_PARTICLE_MODE loadMode );

// pre-calculates the fractional particle count given by 'get_reduced_particle_stream'
boost::int64_t get_fractional_particle_count( boost::int64_t rawNumParticles, double fraction, boost::int64_t limit );

// stream builder that takes a set of input files, and gives back a concatenated stream of all requested files
class prt_stream_builder {
  public:
    prt_stream_builder();
    ~prt_stream_builder();

    void reset_to_defaults();
    boost::shared_ptr<frantic::particles::streams::particle_istream> generate_stream();

    size_t num_errors();
    void get_error( size_t index, frantic::tstring& outError );
    void clear_errors();

  private:
    void get_files_at_frame( int currentFrame, std::vector<frantic::tstring>& outFiles );
    void handle_error( const std::exception& error );
    void error_message( const frantic::tstring& error );

  public:
    // operational flags
    bool m_throwOnError;
    bool m_cacheErrors;
    bool m_logErrors;
    bool m_threadingSupport;

    // times are to be expressed in _frames_
    double m_currentTime;
    double m_framesPerSecond;
    double m_timeDerivative;

    // timing controls
    int m_frameOffset;
    bool m_interpolateSubFrames;
    bool m_keepVelocityChannel;

    // load fraction control
    double m_loadFraction;
    boost::int64_t m_loadLimit;
    render::RENDER_LOAD_PARTICLE_MODE m_loadMode;

    // range control
    int m_rangeStart;
    int m_rangeEnd;
    clamp_mode::clamp_mode_enum m_rangeStartClampMode;
    clamp_mode::clamp_mode_enum m_rangeEndClampMode;

    // input filenames
    std::vector<frantic::tstring> m_fileSequences;
    std::vector<frantic::tstring> m_singleFiles;

    // misc
    frantic::graphics::coordinate_system::option m_coordinateSystem;
    double m_scaleToMeters;

  private:
    std::vector<frantic::tstring> m_errorMessages;
};

} // namespace krakatoa
