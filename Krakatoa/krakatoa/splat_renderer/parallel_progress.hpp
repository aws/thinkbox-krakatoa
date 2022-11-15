// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/logging/render_progress_logger.hpp>

#include <tbb/atomic.h>
#include <tbb/tbb_thread.h>

namespace krakatoa {
namespace splat_renderer {

class parallel_render_progress_logger_master {
    tbb::tbb_thread::id m_loggingThreadId;
    tbb::atomic<std::size_t> m_curTicks;

    boost::shared_ptr<frantic::logging::render_progress_logger> m_delegateLogger;

    std::size_t m_totalTicks;
    std::size_t m_imageOrdering;

    typedef boost::function<void( frantic::graphics2d::framebuffer<frantic::graphics::color6f>& )> watermark_type;

    watermark_type m_watermark;

  public:
    friend class parallel_render_progress_logger;

    void update_progress( std::size_t tickCount ) {
        std::size_t curTicks = tickCount + m_curTicks.fetch_and_add( tickCount );

        if( tbb::this_tbb_thread::get_id() == m_loggingThreadId )
            m_delegateLogger->update_progress( (long long)curTicks, (long long)m_totalTicks );
    }

    void update_progress_and_image( std::size_t tickCount,
                                    frantic::graphics2d::framebuffer<frantic::graphics::color6f>& framebuffer,
                                    std::size_t relativeOrdering ) {
        update_progress( tickCount );

        if( tbb::this_tbb_thread::get_id() == m_loggingThreadId && relativeOrdering <= m_imageOrdering ) {
            m_imageOrdering = relativeOrdering;

            if( m_watermark )
                m_watermark( framebuffer );

            m_delegateLogger->update_frame_buffer( framebuffer );
        }
    }

  public:
    parallel_render_progress_logger_master( boost::shared_ptr<frantic::logging::render_progress_logger> delegateLogger,
                                            std::size_t totalTicks, const watermark_type& watermark )
        : m_delegateLogger( delegateLogger )
        , m_loggingThreadId( tbb::this_tbb_thread::get_id() )
        , m_totalTicks( totalTicks )
        , m_imageOrdering( std::numeric_limits<std::size_t>::max() )
        , m_watermark( watermark ) {
        m_curTicks = 0;
    }
};

class parallel_render_progress_logger {
    std::size_t m_tickCount;
    std::size_t m_updateTickCount;
    parallel_render_progress_logger_master* m_master;

  public:
    parallel_render_progress_logger( parallel_render_progress_logger_master& master, std::size_t updateTickCount )
        : m_master( &master )
        , m_tickCount( 0 )
        , m_updateTickCount( updateTickCount ) {}

    void tick() {
        if( ++m_tickCount >= m_updateTickCount ) {
            m_master->update_progress( m_tickCount );
            m_tickCount = 0;
        }
    }

    void update_image( frantic::graphics2d::framebuffer<frantic::graphics::color6f>& image,
                       std::size_t relativeOrdering ) {
        m_master->update_progress_and_image( m_tickCount, image, relativeOrdering );
        m_tickCount = 0;
    }
};

} // namespace splat_renderer
} // namespace krakatoa
