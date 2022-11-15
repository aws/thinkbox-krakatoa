// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <krakatoasr_datatypes.hpp>
#include <krakatoasr_progress.hpp>

#include <frantic/graphics/color_with_alpha.hpp>
#include <frantic/logging/render_progress_logger.hpp>
#include <frantic/strings/tstring.hpp>

namespace krakatoasr {

class krakatoasr_progress_logger : public frantic::logging::render_progress_logger {
  private:
    progress_logger_interface* m_progressLoggerUpdater;
    frame_buffer_interface* m_frameBufferUpdater;
    cancel_render_interface* m_cancelRenderCheck;
    time_t m_lastProgressUpdateTime;
    time_t m_lastFrameBufferUpdateTime;

  public:
    krakatoasr_progress_logger( progress_logger_interface* progressLoggerUpdater,
                                frame_buffer_interface* frameBufferUpdater, cancel_render_interface* cancelRenderCheck )
        : m_progressLoggerUpdater( progressLoggerUpdater )
        , m_frameBufferUpdater( frameBufferUpdater )
        , m_cancelRenderCheck( cancelRenderCheck )
        , m_lastProgressUpdateTime( 0 )
        , m_lastFrameBufferUpdateTime( 0 ) {}
    virtual ~krakatoasr_progress_logger() {}
    void set_title( const frantic::strings::tstring& title ) {
        if( m_progressLoggerUpdater )
            m_progressLoggerUpdater->set_title( frantic::strings::to_string( title.c_str() ).c_str() );
    }
    void update_progress( long long completed, long long maximum ) {
        update_progress( (float)completed * 100.f / maximum );
    }
    void update_progress( float progressPercent ) {
        if( !m_progressLoggerUpdater && !m_cancelRenderCheck )
            return;
        time_t currentTime = time( NULL ); // is doing a clock call more efficient than calling update? it is if update
                                           // does a console print, but it's tough to tell.
        if( currentTime > m_lastProgressUpdateTime ) { // only call update once a second
            m_lastProgressUpdateTime = currentTime;
            check_for_abort(); // this is here because check_for_abort doesn't get called much by Krakatoa.
            if( m_progressLoggerUpdater ) {
                float totalProgress = get_adjusted_progress( progressPercent );
                m_progressLoggerUpdater->set_progress( totalProgress * 0.01f );
            }
        }
    }
    virtual void update_frame_buffer( frantic::graphics2d::framebuffer<frantic::graphics::color6f>& buffer ) {
        if( !m_frameBufferUpdater )
            return;

        // At least one second must pass before currentTime is greater than m_lastFrameBufferUpdateTime
        // If this is called at 100% completion, we will also display a frame to ensure the last frame is shown. This is
        // a little hacky, but in KSR, we're being sure to set the progress to 100% prior to displaying the last frame
        // buffer.
        time_t currentTime = time( NULL );
        if( currentTime > m_lastFrameBufferUpdateTime || m_progressEnd > 99.999f ) {
            m_lastFrameBufferUpdateTime = currentTime;

            // NOTE: this call assumes color6f has the same memory layout as frame_buffer_pixel_data. Which is does. but
            // if that changes, we're boned.
            const std::vector<frantic::graphics::color6f>& vectorBuffer = buffer.data();
            const frantic::graphics::color6f* rawBuffer = &vectorBuffer[0];

            m_frameBufferUpdater->set_frame_buffer( buffer.width(), buffer.height(),
                                                    (const frame_buffer_pixel_data*)rawBuffer );
        }
    }

    virtual void check_for_abort() {
        if( m_cancelRenderCheck && m_cancelRenderCheck->is_cancelled() ) {
            throw frantic::logging::progress_cancel_exception(
                frantic::strings::to_string( "Operation cancelled by user." ) );
        }
    }
};

} // namespace krakatoasr
