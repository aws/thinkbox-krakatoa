// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_progress.hpp>

#include <krakatoasr_renderer/params.hpp>

namespace krakatoasr {

//
//
// Progress Updating Interface
//
//

/**
 * A console logger.
 * This is Krakatoa SR's default logger. There is only one global instance of this object.
 */
class console_progress_logger : public progress_logger_interface {
  private:
    std::string m_prevTitle;
    int m_prevCompletePercent;

  public:
    console_progress_logger() {
        m_prevTitle = "";
        m_prevCompletePercent = -1;
    }
    virtual void set_title( const char* title ) {
        if( title != m_prevTitle )
            FF_LOG( progress ) << title << std::endl;
        m_prevTitle = title;
    }
    virtual void set_progress( float progress ) {
        int completePercent = (int)( progress * 100 );
        if( completePercent != m_prevCompletePercent )
            FF_LOG( progress ) << completePercent << "% complete" << std::endl;
        m_prevCompletePercent = completePercent;
    }
};

// This is the single global instance of this object, and a function to get a pointer to it.
console_progress_logger g_staticConsoleProgressLogger;
progress_logger_interface* get_console_progress_logger() { return &g_staticConsoleProgressLogger; }

//
//
// Logging Interface
//
//

// we are using a global pointer for logging.
logging_interface* g_loggingInterface = NULL;

void set_global_logging_level( logging_level_t level ) {
    frantic::logging::set_logging_level(
        (frantic::logging::logging_level)
            level ); // assumes enums logging_level_t and frantic::logging::logging_level are the same.
}

// returns the console logger, which is actually a NULL pointer.
logging_interface* get_console_logger() {
    return NULL; // passing in this (NULL) result to "set_global_logging_interface" will actually reset the streams to
                 // print to standard out thus avoiding our custom streams.
}

// functor object used to forward to our user-set global logging interface.
// the only use of this function is for the global functor objects defined below.
class global_logging_interface_functor {
  private:
    krakatoasr::logging_level_t m_level;

  public:
    global_logging_interface_functor( krakatoasr::logging_level_t level )
        : m_level( level ) {}
    void operator()( const frantic::tchar* str ) {
        g_loggingInterface->write_log_line( frantic::strings::to_string( str ).c_str(), m_level );
    }
};
// global static functor objects of the above class.
static global_logging_interface_functor g_debugFuctor( krakatoasr::LOG_DEBUG );
static global_logging_interface_functor g_statFuctor( krakatoasr::LOG_STATS );
static global_logging_interface_functor g_progressFuctor( krakatoasr::LOG_PROGRESS );
static global_logging_interface_functor g_warningFuctor( krakatoasr::LOG_WARNINGS );
static global_logging_interface_functor g_errorFuctor( krakatoasr::LOG_ERRORS );
// global static stream objects for the above functors.
static frantic::logging::ffstreambuf<global_logging_interface_functor> g_debugStream( g_debugFuctor );
static frantic::logging::ffstreambuf<global_logging_interface_functor> g_statStream( g_statFuctor );
static frantic::logging::ffstreambuf<global_logging_interface_functor> g_progressStream( g_progressFuctor );
static frantic::logging::ffstreambuf<global_logging_interface_functor> g_warningStream( g_warningFuctor );
static frantic::logging::ffstreambuf<global_logging_interface_functor> g_errorStream( g_errorFuctor );

void set_global_logging_interface( logging_interface* logger ) {
    g_loggingInterface = logger; // set the global logging interface

    // if the global interface is now null, reset the streams to use standard out. Otherwise, use our globally defined
    // streams that forward to g_loggingInterface.
    if( logger == get_console_logger() ) { // actually NULL.
        frantic::logging::reset_default_streams();
    } else {
        frantic::logging::redirect_stream( frantic::logging::debug, &g_debugStream );
        frantic::logging::redirect_stream( frantic::logging::stats, &g_statStream );
        frantic::logging::redirect_stream( frantic::logging::progress, &g_progressStream );
        frantic::logging::redirect_stream( frantic::logging::warning, &g_warningStream );
        frantic::logging::redirect_stream( frantic::logging::error, &g_errorStream );
    }
}

// all user-defined logging_interface objects have a destructor.
logging_interface::~logging_interface() {
    // A simple safeguard from bad programmers. Make sure they're not destroying the global logging object.
    if( g_loggingInterface == this ) {
        frantic::logging::reset_default_streams();
        g_loggingInterface = NULL;
    }
}

} // namespace krakatoasr
