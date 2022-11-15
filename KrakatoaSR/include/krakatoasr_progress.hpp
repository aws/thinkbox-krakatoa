// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_PROGRESS__
#define __KRAKATOASR_PROGRESS__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * Interface class to be implemented by the user which allows progress tracking during the renderer.
 * It is useful for providing the user with details of what step Krakatoa is currently performing, and the step's
 * "percent completed" details.
 */
class progress_logger_interface {
  public:
    /**
     * Called by the renderer when it changes its current rendering state. This call should reset the progress to 0.0.
     * For example, Krakatoa has multiple steps in rendering (sorting, rendering, drawing, etc.). This function is
     * called between these steps.
     * @param title The name of the step Krakatoa is currently performing.
     */
    virtual void set_title( const char* title ) = 0;

    /**
     * Called by the renderer to update the renderer's progress.
     * Each step will have its own progress and this function will be called to update.
     * @param progress The fraction completed of the current step. This value will be between 0.0 and 1.0.
     */
    virtual void set_progress( float progress ) = 0;
};

/// @return A built-in implmentation of the progress_logger_interface which will print logging data to standard out.
FCNEXPORT progress_logger_interface* get_console_progress_logger();

/**
 * Interface class to be implemented by the user which allows the user to "see" semi-complete versions of the final
 * rendered image during rendering. It is useful to provide visual feedback of the image during the render.
 */
class frame_buffer_interface {
  public:
    /**
     * Called periodically by the renderer and provides the semi-complete rendered image to the user.
     * It is also called after the renderer has completed with the final rendered image.
     * @param width The width of the rendered image.
     * @param height The height of the rendered image.
     * @param data An array of length width*height with color and alpha components making up the rendered image.
     */
    virtual void set_frame_buffer( int width, int height, const frame_buffer_pixel_data* data ) = 0;
};

/**
 * Interface class to be implemented by the user to signal if the current render should be stopped prematurely and
 * control be returned to the user immediately.
 */
class cancel_render_interface {
  public:
    /**
     * Called periodically by the renderer, if the user returns 'true', then the render will cease immediately
     * regardless of its current progress, and control will exit the 'render' function
     * @return the user should return true if they want the render to be canceled now, otherwise, return false to
     * continue
     */
    virtual bool is_cancelled() = 0;
};

/**
 * Interface class to be implemented by the user to access all the logging messages Krakatoa produces.
 * A user must set their logging object using the global "set_global_logging_interface" function.
 * The verbosity of the errors sent to the user is defined by the global "set_global_logging_level" and can range
 * anywhere from statistics to error messages.
 *
 * Example Usage:
 * @code
 *     class my_logger : public krakatoasr::logging_interface {
 *     public:
 *         virtual void write_log_line( const char* line, krakatoasr::logging_level_t level ) {
 *             std::cout << "My Custom Logger Says: " << line << std::endl;
 *         }
 *     };
 *     int main() {
 *         my_logger* myLogger = new my_logger;
 *         krakatoasr::set_global_logging_interface( myLogger );
 *         //DO KRAKATOA RENDER
 *         delete myLogger; //Do not delete until after all Krakatoa calls.
 *     }
 * @endcode
 */
class CLSEXPORT logging_interface {
  public:
    virtual ~logging_interface();
    /**
     * Called periodically by the renderer to provide all statistic/log/warning/error messages to the user.
     * @param line The log message.
     * @param level The logging level that this message corresponds to. Users can supress various levels by using the
     * global "set_global_logging_level" function.
     */
    virtual void write_log_line( const char* line, logging_level_t level ) = 0;
};
//! Default implementation of logging_interface. This implementation is used by default, all messages go to standard
//! out.
FCNEXPORT logging_interface* get_console_logger();

//! Defines the global logging level of the renderer and how verbose it is. Having a high verbosity can slow the render.
FCNEXPORT void set_global_logging_level( logging_level_t level );
//! Defines the global logging interface. A custom "logging_interface" object can be provided. This logger object must
//! not be deleted prior to calling any Krakatoa functions.
FCNEXPORT void set_global_logging_interface( logging_interface* logger );

} // namespace krakatoasr

#endif
