// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_RENDER_SAVING__
#define __KRAKATOASR_RENDER_SAVING__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * This class is for users to implement custom render image saving routines.
 * An object of this class can be passed in to the renderer using krakatoa_renderer::set_render_save_callback.
 * Interface class can be implemented by the user, or the user can use the provided "multi_channel_exr_file_saver" or
 * "file_saver" implementations.
 */
class render_save_interface {
  public:
    virtual ~render_save_interface() {}

    /**
     * Called at the end of a render. This function is passed all the image output from the render.
     * @param width The width of the output image(s)
     * @param height The height of the output image(s)
     * @param imageCount The number of images. There is always at least one image (RGBA). There will be more if the user
     * requested them by setting the renderer's enable_z_depth_render, enable_normal_render, etc.
     * @param listOfTypes An array of length "imageCount". Elements in the array are the image types (eg. RGBA, Z,
     * Normal, etc.) for the next "listOfImages" array.
     * @param listOfImages An array of image pointers. Each array element is a pointer to a width*height image data
     * buffer.
     */
    virtual void save_render_data( int width, int height, int imageCount, const output_type_t* listOfTypes,
                                   const frame_buffer_pixel_data* const* listOfImages ) = 0;
};

/**
 * EXR file implemenation of the render image saving callback.
 * Use an object of this class to save EXR images. If the user has requested multiple outputs (such as Z, Normal, etc.)
 * they will be stored as user-defined channels in the EXR file.
 */
class CLSEXPORT multi_channel_exr_file_saver : public render_save_interface {
  private:
    multi_channel_exr_file_saver_data* m_data;

  public:
    /**
     * Constructor
     * @param exrFilename An image file with this name will be created after the render completes.
     */
    multi_channel_exr_file_saver( const char* exrFilename );
    ~multi_channel_exr_file_saver();
    multi_channel_exr_file_saver( const multi_channel_exr_file_saver& s );
    multi_channel_exr_file_saver& operator=( const multi_channel_exr_file_saver& s );
    /// Implemented internally. Called internally by the renderer.
    virtual void save_render_data( int width, int height, int imageCount, const output_type_t* listOfTypes,
                                   const frame_buffer_pixel_data* const* listOfImages );

    /// Function to override the R,B,G,A color EXR channel names & bit depth
    /// Defaults to "R","G","B","A", krakatoasr::BIT_DEPTH_HALF
    void set_channel_name_rgba( const char* r, const char* g, const char* b, const char* a,
                                const krakatoasr::exr_bit_depth_t d = krakatoasr::BIT_DEPTH_HALF );
    /// Function to override the Z-depth EXR channel name & bit depth. Will only add this channel if
    /// "enable_z_depth_render" was enabled in the renderer. Defaults to "Z", kraktoasr::BIT_DEPTH_FLOAT
    void set_channel_name_z( const char* z, const krakatoasr::exr_bit_depth_t d = krakatoasr::BIT_DEPTH_FLOAT );
    /// Function to override the normal vector (X,Y,Z) EXR channel names & bit dpeth. Will only add these channels if
    /// "enable_normal_render" was enabled in the renderer. Defaults to "normal.X","normal.Y","normal.Z",
    /// kraktoasr::BIT_DEPTH_HALF
    void set_channel_name_normal( const char* x, const char* y, const char* z,
                                  const krakatoasr::exr_bit_depth_t d = krakatoasr::BIT_DEPTH_HALF );
    /// Function to override the velocity vector (X,Y,Z) EXR channel names & bit depth. Will only add these channels if
    /// "enable_velocity_render" was enabled in the renderer. Defaults to "velocity.X","velocity.Y",velocity.Z",
    /// krakatoasr::BIT_DEPTH_HALF
    void set_channel_name_velocity( const char* x, const char* y, const char* z,
                                    const krakatoasr::exr_bit_depth_t d = krakatoasr::BIT_DEPTH_HALF );
    /// Function to override the "occluded render element" R,G,B,A color EXR channel names & bit depth. Will only add
    /// these channels if "enable_occluded_rgba_render" was enabled in the renderer. Defaults to
    /// "occluded.R","occluded.G","occluded.B","occluded.A", kraktoasr::BIT_DEPTH_HALF
    void set_channel_name_rgba_occluded( const char* r, const char* g, const char* b, const char* a,
                                         const krakatoasr::exr_bit_depth_t d = krakatoasr::BIT_DEPTH_HALF );
    /// Function to override compression type for exr file
    /// Defaults to krakatoasr::COMPRESSION_ZIP
    void set_exr_compression_type( krakatoasr::exr_compression_t t = krakatoasr::COMPRESSION_ZIP );
    /// Sets the saver to save the EXR file as a tiled EXR image. Defaults to false.
    void set_tiled( bool tiled );
    /// Sets the width or height of the OpenEXR tiles if using tiled mode. Both default to 256.
    void set_tile_width( int width );
    void set_tile_height( int height );

    multi_channel_exr_file_saver_data* get_data();
};

/**
 * Image file implemenation of the render image saving callback.
 * The image format is based on the filename. Supported formats are: tiff, png, jpg, gif, etc.
 * NOTE: This class also handles EXR files, but unlike multi_channel_exr_file_saver, it will not do multi-layered within
 * a single EXR. Each output image must be a separate EXR file. Use an object of this class save these images. If the
 * user has requested multiple outputs (such as Z, Normal, etc.) they will need to provide filenames for each output
 * during construction.
 */
class CLSEXPORT file_saver : public render_save_interface {
  private:
    file_saver_data* m_data;

  public:
    /**
     * Constructor. Passing in empty strings for any of these parmeters disables that output type. Image format is
     * determined by file extension.
     * @param rbgaFilename The filename of the RBGA output image. Pass a filename if
     * @param zFile The filename for the Z-depth output image. Will only write a file if "enable_z_depth_render" was
     * enabled in the renderer.
     * @param normalFile The filename for the normal output image. Will only write a file if "enable_normal_render" was
     * enabled in the renderer.
     * @param velocityFile The filename for the velocity output image. Will only write a file if
     * "enable_velocity_render" was enabled in the renderer.
     * @param rgbaOccludedFile The filename for the "occluded render element" output image. Will only write a file if
     * "enable_occluded_rgba_render" was enabled in the render.
     */
    file_saver( const char* rgbaFile, const char* zFile = "", const char* normalFile = "",
                const char* velocityFile = "", const char* rgbaOccludedFile = "" );
    ~file_saver();
    file_saver( const file_saver& s );
    file_saver& operator=( const file_saver& s );
    /// Implemented internally. Called internally by the renderer.
    virtual void save_render_data( int width, int height, int imageCount, const output_type_t* listOfTypes,
                                   const frame_buffer_pixel_data* const* listOfImages );
};

} // namespace krakatoasr

#endif
