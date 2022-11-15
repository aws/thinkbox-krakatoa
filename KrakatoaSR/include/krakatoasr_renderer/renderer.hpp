// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/shared_ptr.hpp>

#include <krakatoasr_renderer/params.hpp>

#include <krakatoa/raytrace_renderer/raytrace_renderer.hpp>

namespace krakatoasr {

/**
 * Decides if krakatoa needs to check out a license before rendering.
 * This function can be called prior to render_scene to determine if a license is needed.
 * This will return true if the image is larger than 640x480, and there is some form of output.
 * It will also always return true if params.errorOnMissingLicense is set.
 */
bool needs_license_to_render( const krakatoa_renderer_params& params );

/**
 * Renders a scene.
 * After this call returns, all particle streams are cleared from the params.
 * @param params The scene rendering is based entirely on this parameter object.
 * @param licenseEnforcer The licenser object that will request a license (if not already checked out).
 * @param licenseRequiredForPRTExport If this is false, then an "export" job (no image output) will not actually require
 * a license to exist.
 * @return true if the render completed successfully, false otherwise
 */
bool render_scene( krakatoa_renderer_params& params );

boost::shared_ptr<krakatoa::raytrace_renderer::raytrace_renderer>
setup_raytrace_renderer( krakatoa_renderer_params& params,
                         krakatoa::renderer::particle_container_type& particleBuffer );

} // namespace krakatoasr
