// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/color_with_alpha.hpp>
#include <frantic/graphics2d/framebuffer.hpp>

namespace krakatoa {

/**
 * Function for watermarking output bitmaps
 */
void apply_krakatoa_watermark( frantic::graphics2d::framebuffer<frantic::graphics::color6f>& framebuffer,
                               const frantic::graphics::color3f& watermarkColor );

/**
 * NULL function passed to renderer for not watermarking bitmaps
 */
inline void null_watermark( frantic::graphics2d::framebuffer<frantic::graphics::color6f>& /*framebuffer*/ ) { return; }

/**
 * Checks for a common hacking technique that cut out the code that added the watermark.
 * This function checks if the watermark function has been removed using the same binary hack.
 * There was a crack on cgpersia released on July 26, 2013 that cut out the watermark function.
 * All versions of Krakatoa should run this function to ensure that that crack can no longer be used.
 * @return If it has been hacked.
 */
bool check_for_watermark_crack();

} // namespace krakatoa
