// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/optional.hpp>

#include <krakatoa/particle_render_element_interface.hpp>

namespace krakatoa {

struct mblur_data {
    typedef const frantic::graphics2d::image_channel<frantic::graphics::color6f>& out_image_t;
    typedef const std::vector<particle_render_element_interface_ptr>& render_elements_t;
    typedef std::vector<particle_render_element_interface_ptr>::const_iterator render_element_iterator_t;

    const float m_mblurTime;
    const out_image_t m_outImage;
    const boost::optional<out_image_t> m_behindMatteRenderElement;
    const render_elements_t m_renderElements;
    const std::size_t m_numPasses;

    mblur_data( float mblurTime, const out_image_t outImage, boost::optional<out_image_t> behindMatte,
                render_elements_t renderElements, std::size_t numPasses )
        : m_mblurTime( mblurTime )
        , m_outImage( outImage )
        , m_behindMatteRenderElement( behindMatte )
        , m_renderElements( renderElements )
        , m_numPasses( numPasses ) {}

    mblur_data( const mblur_data& other )
        : m_mblurTime( other.m_mblurTime )
        , m_outImage( other.m_outImage )
        , m_behindMatteRenderElement( other.m_behindMatteRenderElement )
        , m_renderElements( other.m_renderElements )
        , m_numPasses( other.m_numPasses ) {}
};
} // namespace krakatoa