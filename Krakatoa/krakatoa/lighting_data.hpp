// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/color3f.hpp>
#include <frantic/graphics/vector3f.hpp>

namespace krakatoa {

struct lighting_data {
    const frantic::graphics::color3f& m_emission;
    const frantic::graphics::color3f& m_lighting;
    const frantic::graphics::vector3f& m_normal;
    const frantic::graphics::vector3f& m_position;

    lighting_data( const frantic::graphics::color3f& emission, const frantic::graphics::color3f& lighting,
                   const frantic::graphics::vector3f& normal, const frantic::graphics::vector3f& position )
        : m_emission( emission )
        , m_lighting( lighting )
        , m_normal( normal )
        , m_position( position ) {}
};
} // namespace krakatoa