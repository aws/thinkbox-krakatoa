// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/color3f.hpp>
#include <frantic/graphics/vector3f.hpp>

namespace krakatoa {

/**
 * Interface for applying some sort of particpating media (also known as an atmosphere). Typically used for fog, or an
 * underwater effect.
 */
class atmosphere_interface {
  public:
    typedef boost::shared_ptr<atmosphere_interface> ptr_type;

  public:
    virtual ~atmosphere_interface() {}

    /**
     * Applies the effects of the atmosphere along the line segment from 'from' to 'to' to 'inoutLight'.
     * @param inoutLight Before the call, this is the amount of light reaching 'to' from 'from'. Afterwards it is
     * affected by the atmosphere.
     * @param from The point where the light is originating.
     * @param to The point where the light is being evaluated.
     */
    virtual void apply_atomsphere( frantic::graphics::color3f& inoutLight, const frantic::graphics::vector3f& from,
                                   const frantic::graphics::vector3f& to ) const = 0;
};

typedef atmosphere_interface::ptr_type atmosphere_interface_ptr;

} // namespace krakatoa
