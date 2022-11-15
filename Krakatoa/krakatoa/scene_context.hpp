// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/graphics/camera.hpp>

#include <krakatoa/collection_interface.hpp>
#include <krakatoa/geometry_renderer.hpp>
#include <krakatoa/light_object.hpp>
#include <krakatoa/shared_object.hpp>

namespace krakatoa {

/**
 * This interface exists to provide the various disparate parts of krakatoa access to information about the
 * currently rendered scene. I intend this interface tobe a replacement for frantic::max3d::shaders::renderInformation
 * which has been used in similar situations. And used poorly I might add.
 */
class scene_context : public shared_object {
  public:
    typedef boost::intrusive_ptr<scene_context> ptr_type; /**Use this type when storing a scene_context. Never create
                                                             one on the stack, or hold a scene_context*. Please.*/

    typedef collection_interface<matte_primitive_ptr>
        matte_collection; /**The interface exposed for accessing geometry objects*/

    typedef collection_interface<light_object_ptr>
        light_collection; /**The interface exposed for accessing light objects*/

    /**
     * @return the current scene time in seconds. In 3ds Max this can be converted to ticks via the macro SecToTicks()
     */
    virtual double get_time() const = 0;

    /**
     * @param time The new scene time, in seconds.
     */
    virtual void set_time( double time ) = 0;

    /**
     * @return The active camera the scene is being viewed from.
     */
    virtual const frantic::graphics::camera<float>& get_camera() const = 0;

    /**
     * @overload
     */
    virtual frantic::graphics::camera<float>& get_camera() = 0;

    /**
     * @param cam The new camera to associate with this scene_context
     */
    virtual void set_camera( const frantic::graphics::camera<float>& cam ) = 0;

    /**
     * @param cams The list of cameras to associate with this scene_context
     */
    virtual void set_camera( const std::vector<frantic::graphics::camera<float>>& cams ) = 0;

    /**
     * @return A read-only interface to the collection of geometry objects in the scene.
     */
    virtual const matte_collection& get_matte_objects() const = 0;

    /**
     * @return A read-only interface to the collection of light objects in the scene.
     */
    virtual const light_collection& get_light_objects() const = 0;
};

// Use this type for storing all scene_context objects.
typedef scene_context::ptr_type scene_context_ptr;

} // namespace krakatoa
