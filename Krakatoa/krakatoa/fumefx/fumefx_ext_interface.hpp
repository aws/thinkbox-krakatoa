// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/channels/channel_map.hpp>
#include <frantic/particles/streams/particle_istream.hpp>
#include <frantic/volumetrics/voxel_sampler_interface.hpp>

namespace krakatoa {
namespace fumefx {

/**
 * This abstract interface is used to communicate with Krakatoa's FumeFX extension DLLs.
 *
 * @note This interface must never be changed! It can only be extended via subclassing. If you need to add
 *       entirely new functionality, you must try and dynamic_cast this object to the more featured subclass.
 */
class fumefx_ext_interface {
  protected:
    virtual ~fumefx_ext_interface() {}

  public:
    /**
     * This function must be called when you are finished using this object. You do not directly call delete on this
     * object.
     */
    virtual void release() = 0;

    /**
     * Returns the version number of FumeFX that this was expect to work for.
     */
    // virtual __int64 get_fumefx_version() = 0;

    /**
     * Returns the value of PARTICLE_ISTREAM_INTERFACE_VERSION when this extension was created. Before calling
     * this->create_stream() you must verify that get_particle_istream_version() == PARTICLE_ISTREAM_INTERFACE_VERSION;
     */
    virtual __int64 get_particle_istream_version() { return PARTICLE_ISTREAM_INTERFACE_VERSION; }

    /**
     * Creates a new particle_istream object that reads from the FumeFX simulation stored at the given path.
     */
    virtual frantic::particles::streams::particle_istream*
    create_stream( const std::string& srcName, const std::string& simPath, int frameNumber,
                   const frantic::channels::channel_map& pcm,
                   frantic::volumetrics::voxel_sampler_interface_ptr pSampleGen, int disabledChannels, float minDensity,
                   float minFire ) = 0;
};

} // namespace fumefx
} // namespace krakatoa
