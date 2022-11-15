// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/logging/progress_logger.hpp>
#include <frantic/particles/streams/particle_istream.hpp>

namespace krakatoa {

/**
 * Creates a particle repopulation stream from an existing particle stream.
 * When called, it will fully exhaust the pin stream.
 * It is based on creating a metaballs implicit surface around the existing particles, then using seeding particles from
 * the resulting voxel field.
 *
 * @param pin Original particles to be "multiplied".
 * @param fillRadius The radius around each particle to seed new particles.
 * @param fillRadiusSubdivs The number of seeding subdivisions. Increasing this number will exponentially increase
 * output particle count.
 * @param numParticlesPerSubdiv The number of particles placed in each subdivision. Increasing this number will linearly
 * increase output particle count.
 * @param densityFalloffStart A value between zero and one that defines the start (between the center and outter radius)
 * of the linear falloff of density. zero being no falloff, one being a linear falloff from the center of the original
 * particle to the outter radius.
 * @param useSignedDistanceDensityFalloff Base the falloff on a more accurate, but slower signed distance calcuation.
 * This is experimental.
 * @param logger The progress logger. Passing in a logger will possibly result in cancel exceptions to be thrown on user
 * cancel events.
 */
boost::shared_ptr<frantic::particles::streams::particle_istream>
create_particle_repopulation_istream( boost::shared_ptr<frantic::particles::streams::particle_istream> pin,
                                      float fillRadius, int fillRadiusSubdivs, int numParticlesPerSubdiv,
                                      float densityFalloffStart, unsigned randomSeed,
                                      frantic::logging::progress_logger& logger );

} // namespace krakatoa
