// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/smart_ptr.hpp>
#include <frantic/channels/channel_map.hpp>
#include <set>

namespace krakatoa {

/**
 * This abstract class defines an interface for Krakatoa shaders that computes the
 * light scattered into the direction of the eye at a specific evaluation position.
 *
 * @note For a piecewise Volume Rendering Equation:
 *   L(x,w) = Integral[0,s]( e^(-Tau(x,x')) * sigmaA(x') * Le(x',w) )dx'
 *            + Integral[0,s]( e^(-Tau(x,x')) * F(x',w) )dx'
 *            + e^(-Tau(x,x+s*w)) * L(x - s*w,w)
 *   The shade() call is providing F(x',w) which is the inscattered light at x' in the direction w.
 */
class krakatoa_shader {
    std::set<frantic::tstring> m_defaultChannels;

  protected:
    frantic::channels::channel_map m_requiredChannels;
    boost::scoped_array<char> m_defaultValues;

    bool has_default( const frantic::tstring& name ) const {
        return ( m_defaultChannels.find( name ) != m_defaultChannels.end() );
    }

  public:
    typedef boost::shared_ptr<krakatoa_shader> ptr_type;

  public:
    krakatoa_shader() {}

    virtual ~krakatoa_shader() {}

    /**
     * This function will finish the shader creation and MUST be called after all set_channel_default()
     * calls. The subclass shader will set up its accessors using the channel map passed in. Any channels
     * missing from the passed map will be set to constants given by set_constant_property(). If neither
     * is available, an exception will be thrown.
     * @param pcm A channel map describing the input layout for calls to shade()
     */
    virtual void set_channel_map( const frantic::channels::channel_map& pcm ) = 0;

    /**
     * This function will compute the amount of inscattered light from a particular light-source.
     *
     * @param toEye A normalized vector pointing towards the observer (ie. The eye, the camera, etc.)
     * @param toLight A normalized vector pointong towards the light-source
     * @param incidentLight The amount and variety of light incident upon the shading point. This value
     *                       has been attenuated by marching through the density field from the light to
     *                       the shading point.
     * @param scatterCoefficient The density field's scattering co-efficient at the shading location.
     *                            This is analogous to the diffuse color of a BRDF, except weighted by the
     *                            density of the volume at the shading location.
     * @param renderData A pointer to the various data that are sampled at this shading location.
     * @return A color3f object that represents the amount of light reflected towards the eye.
     */
    virtual frantic::graphics::color3f shade( const frantic::graphics::vector3f& toEye,
                                              const frantic::graphics::vector3f& toLight,
                                              const frantic::graphics::color3f& incidentLight,
                                              const frantic::graphics::color3f& scatterCoefficient,
                                              const char* renderData ) const = 0;

    /**
     * Some shaders produce extra output that is stored back into a particle (ie. Render elements). This routine is
     * called once per particle in order to initialize the default values for these fields.
     * @param particle The particle to initialize the defaults for.
     */
    virtual void set_particle_defaults( char* /*particle*/ ) const {}

    /**
     * This function will specify a default value for a given shader property.
     */
    template <class T>
    void set_channel_default( const frantic::tstring& propName, typename boost::call_traits<T>::param_type prop ) {
        frantic::channels::channel_cvt_accessor<T> accessor = m_requiredChannels.get_cvt_accessor<T>( propName );
        if( !m_defaultValues.get() )
            m_defaultValues.reset( new char[m_requiredChannels.structure_size()] );
        accessor.set( m_defaultValues.get(), prop );
        m_defaultChannels.insert( propName );
    }

    /**
     * This function will return a channel map describing the channels consumed
     * by this shader.
     * @return A channel map with the all the channels consumed by this shader.
     */
    const frantic::channels::channel_map& get_input_channels() const { return m_requiredChannels; }

    /**
     * This function will derfine all the channels required by this shader into the given channel_map.
     * values.
     * @param inoutMap A reference to a channel_map which will be modified to include the shader channels
     *                which do not have override values. The map must not be finished.
     */
    void define_required_channels(
        frantic::channels::channel_map& inoutMap,
        frantic::channels::data_type_t overrideType = frantic::channels::data_type_invalid ) const {
        if( inoutMap.channel_definition_complete() )
            throw std::runtime_error(
                "krakatoa_shader::get_required_channels() - The provided channel map could not be modified." );
        for( std::size_t i = 0, iEnd = m_requiredChannels.channel_count(); i < iEnd; ++i ) {
            const frantic::channels::channel& ch = m_requiredChannels[i];
            if( !inoutMap.has_channel( ch.name() ) && !has_default( ch.name() ) )
                inoutMap.define_channel( ch.name(), ch.arity(),
                                         overrideType == frantic::channels::data_type_invalid ? ch.data_type()
                                                                                              : overrideType );
        }
    }
};

} // namespace krakatoa

namespace krakatoa {
typedef krakatoa::krakatoa_shader shader;
typedef shader::ptr_type shader_ptr;
} // namespace krakatoa
