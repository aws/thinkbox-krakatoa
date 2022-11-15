// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/raytrace_renderer/openvdb_renderer.hpp>
#include <krakatoa/raytrace_renderer/raytrace_renderer.hpp>

#include <boost/shared_array.hpp>
#include <boost/unordered_map.hpp>

#include <frantic/channels/channel_map.hpp>
#include <frantic/graphics/raw_byte_buffer.hpp>
#include <frantic/graphics/size3.hpp>
#include <frantic/volumetrics/grid_tree_base.hpp>
#include <frantic/volumetrics/hash_grid.hpp>

namespace krakatoa {
namespace raytrace_renderer {

using frantic::graphics::vector3;
using frantic::graphics::vector3f;

class raytrace_impl : public raytrace_renderer {
    frantic::volumetrics::hash_grid m_voxels;
    frantic::graphics::boundbox3f m_voxelBounds;

    frantic::channels::channel_accessor<float> m_voxelDensityAccessor;
    frantic::channels::channel_accessor<color_type> m_colorAccessor, m_emissionAccessor, m_extinctionAccessor;

  private:
    struct voxel_sample {
        float density;
        color_type color, emission, extinction;
    };

    /**
     * This function does recursive Simpson's rule integration of of a linearly interpolated density function and light
     * function. It recursively divides the interval until it an absolute error tolerance is exceeded. This recurses on
     * two intervals to only require extensive recursion on parts of the interval that require it, since a exp()
     * function tends to bias towards a small region at one of the interval.
     *
     * This solves the equation: integral(0,1)[ (lightA + x (lightB - lightA)) * e^-integral(0,x){extinctionA + y
     * (extinctionB - extinctionA) dy} dx ] by using integral(0,y){extinctionA + y (extinctionB - extinctionA) dy} = y *
     * extinctionA + 0.5 y^2 (extinctionB - extinctionA)
     *
     * @param lightA the amount of light reflected at the start of the interval
     * @param lightB the amount of light reflected at the end of the interval
     * @param extinctionA the coefficient of extinction at the start of the interval
     * @param extinctionB the coefficient of exrinction at the end of the interval
     * @return the total amount of reflected light, and the alpha transparency associated with this interval. These
     * values can be considered pre-multiplied color values for the purposes of alpha-blending.
     */
    inline static std::pair<float, float> integrate_scattered_light( float lightA, float lightB, float extinctionA,
                                                                     float extinctionB ) {
        struct impl {
            float lightA, lightB;
            float extinctionA, extinctionB;

            typedef float value_type;
            typedef boost::call_traits<value_type>::param_type param_type;

            std::pair<value_type, float> apply() {
                float endT = std::exp( -0.5f * ( extinctionA + extinctionB ) );

                value_type middleLight = std::exp( -( 0.5f * extinctionA + 0.125f * ( extinctionB - extinctionA ) ) ) *
                                         0.5f * ( lightA + lightB );
                value_type endLight = endT * lightB;

                value_type sum = lightA + endLight + 4 * middleLight;

                return std::pair<value_type, float>(
                    apply_recursive( 0, 0, 1.f, lightA, middleLight, endLight, sum ) / 6.f, 1.f - endT );
            }

            value_type apply_recursive( int depth, float a, float b, param_type left, param_type middle,
                                        param_type right, param_type sum ) {
                float tLeft = a + 0.25f * ( b - a );
                float tLeft2_2 = 0.5f * tLeft * tLeft;

                value_type leftLight = std::exp( -( tLeft * extinctionA + tLeft2_2 * ( extinctionB - extinctionA ) ) ) *
                                       ( lightA + tLeft * ( lightB - lightA ) );
                value_type leftSum = left + middle + 4 * leftLight;

                float tRight = a + 0.75f * ( b - a );
                float tRight2_2 = 0.5f * tRight * tRight;

                value_type rightLight =
                    std::exp( -( tRight * extinctionA + tRight2_2 * ( extinctionB - extinctionA ) ) ) *
                    ( lightA + tRight * ( lightB - lightA ) );
                value_type rightSum = middle + right + 4 * rightLight;

                value_type totalSum = 0.5f * ( leftSum + rightSum );

                if( depth > 10 || fabsf( sum - totalSum ) < 1e-3f )
                    return totalSum;

                float result;

                float c = 0.5f * ( a + b );
                result = apply_recursive( depth + 1, a, c, left, leftLight, middle, leftSum );
                result += apply_recursive( depth + 1, c, b, middle, rightLight, right, rightSum );

                return 0.5f * result;
            }
        } theImpl;

        theImpl.lightA = lightA;
        theImpl.lightB = lightB;
        theImpl.extinctionA = extinctionA;
        theImpl.extinctionB = extinctionB;

        return theImpl.apply();
    }

    inline float sample_density( const frantic::graphics::vector3f& worldPos ) const {
        using frantic::graphics::vector3;
        using frantic::graphics::vector3f;

        vector3f voxelPos = m_vcs.get_voxel_coord( worldPos );
        vector3f floorPos = voxelPos.to_floor();
        vector3f alpha = ( voxelPos - floorPos );
        vector3 voxelCoord( (int)floorPos.x, (int)floorPos.y, (int)floorPos.z );

        const char* v;
        float nextDensity = 0.f;

        v = m_voxels.get_voxel_ptr( voxelCoord );
        if( v )
            nextDensity += ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ) * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 1, 0, 0 ) );
        if( v )
            nextDensity += alpha.x * ( 1.f - alpha.y ) * ( 1.f - alpha.z ) * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 0, 1, 0 ) );
        if( v )
            nextDensity += ( 1.f - alpha.x ) * alpha.y * ( 1.f - alpha.z ) * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 1, 1, 0 ) );
        if( v )
            nextDensity += alpha.x * alpha.y * ( 1.f - alpha.z ) * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 0, 0, 1 ) );
        if( v )
            nextDensity += ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * alpha.z * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 1, 0, 1 ) );
        if( v )
            nextDensity += alpha.x * ( 1.f - alpha.y ) * alpha.z * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 0, 1, 1 ) );
        if( v )
            nextDensity += ( 1.f - alpha.x ) * alpha.y * alpha.z * m_voxelDensityAccessor.get( v );
        v = m_voxels.get_voxel_ptr( voxelCoord + vector3( 1, 1, 1 ) );
        if( v )
            nextDensity += alpha.x * alpha.y * alpha.z * m_voxelDensityAccessor.get( v );

        return nextDensity;
    }

    inline bool do_sample( const frantic::graphics::vector3f& worldPos, voxel_sample& outSample ) const {
        using frantic::graphics::vector3;
        using frantic::graphics::vector3f;

        vector3f voxelPos = m_vcs.get_voxel_coord( worldPos );
        vector3f floorPos = voxelPos.to_floor();
        vector3f alpha = ( voxelPos - floorPos );
        vector3 voxelCoord( (int)floorPos.x, (int)floorPos.y, (int)floorPos.z );

        const char* voxelPtrs[8];

        m_voxels.get_voxel_ptrs( voxelCoord, 2, voxelPtrs );

        float weights[] = { ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
                            ( alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
                            ( 1.f - alpha.x ) * ( alpha.y ) * ( 1.f - alpha.z ),
                            ( alpha.x ) * ( alpha.y ) * ( 1.f - alpha.z ),
                            ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( alpha.z ),
                            ( alpha.x ) * ( 1.f - alpha.y ) * ( alpha.z ),
                            ( 1.f - alpha.x ) * ( alpha.y ) * ( alpha.z ),
                            ( alpha.x ) * ( alpha.y ) * ( alpha.z ) };

        outSample.density = 0.f;
        if( m_colorAccessor.is_valid() )
            outSample.color = color_type::black();
        if( m_emissionAccessor.is_valid() )
            outSample.emission = color_type::black();
        if( m_extinctionAccessor.is_valid() )
            outSample.extinction = color_type::black();

        bool result = false;

        for( int i = 0; i < 8; ++i ) {
            if( !voxelPtrs[i] )
                continue;

            result = true;

            outSample.density += weights[i] * m_voxelDensityAccessor.get( voxelPtrs[i] );
            if( m_colorAccessor.is_valid() )
                outSample.color += weights[i] * m_colorAccessor.get( voxelPtrs[i] );
            if( m_emissionAccessor.is_valid() )
                outSample.emission += weights[i] * m_emissionAccessor.get( voxelPtrs[i] );
            if( m_extinctionAccessor.is_valid() )
                outSample.extinction += weights[i] * m_extinctionAccessor.get( voxelPtrs[i] );
        }

        return result;
    }

    inline float integrate_density( const frantic::graphics::ray3f& r, double tStart, double tEnd ) const {
        using frantic::graphics::vector3;
        using frantic::graphics::vector3f;

        const double tStep = (double)m_maxStepSize / (double)r.direction().get_magnitude();
        float dist = m_maxStepSize;

        float prevDensity = sample_density( r.origin() );
        float nextDensity;
        float accum = 0.f;

        double tCur = std::min( tStart + tStep, tEnd );
        while( tCur <= tEnd ) {
            nextDensity = sample_density( r.at( (float)tCur ) );

            accum += 0.5f * dist * ( prevDensity + nextDensity );

            prevDensity = nextDensity;

            tCur += tStep;
        }

        return accum;
    }

    inline color_type integrate_extinction( const frantic::graphics::ray3f& r, double tStart, double tEnd ) const {
        using frantic::graphics::vector3;
        using frantic::graphics::vector3f;

        const double tStep = (double)m_maxStepSize / (double)r.direction().get_magnitude();
        float dist = m_maxStepSize;

        voxel_sample prevSample, nextSample;
        color_type accum;

        do_sample( r.origin(), prevSample );

        double tCur = std::min( tStart + tStep, tEnd );
        while( tCur <= tEnd ) {
            do_sample( r.at( (float)tCur ), nextSample );

            accum += 0.5f * dist * ( prevSample.extinction + nextSample.extinction );

            prevSample = nextSample;

            tCur += tStep;
        }

        return accum;
    }

    inline bool calculate_lighting_at( const vector3f& worldPos, voxel_sample& outSample, color_type& outLight ) const {
        if( !do_sample( worldPos, outSample ) )
            return false;

        outSample.density *= m_cameraDensityScale;

        if( m_emissionAccessor.is_valid() )
            outSample.emission *= m_cameraEmissionScale;
        if( m_colorAccessor.is_valid() )
            outSample.color *= m_cameraDensityScale;
        if( m_extinctionAccessor.is_valid() )
            outSample.extinction *= m_cameraDensityScale;

        outLight = color_type::black();

        if( outSample.density > 0 ) {
            for( int i = 0, iEnd = (int)m_sceneContext->get_light_objects().size(); i < iEnd; ++i ) {
                light_object_ptr lightObj = m_sceneContext->get_light_objects().get( i );

                frantic::graphics::ray3f rLight( worldPos, lightObj->get_light_impl().position() - worldPos );

                double tStart = 0.0, tEnd = 1.0;
                if( !rLight.clamp_to_box( m_voxelBounds, tStart, tEnd ) )
                    continue;

                color_type thisLight = lightObj->eval_lighting( worldPos );

                if( m_extinctionAccessor.is_valid() ) {
                    color_type extinction = integrate_extinction( rLight, tStart, tEnd );
                    thisLight.r *= std::exp( -m_lightDensityScale * extinction.r );
                    thisLight.g *= std::exp( -m_lightDensityScale * extinction.g );
                    thisLight.b *= std::exp( -m_lightDensityScale * extinction.b );
                } else {
                    thisLight *= std::exp( -m_lightDensityScale * integrate_density( rLight, tStart, tEnd ) );
                }

                outLight += thisLight;
            }

            outLight += m_ambientLight;

            if( m_colorAccessor.is_valid() )
                outLight *= outSample.color;
            else
                outLight *= outSample.density;
        }

        if( m_emissionAccessor.is_valid() )
            outLight += outSample.emission;

        return true;
    }

    inline pixel_type raymarch_impl( const frantic::graphics::ray3f& r, double tLeft, double tRight,
                                     pixel_type& fullVal, voxel_sample& prevSample, voxel_sample& nextSample,
                                     color_type& prevLight, color_type& nextLight );

  public:
    raytrace_impl() {}

    virtual ~raytrace_impl() {}

    virtual void add_render_element( krakatoa::render_element_interface_ptr renderElement ) {}

    virtual void precompute_lighting() {}

    void initialize();

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha );

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha, float& nearestDepth, float alphaThreshold = 0.9999f );

    virtual void raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1, alpha_type& accumAlpha );

    virtual void render( image_type& outImage );
};

void raytrace_impl::initialize() {
    using frantic::graphics::color3f;
    using frantic::graphics::vector3;
    using frantic::graphics::vector3f;

    frantic::channels::channel_accessor<vector3f> posAccessor =
        m_particles->get_channel_map().get_accessor<vector3f>( _T("Position") );
    frantic::channels::channel_const_cvt_accessor<float> densityAccessor =
        m_particles->get_channel_map().get_const_cvt_accessor<float>( _T("Density") );
    frantic::channels::channel_const_cvt_accessor<color_type> colorAccessor;
    frantic::channels::channel_const_cvt_accessor<color_type> emissionAccessor;
    frantic::channels::channel_const_cvt_accessor<color_type> absorptionAccessor;

    frantic::channels::channel_map voxelChannels;
    voxelChannels.define_channel<float>( _T("Density") );
    if( m_particles->get_channel_map().has_channel( _T("Color") ) ) {
        voxelChannels.define_channel<color_type>( _T("Color") );
        colorAccessor = m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Color") );
    }
    if( m_useEmissionChannel && m_particles->get_channel_map().has_channel( _T("Emission") ) ) {
        voxelChannels.define_channel<color_type>( _T("Emission") );
        emissionAccessor = m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Emission") );
    }
    if( m_useAbsorptionChannel && m_particles->get_channel_map().has_channel( _T("Absorption") ) ) {
        voxelChannels.define_channel<color_type>( _T("Extinction") );
        absorptionAccessor = m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Absorption") );
    }
    voxelChannels.end_channel_definition();

    m_voxels.reset( voxelChannels );

    m_colorAccessor.reset();
    m_emissionAccessor.reset();
    m_extinctionAccessor.reset();

    m_voxelDensityAccessor = m_voxels.get_channel_map().get_accessor<float>( _T("Density") );

    if( voxelChannels.has_channel( _T("Color") ) )
        m_colorAccessor = voxelChannels.get_accessor<color_type>( _T("Color") );

    if( m_useEmissionChannel && voxelChannels.has_channel( _T("Emission") ) )
        m_emissionAccessor = voxelChannels.get_accessor<color_type>( _T("Emission") );

    if( voxelChannels.has_channel( _T("Extinction") ) )
        m_extinctionAccessor = voxelChannels.get_accessor<color_type>( _T("Extinction") );

    float density;
    color_type color, emission, absorption;

    float voxelLengthFactor = ( m_vcs.voxel_length() * m_vcs.voxel_length() * m_vcs.voxel_length() );

    for( particle_container_type::const_iterator it = m_particles->begin(), itEnd = m_particles->end(); it != itEnd;
         ++it ) {
        vector3f particlePos = posAccessor.get( *it );

        m_voxelBounds += particlePos;

        vector3f voxelPos = m_vcs.get_voxel_coord( particlePos );
        vector3f floorPos = voxelPos.to_floor();
        vector3f alpha = ( voxelPos - floorPos );
        vector3 voxelCoord( (int)floorPos.x, (int)floorPos.y, (int)floorPos.z );

        char* voxelPtrs[8];

        m_voxels.get_or_make_voxel_ptrs( voxelCoord, 2, voxelPtrs );

        float weights[] = { ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
                            ( alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
                            ( 1.f - alpha.x ) * ( alpha.y ) * ( 1.f - alpha.z ),
                            ( alpha.x ) * ( alpha.y ) * ( 1.f - alpha.z ),
                            ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( alpha.z ),
                            ( alpha.x ) * ( 1.f - alpha.y ) * ( alpha.z ),
                            ( 1.f - alpha.x ) * ( alpha.y ) * ( alpha.z ),
                            ( alpha.x ) * ( alpha.y ) * ( alpha.z ) };

        density = densityAccessor.get( *it );
        density /= voxelLengthFactor;

        if( colorAccessor.is_valid() )
            color = colorAccessor.get( *it );
        if( emissionAccessor.is_valid() )
            emission = emissionAccessor.get( *it ) /
                       voxelLengthFactor; // Not multiplied by density, so need to apply the voxelLengthFactor here.
        if( absorptionAccessor.is_valid() )
            absorption = absorptionAccessor.get( *it );

        for( int i = 0; i < 8; ++i ) {
            m_voxelDensityAccessor.get( voxelPtrs[i] ) += density * weights[i];
            if( m_colorAccessor.is_valid() )
                m_colorAccessor.get( voxelPtrs[i] ) += density * weights[i] * color;
            if( m_emissionAccessor.is_valid() )
                m_emissionAccessor.get( voxelPtrs[i] ) += weights[i] * emission;
            if( m_extinctionAccessor.is_valid() )
                m_extinctionAccessor.get( voxelPtrs[i] ) += density * weights[i] * ( color + absorption );
        }
    }

    if( !m_voxelBounds.is_empty() ) {
        m_voxelBounds += m_voxelBounds.minimum() - vector3f( m_vcs.voxel_length() );
        m_voxelBounds += m_voxelBounds.maximum() + vector3f( m_vcs.voxel_length() );
    }

    for( std::size_t i = 0, iEnd = m_sceneContext->get_light_objects().size(); i < iEnd; ++i ) {
        light_object_ptr light = m_sceneContext->get_light_objects().get( (int)i );

        light->begin( m_sceneContext );

        light->update( 0.5f );
    }
}

raytrace_impl::pixel_type raytrace_impl::raymarch_impl( const frantic::graphics::ray3f& r, double tLeft, double tRight,
                                                        pixel_type& fullVal, voxel_sample& prevSample,
                                                        voxel_sample& nextSample, color_type& prevLight,
                                                        color_type& nextLight ) {
    float dist = (float)( r.direction().get_magnitude() * ( tRight - tLeft ) );
    if( dist <= m_minStepSize )
        return fullVal;

    voxel_sample midSample;
    color_type midLight;

    double tCur = 0.5f * ( tLeft + tRight );

    vector3f worldPos = r.at( (float)tCur );

    calculate_lighting_at( worldPos, midSample, midLight );

    pixel_type left, right;

    if( m_extinctionAccessor.is_valid() ) {
        boost::tie( left.c.r, left.a.ar ) = integrate_scattered_light(
            prevLight.r, midLight.r, 0.5f * dist * prevSample.extinction.r, 0.5f * dist * midSample.extinction.r );
        boost::tie( left.c.g, left.a.ag ) = integrate_scattered_light(
            prevLight.g, midLight.g, 0.5f * dist * prevSample.extinction.g, 0.5f * dist * midSample.extinction.g );
        boost::tie( left.c.b, left.a.ab ) = integrate_scattered_light(
            prevLight.b, midLight.b, 0.5f * dist * prevSample.extinction.b, 0.5f * dist * midSample.extinction.b );

        boost::tie( right.c.r, right.a.ar ) = integrate_scattered_light(
            midLight.r, nextLight.r, 0.5f * dist * midSample.extinction.r, 0.5f * dist * nextSample.extinction.r );
        boost::tie( right.c.g, right.a.ag ) = integrate_scattered_light(
            midLight.g, nextLight.g, 0.5f * dist * midSample.extinction.g, 0.5f * dist * nextSample.extinction.g );
        boost::tie( right.c.b, right.a.ab ) = integrate_scattered_light(
            midLight.b, nextLight.b, 0.5f * dist * midSample.extinction.b, 0.5f * dist * nextSample.extinction.b );
    } else {
        boost::tie( left.c.r, left.a.ar ) = integrate_scattered_light(
            prevLight.r, midLight.r, 0.5f * dist * prevSample.density, 0.5f * dist * midSample.density );
        boost::tie( left.c.g, left.a.ag ) = integrate_scattered_light(
            prevLight.g, midLight.g, 0.5f * dist * prevSample.density, 0.5f * dist * midSample.density );
        boost::tie( left.c.b, left.a.ab ) = integrate_scattered_light(
            prevLight.b, midLight.b, 0.5f * dist * prevSample.density, 0.5f * dist * midSample.density );

        boost::tie( right.c.r, right.a.ar ) = integrate_scattered_light(
            midLight.r, nextLight.r, 0.5f * dist * midSample.density, 0.5f * dist * nextSample.density );
        boost::tie( right.c.g, right.a.ag ) = integrate_scattered_light(
            midLight.g, nextLight.g, 0.5f * dist * midSample.density, 0.5f * dist * nextSample.density );
        boost::tie( right.c.b, right.a.ab ) = integrate_scattered_light(
            midLight.b, nextLight.b, 0.5f * dist * midSample.density, 0.5f * dist * nextSample.density );
    }

    left.c *= 0.5f * dist;
    right.c *= 0.5f * dist;

    pixel_type accum = left;
    accum.blend_over( right );

    if( ( fullVal.c - accum.c ).max_abs_component() < 1e-3f )
        return accum;

    left = raymarch_impl( r, tLeft, tCur, left, prevSample, midSample, prevLight, midLight );
    right = raymarch_impl( r, tCur, tRight, right, midSample, nextSample, midLight, nextLight );

    left.blend_over( right );

    return left;
}

void raytrace_impl::raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                              alpha_type& accumAlpha ) {
    float depth = 0.f;
    raymarch( r, t0, t1, accum, accumAlpha, depth );
}

void raytrace_impl::raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                              alpha_type& accumAlpha, float& nearestDepth, float alphaThreshold ) {
    nearestDepth = static_cast<float>( t1 );
    double tStart = t0, tEnd = t1;
    if( !r.clamp_to_box( m_voxelBounds, tStart, tEnd ) )
        return;

    const double dirMag = r.direction().get_magnitude();
    const double tStep = 2.0 * (double)m_maxStepSize /
                         dirMag; // Double, because we always subdivide at least once to see if we are converging.

    voxel_sample nextSample, prevSample;
    color_type prevLight, nextLight;

    bool prevValid = calculate_lighting_at( r.at( (float)tStart ), prevSample, prevLight );

    double tPrev = tStart;
    double tCur = tStart + tStep;
    if( tCur > tEnd )
        tCur = tEnd;

    while( tCur <= tEnd ) {
        float dist = (float)( dirMag * ( tCur - tPrev ) );
        vector3f worldPos = r.at( (float)tCur );

        bool nextValid = calculate_lighting_at( worldPos, nextSample, nextLight );

        if( nextValid || prevValid ) {
            pixel_type result;

            if( m_renderMode == mode_type::additive ) {
                color_type addent = nextSample.density * nextSample.color;
                if( addent.component_sum() > 0.f ) {
                    accum += addent;
                    nearestDepth = std::min( vector3f::distance( worldPos, r.origin() ), nearestDepth );
                }
            } else {
                if( m_extinctionAccessor.is_valid() ) {
                    boost::tie( result.c.r, result.a.ar ) = integrate_scattered_light(
                        prevLight.r, nextLight.r, dist * prevSample.extinction.r, dist * nextSample.extinction.r );
                    boost::tie( result.c.g, result.a.ag ) = integrate_scattered_light(
                        prevLight.g, nextLight.g, dist * prevSample.extinction.g, dist * nextSample.extinction.g );
                    boost::tie( result.c.b, result.a.ab ) = integrate_scattered_light(
                        prevLight.b, nextLight.b, dist * prevSample.extinction.b, dist * nextSample.extinction.b );
                } else {
                    boost::tie( result.c.r, result.a.ar ) = integrate_scattered_light(
                        prevLight.r, nextLight.r, dist * prevSample.density, dist * nextSample.density );
                    boost::tie( result.c.g, result.a.ag ) = integrate_scattered_light(
                        prevLight.g, nextLight.g, dist * prevSample.density, dist * nextSample.density );
                    boost::tie( result.c.b, result.a.ab ) = integrate_scattered_light(
                        prevLight.b, nextLight.b, dist * prevSample.density, dist * nextSample.density );
                }

                result.c *= dist;
                result = raymarch_impl( r, tPrev, tCur, result, prevSample, nextSample, prevLight, nextLight );

                if( result.a.component_sum() > 0.f ) {
                    nearestDepth = std::min( vector3f::distance( worldPos, r.origin() ), nearestDepth );
                }

                accum += accumAlpha.occlude( result.c );
                accumAlpha.blend_over( result.a );

                if( accumAlpha.ar > alphaThreshold && accumAlpha.ag > alphaThreshold && accumAlpha.ab > alphaThreshold )
                    break;
            }

            prevSample = nextSample;
            prevLight = nextLight;
            prevValid = nextValid;
        }

        // I thought about having the stepsize grow as alpha grows.
        // tStep = std::max((double)m_minStepSize, (double)(accumAlpha.ar * m_maxStepSize) / dirMag);

        tPrev = tCur;
        tCur += tStep;
        if( tCur > tEnd ) {
            if( tEnd - tPrev > 1e-5 )
                tCur = tEnd;
        }
    }
}

void raytrace_impl::raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1,
                                      alpha_type& accumAlpha ) {
    double tStart = t0, tEnd = t1;
    if( !r.clamp_to_box( m_voxelBounds, tStart, tEnd ) )
        return;

    if( m_extinctionAccessor.is_valid() ) {
        color_type extinction = integrate_extinction( r, tStart, tEnd );
        accumAlpha.ar = 1.f - std::exp( -m_lightDensityScale * extinction.r );
        accumAlpha.ag = 1.f - std::exp( -m_lightDensityScale * extinction.g );
        accumAlpha.ab = 1.f - std::exp( -m_lightDensityScale * extinction.b );
    } else {
        accumAlpha.ar = accumAlpha.ag = accumAlpha.ab =
            1.f - std::exp( -m_lightDensityScale * integrate_density( r, tStart, tEnd ) );
    }
}

void raytrace_impl::render( image_type& outImage ) {
    using frantic::graphics::color3f;
    using frantic::graphics::vector3;
    using frantic::graphics::vector3f;

    initialize();

    for( int y = 0, yEnd = outImage.height(); y < yEnd; ++y ) {
        for( int x = 0, xEnd = outImage.width(); x < xEnd; ++x ) {
            pixel_type result;
            bool isValid = true;
            frantic::graphics::ray3f pixelRay = m_sceneContext->get_camera().get_worldspace_ray(
                frantic::graphics2d::vector2f( (float)x + 0.5f, (float)y + 0.5f ), 0.5f, isValid );

            if( !isValid )
                continue;

            raymarch( pixelRay, 0, std::numeric_limits<float>::max(), result.c, result.a );

            outImage.set_pixel( x, y, result );
        }

        m_progress->update_progress( y, yEnd );
        if( yEnd > 10 && ( ( y + 1 ) % ( yEnd / 10 ) == 0 ) )
            m_progress->update_frame_buffer( outImage );
    }

    m_progress->update_frame_buffer( outImage );
}

raytrace_renderer::ptr_type raytrace_renderer::create_instance( bool useOpenVDB ) {
#if defined( OPENVDB_AVAILABLE )
    return useOpenVDB ? raytrace_renderer::ptr_type( new openvdb_renderer )
                      : raytrace_renderer::ptr_type( new raytrace_impl );
#else
    if( useOpenVDB ) {
        throw std::runtime_error( "Tried to initialize OpenVDB, but Krakatoa was not built with OpenVDB" );
    }
    return raytrace_renderer::ptr_type( new raytrace_impl );
#endif
}

} // namespace raytrace_renderer
} // namespace krakatoa
