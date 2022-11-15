// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#if defined( OPENVDB_AVAILABLE )

#include <krakatoa/raytrace_renderer/openvdb_renderer.hpp>

#include <boost/function.hpp>
#include <boost/math/special_functions/pow.hpp>
#include <boost/thread/detail/tss_hooks.hpp>
#include <boost/thread/tss.hpp>
#include <boost/type_traits/conditional.hpp>
#include <openvdb/tools/Interpolation.h>

using namespace frantic::channels;
using namespace frantic::graphics;
using namespace krakatoa::raytrace_renderer::detail;
using krakatoa::raytrace_renderer::openvdb_renderer;

/////////////
// Helpers //
/////////////

namespace {

/**
 * Get the affected neighbour voxels for trilinear interpolation.
 *
 * @param vc The floor of the index space position to do interpolation on.
 * @param neighbours An array which will be filled with the affected neighbour voxels.
 */
inline void get_openvdb_coord_neighbours( const openvdb::Coord& vc, openvdb::Coord neighbours[8] ) {
    neighbours[0].reset( vc.x() + 0, vc.y() + 0, vc.z() + 0 );
    neighbours[1].reset( vc.x() + 1, vc.y() + 0, vc.z() + 0 );
    neighbours[2].reset( vc.x() + 0, vc.y() + 1, vc.z() + 0 );
    neighbours[3].reset( vc.x() + 1, vc.y() + 1, vc.z() + 0 );
    neighbours[4].reset( vc.x() + 0, vc.y() + 0, vc.z() + 1 );
    neighbours[5].reset( vc.x() + 1, vc.y() + 0, vc.z() + 1 );
    neighbours[6].reset( vc.x() + 0, vc.y() + 1, vc.z() + 1 );
    neighbours[7].reset( vc.x() + 1, vc.y() + 1, vc.z() + 1 );
}

typedef boost::tuple<float, color3f> float_and_1_color_t;
typedef boost::tuple<float, color3f, color3f> float_and_2_color_t;
typedef boost::tuple<float, color3f, color3f, color3f> float_and_3_color_t;

typedef openvdb_renderer::grid_of_t<float_and_1_color_t>::type float_and_1_color_grid;
typedef openvdb_renderer::grid_of_t<float_and_2_color_t>::type float_and_2_color_grid;
typedef openvdb_renderer::grid_of_t<float_and_3_color_t>::type float_and_3_color_grid;

/**
 * The type of OpenVDB grid to use based on whether the Emission and Absorption channels should be taken into account.
 */
template <bool UseEmission, bool UseAbsorption>
struct conditional_grid_t {
    typedef typename boost::conditional<
        UseEmission, //
        typename boost::conditional<UseAbsorption, float_and_3_color_grid, float_and_2_color_grid>::type,
        typename boost::conditional<UseAbsorption, float_and_2_color_grid, float_and_1_color_grid>::type>::type type;
};

/**
 * Create a tuple appropriate for insertion into conditional_grid_t<UseEmission, UseAbsorption>. Unneeded arguments are
 * discarded.
 *
 * NOTE: When we have C++17, replace this with a single constexpr function that uses constexpr if.
 */
template <bool UseEmission, bool UseAbsorption>
typename conditional_grid_t<UseEmission, UseAbsorption>::type::ValueType
make_value_type( float density, const color3f& color, const color3f& emission, const color3f& absorption );

template <>
typename conditional_grid_t<false, false>::type::ValueType
make_value_type<false, false>( float density, const color3f& color, const color3f&, const color3f& ) {
    return conditional_grid_t<false, false>::type::ValueType( density, color );
}

template <>
typename conditional_grid_t<true, false>::type::ValueType
make_value_type<true, false>( float density, const color3f& color, const color3f& emission, const color3f& ) {
    return conditional_grid_t<true, false>::type::ValueType( density, color, emission );
}

template <>
typename conditional_grid_t<false, true>::type::ValueType
make_value_type<false, true>( float density, const color3f& color, const color3f&, const color3f& absorption ) {
    return conditional_grid_t<false, true>::type::ValueType( density, color, absorption );
}

template <>
typename conditional_grid_t<true, true>::type::ValueType
make_value_type<true, true>( float density, const color3f& color, const color3f& emission, const color3f& absorption ) {
    return conditional_grid_t<true, true>::type::ValueType( density, color, emission, absorption );
}

/**
 * Create a (density, lighting, emission, attenuation) tuple from a tuple that may or may not contain emission and / or
 * attenuation data.
 */
template <bool HasEmission, bool HasAbsorption>
float_and_3_color_grid::ValueType
make_float_and_3_color_type( const typename conditional_grid_t<HasEmission, HasAbsorption>::type::ValueType& tup );

template <>
float_and_3_color_grid::ValueType
make_float_and_3_color_type<false, false>( const typename conditional_grid_t<false, false>::type::ValueType& tup ) {
    return float_and_3_color_grid::ValueType( boost::get<0>( tup ), boost::get<1>( tup ), color3f(), color3f() );
}

template <>
float_and_3_color_grid::ValueType
make_float_and_3_color_type<true, false>( const typename conditional_grid_t<true, false>::type::ValueType& tup ) {
    return float_and_3_color_grid::ValueType( boost::get<0>( tup ), boost::get<1>( tup ), boost::get<2>( tup ),
                                              color3f() );
}

template <>
float_and_3_color_grid::ValueType
make_float_and_3_color_type<false, true>( const typename conditional_grid_t<false, true>::type::ValueType& tup ) {
    return float_and_3_color_grid::ValueType( boost::get<0>( tup ), boost::get<1>( tup ), color3f(),
                                              boost::get<2>( tup ) );
}

template <>
float_and_3_color_grid::ValueType
make_float_and_3_color_type<true, true>( const typename conditional_grid_t<true, true>::type::ValueType& tup ) {
    return tup;
}

template <bool UseEmission, bool UseAbsorption>
struct openvdb_functors {
    typedef typename conditional_grid_t<UseEmission, UseAbsorption>::type grid_t;

    /**
     * Functor used by OpenVDB to transform grids of one type to another.
     */
    static void to_float_and_3_color_transform( const typename grid_t::ValueOnCIter& iter,
                                                typename float_and_3_color_grid::Accessor& accessor ) {
        float_and_3_color_grid::ValueType output = make_float_and_3_color_type<UseEmission, UseAbsorption>( *iter );
        if( iter.isVoxelValue() ) {
            // Set a single voxel
            accessor.setValue( iter.getCoord(), output );
        } else {
            // Fill an entire tile
            openvdb::CoordBBox bbox;
            iter.getBoundingBox( bbox );
            accessor.getTree()->fill( bbox, output );
        }
    }
};

openvdb::math::Vec3<double> to_openvdb( const frantic::graphics::vector3f& v ) {
    return openvdb::math::Vec3<double>( v.x, v.y, v.z );
}

} // namespace

namespace krakatoa {
namespace raytrace_renderer {

/////////////////////////////////
// Ray Marching Implementation //
/////////////////////////////////

namespace detail {

/**
 * The purpose of this class is to bridge OpenVDB's compile-time template mechanisms and our dynamic runtime
 * polymorphism.
 *
 * The underlying `openvdb::Grid` type to use as our central ray tracing data structure is not known until runtime
 * because of the `set_use_emission` and `set_use_absorption` methods of the `krakatoa::renderer` class, which change
 * the number of color3f channels we need to allocate inside our Grid.  We instead create a `openvdb_renderer_impl`
 * templated class, which inherits `openvdb_renderer_impl_interface` and allows operations to be performed through a
 * common interface agnostic to the underlying Grid type.  Methods which do have behaviour specific to the type can be
 * done through explicit template specialization (though methods using `constexpr if` should be preferred once we have
 * C++17 support).
 *
 * This approach requires an additional virtual method call, but I assume this is dwarfed by the time taken to do the
 * ray marching.
 */
class openvdb_renderer_impl_interface {
  protected:
    typedef renderer::alpha_type alpha_type;
    typedef renderer::color_type color_type;
    typedef renderer::particle_container_type particle_container_type;

  public:
    openvdb_renderer_impl_interface() {}

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, color_type& accum,
                           alpha_type& accumAlpha, float& nearestDepth, float alphaThreshold = 1e-4f ) = 0;

    virtual void raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1,
                                   alpha_type& accumAlpha ) = 0;

    virtual float_and_3_color_grid::ConstPtr grid() const = 0;

    virtual ~openvdb_renderer_impl_interface() {}

  private:
    // Disable copying and assignment
    openvdb_renderer_impl_interface( const openvdb_renderer_impl_interface& );
    openvdb_renderer_impl_interface& operator=( const openvdb_renderer_impl_interface& );
};

/**
 * @warning Make sure OpenVDB is initialized (with `openvdb::initialize()`) before constructing an
 *          `openvdb_renderer_impl`!
 */
template <bool UseEmission, bool UseAbsorption>
class openvdb_renderer_impl : public openvdb_renderer_impl_interface {
  public:
    openvdb_renderer_impl( const openvdb_renderer& r )
        : m_grid( grid_t::create() )
        , m_gridAcc( m_grid->getAccessor() )
        , self( r ) {

        m_grid->setTransform( openvdb::math::Transform::createLinearTransform( self.m_vcs.voxel_length() ) );

        channel_accessor<vector3f> posAccessor =
            self.m_particles->get_channel_map().get_accessor<vector3f>( _T("Position") );
        channel_const_cvt_accessor<float> densityAccessor =
            self.m_particles->get_channel_map().get_const_cvt_accessor<float>( _T("Density") );

        channel_const_cvt_accessor<color_type> colorAccessor;
        channel_const_cvt_accessor<color_type> lightingAccessor;
        channel_const_cvt_accessor<color_type> emissionAccessor;
        channel_const_cvt_accessor<color_type> absorptionAccessor;
        if( self.m_particles->get_channel_map().has_channel( _T("Color") ) )
            colorAccessor = self.m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Color") );
        if( self.m_renderMode == krakatoa::renderer::mode_type::additive ) {
            lightingAccessor = colorAccessor;
        } else if( self.m_particles->get_channel_map().has_channel( _T("Lighting") ) ) {
            lightingAccessor = self.m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Lighting") );
        } else {
            FF_LOG( warning )
                << "Krakatoa Lighting channel not present! openvdb_renderer will be unable to calculate shadows.";
        }
        if( UseEmission && self.m_particles->get_channel_map().has_channel( _T("Emission") ) )
            emissionAccessor = self.m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Emission") );
        if( UseAbsorption && self.m_particles->get_channel_map().has_channel( _T("Absorption") ) )
            absorptionAccessor =
                self.m_particles->get_channel_map().get_const_cvt_accessor<color_type>( _T("Absorption") );

        const float invVoxelVolume = 1.f / boost::math::pow<3>( self.m_vcs.voxel_length() );
        for( particle_container_type::const_iterator it = self.m_particles->begin(), itEnd = self.m_particles->end();
             it != itEnd; ++it ) {
            const vector3f particlePos = posAccessor.get( *it );

            const vector3f voxelPos = self.m_vcs.get_voxel_coord( particlePos );
            const vector3f floorPos = voxelPos.to_floor();
            const vector3f alpha = ( voxelPos - floorPos );
            const openvdb::Coord voxelCoord( (int)floorPos.x, (int)floorPos.y, (int)floorPos.z );

            const float weights[] = {
                // clang-format off
			    ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
			    ( alpha.x )       * ( 1.f - alpha.y ) * ( 1.f - alpha.z ),
			    ( 1.f - alpha.x ) * ( alpha.y )       * ( 1.f - alpha.z ),
			    ( alpha.x )       * ( alpha.y )       * ( 1.f - alpha.z ),
			    ( 1.f - alpha.x ) * ( 1.f - alpha.y ) * ( alpha.z ),
			    ( alpha.x )       * ( 1.f - alpha.y ) * ( alpha.z ),
			    ( 1.f - alpha.x ) * ( alpha.y )       * ( alpha.z ),
			    ( alpha.x )       * ( alpha.y )       * ( alpha.z )
                // clang-format on
            };

            const float density = densityAccessor.get( *it ) * invVoxelVolume;
            const color_type color = ( colorAccessor.is_valid() ? colorAccessor.get( *it ) : color_type() );
            const color_type lighting = ( lightingAccessor.is_valid() ? lightingAccessor.get( *it ) : color_type() );
            const color_type emission = // Not multiplied by density, so need to apply the voxelVolume here.
                ( emissionAccessor.is_valid() ? ( emissionAccessor.get( *it ) * invVoxelVolume ) : color_type() );
            const color_type absorption =
                ( absorptionAccessor.is_valid() ? absorptionAccessor.get( *it ) : color_type() );

            openvdb::Coord voxelCoords[8];
            get_openvdb_coord_neighbours( voxelCoord, voxelCoords );

            for( int i = 0; i < 8; ++i ) {
                typename grid_t::ValueType tup = make_value_type<UseEmission, UseAbsorption>(
                    density * weights[i],                            // density
                    density * weights[i] * lighting,                 // lighting
                    weights[i] * emission,                           // emission
                    density * weights[i] * ( color + absorption ) ); // attenuation
                m_gridAcc.setValue( voxelCoords[i], m_gridAcc.getValue( voxelCoords[i] ) + tup );
            }
        }
    }

    virtual void raymarch( const frantic::graphics::ray3f& r, double t0, double t1, renderer::color_type& accum,
                           renderer::alpha_type& accumAlpha, float& nearestDepth,
                           float alphaThreshold = 1e-4f ) override {
        nearestDepth = float( t1 );

        if( m_grid->empty() )
            return;

        // Index space to world space transformation: multiply by m_vcs.voxel_length()
        // World space to index space transformation: divide by m_vcs.voxel_length()
        const float invVoxelLen = 1.f / self.m_vcs.voxel_length();
        openvdb::math::Ray<openvdb::Real> indexSpaceRay( to_openvdb( r.origin() * invVoxelLen ),
                                                         to_openvdb( r.direction() * invVoxelLen ), t0 * invVoxelLen,
                                                         t1 * invVoxelLen );

        if( !m_intersector.get() ) {
            // TODO: These resources may not be cleaned up automatically on Windows.
            m_intersector.reset( new openvdb::tools::VolumeRayIntersector<grid_t>( *m_grid ) );
            m_threadGridAcc.reset( new typename grid_t::Accessor( m_grid->getAccessor() ) );
            m_sampler.reset( new sampler_t( *m_threadGridAcc, m_grid->transform() ) );
        }

        if( !m_intersector->setIndexRay( indexSpaceRay ) )
            return; // It misses the bounding box grid

        // last sampled position in world space
        openvdb::Real lastSamplePos = -std::numeric_limits<openvdb::Real>::infinity();
        // last sampled value, for trapezoidal rule integration
        typename grid_t::ValueType lastSample;
        // ∫ Attenuation(x) dx from t0 to the current sample, for each of R, G, B
        double attenuationIntegralR = 0.0, attenuationIntegralG = 0.0, attenuationIntegralB = 0.0;
        // node hit positions in index space
        openvdb::Real nodeHitStart, nodeHitEnd;
        while( m_intersector->march( nodeHitStart, nodeHitEnd ) ) {
            // Note: Each intersector hit represents a hit against a node in the tree.  In the case of a leaf node, we
            // must
            //       sample the dense grid associated with the leaf node.  In the case of a non-leaf node, we could get
            //       away with only sampling it once, since our background value is always 0, but Ken Museth (author of
            //       OpenVDB) opines that this is probably [more trouble than it is
            //       worth](https://groups.google.com/forum/#!topic/openvdb-forum/bd0M4Q8YeTw).

            // Like `raytrace_impl`, I double the number of samples here which helps keep scenes looking consistent.
            const int numSamples =
                2 * std::ceil( ( nodeHitEnd - nodeHitStart ) * self.m_vcs.voxel_length() / self.m_maxStepSize );
            for( int i = 0; i < numSamples; ++i ) {
                const openvdb::Real normalized_i = (openvdb::Real)i / numSamples;
                const openvdb::Real samplePosIS = ( 1.0 - normalized_i ) * nodeHitStart + normalized_i * nodeHitEnd;
                typename grid_t::ValueType sample = m_sampler->isSample( indexSpaceRay( samplePosIS ) );

                const openvdb::Real samplePos = samplePosIS * self.m_vcs.voxel_length();
                if( lastSamplePos == -std::numeric_limits<openvdb::Real>::infinity() ) {
                    lastSamplePos = samplePos;
                    lastSample = sample;
                    continue;
                }
                const openvdb::Real dt = samplePos - lastSamplePos;
                typename grid_t::ValueType value = ( sample + lastSample ) * 0.5f; // simple trapezoidal rule

                const color_type attenuation = get_attenuation_term( value ) * self.m_cameraDensityScale;
                attenuationIntegralR += attenuation.r * dt;
                attenuationIntegralG += attenuation.g * dt;
                attenuationIntegralB += attenuation.b * dt;
                lastSamplePos = samplePos;
                lastSample = sample;

                if( self.m_renderMode != krakatoa::renderer::mode_type::additive ) {
                    const color_type alpha( 1.f - std::exp( -attenuationIntegralR ),
                                            1.f - std::exp( -attenuationIntegralG ),
                                            1.f - std::exp( -attenuationIntegralB ) );
                    if( alpha.component_sum() > 0.f ) {
                        nearestDepth = std::min( float( samplePos * indexSpaceRay.dir().length() ), nearestDepth );

                        const color_type lighting = get_lighting_term( value ) * self.m_cameraDensityScale;
                        const color_type emission = get_emission_term( value ) * self.m_cameraEmissionScale;
                        const color_type color = ( lighting + emission ) * dt;

                        accum += accumAlpha.occlude( color );
                        accumAlpha = alpha_type( alpha );
                        if( accumAlpha.ar > alphaThreshold && accumAlpha.ag > alphaThreshold &&
                            accumAlpha.ab > alphaThreshold ) {
                            return;
                        }
                    }
                } else {
                    // FIXME: This doesn't match the particle rendering mode

                    // We already multiply by density when creating the volume, here we only need to scale by
                    // cameraDensityScale
                    const color_type contrib = self.m_cameraDensityScale * get_lighting_term( sample ) * dt;
                    if( contrib.component_sum() > 0.f ) {
                        nearestDepth = std::min( float( samplePos * indexSpaceRay.dir().length() ), nearestDepth );

                        accum += contrib;
                        accumAlpha.blend_over( alpha_type( contrib ) );
                    }
                }
            }
        }
    }

    virtual void raymarch_opacity( const frantic::graphics::ray3f& r, double t0, double t1,
                                   renderer::alpha_type& accumAlpha ) override {
        // This isn't as efficient as it could be, but I don't think this method is used.
        float nearestDepth;
        color3f accum;
        raymarch( r, t0, t1, accum, accumAlpha, nearestDepth );
    }

    virtual float_and_3_color_grid::ConstPtr grid() const override {
        typename float_and_3_color_grid::Ptr outGrid = float_and_3_color_grid::create();
        std::function<void( const typename grid_t::ValueOnCIter& iter, float_and_3_color_grid::Accessor& accessor )>
            functor( openvdb_functors<UseEmission, UseAbsorption>::to_float_and_3_color_transform );
        openvdb::tools::transformValues( m_grid->cbeginValueOn(), *outGrid, functor );
        outGrid->setTransform( m_grid->transformPtr() );
        return outGrid;
    }

    // The type of the grid to use
    typedef typename conditional_grid_t<UseEmission, UseAbsorption>::type grid_t;

  private:
    static float get_density_term( const typename grid_t::ValueType& val ) { return boost::get<0>( val ); }
    static color3f get_lighting_term( const typename grid_t::ValueType& val ) { return boost::get<1>( val ); }
    static color3f get_emission_term( const typename grid_t::ValueType& );
    static color3f get_attenuation_term( const typename grid_t::ValueType& );

    // Smart pointer to an OpenVDB grid.
    typename grid_t::Ptr m_grid;

    // Accessor which provides cached random access to `m_grid`. Not thread-safe.
    typename grid_t::Accessor m_gridAcc;

    // A topology grid to perform intersection tests against.  Each thread needs its own as it does caching.
    boost::thread_specific_ptr<openvdb::tools::VolumeRayIntersector<grid_t>> m_intersector;

    // Can be used for thread-safe access to a grid's voxels and nodes.
    boost::thread_specific_ptr<typename grid_t::Accessor> m_threadGridAcc;

    // The sampler to use. A BoxSampler provides trilinear interpolation similar to `raytrace_impl`. We could also
    // switch to a StaggedBoxSampler which might combat moiré effects.
    typedef openvdb::tools::GridSampler<openvdb::tree::ValueAccessor<typename grid_t::TreeType>,
                                        openvdb::tools::BoxSampler>
        sampler_t;

    // A sampler which provides interpolated reads of `m_grid`'s values.
    boost::thread_specific_ptr<sampler_t> m_sampler;

    // Refer back to the original class to access its protected members.
    const openvdb_renderer& self;
};

template <>
color3f openvdb_renderer_impl<false, false>::get_emission_term( const typename grid_t::ValueType& val ) {
    return color3f::black();
}
template <>
color3f openvdb_renderer_impl<false, true>::get_emission_term( const typename grid_t::ValueType& val ) {
    return color3f::black();
}
template <>
color3f openvdb_renderer_impl<true, false>::get_emission_term( const typename grid_t::ValueType& val ) {
    return boost::get<2>( val );
}
template <>
color3f openvdb_renderer_impl<true, true>::get_emission_term( const typename grid_t::ValueType& val ) {
    return boost::get<2>( val );
}

template <>
color3f openvdb_renderer_impl<false, false>::get_attenuation_term( const typename grid_t::ValueType& val ) {
    return color3f( boost::get<0>( val ) );
}
template <>
color3f openvdb_renderer_impl<true, false>::get_attenuation_term( const typename grid_t::ValueType& val ) {
    return color3f( boost::get<0>( val ) );
}
template <>
color3f openvdb_renderer_impl<false, true>::get_attenuation_term( const typename grid_t::ValueType& val ) {
    return boost::get<2>( val );
}
template <>
color3f openvdb_renderer_impl<true, true>::get_attenuation_term( const typename grid_t::ValueType& val ) {
    return boost::get<3>( val );
}

} // namespace detail

/////////////////
// Pimpl idiom //
/////////////////

openvdb_renderer::openvdb_renderer()
    : m_initialized( false ) {}

void openvdb_renderer::add_render_element( krakatoa::render_element_interface_ptr renderElement ) {
    m_renderElements.push_back( renderElement );
}

void openvdb_renderer::precompute_lighting() {
    if( !m_particles || !m_sceneContext || !m_lightingEngine || !m_shader )
        return;

    // Set a default splat filter if the user hasn't
    if( !m_splatFilter )
        m_splatFilter = splat_renderer::filter2f::create_instance( _T("Bilinear") );

    // Set a default progress logger if the user hasn't
    if( !m_progress )
        m_progress.reset( new frantic::logging::null_render_progress_logger );

    m_shader->set_channel_map( m_particles->get_channel_map() );
    for( std::vector<renderer::shader_ptr_type>::iterator it = m_shaders.begin(); it != m_shaders.end(); ++it ) {
        ( *it )->set_channel_map( m_particles->get_channel_map() );
    }

    if( m_renderMode != mode_type::additive ) {
        m_lightingEngine->set_density_scale( m_lightDensityScale );
        m_lightingEngine->set_progress_logger( m_progress );
        m_lightingEngine->set_scene_context( m_sceneContext );
        if( m_useMixedShaders ) {
            m_lightingEngine->set_use_mixed_shaders( true, m_particles->get_channel_map() );
            m_lightingEngine->set_shaders( m_shaders );
        }
        m_lightingEngine->set_shader( m_shader );
        m_lightingEngine->set_splat_filter( m_splatFilter );
        m_lightingEngine->add_render_elements( m_renderElements.begin(), m_renderElements.end() );
        m_lightingEngine->compute_particle_lighting( *m_particles, m_useAbsorptionChannel );
    }
}

void openvdb_renderer::initialize() {
    if( !m_initialized ) {
        if( !m_particles ) {
            throw std::runtime_error(
                "openvdb_renderer::initialize: A particle container was not provided at time of initialization." );
        }

        openvdb::initialize(); // May safely be called multiple times.

        if( m_useEmissionChannel && m_useAbsorptionChannel )
            m_pImpl.reset( new openvdb_renderer_impl<true, true>( *this ) );
        else if( m_useEmissionChannel )
            m_pImpl.reset( new openvdb_renderer_impl<true, false>( *this ) );
        else if( m_useAbsorptionChannel )
            m_pImpl.reset( new openvdb_renderer_impl<false, true>( *this ) );
        else
            m_pImpl.reset( new openvdb_renderer_impl<false, false>( *this ) );
        m_initialized = true;
    }
}

void openvdb_renderer::raymarch( const ray3f& r, double t0, double t1, color_type& accum, alpha_type& accumAlpha ) {
    float depth = 0.f;
    m_pImpl->raymarch( r, t0, t1, accum, accumAlpha, depth );
}

void openvdb_renderer::raymarch( const ray3f& r, double t0, double t1, color_type& accum, alpha_type& accumAlpha,
                                 float& nearestDepth, float alphaThreshold ) {
    m_pImpl->raymarch( r, t0, t1, accum, accumAlpha, nearestDepth, alphaThreshold );
}

void openvdb_renderer::raymarch_opacity( const ray3f& r, double t0, double t1, alpha_type& accumAlpha ) {
    m_pImpl->raymarch_opacity( r, t0, t1, accumAlpha );
}

void openvdb_renderer::render( image_type& outImage ) {
    initialize();

    const int height = outImage.height();
    const int width = outImage.width();
    for( int y = 0; y < height; ++y ) {
        for( int x = 0; x < width; ++x ) {
            bool isValid = true;
            ray3f pixelRay = m_sceneContext->get_camera().get_worldspace_ray(
                frantic::graphics2d::vector2f( x + 0.5f, y + 0.5f ), 0.5f, isValid );

            if( !isValid )
                continue;

            pixel_type result;
            raymarch( pixelRay, 0, std::numeric_limits<float>::max(), result.c, result.a );

            outImage.set_pixel( x, y, result );
        }

        m_progress->update_progress( y, height );
        if( height > 10 && ( ( y + 1 ) % ( height / 10 ) == 0 ) )
            m_progress->update_frame_buffer( outImage );
    }

    m_progress->update_frame_buffer( outImage );
}

float_and_3_color_grid::ConstPtr openvdb_renderer::grid() const {
    return ( m_initialized ? m_pImpl->grid() : nullptr );
}

openvdb_renderer::~openvdb_renderer() {}

} // namespace raytrace_renderer
} // namespace krakatoa

#endif