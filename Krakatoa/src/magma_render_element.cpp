// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/magma_render_element.hpp>

#include <krakatoa/render_element_impl.hpp>

#include <frantic/channels/channel_operation_nodes.hpp>

#include <memory>

namespace krakatoa {

magma_render_element::magma_render_element( bool doAntialias )
    : m_doAntialias( doAntialias ) {}

magma_render_element::magma_render_element( bool doAntialias,
                                            const std::vector<frantic::channels::channel_op_node*>& exprTree )
    : m_doAntialias( doAntialias ) {
    m_exprTree.assign( exprTree.begin(), exprTree.end() );
}

magma_render_element::~magma_render_element() {
    std::vector<frantic::channels::channel_op_node*>::iterator it = m_exprTree.begin(), itEnd = m_exprTree.end();
    for( ; it != itEnd; ++it )
        delete *it;
}

magma_render_element::draw_type magma_render_element::get_drawing_type() const {
    if( m_doAntialias ) {
        return draw_type_antialias;
    } else {
        return draw_type_solid;
    }
}

render_element_interface* magma_render_element::clone() {
    std::unique_ptr<magma_render_element> pResult( new magma_render_element( m_doAntialias ) );

    pResult->m_pCompiledExpr = m_pCompiledExpr;
    pResult->m_outputMap = m_outputMap;
    pResult->m_accessor = m_accessor;

    pResult->get_framebuffer().set_size( get_framebuffer().size() );
    pResult->get_framebuffer().fill( frantic::graphics::color6f( 0.f ) );

    return pResult.release();
}

void magma_render_element::set_channel_map( const frantic::channels::channel_map& pcm ) {
    m_pCompiledExpr.reset( new frantic::channels::channel_operation_compiler );
    m_pCompiledExpr->reset( pcm, pcm );

    if( !m_exprTree.empty() ) {
        frantic::channels::output_channel_op_node* pOutputNode =
            dynamic_cast<frantic::channels::output_channel_op_node*>( m_exprTree[0] );
        if( pOutputNode ) {
            pOutputNode->compile( m_exprTree, *m_pCompiledExpr );
            m_accessor = m_pCompiledExpr->get_channel_map().get_const_cvt_accessor<frantic::graphics::color3f>(
                frantic::strings::to_tstring( pOutputNode->get_channel_name() ) );
        }
    }

    m_outputMap = pcm;
}

/**
#("Position", "float32", 3),
#("Velocity", "float16", 3),
#("Density", "float16", 1),
#("Color", "float16", 3),

#("Absorption", "float16", 3),
#("Emission", "float16", 3),
#("Eccentricity", "float16", 1),
#("SpecularPower", "float16", 1),
#("SpecularLevel", "float16", 1),

#("Normal", "float16", 3),
#("Tangent", "float16", 3),
#("TextureCoord", "float16", 3),
--#("Lighting", "float16", 3),

#("Selection", "float16", 1),

#("Age", "float32", 1),
#("LifeSpan", "float32", 1),
#("ID", "int32", 1),

#("MtlIndex","int16", 1),
#("MXSInteger", "int16", 1),
#("MXSFloat", "float16", 1),
#("MXSVector", "float16", 3),
#("Orientation", "float16", 4),
#("Scale", "float16", 3),
#("Acceleration", "float16", 3),
#("SignedDistance", "float16", 1),

#("Fire", "float16", 1),
#("Fuel", "float16", 1),
#("Temperature", "float16", 1),
#("DensityGradient", "float16", 3)
 */

void magma_render_element::add_required_channels( frantic::channels::channel_map& pcm ) {
    typedef std::map<frantic::tstring, std::pair<frantic::channels::data_type_t, std::size_t>> type_map;
    static type_map g_channelDefaults;

    // HACK: This should be some sort of globally accessible thing at the very least. Possibly through
    //        krakatoa_context. A hard-coded list doesn't really make a ton of sense anyways though.
    if( g_channelDefaults.empty() ) {
        g_channelDefaults[_T("Position")] = std::make_pair( frantic::channels::data_type_float32, 3 );
        g_channelDefaults[_T("Velocity")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Color")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Absorption")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Emission")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Normal")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Tangent")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("TextureCoord")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("MXSVector")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Scale")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("Acceleration")] = std::make_pair( frantic::channels::data_type_float16, 3 );
        g_channelDefaults[_T("DensityGradient")] = std::make_pair( frantic::channels::data_type_float16, 3 );

        g_channelDefaults[_T("Density")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("Eccentricity")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("SpecularPower")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("SpecularLevel")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("Selection")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("MXSFloat")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("SignedDistance")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("Fire")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("Fuel")] = std::make_pair( frantic::channels::data_type_float16, 1 );
        g_channelDefaults[_T("Temperature")] = std::make_pair( frantic::channels::data_type_float16, 1 );

        g_channelDefaults[_T("Age")] = std::make_pair( frantic::channels::data_type_float32, 1 );
        g_channelDefaults[_T("LifeSpan")] = std::make_pair( frantic::channels::data_type_float32, 1 );
        g_channelDefaults[_T("ID")] = std::make_pair( frantic::channels::data_type_int32, 1 );

        g_channelDefaults[_T("MtlIndex")] = std::make_pair( frantic::channels::data_type_int16, 1 );
        g_channelDefaults[_T("MXSInteger")] = std::make_pair( frantic::channels::data_type_int16, 1 );
    }

    std::vector<frantic::channels::channel_op_node*>::const_iterator it = m_exprTree.begin(), itEnd = m_exprTree.end();
    for( ; it != itEnd; ++it ) {
        if( const frantic::channels::input_channel_op_node* pInputChannelNode =
                dynamic_cast<const frantic::channels::input_channel_op_node*>( *it ) ) {
            frantic::tstring chName = frantic::strings::to_tstring( pInputChannelNode->get_channel() );

            if( !pcm.has_channel( chName ) ) {
                type_map::const_iterator it = g_channelDefaults.find( chName );
                if( it != g_channelDefaults.end() )
                    pcm.define_channel( it->first, it->second.second, it->second.first );
                else if( chName.substr( 0, 7 ) == _T("Mapping") )
                    pcm.define_channel( chName, 3, frantic::channels::data_type_float16 );
                else
                    throw std::runtime_error(
                        "A CustomData render element requires channel \"" + frantic::strings::to_string( chName ) +
                        "\" which is not a channel that is available while rendering.\nIf this channel is actually "
                        "available via your PRT object, you can use a KCM to move that channel into one of the "
                        "Mapping## channels which are available at render time." );
            }
        }
    }
}

frantic::graphics::color3f magma_render_element::evaluate( const char* particle ) {
    char* tempBuffer = (char*)alloca( m_pCompiledExpr->get_channel_map().structure_size() );

    // We can memcpy here since the destination particle is a subset of the particle layout used in m_compiledExpr.
    memcpy( tempBuffer, particle, m_outputMap.structure_size() );

    m_pCompiledExpr->eval( tempBuffer, (std::size_t)-1 );

    return m_accessor.get( tempBuffer );
}

} // namespace krakatoa
