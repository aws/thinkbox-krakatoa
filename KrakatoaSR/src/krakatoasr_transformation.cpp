// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <krakatoasr_transformation.hpp>

#include <krakatoasr_renderer/params.hpp>

using namespace frantic::graphics;

namespace krakatoasr {

animated_transform::animated_transform() {
    m_data = new animated_transform_params;
    m_data->unanimated = false;
}

animated_transform::animated_transform( const float* elements ) {
    m_data = new animated_transform_params;
    m_data->unanimated = true;
    m_data->tm[0.0f] = transform4f( elements ); // unanimated
}

animated_transform::animated_transform( float e11, float e21, float e31, float e41, float e12, float e22, float e32,
                                        float e42, float e13, float e23, float e33, float e43, float e14, float e24,
                                        float e34, float e44 ) {
    m_data = new animated_transform_params;
    m_data->unanimated = true;
    m_data->tm[0.0f] =
        transform4f( e11, e21, e31, e41, e12, e22, e32, e42, e13, e23, e33, e43, e14, e24, e34, e44 ); // unanimated
}

animated_transform::~animated_transform() { delete m_data; }

animated_transform::animated_transform( const animated_transform& t ) {
    m_data = new animated_transform_params;
    *this = t;
}

animated_transform& animated_transform::operator=( const animated_transform& t ) {
    *m_data = *t.m_data;
    return *this;
}

const animated_transform_params* animated_transform::get_data() const { return m_data; }

animated_transform_params* animated_transform::get_data() { return m_data; }

void animated_transform::add_transform( const float* elements, float shutterTime ) {
    if( m_data->unanimated )
        throw std::runtime_error(
            "This transformation has been defined as unanimated. No new transformations can be added to it. To use "
            "specify an animated transformation, no not provide an initial transformation on creation time." );
    m_data->tm[shutterTime] = transform4f( elements );
}

void animated_transform::add_transform( float e11, float e21, float e31, float e41, float e12, float e22, float e32,
                                        float e42, float e13, float e23, float e33, float e43, float e14, float e24,
                                        float e34, float e44, float shutterTime ) {
    if( m_data->unanimated )
        throw std::runtime_error(
            "This transformation has been defined as unanimated. No new transformations can be added to it. To use "
            "specify an animated transformation, no not provide an initial transformation on creation time." );
    m_data->tm[shutterTime] =
        transform4f( e11, e21, e31, e41, e12, e22, e32, e42, e13, e23, e33, e43, e14, e24, e34, e44 );
}

void animated_transform::get_transform( float* outElements, float shutterTime ) const {
    transform4f outTm;
    if( m_data->tm.empty() ) {
        outTm.set_to_identity();
    } else if( m_data->tm.size() == 1 ) {
        outTm = m_data->tm.begin()->second;
    } else {
        std::map<float, transform4f>::const_iterator iter = m_data->tm.lower_bound( shutterTime );
        if( iter == m_data->tm.end() ) {
            // values for z that are greater than the last value in zValuesData
            --iter;
            outTm = iter->second;
        } else if( iter == m_data->tm.begin() ) {
            // values for z that are less than or equal to the first value in zValuesData
            outTm = iter->second;
        } else {
            // values for z that fall between two samples in zValuesData (interpolate)
            std::map<float, transform4f>::const_iterator prevIter = iter;
            --prevIter;
            float alpha = ( shutterTime - prevIter->first ) / ( iter->first - prevIter->first );
            outTm = ( ( 1.0f - alpha ) * prevIter->second + alpha * iter->second );
        }
    }
    // convert from transform4f to float[16]. I know I could probably just do weird casting to make it work, but this
    // seems better.
    outElements[0] = outTm[0];
    outElements[1] = outTm[1];
    outElements[2] = outTm[2];
    outElements[3] = outTm[3];
    outElements[4] = outTm[4];
    outElements[5] = outTm[5];
    outElements[6] = outTm[6];
    outElements[7] = outTm[7];
    outElements[8] = outTm[8];
    outElements[9] = outTm[9];
    outElements[10] = outTm[10];
    outElements[11] = outTm[11];
    outElements[12] = outTm[12];
    outElements[13] = outTm[13];
    outElements[14] = outTm[14];
    outElements[15] = outTm[15];
}

bool animated_transform::is_animated() const { return ( m_data->tm.size() > 1 ); }

bool animated_transform::is_identity() const {
    if( !m_data->tm.empty() ) {
        std::map<float, transform4f>::const_iterator iter = m_data->tm.begin(), iterEnd = m_data->tm.end();
        for( ; iter != iterEnd; ++iter ) {
            if( !iter->second.is_identity() )
                return false;
        }
    }
    return true;
}

} // namespace krakatoasr
