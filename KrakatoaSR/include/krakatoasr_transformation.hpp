// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#ifndef __KRAKATOASR_TRANSFORMATION__
#define __KRAKATOASR_TRANSFORMATION__

#include <krakatoasr_datatypes.hpp>

namespace krakatoasr {

/**
 * Transformation matrix.
 * This is a helper class for the renderer. It represents an animated or unanimated transformation matrix.
 */
class CLSEXPORT animated_transform {
  private:
    animated_transform_params* m_data;

  public:
    /**
     * Default constructor.
     * Use this constructor if you want to create animated transformations.
     * To set the matrix to non-identity, the user will want to call "add_transform" one or more times.
     * If the user does not call "add_transform", it will default to the identity matrix.
     */
    animated_transform();

    /**
     * Unanimated matrix constructor from element array.
     * When using this constructor, the matrix will be unanimated.
     * @param elements An array of 16 floating point elements representing a 4x4 transformation matrix.
     */
    animated_transform( const float* elements );

    /**
     * Unanimated matrix constructor from list of elements.
     * When using this constructor, the matrix will be unanimated.
     * @param "eXY" the "X" and "Y" elements of the 4x4 transformation matrix.
     */
    animated_transform( float e11, float e21, float e31, float e41, float e12, float e22, float e32, float e42,
                        float e13, float e23, float e33, float e43, float e14, float e24, float e34,
                        float e44 ); // unanimated

    ~animated_transform();
    animated_transform( const animated_transform& t );
    animated_transform& operator=( const animated_transform& t );
    const animated_transform_params* get_data() const;
    animated_transform_params* get_data();

    /**
     * Adds a transformation from element array at a specific shutter time.
     * Used for constructing animated transformations. This function can be called multiple times for different shutter
     * times.
     * @param elements An array of 16 floating point elements representing a 4x4 transformation matrix.
     * @param shutterTime The shutter time in seconds of this matrix.
     */
    void add_transform( const float* elements, float shutterTime );

    /**
     * Adds a transformation from element array at a specific shutter time.
     * Used for constructing animated transformations. This function can be called multiple times for different shutter
     * times.
     * @param "eXY" the "X" and "Y" elements of the 4x4 transformation matrix.
     * @param shutterTime The shutter time in seconds of this matrix.
     */
    void add_transform( float e11, float e21, float e31, float e41, float e12, float e22, float e32, float e42,
                        float e13, float e23, float e33, float e43, float e14, float e24, float e34, float e44,
                        float shutterTime );

    /**
     * Retrieves a transformation matrix at a given shutter time.
     * When the transformation matrix is animated, it will use linear interpolation to create a transformation at the
     * given shutter time.
     * @param outElements The user must pass in a 16 element float array. The resulting transformation matrix will be
     * placed in this array.
     * @param shutterTime The desired matrix's shutter time in seconds. If it is unanimated, this parameter will not be
     * used.
     */
    void get_transform( float* outElements, float shutterTime = 0.0f ) const;

    /**
     * Test to see if this matrix is animated.
     * @return True if the matrix is animated.
     */
    bool is_animated() const;

    /**
     * Test if the matrix is the identity matrix.
     * @return True if the matrix is always the identity matrix.
     */
    bool is_identity() const;
};

} // namespace krakatoasr

#endif
