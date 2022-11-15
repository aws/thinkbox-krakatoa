// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/call_traits.hpp>

namespace krakatoa {

namespace detail {
template <class T>
class collection_interface_traits {
  public:
    typedef T& return_type;
    typedef const T& const_return_type;
};
} // namespace detail

template <class T, class Traits = detail::collection_interface_traits<T>>
class collection_interface {
  protected:
    typedef Traits traits_type;

  public:
    typedef typename traits_type::return_type return_type;
    typedef typename traits_type::const_return_type const_return_type;

  public:
    virtual ~collection_interface() {}

    virtual std::size_t size() const = 0;

    virtual return_type get( int index ) = 0;

    virtual const_return_type get( int index ) const = 0;
};

template <class T, template <typename, typename> class Container, class Allocator = std::allocator<T>>
class collection_wrapper : public Container<T, Allocator>, public collection_interface<T> {
  public:
    virtual ~collection_wrapper() {}

    virtual std::size_t size() const { return static_cast<const Container<T, Allocator>*>( this )->size(); }

    virtual typename collection_interface<T>::return_type get( int index ) {
        return ( *static_cast<Container<T, Allocator>*>( this ) )[index];
    }

    virtual typename collection_interface<T>::const_return_type get( int index ) const {
        return ( *static_cast<const Container<T, Allocator>*>( this ) )[index];
    }
};

} // namespace krakatoa
