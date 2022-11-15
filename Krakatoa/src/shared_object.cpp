// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <krakatoa/shared_object.hpp>

namespace krakatoa {

shared_object::shared_object()
    : m_refCount( 0 ) {}

shared_object::~shared_object() {}

void intrusive_ptr_add_ref( shared_object* pObj ) { ++pObj->m_refCount; }

void intrusive_ptr_release( shared_object* pObj ) {
    if( --pObj->m_refCount == 0 )
        delete pObj;
}

} // namespace krakatoa
