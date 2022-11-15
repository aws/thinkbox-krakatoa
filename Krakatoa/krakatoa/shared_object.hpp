// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/intrusive_ptr.hpp>

namespace krakatoa {

class shared_object {
  private:
    int m_refCount;

    friend void intrusive_ptr_add_ref( shared_object* pObj );
    friend void intrusive_ptr_release( shared_object* pObj );

  public:
    shared_object();
    virtual ~shared_object();
};

void intrusive_ptr_add_ref( shared_object* pObj );
void intrusive_ptr_release( shared_object* pObj );

} // namespace krakatoa
