// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <cstdio>

// This can be removed when we get a version of Flex which supports Visual Studio 2015
#if defined( THINKBOX_DEF_STDIO ) && defined( _WIN32 ) && _MSC_VER >= 1900
extern "C" {
FILE __iob_func[3] = { *stdin, *stdout, *stderr };
}
#endif