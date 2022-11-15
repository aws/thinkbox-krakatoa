// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <map>
#include <string>

#include <boost/cstdint.hpp>
#include <boost/shared_array.hpp>

#include <frantic/files/filename_sequence.hpp>

namespace birth_channel_gen {
namespace scope_answer {
enum scope_answer { ask, yes, no, yes_all, no_all };
}

struct options {
    frantic::tstring m_inSequence;
    frantic::tstring m_outSequence;
    frantic::tstring m_inChannel;
    frantic::tstring m_outChannel;
    frantic::tstring m_idChannel;
    double m_startFrame;
    bool m_ignoreError;
    scope_answer::scope_answer m_overwriteChannel;
    scope_answer::scope_answer m_overwriteFile;

    options()
        : m_inChannel( _T( "Position" ) )
        , m_outChannel( _T( "BirthPosition" ) )
        , m_idChannel( _T( "ID" ) )
        , m_startFrame( 0.0 )
        , m_ignoreError( false )
        , m_overwriteChannel( scope_answer::ask )
        , m_overwriteFile( scope_answer::ask ) {}
};

class gen_exception : public std::runtime_error {
  public:
    gen_exception( const frantic::tstring& msg )
        : std::runtime_error( frantic::strings::to_string( msg ) ) {}
};

void generate_sticky_channels( const birth_channel_gen::options& opts );

namespace detail {
void generate_sticky_channels_frame( const birth_channel_gen::options& opts, double currentFrame,
                                     const frantic::files::filename_sequence& inputFileSequence,
                                     const frantic::files::filename_sequence& outputFileSequence,
                                     std::map<boost::int64_t, boost::shared_array<char>>& birthValues,
                                     scope_answer::scope_answer& overwriteChannels,
                                     scope_answer::scope_answer& overwriteFiles );

scope_answer::scope_answer yes_no_all( const frantic::tstring& question );

void handle_error( const birth_channel_gen::options& opts, const frantic::tstring& error );
} // namespace detail
} // namespace birth_channel_gen