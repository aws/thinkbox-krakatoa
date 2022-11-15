// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <iostream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_array.hpp>

#include <frantic/files/files.hpp>
#include <frantic/files/paths.hpp>
#include <frantic/logging/console_progress_logger.hpp>
#include <frantic/particles/particle_file_stream_factory.hpp>
#include <frantic/particles/prt_metadata.hpp>
#include <frantic/strings/tstring.hpp>

#include "krakatoa/sticky_channel_particle_istream.hpp"

#include "krakatoa/birth_channel_gen.hpp"

void birth_channel_gen::generate_sticky_channels( const birth_channel_gen::options& opts ) {
    frantic::files::filename_sequence inputFileSequence( opts.m_inSequence );
    try {
        inputFileSequence.sync_frame_set();
    } catch( std::runtime_error ) {
        throw birth_channel_gen::gen_exception( _T( "Error: The input sequence \"" ) + opts.m_inSequence +
                                                _T( "\"doesn't exist." ) );
    }

    std::vector<double> frames;
    inputFileSequence.allframe_numbers( frames );

    if( frames.size() == 0 ) {
        throw birth_channel_gen::gen_exception( _T( "Error: The input sequence \"" ) + opts.m_inSequence +
                                                _T( "\"doesn't exist." ) );
    }

    frantic::files::filename_sequence outputFileSequence( opts.m_outSequence );

    std::map<boost::int64_t, boost::shared_array<char>> birthValues;

    scope_answer::scope_answer overwriteChannels = opts.m_overwriteChannel;
    scope_answer::scope_answer overwriteFiles = opts.m_overwriteFile;

    volatile double startFrame = opts.m_startFrame;

    if( opts.m_startFrame == 0 && frames.front() > 0 ) {
        startFrame = frames.front();
    } else if( opts.m_startFrame == 0 && frames.back() < 0 ) {
        startFrame = frames.back();
    }

    if( startFrame < frames.front() || startFrame > frames.back() ) {
        throw birth_channel_gen::gen_exception( _T( "Error: Start frame out of sequence range." ) );
    }

    double currentFrame = startFrame;
    boost::int64_t currentFile = 0;
    frantic::logging::console_progress_logger progress;
    progress.set_title( _T( "Adding Channel..." ) );
    progress.update_progress( 0.0f );

    // Generate for the start Frame.
    detail::generate_sticky_channels_frame( opts, currentFrame, inputFileSequence, outputFileSequence, birthValues,
                                            overwriteChannels, overwriteFiles );

    ++currentFile;
    progress.update_progress( static_cast<boost::int32_t>(
        ( static_cast<float>( currentFile ) / static_cast<float>( frames.size() ) ) * 100.0f ) );

    // If the first frame is lower than the start frame...
    if( frames.front() < currentFrame ) {
        //...moving backwards from the end, skip all frames, including the start frame, then generate for frames before
        //the start frame.
        for( std::vector<double>::reverse_iterator iter = frames.rbegin(); iter != frames.rend(); ++iter ) {
            if( *iter >= startFrame ) {
                continue;
            }

            currentFrame = *iter;
            detail::generate_sticky_channels_frame( opts, currentFrame, inputFileSequence, outputFileSequence,
                                                    birthValues, overwriteChannels, overwriteFiles );

            ++currentFile;
            progress.update_progress( static_cast<boost::int32_t>(
                ( static_cast<float>( currentFile ) / static_cast<float>( frames.size() ) ) * 100.0f ) );
        }
    }

    // Moving forwards from the beginning, skip all frames, including the start frame, then generate for frames after
    // the start frame.
    for( std::vector<double>::iterator iter = frames.begin(); iter != frames.end(); ++iter ) {
        if( *iter <= startFrame ) {
            continue;
        }

        currentFrame = *iter;
        detail::generate_sticky_channels_frame( opts, currentFrame, inputFileSequence, outputFileSequence, birthValues,
                                                overwriteChannels, overwriteFiles );

        ++currentFile;
        progress.update_progress( static_cast<boost::int32_t>(
            ( static_cast<float>( currentFile ) / static_cast<float>( frames.size() ) ) * 100.0f ) );
    }

    progress.update_progress( 100.0f );
}

void birth_channel_gen::detail::generate_sticky_channels_frame(
    const birth_channel_gen::options& opts, double currentFrame,
    const frantic::files::filename_sequence& inputFileSequence,
    const frantic::files::filename_sequence& outputFileSequence,
    std::map<boost::int64_t, boost::shared_array<char>>& birthValues, scope_answer::scope_answer& overwriteChannels,
    scope_answer::scope_answer& overwriteFiles ) {

    frantic::tstring inputFile = inputFileSequence[currentFrame];
    frantic::tstring outputFile = outputFileSequence[currentFrame];

    frantic::particles::streams::particle_istream_ptr inFileStream;

    frantic::particles::particle_file_stream_factory_object inputFactory;
    frantic::particles::particle_file_metadata meta;

    if( !frantic::files::file_exists( inputFile ) ) {
        handle_error( opts, _T( "Error: the input file \"" ) + inputFile + _T( "\" does not exist." ) );
        return;
    }

    inFileStream = inputFactory.create_istream( inputFile, &meta );

    frantic::channels::channel_map channels = inFileStream->get_channel_map();
    if( channels.has_channel( opts.m_outChannel ) ) {
        if( overwriteChannels != scope_answer::yes_all && overwriteChannels != scope_answer::no_all ) {
            scope_answer::scope_answer answer =
                yes_no_all( _T( "The input file already has a \"" ) + opts.m_outChannel +
                            _T( "\" channel. Should this be overwritten in the output file?" ) );
            if( answer == scope_answer::yes_all || answer == scope_answer::no_all ) {
                overwriteChannels = answer;
            }
            if( answer == scope_answer::no || answer == scope_answer::no_all ) {
                handle_error( opts, _T( "Error: the input file \"" ) + inputFile + _T( "\" already has a \"" ) +
                                        opts.m_outChannel + _T( "\" channel and overwrite is turned off." ) );
                return;
            }

        } else if( overwriteChannels == scope_answer::no_all ) {
            handle_error( opts, _T( "Error: the input file \"" ) + inputFile + _T( "\" already has a \"" ) +
                                    opts.m_outChannel + _T( "\" channel and overwrite is turned off." ) );
            return;
        }
    }

    if( frantic::files::file_exists( outputFile ) ) {
        if( overwriteFiles != scope_answer::yes_all && overwriteFiles != scope_answer::no_all ) {
            scope_answer::scope_answer answer = yes_no_all(
                _T( "The output file \"" ) + outputFile + _T( "\" already exists. Should this file be overwritten?" ) );
            if( answer == scope_answer::yes_all || answer == scope_answer::no_all ) {
                overwriteFiles = answer;
            }
            if( answer == scope_answer::no || answer == scope_answer::no_all ) {
                handle_error( opts, _T( "Error: The output file \"" ) + outputFile +
                                        _T( "\" already exists and overwrite is turned off." ) );
                return;
            }

        } else if( overwriteFiles == scope_answer::no_all ) {
            handle_error( opts, _T( "Error: The output file \"" ) + outputFile +
                                    _T( "\" already exists and overwrite is turned off." ) );
            return;
        }
    }

    if( !channels.has_channel( opts.m_inChannel ) ) {
        handle_error( opts, _T( "Error: the input file \"" ) + inputFile + _T( "\" is missing a \"" ) +
                                opts.m_inChannel + _T( "\" channel." ) );
        return;
    }

    if( !channels.has_channel( opts.m_idChannel ) ) {
        handle_error( opts, _T( "Error: the input file \"" ) + inputFile + _T( "\" is missing a \"" ) +
                                opts.m_idChannel + _T( "\" channel." ) );
        return;
    }

    frantic::particles::streams::particle_istream_ptr stickyStream( new stickychannel_particle_istream(
        inFileStream, birthValues, opts.m_inChannel, opts.m_outChannel, opts.m_idChannel ) );

    frantic::particles::streams::particle_ostream_ptr outFileStream;
    frantic::particles::particle_file_stream_factory_object outputFactory;
    outFileStream = outputFactory.create_ostream( outputFile, stickyStream->get_native_channel_map(),
                                                  stickyStream->get_native_channel_map(), &meta,
                                                  stickyStream->particle_count(), -1 );

    boost::scoped_array<char> particleBuffer( new char[stickyStream->get_native_channel_map().structure_size()] );
    while( stickyStream->get_particle( particleBuffer.get() ) ) {
        outFileStream->put_particle( particleBuffer.get() );
    }

    stickyStream->close();
    outFileStream->close();
}

birth_channel_gen::scope_answer::scope_answer
birth_channel_gen::detail::yes_no_all( const frantic::tstring& question ) {
    char answer;
    char allAnswer;

    std::cout << frantic::strings::to_string( question ) << std::endl;
    std::cout << "Enter 'y' or 'Y' for yes, anything else will be considered no." << std::endl;
    std::cin >> answer;

    std::cout << "Should this apply to all frames?" << std::endl;
    std::cout << "Enter 'a' or 'A' for yes, anything else will be considered no." << std::endl;
    std::cin >> allAnswer;

    std::tolower( answer );
    std::tolower( allAnswer );

    if( answer == 'y' ) {
        if( allAnswer == 'a' ) {
            return scope_answer::yes_all;
        } else {
            return scope_answer::yes;
        }
    } else {
        if( allAnswer == 'a' ) {
            return scope_answer::no_all;
        } else {
            return scope_answer::no;
        }
    }
}

void birth_channel_gen::detail::handle_error( const birth_channel_gen::options& opts, const frantic::tstring& error ) {
    if( opts.m_ignoreError ) {
        std::cerr << frantic::strings::to_string( error ) << std::endl;
    } else {
        throw birth_channel_gen::gen_exception( error );
    }
}