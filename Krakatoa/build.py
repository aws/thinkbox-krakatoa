# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from __future__ import annotations
from typing import Any
from cpt.packager import ConanMultiPackager

import argparse
import platform
import pprint


COMMON_PACKAGER_ARGS: dict[str, Any] = {
    'build_types': ['Release'],
    'archs': ['x86_64'],
    'build_policy': 'missing'
}

WINDOWS_PACKAGER_ARGS: dict[str, Any] = {
    'visual_versions': ['15', '16'],
    'visual_runtimes': ['MD']
}

LINUX_PACKAGER_ARGS: dict[str, Any] = {
    'gcc_versions': ['7']
}

MACOS_PACKAGER_ARGS: dict[str, Any] = {
    'apple_clang_versions': ['10']
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('-u', '--username', default=None, help='The Conan username to use for the built package.')
    parser.add_argument('-c', '--channel', default=None, help='The Conan channel to use for the built package.')
    parser.add_argument('-o', '--option', action='append', dest='options', help='Specify package options to be used by the build.')
    parser.add_argument('--dry-run', action='store_true', help='Print the configurations that would be built without actually building them.')
    return parser.parse_args()

def main() -> None:
    args = parse_arguments()

    packager_args = {
        'username': args.username,
        'channel': args.channel,
        'options': args.options
    }
    packager_args.update(COMMON_PACKAGER_ARGS)

    if platform.system() == 'Windows':
        packager_args.update(WINDOWS_PACKAGER_ARGS)
    elif platform.system() == 'Linux':
        packager_args.update(LINUX_PACKAGER_ARGS)
    elif platform.system() == 'Darwin':
        packager_args.update(MACOS_PACKAGER_ARGS)
    else:
        raise Exception('Platform not supported.')

    builder = ConanMultiPackager(**packager_args)
    builder.add_common_builds(pure_c=False)

    # Remove the legacy libstdc++ build.
    if platform.system() == 'Linux':
        builder.remove_build_if(lambda build: build.settings['compiler.libcxx'] == 'libstdc++')

    if args.dry_run:
        pprint.pprint(builder.builds, indent=4)
    else:
        builder.run()


if __name__ == '__main__':
    main()
