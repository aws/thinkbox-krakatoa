# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
import os
from typing import Any
from conans import ConanFile, CMake


SETTINGS: list[str] = [
    'os',
    'compiler',
    'build_type',
    'arch'
]

TOOL_REQUIRES: list[str] = [
    'cmake/3.24.1',
    'thinkboxcmlibrary/1.0.0',
]

REQUIRES: list[str] = [
    'boost/1.78.0',
    'zlib/1.2.12',
    'thinkboxlibrary/1.0.0',
    'krakatoa/1.0.0',
    'openimageio/2.3.7.2'
]


class KrakatoaSRConan(ConanFile):
    name: str = 'krakatoasr'
    version: str = '1.0.0'
    license: str = 'Apache-2.0'
    settings: list[str] = SETTINGS
    tool_requires: list[str] = TOOL_REQUIRES
    requires: list[str] = REQUIRES
    generators: str | list[str] = 'cmake_find_package'
    options: dict[str, Any] = {
        'shared': [True, False],
        'build_examples': [True, False]
    }
    default_options: dict[str, Any] = {
        'shared': False,
        'build_examples': False,
        'openimageio:with_libjpeg': 'libjpeg',
        'openimageio:with_libpng': True,
        'openimageio:with_freetype': False,
        'openimageio:with_hdf5': False,
        'openimageio:with_opencolorio': False,
        'openimageio:with_opencv': False,
        'openimageio:with_tbb': False,
        'openimageio:with_dicom': False,
        'openimageio:with_ffmpeg': False,
        'openimageio:with_giflib': False,
        'openimageio:with_libheif': False,
        'openimageio:with_raw': False,
        'openimageio:with_openjpeg': False,
        'openimageio:with_openvdb': False,
        'openimageio:with_ptex': False,
        'openimageio:with_libwebp': False,
        'libtiff:lzma': False,
        'libtiff:jpeg': 'libjpeg',
        'libtiff:zlib': True,
        'libtiff:libdeflate': False,
        'libtiff:zstd': False,
        'libtiff:jbig': False,
        'libtiff:webp': False
    }

    def build(self) -> None:
        cmake = CMake(self)
        cmake.configure(defs={
            'LIBRARY_TYPE': 'SHARED' if self.options.shared else 'STATIC',
            'BUILD_EXAMPLES': 'ON' if self.options.build_examples else 'OFF'
        })
        cmake.build()

    def export_sources(self) -> None:
        self.copy('**.h', src='', dst='')
        self.copy('**.hpp', src='', dst='')
        self.copy('**.cpp', src='', dst='')
        self.copy('KrakatoaSR.exp', src='', dst='')
        self.copy('KrakatoaSR.map', src='', dst='')
        self.copy('CMakeLists.txt', src='', dst='')
        self.copy('../NOTICE.txt', src='', dst='')
        self.copy('../LICENSE.txt', src='', dst='')

    def imports(self) -> None:
        # Copy DLLs to the Example binary directory
        self.copy('*.dll', dst='Release', src='bin')

    def package(self) -> None:
        cmake = CMake(self)
        cmake.install()

        with open(os.path.join(self.source_folder, 'NOTICE.txt'), 'r', encoding='utf8') as notice_file:
            notice_contents = notice_file.readlines()
        with open(os.path.join(self.source_folder, 'LICENSE.txt'), 'r', encoding='utf8') as license_file:
            license_contents = license_file.readlines()
        os.makedirs(os.path.join(self.package_folder, 'licenses'), exist_ok=True)
        with open(os.path.join(self.package_folder, 'licenses', 'LICENSE'), 'w', encoding='utf8') as cat_license_file:
            cat_license_file.writelines(notice_contents)
            cat_license_file.writelines(license_contents)

    def deploy(self) -> None:
        self.copy("*", dst="bin", src="bin")
        self.copy("*", dst="lib", src="lib")
        self.copy("*", dst="include", src="include")

    def package_info(self) -> None:
        self.cpp_info.libs = ["krakatoasr"]
