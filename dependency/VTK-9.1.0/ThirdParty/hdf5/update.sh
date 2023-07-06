#!/usr/bin/env bash

set -e
set -x
shopt -s dotglob

readonly name="hdf5"
readonly ownership="HDF Upstream <kwrobot@kitware.com>"
readonly subtree="ThirdParty/$name/vtk$name"
readonly repo="https://gitlab.kitware.com/third-party/hdf5.git"
readonly tag="for/vtk-20210916.1-1.12.1"
readonly paths="
CMakeFilters.cmake
CMakeInstallation.cmake
CMakeLists.txt
CMakePlugins.cmake
UserMacros.cmake

src/
hl/CMakeLists.txt
hl/src/

config/lt_vers.am
config/clang-warnings
config/gnu-warnings
config/intel-warnings
config/cmake/ConfigureChecks.cmake
config/cmake/ConversionTests.c
config/cmake/H5pubconf.h.in
config/cmake/HDF5Macros.cmake
config/cmake/HDF5PluginMacros.cmake
config/cmake/HDFCompilerFlags.cmake
config/cmake/libhdf5.settings.cmake.in

config/cmake_ext_mod/ConfigureChecks.cmake
config/cmake_ext_mod/HDFMacros.cmake
config/cmake_ext_mod/HDFLibMacros.cmake
config/cmake_ext_mod/HDFTests.c

.gitattributes
ACKNOWLEDGMENTS
COPYING
COPYING_LBNL_HDF5
README.txt
README.kitware.md
"

extract_source () {
    git_archive
    pushd "$extractdir/$name-reduced"
    find . -name Makefile.am -delete
    find . -name Makefile.in -delete
    find . -name "*.lnt" -delete
    rm -v src/.indent.pro
    # Add missing newline at EOF.
    echo >> config/intel-warnings/general
    popd
}

. "${BASH_SOURCE%/*}/../update-common.sh"
