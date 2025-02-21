#!/usr/bin/env bash

set -e
set -x
shopt -s dotglob

readonly name="lz4"
readonly ownership="lz4 Upstream <kwrobot@kitware.com>"
readonly subtree="ThirdParty/$name/vtk$name"
readonly repo="https://gitlab.kitware.com/third-party/lz4.git"
readonly tag="for/vtk-20210727-1.9.3"
readonly paths="
lib/*.c
lib/*.h

.gitattributes
lib/LICENSE
lib/README.md
README.md
README.kitware.md
CMakeLists.txt
"

extract_source () {
    git_archive
}

. "${BASH_SOURCE%/*}/../update-common.sh"
