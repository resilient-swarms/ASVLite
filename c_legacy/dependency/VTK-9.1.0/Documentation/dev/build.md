# Building VTK

This page describes how to build and install VTK. It covers building for
development, on both Unix-type systems (Linux, HP-UX, Solaris, macOS), and
Windows. Note that Unix-like environments such as Cygwin and MinGW are not
officially supported. However, patches to fix problems with these platforms
will be considered for inclusion. It is recommended that users which require
VTK to work on these platforms to submit nightly testing results for them.

A full-featured build of VTK depends on several open source tools and libraries
such as Python, Qt, CGNS, HDF5, etc. Some of these are included in the VTK
source itself (e.g., HDF5), while others are expected to be present on the
machine on which VTK is being built (e.g., Python, Qt).

## Obtaining the source

To obtain VTK's sources locally, clone this repository using
[Git][git].

```sh
git clone --recursive https://gitlab.kitware.com/vtk/vtk.git
```

## Building

VTK supports all of the common generators supported by CMake. The Ninja,
Makefiles, and Visual Studio generators are the most well-tested however.

Note that VTK does not support in-source builds, so you must have a build tree
that is not the source tree.

### Prerequisites

VTK only requires a few packages in order to build in general, however
specific features may require additional packages to be provided to VTK's
build configuration.

Required:

  * [CMake][cmake]
    - Version 3.12 or newer, however, the latest version is always recommended
  * Supported compiler
    - GCC 4.8 or newer
    - Clang 3.3 or newer
    - Apple Clang 5.0 (from Xcode 5.0) or newer
    - Microsoft Visual Studio 2015 or newer
    - Intel 14.0 or newer

Optional dependencies:

  * [Python][python]
    - When using Python 2, at least 2.7 is required
    - When using Python 3, at least 3.4 is required
  * [Qt5][qt]
    - Version 5.9 or newer

#### Installing CMake

CMake is a tool that makes cross-platform building simple. On several systems
it will probably be already installed or available through system package
management utilities. If it is not, there are precompiled binaries available on
[CMake's download page][cmake-download].

#### Installing Qt

VTK uses Qt as its GUI library (if the relevant modules are enabled).
Precompiled binaries are available on [Qt's website][qt-download].

Note that on Windows, the compiler used for building VTK must match the
compiler version used to build Qt.

### Optional Additions

#### Download And Install ffmpeg (`.avi`) movie libraries

When the ability to write `.avi` files is desired, and writing these files is
not supported by the OS, VTK can use the ffmpeg library. This is generally
true for Unix-like operating systems. Source code for ffmpeg can be obtained
from [the website][ffmpeg].

#### MPI

To run VTK in parallel, an [MPI][mpi] implementation is required. If an MPI
implementation that exploits special interconnect hardware is provided on your
system, we suggest using it for optimal performance. Otherwise, on Linux/Mac,
we suggest either [OpenMPI][openmpi] or [MPICH][mpich]. On Windows, [Microsoft
MPI][msmpi] is required.

#### Python

In order to use scripting, [Python][python] is required (versions 2.7 and 3.3).

#### OSMesa

Off-screen Mesa can be used as a software-renderer for running VTK on a server
without hardware OpenGL acceleration. This is usually available in system
packages on Linux. For example, the `libosmesa6-dev` package on Debian and
Ubuntu. However, for older machines, building a newer version of Mesa is
likely necessary for bug fixes and support. Its source and build instructions
can be found on [its website][mesa].

## Creating the Build Environment

### Linux (Ubuntu/Debian)

  * `sudo apt install` the following packages:
    - `build-essential`
    - `cmake`
    - `mesa-common-dev`
    - `mesa-utils`
    - `freeglut3-dev`
    - `ninja-build`
      - `ninja` is a speedy replacement for `make`, highly recommended.

### Windows

  * [Visual Studio Community Edition][visual-studio]
  * Use "x64 Native Tools Command Prompt" for the installed Visual Studio
    version to configure with CMake and to build with ninja.
  * Get [ninja][ninja]. Unzip the binary and put it in `PATH`. Note that newer
    Visual Studio releases come with a version of `ninja` already and should
    already exist in `PATH` within the command prompt.

## Building

In order to build, CMake requires two steps, configure and build. VTK itself
does not support what are known as in-source builds, so the first step is to
create a build directory.

```sh
mkdir -p vtk/build
cd vtk/build
ccmake ../path/to/vtk/source # -GNinja may be added to use the Ninja generator
```

CMake's GUI has input entries for the build directory and the generator
already. Note that on Windows, the GUI must be launched from a "Native Tools
Command Prompt" available with Visual Studio in the start menu.

### Missing dependencies

CMake may not find all dependencies automatically in all cases. The steps
needed to find any given package depends on the package itself. For general
assistance, please see the documentation for
[`find_package`'s search procedure][cmake-find_package-search] and
[the relevant Find module][cmake-modules-find] (as available).

### Build Settings

VTK has a number of settings available for its build. The common variables
to modify include:

  * `BUILD_SHARED_LIBS` (default `ON`): If set, shared libraries will be
    built. This is usually what is wanted.
  * `VTK_USE_CUDA` (default `OFF`): Whether [CUDA][cuda] support will be available or
    not.
  * `VTK_USE_MPI` (default `OFF`): Whether [MPI][mpi] support will be available or
    not.
  * `VTK_WRAP_PYTHON` (default `OFF`; requires `VTK_ENABLE_WRAPPING`): Whether
    Python support will be available or not.
  * `VTK_PYTHON_VERSION` (default `3`): The major version of Python to
    support. Must be either `2` or `3`.

Less common, but variables which may be of interest to some:

  * `VTK_BUILD_EXAMPLES` (default `OFF`): If set, VTK's example code will be
    added as tests to the VTK test suite.
  * `VTK_ENABLE_LOGGING` (default `ON`): If set, enhanced logging will be
    enabled.
  * `VTK_BUILD_TESTING` (default `OFF`): Whether to build tests or not. Valid
    values are `OFF` (no testing), `WANT` (enable tests as possible), and `ON`
    (enable all tests; may error out if features otherwise disabled are
    required by test code).
  * `VTK_ENABLE_KITS` (default `OFF`; requires `BUILD_SHARED_LIBS`): Compile
    VTK into a smaller set of libraries. Can be useful on platforms where VTK
    takes a long time to launch due to expensive disk access.
  * `VTK_ENABLE_WRAPPING` (default `ON`): Whether any wrapping support will be
    available or not.
  * `VTK_WRAP_JAVA` (default `OFF`; requires `VTK_ENABLE_WRAPPING`):
    Whether Java support will be available or not.
  * `VTK_SMP_IMPLEMENTATION_TYPE` (default `Sequential`): Set which SMPTools
    will be implemented by default. Must be either `Sequential`, `STDThread`,
    `OpenMP` or `TBB`. The backend can be changed at runtime if the desired
    backend has his option `VTK_SMP_ENABLE_<backend_name>` set to `ON`.

More advanced options:

  * `VTK_BUILD_DOCUMENTATION` (default `OFF`): If set, VTK will build its API
    documentation using Doxygen.
  * `VTK_BUILD_ALL_MODULES` (default `OFF`): If set, VTK will enable all
    modules not disabled by other features.
  * `VTK_ENABLE_REMOTE_MODULES` (default `ON`): If set, VTK will try to build
    remote modules (the `Remote` directory). If unset, no remote modules will
    build.
  * `VTK_USE_EXTERNAL` (default `OFF`): Whether to prefer external third
    party libraries or the versions VTK's source contains.
  * `VTK_TARGET_SPECIFIC_COMPONENTS` (default `OFF`): Whether to install
    files into target-specific components (`<TARGET>-runtime`,
    `<TARGET>-development`, etc.) or general components (`runtime`,
    `development`, etc.)
  * `VTK_VERSIONED_INSTALL` (default `ON`): Whether to add version numbers to
    VTK's include directories and library names in the install tree.
  * `VTK_CUSTOM_LIBRARY_SUFFIX` (default depends on `VTK_VERSIONED_INSTALL`):
    The custom suffix for libraries built by VTK. Defaults to either an empty
    string or `X.Y` where `X` and `Y` are VTK's major and minor version
    components, respectively.
  * `VTK_INSTALL_SDK` (default `ON`): If set, VTK will install its headers,
    CMake API, etc. into its install tree for use.
  * `VTK_RELOCATABLE_INSTALL` (default `ON`): If set, the install tree will be
    relocatable to another path. If unset, the install tree may be tied to the
    build machine with absolute paths, but finding dependencies in
    non-standard locations may require work without passing extra information
    when consuming VTK.
  * `VTK_UNIFIED_INSTALL_TREE` (default `OFF`): If set, the install tree is
    stipulated to be a unified install tree of VTK and all of its dependencies;
    a unified tree usually simplifies things including, but not limited to,
    the Python module paths, library search paths, and plugin searching. This
    option is irrelevant if a relocatable install is requested as such setups
    assume that dependencies are set up either via a unified tree or some other
    mechanism such as modules).
  * `VTK_ENABLE_SANITIZER` (default `OFF`): Whether to enable sanitization of
    the VTK codebase or not.
  * `VTK_SANITIZER` (default `address`; requires `VTK_ENABLE_SANITIZER`): The
    sanitizer to use.
  * `VTK_USE_LARGE_DATA` (default `OFF`; requires `VTK_BUILD_TESTING`):
    Whether to enable tests which use "large" data or not (usually used to
    reduce the amount of data downloading required for the test suite).
  * `VTK_LEGACY_REMOVE` (default `OFF`): If set, VTK will disable legacy,
    deprecated APIs.
  * `VTK_LEGACY_SILENT` (default `OFF`; requires `VTK_LEGACY_REMOVE` to be
    `OFF`): If set, usage of legacy, deprecated APIs will not cause warnings.
  * `VTK_USE_TK` (default `OFF`; requires `VTK_WRAP_PYTHON`): If set, VTK will
    enable Tkinter support for VTK widgets.
  * `VTK_BUILD_COMPILE_TOOLS_ONLY` (default `OFF`): If set, VTK will compile
    just its compile tools for use in a cross-compile build.
  * `VTK_SERIAL_TESTS_USE_MPIEXEC` (default `OFF`): Used on HPC to run
    serial tests on compute nodes. If set, it prefixes serial tests with
    "${MPIEXEC_EXECUTABLE}" "${MPIEXEC_NUMPROC_FLAG}" "1" ${MPIEXEC_PREFLAGS}
  * `VTK_WINDOWS_PYTHON_DEBUGGABLE` (default `OFF`): Set to `ON` if using a
    debug build of Python.
  * `VTK_DLL_PATHS` (default `""`): If set, these paths will be added via
    Python 3.8's `os.add_dll_directory` mechanism in order to find dependent
    DLLs when loading VTK's Python modules.
  * `VTK_ENABLE_VR_COLLABORATION` (default `OFF`): If `ON`, includes support
    for multi client VR collaboration. Requires libzmq and cppzmq external libraries.
  * `VTK_SMP_ENABLE_<backend_name>` (default `OFF` if needs an external library otherwise `ON`):
    If set, builds with the specified SMPTools backend implementation that can be
    changed on runtime with `VTK_SMP_BACKEND_IN_USE` environment variable.
  * `VTK_USE_VIDEO_FOR_WINDOWS` (default `OFF`; requires Windows): Enable the
    `vtkAVIWriter` class in the `VTK::IOMovie` module.
  * `VTK_USE_VIDEO_FOR_WINDOWS_CAPTURE` (default `OFF`; requires Windows):
    Enable the `vtkWin32VideoSource` class in the `VTK::IOVideo` module.
  * `VTK_USE_MICROSOFT_MEDIA_FOUNDATION` (default `OFF`; requires Windows):
    Enable the `vtkMP4Writer` class in the `VTK::IOMovie` module.
  * `VTK_USE_64BIT_TIMESTAMPS` (default `OFF`; forced on for 64-bit builds):
    Build with 64-bit `vtkMTimeType`.
  * `VTK_USE_64BIT_IDS` (default `OFF` for 32-bit builds; `ON` for 64-bit
    builds): Whether `vtkIdType` should be 32-bit or 64-bit.
  * `VTK_DEBUG_LEAKS` (default `OFF`): Whether VTK will report leaked
    `vtkObject` instances at process destruction or not.
  * `VTK_DEBUG_RANGE_ITERATORS` (default `OFF`; requires a `Debug` build):
    Detect errors with `for-range` iterators in VTK (note that this is very
    slow).
  * `VTK_ALWAYS_OPTIMIZE_ARRAY_ITERATORS` (default `OFF`; requires `NOT
    VTK_DEBUG_RANGE_ITERATORS`): Optimize `for-range` array iterators even in
    `Debug` builds.
  * `VTK_ALL_NEW_OBJECT_FACTORY` (default `OFF`): If `ON`, classes using
    `vtkStandardNewMacro` will use `vtkObjectFactoryNewMacro` allowing
    overrides to be available even when not explicitly requested through
    `vtkObjectFactoryNewMacro` or `vtkAbstractObjectFactoryNewMacro`.

The VTK module system provides a number of variables to control modules which
are not otherwise controlled by the other options provided.

  * `VTK_MODULE_USE_EXTERNAL_<name>` (default depends on `VTK_USE_EXTERNAL`):
    Use an external source for the named third-party module rather than the
    copy contained within the VTK source tree.

    > **_WARNING:_**
    >
    > Activating this option within an interactive cmake configuration (i.e. ccmake, cmake-gui)
    > could end up finding libraries in the standard locations rather than copies
    > in non-standard locations.
    >
    > It is recommended to pass the variables necessary to find the intended external package to
    > the first configure to avoid finding unintended copies of the external package.
    > The variables which matter depend on the package being found, but those ending with
    > `_LIBRARY` and `_INCLUDE_DIR` as well as the general CMake `find_package` variables ending
    > with `_DIR` and `_ROOT` are likely candidates.
    >
    > ```
    > Example:
    > ccmake -D HDF5_ROOT:PATH=/home/user/myhdf5 ../vtk/sources
    > ```

  * `VTK_MODULE_ENABLE_<name>` (default `DEFAULT`): Change the build settings
    for the named module. Valid values are those for the module system's build
    settings (see below).
  * `VTK_GROUP_ENABLE_<name>` (default `DEFAULT`): Change the default build
    settings for modules belonging to the named group. Valid values are those
    for the module system's build settings (see below).

For variables which use the module system's build settings, the valid values are as follows:

  * `YES`: Require the module to be built.
  * `WANT`: Build the module if possible.
  * `DEFAULT`: Use the settings by the module's groups and
    `VTK_BUILD_ALL_MODULES`.
  * `DONT_WANT`: Don't build the module unless required as a dependency.
  * `NO`: Do not build the module.

If any `YES` module requires a `NO` module, an error is raised.

#### Mobile devices

VTK supports mobile devices in its build. These are triggered by a top-level
flag which then exposes some settings for a cross-compiled VTK that is
controlled from the top-level build.

iOS builds may be enabled by setting the `VTK_IOS_BUILD` option. The following
settings than affect the iOS build:

  * `IOS_SIMULATOR_ARCHITECTURES`
  * `IOS_DEVICE_ARCHITECTURES`
  * `IOS_DEPLOYMENT_TARGET`
  * `IOS_EMBED_BITCODE`

Android builds may be enabled by setting the `VTK_ANDROID_BUILD` option. The
following settings affect the Android build:

  * `ANDROID_NDK`
  * `ANDROID_NATIVE_API_LEVEL`
  * `ANDROID_ARCH_ABI`

#### Python wheels

VTK also supports creating a Python wheel containing its Python wrappers for
Python3 (Python2 wheels are no longer supported). This is supported by setting
the `VTK_WHEEL_BUILD` flag. This changes the build directory structure around
to match that expected by wheels. Once configured, the build tree may be built
as it would be normally and then the generated `setup.py` file used to create
the wheel.

```sh
cmake -GNinja -DVTK_WHEEL_BUILD=ON -DVTK_WRAP_PYTHON=ON path/to/vtk/source
ninja
python3 setup.py bdist_wheel
```

Any modules may be turned on or off as in a normal VTK build. Certain modules
add features to the generated wheel to indicate their availability. These flags
are not meant to be comprehensive, but any reasonable feature flags may be
added to `CMake/vtkWheelFinalization.cmake` as needed.

Note that the wheel will not include any external third party libraries in its
wheel (e.g., X11, OpenGL, etc.) to avoid conflicts with systems or other wheels
doing the same.

## Building documentation

The following targets are used to build documentation for VTK:

  * `DoxygenDoc` - build the doxygen documentation from VTK's C++ source files.

[cmake]: https://cmake.org
[cmake-download]: https://cmake.org/download
[cmake-find_package-search]: https://cmake.org/cmake/help/latest/command/find_package.html#search-procedure
[cmake-modules-find]: https://cmake.org/cmake/help/latest/manual/cmake-modules.7.html#find-modules
[cuda]: https://developer.nvidia.com/cuda-zone
[ffmpeg]: https://ffmpeg.org
[git]: https://git-scm.org
[mesa]: https://www.mesa3d.org
[mpi]: https://www.mcs.anl.gov/research/projects/mpi
[ninja]: https://ninja-build.org
[msmpi]: https://docs.microsoft.com/en-us/message-passing-interface/microsoft-mpi
[mpich]: https://www.mpich.org
[nvpipe]: https://github.com/NVIDIA/NvPipe
[openmpi]: https://www.open-mpi.org
[python]: https://python.org
[qt]: https://qt.io
[qt-download]: https://download.qt.io/official_releases/qt
[visual-studio]: https://visualstudio.microsoft.com/vs
