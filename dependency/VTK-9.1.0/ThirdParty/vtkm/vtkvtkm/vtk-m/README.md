# VTK-m #

VTK-m is a toolkit of scientific visualization algorithms for emerging
processor architectures. VTK-m supports the fine-grained concurrency for
data analysis and visualization algorithms required to drive extreme scale
computing by providing abstract models for data and execution that can be
applied to a variety of algorithms across many different processor
architectures.

You can find out more about the design of VTK-m on the [VTK-m Wiki].


## Learning Resources ##

  + A high-level overview is given in the IEEE Vis talk "[VTK-m:
    Accelerating the Visualization Toolkit for Massively Threaded
    Architectures][VTK-m Overview]."

  + The [VTK-m Users Guide] provides extensive documentation. It is broken
    into multiple parts for learning and references at multiple different
    levels.
      + "Part 1: Getting Started" provides the introductory instruction for
        building VTK-m and using its high-level features.
      + "Part 2: Using VTK-m" covers the core fundamental components of
        VTK-m including data model, worklets, and filters.
      + "Part 3: Developing with VTK-m" covers how to develop new worklets
        and filters.
      + "Part 4: Advanced Development" covers topics such as new worklet
        types and custom device adapters.

  + A practical [VTK-m Tutorial] based in what users want to accomplish with
    VTK-m:
      + Building VTK-m and using existing VTK-m data structures and filters.
      + Algorithm development with VTK-m.
      + Writing new VTK-m filters.

  + Community discussion takes place on the [VTK-m users email list].

  + Doxygen-generated nightly reference documentation is available
    [online][VTK-m Doxygen].


## Contributing ##

There are many ways to contribute to [VTK-m], with varying levels of
effort.

  + Ask a question on the [VTK-m users email list].

  + Submit new or add to discussions of a feature requests or bugs on the
    [VTK-m Issue Tracker].

  + Submit a Pull Request to improve [VTK-m]
      + See [CONTRIBUTING.md] for detailed instructions on how to create a
        Pull Request.
      + See the [VTK-m Coding Conventions] that must be followed for
        contributed code.

  + Submit an Issue or Pull Request for the [VTK-m Users Guide]


## Dependencies ##

VTK-m Requires:

  + C++11 Compiler. VTK-m has been confirmed to work with the following
      + GCC 5.4+
      + Clang 5.0+
      + XCode 5.0+
      + MSVC 2015+
      + Intel 17.0.4+
  + [CMake](http://www.cmake.org/download/)
      + CMake 3.12+
      + CMake 3.13+ (for CUDA support)

Optional dependencies are:

  + CUDA Device Adapter
      + [Cuda Toolkit 9.2, >= 10.2](https://developer.nvidia.com/cuda-toolkit)
      + Note CUDA >= 10.2 is required on Windows
  + TBB Device Adapter
      + [TBB](https://www.threadingbuildingblocks.org/)
  + OpenMP Device Adapter
      + Requires a compiler that supports OpenMP >= 4.0.
  + OpenGL Rendering
      + The rendering module contains multiple rendering implementations
        including standalone rendering code. The rendering module also
        includes (optionally built) OpenGL rendering classes.
      + The OpenGL rendering classes require that you have a extension
        binding library and one rendering library. A windowing library is
        not needed except for some optional tests.
  + Extension Binding
      + [GLEW](http://glew.sourceforge.net/)
  + On Screen Rendering
      + OpenGL Driver
      + Mesa Driver
  + On Screen Rendering Tests
      + [GLFW](http://www.glfw.org/)
      + [GLUT](http://freeglut.sourceforge.net/)
  + Headless Rendering
      + [OS Mesa](https://www.mesa3d.org/osmesa.html)
      + EGL Driver

VTK-m has been tested on the following configurations:c
  + On Linux
      + GCC 5.4.0, 5.4, 6.5, 7.4, 8.2, 9.2; Clang 5, 8; Intel 17.0.4; 19.0.0
      + CMake 3.12, 3.13, 3.16, 3.17
      + CUDA 9.2, 10.2, 11.0, 11.1 
      + TBB 4.4 U2, 2017 U7
  + On Windows
      + Visual Studio 2015, 2017
      + CMake 3.12, 3.17
      + CUDA 10.2
      + TBB 2017 U3, 2018 U2
  + On MacOS
      + AppleClang 9.1
      + CMake 3.12
      + TBB 2018


## Building ##

VTK-m supports all majors platforms (Windows, Linux, OSX), and uses CMake
to generate all the build rules for the project. The VTK-m source code is
available from the [VTK-m download page] or by directly cloning the [VTK-m
git repository].

The basic procedure for building VTK-m is to unpack the source, create a
build directory, run CMake in that build directory (pointing to the source)
and then build. Here are some example *nix commands for the process
(individual commands may vary).

```sh
$ tar xvzf ~/Downloads/vtk-m-v1.4.0.tar.gz
$ mkdir vtkm-build
$ cd vtkm-build
$ cmake-gui ../vtk-m-v1.4.0
$ cmake --build -j .              # Runs make (or other build program)
```

A more detailed description of building VTK-m is available in the [VTK-m
Users Guide].


## Example ##

The VTK-m source distribution includes a number of examples. The goal of the
VTK-m examples is to illustrate specific VTK-m concepts in a consistent and
simple format. However, these examples only cover a small part of the
capabilities of VTK-m.

Below is a simple example of using VTK-m to load a VTK image file, run the
Marching Cubes algorithm on it, and render the results to an image:

```cpp
#include <vtkm/Bounds.h>
#include <vtkm/Range.h>
#include <vtkm/cont/ColorTable.h>
#include <vtkm/filter/Contour.h>
#include <vtkm/io/VTKDataSetReader.h>
#include <vtkm/rendering/Actor.h>
#include <vtkm/rendering/Camera.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/rendering/Color.h>
#include <vtkm/rendering/MapperRayTracer.h>
#include <vtkm/rendering/Scene.h>
#include <vtkm/rendering/View3D.h>

vtkm::io::VTKDataSetReader reader("path/to/vtk_image_file.vtk");
vtkm::cont::DataSet inputData = reader.ReadDataSet();
std::string fieldName = "scalars";

vtkm::Range range;
inputData.GetPointField(fieldName).GetRange(&range);
vtkm::Float64 isovalue = range.Center();

// Create an isosurface filter
vtkm::filter::Contour filter;
filter.SetIsoValue(0, isovalue);
filter.SetActiveField(fieldName);
vtkm::cont::DataSet outputData = filter.Execute(inputData);

// compute the bounds and extends of the input data
vtkm::Bounds coordsBounds = inputData.GetCoordinateSystem().GetBounds();

// setup a camera and point it to towards the center of the input data
vtkm::rendering::Camera camera;
camera.ResetToBounds(coordsBounds);
vtkm::cont::ColorTable colorTable("inferno");

// Create a mapper, canvas and view that will be used to render the scene
vtkm::rendering::Scene scene;
vtkm::rendering::MapperRayTracer mapper;
vtkm::rendering::CanvasRayTracer canvas(512, 512);
vtkm::rendering::Color bg(0.2f, 0.2f, 0.2f, 1.0f);

// Render an image of the output isosurface
scene.AddActor(vtkm::rendering::Actor(outputData.GetCellSet(),
                                      outputData.GetCoordinateSystem(),
                                      outputData.GetField(fieldName),
                                      colorTable));
vtkm::rendering::View3D view(scene, mapper, canvas, camera, bg);
view.Paint();
view.SaveAs("demo_output.png");
```

A minimal CMakeLists.txt such as the following one can be used to build this
example.

```CMake
project(example)

set(VTKm_DIR "/somepath/lib/cmake/vtkm-XYZ")

find_package(VTKm REQUIRED)

add_executable(example example.cxx)
target_link_libraries(example vtkm_cont vtkm_rendering)
```

## License ##

VTK-m is distributed under the OSI-approved BSD 3-clause License.
See [LICENSE.txt](LICENSE.txt) for details.


[VTK-m]:                    https://gitlab.kitware.com/vtk/vtk-m/
[VTK-m Coding Conventions]: docs/CodingConventions.md
[VTK-m Doxygen]:            http://m.vtk.org/documentation/
[VTK-m download page]:      http://m.vtk.org/index.php/VTK-m_Releases
[VTK-m git repository]:     https://gitlab.kitware.com/vtk/vtk-m/
[VTK-m Issue Tracker]:      https://gitlab.kitware.com/vtk/vtk-m/-/issues
[VTK-m Overview]:           http://m.vtk.org/images/2/29/VTKmVis2016.pptx
[VTK-m Users Guide]:        http://m.vtk.org/images/c/c8/VTKmUsersGuide.pdf
[VTK-m users email list]:   http://vtk.org/mailman/listinfo/vtkm
[VTK-m Wiki]:               http://m.vtk.org/
[VTK-m Tutorial]:           http://m.vtk.org/index.php/Tutorial
[CONTRIBUTING.md]:          CONTRIBUTING.md
