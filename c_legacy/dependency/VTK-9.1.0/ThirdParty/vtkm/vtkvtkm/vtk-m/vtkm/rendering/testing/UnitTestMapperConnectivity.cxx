//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#include <vtkm/cont/DeviceAdapter.h>
#include <vtkm/cont/testing/MakeTestDataSet.h>
#include <vtkm/cont/testing/Testing.h>
#include <vtkm/rendering/Actor.h>
#include <vtkm/rendering/Canvas.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/rendering/MapperConnectivity.h>
#include <vtkm/rendering/Scene.h>
#include <vtkm/rendering/View3D.h>
#include <vtkm/rendering/raytracing/Logger.h>
#include <vtkm/rendering/testing/RenderTest.h>

namespace
{

void RenderTests()
{
  try
  {
    vtkm::cont::testing::MakeTestDataSet maker;
    vtkm::cont::ColorTable colorTable("inferno");
    using M = vtkm::rendering::MapperConnectivity;
    using C = vtkm::rendering::CanvasRayTracer;
    using V3 = vtkm::rendering::View3D;

    vtkm::rendering::testing::Render<M, C, V3>(
      maker.Make3DRegularDataSet0(), "pointvar", colorTable, "reg3D.pnm");
    vtkm::rendering::testing::Render<M, C, V3>(
      maker.Make3DRectilinearDataSet0(), "pointvar", colorTable, "rect3D.pnm");
    vtkm::rendering::testing::Render<M, C, V3>(
      maker.Make3DExplicitDataSetZoo(), "pointvar", colorTable, "explicit3D.pnm");
  }
  catch (const std::exception& e)
  {
    std::cout << vtkm::rendering::raytracing::Logger::GetInstance()->GetStream().str() << "\n";
    std::cout << e.what() << "\n";
  }
}

} //namespace

int UnitTestMapperConnectivity(int argc, char* argv[])
{
  return vtkm::cont::testing::Testing::Run(RenderTests, argc, argv);
}
