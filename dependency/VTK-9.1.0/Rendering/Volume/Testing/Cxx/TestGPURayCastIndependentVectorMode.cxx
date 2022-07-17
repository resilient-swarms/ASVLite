/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestGPURayCastIndependentVectorMode.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

/** \description
 * Tests vector rendering mode in vtkVolumeSmartMapper. VectorMode builds
 * on the independent component support provided by GPURayCastMapper. Each of
 * the components are considered independent vector components. To render the
 * vector's magnitude, an additional data array is computed through vtkImageMagnitude.
 * This test renders a component and the vector magnitude in two separate volumes.
 * Each mapper switches modes and components to ensure the internal mapper's
 * resources are managed correctly and no errors are generated.
 */

#include "vtkCamera.h"
#include "vtkColorTransferFunction.h"
#include "vtkImageData.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkNew.h"
#include "vtkPiecewiseFunction.h"
#include "vtkPointDataToCellData.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSmartVolumeMapper.h"
#include "vtkVolume.h"
#include "vtkVolumeProperty.h"

int TestGPURayCastIndependentVectorMode(int argc, char* argv[])
{
  // cout << "CTEST_FULL_OUTPUT (Avoid ctest truncation of output)" << endl;

  int dims[3] = { 20, 20, 20 };

  // Create a vtkImageData with two components
  vtkNew<vtkImageData> image;
  image->SetDimensions(dims[0], dims[1], dims[2]);
  image->AllocateScalars(VTK_DOUBLE, 3);

  for (int z = 0; z < dims[2]; ++z)
  {
    for (int y = 0; y < dims[1]; ++y)
    {
      for (int x = 0; x < dims[0]; ++x)
      {
        // The 3-component vector field is described by:
        // V = coords_x *  iHat   +   10 * coords_y * jHat   +   coords_z * kHat
        float const valueX = static_cast<float>(x) - dims[0] / 2.0f;
        float const valueY = static_cast<float>(y) - dims[1] / 2.0f;
        float const valueZ = static_cast<float>(z) - dims[2] / 2.0f;
        image->SetScalarComponentFromFloat(x, y, z, 0, valueX);
        image->SetScalarComponentFromFloat(x, y, z, 1, valueY * 10.f);
        image->SetScalarComponentFromFloat(x, y, z, 2, valueZ);
      }
    }
  }

  vtkNew<vtkRenderWindow> renWin;
  renWin->SetSize(400, 400);
  renWin->SetMultiSamples(0);

  vtkNew<vtkRenderer> ren;
  ren->SetBackground(0.3176, 0.3412, 0.4314);
  renWin->AddRenderer(ren);

  vtkNew<vtkRenderWindowInteractor> iren;
  vtkNew<vtkInteractorStyleTrackballCamera> style;
  iren->SetInteractorStyle(style);
  iren->SetRenderWindow(renWin);

  renWin->Render();

  // Mapper 1 (final render as component)
  vtkNew<vtkSmartVolumeMapper> mapper;
  mapper->AutoAdjustSampleDistancesOff();
  mapper->SetSampleDistance(0.5);
  mapper->SetInputData(image);

  // TFs (known x values from V)
  vtkNew<vtkColorTransferFunction> ctf1;
  ctf1->AddRGBPoint(-100, 0.0, 0.0, 1.0);
  ctf1->AddRGBPoint(0, 0.86, 0.86, 0.86);
  ctf1->AddRGBPoint(100, 1.0, 0.0, 0.0);

  vtkNew<vtkPiecewiseFunction> pf1;
  pf1->AddPoint(-100.0, 0.0);
  pf1->AddPoint(0, 0.5);
  pf1->AddPoint(100, 1.0);

  vtkNew<vtkVolumeProperty> property;
  property->IndependentComponentsOn();
  property->SetColor(0, ctf1);
  property->SetScalarOpacity(0, pf1);

  vtkNew<vtkVolume> volume;
  volume->SetMapper(mapper);
  volume->SetProperty(property);
  ren->AddVolume(volume);

  // Mapper 2 (final render as magnitude)
  vtkNew<vtkSmartVolumeMapper> mapperMag;
  mapperMag->AutoAdjustSampleDistancesOff();
  mapperMag->SetSampleDistance(0.5);
  mapperMag->SetInputData(image);

  // TFs (known x values from V)
  vtkNew<vtkColorTransferFunction> ctf2;
  ctf2->AddRGBPoint(0, 0.0, 0.0, 1.0);
  ctf2->AddRGBPoint(50, 0.86, 0.86, 0.86);
  ctf2->AddRGBPoint(101, 1.0, 0.0, 0.0);

  vtkNew<vtkPiecewiseFunction> pf2;
  pf2->AddPoint(0, 0.0);
  pf2->AddPoint(50, 0.3);
  pf2->AddPoint(101, 1.0);

  vtkNew<vtkVolumeProperty> propertyMag;
  propertyMag->SetColor(0, ctf2);
  propertyMag->SetScalarOpacity(0, pf2);

  vtkNew<vtkVolume> volumeMag;
  volumeMag->SetMapper(mapperMag);
  volumeMag->SetProperty(propertyMag);
  ren->AddVolume(volumeMag);
  volumeMag->SetPosition(20.0, 20.0, 0.0);
  ren->ResetCamera();

  // Mapper 3 (final render as magnitude - cell data)
  vtkNew<vtkPointDataToCellData> pointsToCells;
  pointsToCells->SetInputData(image);
  pointsToCells->Update();

  vtkNew<vtkSmartVolumeMapper> mapperMagCells;
  mapperMagCells->AutoAdjustSampleDistancesOff();
  mapperMagCells->SetSampleDistance(0.5);
  mapperMagCells->SetInputData(pointsToCells->GetOutput());

  vtkNew<vtkVolume> volumeMagCells;
  volumeMagCells->SetMapper(mapperMagCells);
  volumeMagCells->SetProperty(propertyMag);
  ren->AddVolume(volumeMagCells);
  volumeMagCells->SetPosition(20.0, 0.0, 0.0);
  ren->ResetCamera();

  // Switch between components and magnitude to ensure no errors
  // are generated
  mapper->SetVectorMode(vtkSmartVolumeMapper::COMPONENT);
  mapper->SetVectorComponent(0);
  mapperMag->SetVectorMode(vtkSmartVolumeMapper::MAGNITUDE);
  renWin->Render();

  mapper->SetVectorMode(vtkSmartVolumeMapper::MAGNITUDE);
  mapperMag->SetVectorMode(vtkSmartVolumeMapper::COMPONENT);
  mapperMag->SetVectorComponent(2);
  renWin->Render();

  mapper->SetVectorMode(vtkSmartVolumeMapper::COMPONENT);
  mapper->SetVectorComponent(1);
  mapperMag->SetVectorMode(vtkSmartVolumeMapper::MAGNITUDE);
  renWin->Render();

  mapperMagCells->SetVectorMode(vtkSmartVolumeMapper::COMPONENT);
  mapperMagCells->SetVectorComponent(2);
  renWin->Render();

  mapperMagCells->SetVectorMode(vtkSmartVolumeMapper::MAGNITUDE);
  renWin->Render();

  int retVal = vtkRegressionTestImage(renWin);
  if (retVal == vtkRegressionTester::DO_INTERACTOR)
  {
    iren->Start();
  }

  return !retVal;
}
