/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestPathTracerMaterials.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// This test verifies that we can load a set of materials specification
// from disk and use them.

#include "vtkTestUtilities.h"

#include "vtkActor.h"
#include "vtkOSPRayMaterialLibrary.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"

#include <string>

int TestPathTracerMaterialLibrary(int argc, char* argv[])
{
  // read an ospray material file
  const char* materialFile =
    vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/ospray_mats.json");
  vtkSmartPointer<vtkOSPRayMaterialLibrary> lib = vtkSmartPointer<vtkOSPRayMaterialLibrary>::New();
  cout << "Open " << materialFile << endl;
  lib->ReadFile(materialFile);
  cout << "Parsed file OK, now check for expected contents." << endl;
  std::set<std::string> mats = lib->GetMaterialNames();

  cout << "Materials are:" << endl;
  std::set<std::string>::iterator it = mats.begin();
  while (it != mats.end())
  {
    cout << *it << endl;
    ++it;
  }
  if (mats.find("Water") == mats.end())
  {
    cerr << "Problem, could not find expected material named water." << endl;
    return VTK_ERROR;
  }
  cout << "Found Water material." << endl;
  if (lib->LookupImplName("Water") != "glass")
  {
    cerr << "Problem, expected Water to be implemented by the glass material." << endl;
    return VTK_ERROR;
  }
  cout << "Water is the right type." << endl;
  if (lib->GetDoubleShaderVariable("Water", "attenuationColor").size() != 3)
  {
    cerr << "Problem, expected Water to have a 3 component variable called attentuationColor."
         << endl;
    return VTK_ERROR;
  }
  cout << "Water has an expected variable." << endl;
  if (lib->GetTexture("Bumpy", "map_bump") == nullptr)
  {
    cerr << "Problem, expected Bumpy to have a texture called map_bump." << endl;
    return VTK_ERROR;
  }
  cout << "Bumpy has a good texture too." << endl;
  std::string textureName = lib->GetTextureName("Bumpy", "map_bump");
  if (textureName != "vtk")
  {
    cerr << "Problem, expected Bumpy to have a 'map_bump' texture named 'vtk'." << endl;
    return VTK_ERROR;
  }
  cout << "Bumpy has a good texture name too." << endl;
  std::string textureFilename = lib->GetTextureFilename("Bumpy", "map_bump");
  if (textureFilename != vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/vtk.png"))
  {
    cerr << "Problem, expected Bumpy to have a 'map_bump' texture with filename named vtk.png"
         << endl;
    return VTK_ERROR;
  }
  cout << "Bumpy has a good texture filename too." << endl;
  // read a wavefront mtl file
  const char* materialFile2 =
    vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/ospray_mats.mtl");
  cout << "Open " << materialFile2 << endl;

  lib->ReadFile(materialFile2);
  cout << "Parsed file OK, now check for expected contents." << endl;

  mats = lib->GetMaterialNames();
  cout << "Materials are now:" << endl;
  it = mats.begin();
  while (it != mats.end())
  {
    cout << *it << endl;
    ++it;
  }

  auto ks = lib->GetDoubleShaderVariable("mat1", "Ks");
  if (ks[2] != 0.882353)
  {
    cerr << "Problem, could not find expected material mat1 ks component." << endl;
    return VTK_ERROR;
  }

  if (mats.find("mat2") == mats.end())
  {
    cerr << "Problem, could not find expected material named mat2." << endl;
    return VTK_ERROR;
  }
  if (lib->GetDoubleShaderVariable("mat2", "Kd").size() == 0)
  {
    cerr << "Problem, expected mat2 to have a variable called Kd." << endl;
    return VTK_ERROR;
  }

  lib->RemoveAllShaderVariables("mat2");
  if (lib->GetDoubleShaderVariable("mat2", "Kd").size() > 0)
  {
    cerr << "Problem, expected mat2 to have Kd removed." << endl;
    return VTK_ERROR;
  }

  cout << "mat2 has an expected variable." << endl;
  if (lib->GetTexture("mat2", "map_Kd") == nullptr)
  {
    cerr << "Problem, expected mat2 to have a texture called map_Kd." << endl;
    return VTK_ERROR;
  }
  cout << "mat2 has a good texture too." << endl;

  textureName = lib->GetTextureName("mat2", "map_Kd");
  if (textureName != "vtk")
  {
    cerr << "Problem, expected mat2 to have a texture named 'vtk'." << endl;
    return VTK_ERROR;
  }
  cout << "mat2 has a good texture name too." << endl;
  textureFilename = lib->GetTextureFilename("mat2", "map_Kd");
  if (textureFilename != vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/vtk.png"))
  {
    cerr << "Problem, expected Bumpy to have a 'map_bump' texture with filename named vtk.png"
         << endl;
    return VTK_ERROR;
  }
  cout << "mat2 has a good texture filename too." << endl;
  lib->RemoveAllTextures("mat2");
  if (lib->GetTexture("mat2", "map_Kd") != nullptr)
  {
    cerr << "Problem, expected mat2 to have map_Kd removed." << endl;
    return VTK_ERROR;
  }

  if (mats.find("mat3") == mats.end())
  {
    cerr << "Problem, could not find expected material named mat3." << endl;
    return VTK_ERROR;
  }
  if (lib->LookupImplName("mat3") != "metal")
  {
    cerr << "Problem, expected mat3 to be implemented by the metal material." << endl;
    return VTK_ERROR;
  }
  cout << "mat3 is the right type." << endl;

  cout << "We're all clear kid." << endl;

  // serialize and deserialize
  cout << "Serialize" << endl;
  const char* buf = lib->WriteBuffer();

  cout << "Deserialize" << endl;
  lib->ReadBuffer(buf);

  return 0;
}
