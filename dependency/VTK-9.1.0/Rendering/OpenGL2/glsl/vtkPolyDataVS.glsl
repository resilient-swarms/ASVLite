//VTK::System::Dec

/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPolyDataVS.glsl

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

in vec4 vertexMC;

//VTK::CustomUniforms::Dec

// frag position in VC
//VTK::PositionVC::Dec

// optional normal declaration
//VTK::Normal::Dec

// extra lighting parameters
//VTK::Light::Dec

// Texture coordinates
//VTK::TCoord::Dec

// material property values
//VTK::Color::Dec

// clipping plane vars
//VTK::Clip::Dec

// camera and actor matrix values
//VTK::Camera::Dec

// Apple Bug
//VTK::PrimID::Dec

// Value raster
//VTK::ValuePass::Dec

// picking support
//VTK::Picking::Dec

void main()
{
  //VTK::CustomBegin::Impl

  //VTK::Color::Impl

  //VTK::Normal::Impl

  //VTK::TCoord::Impl

  //VTK::Clip::Impl

  //VTK::PrimID::Impl

  //VTK::PositionVC::Impl

  //VTK::ValuePass::Impl

  //VTK::Light::Impl

  //VTK::Picking::Impl

  //VTK::CustomEnd::Impl
}
