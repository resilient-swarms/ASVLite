/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkGaussianRandomSequence.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.
=========================================================================*/
#include "vtkGaussianRandomSequence.h"

//------------------------------------------------------------------------------
vtkGaussianRandomSequence::vtkGaussianRandomSequence() = default;

//------------------------------------------------------------------------------
vtkGaussianRandomSequence::~vtkGaussianRandomSequence() = default;

//------------------------------------------------------------------------------
double vtkGaussianRandomSequence::GetScaledValue(double mean, double standardDeviation)
{
  return mean + standardDeviation * this->GetValue();
}

//------------------------------------------------------------------------------
double vtkGaussianRandomSequence::GetNextScaledValue(double mean, double standardDeviation)
{
  this->Next();
  return this->GetScaledValue(mean, standardDeviation);
}

//------------------------------------------------------------------------------
void vtkGaussianRandomSequence::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
