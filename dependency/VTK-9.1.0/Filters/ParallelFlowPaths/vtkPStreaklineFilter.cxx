/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkPStreaklineFilter.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPStreaklineFilter.h"
#include "vtkAppendPolyData.h"
#include "vtkCell.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkFloatArray.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkIntArray.h"
#include "vtkMultiProcessController.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkSetGet.h"
#include "vtkSmartPointer.h"

#include <cassert>
#include <vector>

vtkStandardNewMacro(vtkPStreaklineFilter);

vtkPStreaklineFilter::vtkPStreaklineFilter()
{
  this->It.Initialize(this);
}

int vtkPStreaklineFilter::OutputParticles(vtkPolyData* particles)
{
  return this->It.OutputParticles(particles);
}

void vtkPStreaklineFilter::Finalize()
{
  int leader = 0;
  int tag = 129;

  if (this->Controller->GetLocalProcessId() == leader) // process 0 do the actual work
  {
    vtkNew<vtkAppendPolyData> append;
    int totalNumPts(0);
    for (int i = 0; i < this->Controller->GetNumberOfProcesses(); i++)
    {
      if (i != this->Controller->GetLocalProcessId())
      {
        vtkSmartPointer<vtkPolyData> output_i = vtkSmartPointer<vtkPolyData>::New();
        this->Controller->Receive(output_i, i, tag);
        append->AddInputData(output_i);
        totalNumPts += output_i->GetNumberOfPoints();
      }
      else
      {
        append->AddInputData(this->Output);
        totalNumPts += this->Output->GetNumberOfPoints();
      }
    }
    append->Update();
    vtkPolyData* appoutput = append->GetOutput();
    this->Output->Initialize();
    this->Output->ShallowCopy(appoutput);
    assert(this->Output->GetNumberOfPoints() == totalNumPts);
    this->It.Finalize();
  }
  else
  {
    // send everything to rank 0
    this->Controller->Send(this->Output, leader, tag);
    this->Output->Initialize();
  }
}

void vtkPStreaklineFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}
