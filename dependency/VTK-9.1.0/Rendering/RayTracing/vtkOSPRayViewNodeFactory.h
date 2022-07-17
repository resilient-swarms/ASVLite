/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayViewNodeFactory.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkOSPRayViewNodeFactory
 * @brief   matches vtk rendering classes to
 * specific ospray ViewNode classes
 *
 * Ensures that vtkOSPRayPass makes ospray specific translator instances
 * for every VTK rendering pipeline class instance it encounters.
 */

#ifndef vtkOSPRayViewNodeFactory_h
#define vtkOSPRayViewNodeFactory_h

#include "vtkRenderingRayTracingModule.h" // For export macro
#include "vtkViewNodeFactory.h"

class VTKRENDERINGRAYTRACING_EXPORT vtkOSPRayViewNodeFactory : public vtkViewNodeFactory
{
public:
  static vtkOSPRayViewNodeFactory* New();
  vtkTypeMacro(vtkOSPRayViewNodeFactory, vtkViewNodeFactory);
  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  vtkOSPRayViewNodeFactory();
  ~vtkOSPRayViewNodeFactory() override;

private:
  vtkOSPRayViewNodeFactory(const vtkOSPRayViewNodeFactory&) = delete;
  void operator=(const vtkOSPRayViewNodeFactory&) = delete;
};

#endif
