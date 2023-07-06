/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkOSPRayVolumeMapper.h
  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen

  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkOSPRayVolumeMapper
 * @brief   Standalone OSPRayVolumeMapper.
 *
 * This is a standalone interface for ospray volume rendering to be used
 * within otherwise OpenGL rendering contexts such as within the
 * SmartVolumeMapper.
 */

#ifndef vtkOSPRayVolumeMapper_h
#define vtkOSPRayVolumeMapper_h

#include "vtkOSPRayVolumeInterface.h"
#include "vtkRenderingRayTracingModule.h" // For export macro

class vtkOSPRayPass;
class vtkRenderer;
class vtkWindow;

class VTKRENDERINGRAYTRACING_EXPORT vtkOSPRayVolumeMapper : public vtkOSPRayVolumeInterface
{
public:
  static vtkOSPRayVolumeMapper* New();
  vtkTypeMacro(vtkOSPRayVolumeMapper, vtkOSPRayVolumeInterface);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Release any graphics resources that are being consumed by this mapper.
   * The parameter window could be used to determine which graphic
   * resources to release.
   */
  void ReleaseGraphicsResources(vtkWindow*) override;

  // Initialize internal constructs
  virtual void Init();

  /**
   * Render the volume onto the screen.
   * Overridden to use OSPRay to do the work.
   */
  void Render(vtkRenderer*, vtkVolume*) override;

protected:
  vtkOSPRayVolumeMapper();
  ~vtkOSPRayVolumeMapper() override;

  vtkOSPRayPass* InternalOSPRayPass;
  vtkRenderer* InternalRenderer;
  bool Initialized;

private:
  vtkOSPRayVolumeMapper(const vtkOSPRayVolumeMapper&) = delete;
  void operator=(const vtkOSPRayVolumeMapper&) = delete;
};

#endif
