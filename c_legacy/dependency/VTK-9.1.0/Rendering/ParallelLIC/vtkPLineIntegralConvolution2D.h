/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPLineIntegralConvolution2D.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkPLineIntegralConvolution2D
 * @brief   parallel part of GPU-based
 * implementation of Line Integral Convolution (LIC)
 *
 *
 * Implements the parallel parts of the algorithm.
 *
 * @sa
 *  vtkPLineIntegralConvolution2D
 */

#ifndef vtkPLineIntegralConvolution2D_h
#define vtkPLineIntegralConvolution2D_h

#include "vtkLineIntegralConvolution2D.h"
#include "vtkRenderingParallelLICModule.h" // for export macro
#include <string>                          // for string

class vtkPainterCommunicator;
class vtkPPainterCommunicator;

class VTKRENDERINGPARALLELLIC_EXPORT vtkPLineIntegralConvolution2D
  : public vtkLineIntegralConvolution2D
{
public:
  static vtkPLineIntegralConvolution2D* New();
  vtkTypeMacro(vtkPLineIntegralConvolution2D, vtkLineIntegralConvolution2D);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Set the communicator to use during parallel operation
   * The communicator will not be duplicated or reference
   * counted for performance reasons thus caller should
   * hold/manage reference to the communicator during use
   * of the LIC object.
   */
  void SetCommunicator(vtkPainterCommunicator*) override;
  vtkPainterCommunicator* GetCommunicator() override;
  ///@}

  /**
   * For parallel operation, find global min/max
   * min/max are in/out.
   */
  void GetGlobalMinMax(vtkPainterCommunicator* comm, float& min, float& max) override;

  /**
   * Methods used for parallel benchmarks. Use cmake to define
   * vtkLineIntegralConviolution2DTIME to enable benchmarks.
   * During each update timing information is stored, it can
   * be written to disk by calling WriteLog.
   */
  void WriteTimerLog(VTK_FILEPATH const char* fileName) override;

protected:
  vtkPLineIntegralConvolution2D();
  ~vtkPLineIntegralConvolution2D() override;

  ///@{
  /**
   * Methods used for parallel benchmarks. Use cmake to define
   * vtkSurfaceLICPainterTIME to enable benchmarks. During each
   * update timing information is stored, it can be written to
   * disk by calling WriteLog. Note: Some of the timings are
   * enabled by the surface lic painter.
   */
  void StartTimerEvent(const char* name) override;
  void EndTimerEvent(const char* name) override;
  ///@}

private:
  std::string LogFileName;

private:
  vtkPLineIntegralConvolution2D(const vtkPLineIntegralConvolution2D&) = delete;
  void operator=(const vtkPLineIntegralConvolution2D&) = delete;
};

#endif
