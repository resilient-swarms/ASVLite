/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkGenericImageInterpolator.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkGenericImageInterpolator
 * @brief   interpolate data values from images using vtkGenericDataArray API
 *
 * vtkGenericImageInterpolator provides a simple interface for interpolating image
 * data.  It provides linear, cubic, and nearest-neighbor interpolation. The only
 * difference between it and vtkImageInterpolator is vtkGenericImageInterpolator
 * does not assume an underlying data structure for its data arrays; instead, it
 * uses the API from vtkGenericDataArray to perform calculations.
 * @sa
 * vtkImageInterpolator vtkImageReslice
 */

#ifndef vtkGenericImageInterpolator_h
#define vtkGenericImageInterpolator_h

#include "vtkImageInterpolator.h"
#include "vtkImagingCoreModule.h" // For export macro

class VTKIMAGINGCORE_EXPORT vtkGenericImageInterpolator : public vtkImageInterpolator
{
public:
  static vtkGenericImageInterpolator* New();
  vtkTypeMacro(vtkGenericImageInterpolator, vtkImageInterpolator);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Update the interpolator.  If the interpolator has been modified by
   * a Set method since Initialize() was called, you must call this method
   * to update the interpolator before you can use it.
   */
  void Update() override;

protected:
  vtkGenericImageInterpolator();
  ~vtkGenericImageInterpolator() override;

  ///@{
  /**
   * Get the interpolation functions.
   */
  void GetInterpolationFunc(
    void (**doublefunc)(vtkInterpolationInfo*, const double[3], double*)) override;
  void GetInterpolationFunc(
    void (**floatfunc)(vtkInterpolationInfo*, const float[3], float*)) override;
  ///@}

  ///@{
  /**
   * Get the row interpolation functions.
   */
  void GetRowInterpolationFunc(
    void (**doublefunc)(vtkInterpolationWeights*, int, int, int, double*, int)) override;
  void GetRowInterpolationFunc(
    void (**floatfunc)(vtkInterpolationWeights*, int, int, int, float*, int)) override;
  ///@}

private:
  vtkGenericImageInterpolator(const vtkGenericImageInterpolator&) = delete;
  void operator=(const vtkGenericImageInterpolator&) = delete;
};

#endif
