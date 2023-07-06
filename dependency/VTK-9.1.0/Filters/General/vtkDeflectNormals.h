/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDeflectNormals.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkDeflectNormals
 * @brief   deflect normals using a 3 component vector field
 *
 * vtkDeflectNormals is a filter that modifies the normals using a vector field.
 * It is useful to give a 3D perception of a flat surface using shading of the mapper.
 *
 * The filter passes both its point data and cell data to its output.
 */

#ifndef vtkDeflectNormals_h
#define vtkDeflectNormals_h

#include "vtkDataSetAlgorithm.h"
#include "vtkFiltersGeneralModule.h" // For export macro

class VTKFILTERSGENERAL_EXPORT vtkDeflectNormals : public vtkDataSetAlgorithm
{
public:
  static vtkDeflectNormals* New();
  vtkTypeMacro(vtkDeflectNormals, vtkDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Specify value to scale deflection.
   * Default is 1.
   */
  vtkSetMacro(ScaleFactor, double);
  vtkGetMacro(ScaleFactor, double);
  ///@}

  ///@{
  /**
   * Specify user defined normal.
   * Default is (0, 0, 1).
   */
  vtkSetVector3Macro(UserNormal, double);
  vtkGetVector3Macro(UserNormal, double);
  ///@}

  ///@{
  /**
   * Specify value of the user defined normal.
   * Default is false.
   */
  vtkSetMacro(UseUserNormal, bool);
  vtkGetMacro(UseUserNormal, bool);
  vtkBooleanMacro(UseUserNormal, bool);
  ///@}

protected:
  vtkDeflectNormals();
  ~vtkDeflectNormals() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  double ScaleFactor = 1.0;
  double UserNormal[3] = { 0.0, 0.0, 1.0 };
  bool UseUserNormal = false;

private:
  vtkDeflectNormals(const vtkDeflectNormals&) = delete;
  void operator=(const vtkDeflectNormals&) = delete;
};

#endif
