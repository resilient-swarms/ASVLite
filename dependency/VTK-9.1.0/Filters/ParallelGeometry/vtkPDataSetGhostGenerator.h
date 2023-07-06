/*=========================================================================

 Program:   Visualization Toolkit
 Module:    vtkPDataSetGhostGenerator.h

 Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
 All rights reserved.
 See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

 This software is distributed WITHOUT ANY WARRANTY; without even
 the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the above copyright notice for more information.

 =========================================================================*/
/**
 * @class   vtkPDataSetGhostGenerator
 *
 *
 *  An abstract class that provides common functionality and implements an
 *  interface for all parallel ghost data generators.
 *
 * @sa
 * vtkDataSetGhostGenerator, vtkPUniformGridGhostDataGenerator,
 * vtkPStructuredGridGhostDataGenerator, vtkPRectilinearGridGhostDataGenerator
 */

#ifndef vtkPDataSetGhostGenerator_h
#define vtkPDataSetGhostGenerator_h

#include "vtkDataSetGhostGenerator.h"
#include "vtkDeprecation.h"                   // For VTK_DEPRECATED_IN_9_1_0
#include "vtkFiltersParallelGeometryModule.h" // For export macro

class vtkMultiProcessController;
class vtkMultiBlockDataSet;

class VTK_DEPRECATED_IN_9_1_0(
  "Use vtkGhostCellsGenerator instead") VTKFILTERSPARALLELGEOMETRY_EXPORT vtkPDataSetGhostGenerator
  : public vtkDataSetGhostGenerator
{
public:
  vtkTypeMacro(vtkPDataSetGhostGenerator, vtkDataSetGhostGenerator);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Get/Set macro for the multi-process controller. If a controller is not
   * supplied, then, the global controller is assumed.
   */
  vtkSetMacro(Controller, vtkMultiProcessController*);
  vtkGetMacro(Controller, vtkMultiProcessController*);
  ///@}

  /**
   * Initializes
   */
  void Initialize();

  /**
   * Barrier synchronization
   */
  void Barrier();

protected:
  vtkPDataSetGhostGenerator();
  ~vtkPDataSetGhostGenerator() override;

  /**
   * Creates ghost layers. Implemented by concrete implementations.
   */
  virtual void GenerateGhostLayers(
    vtkMultiBlockDataSet* in, vtkMultiBlockDataSet* out) override = 0;

  int Rank;
  bool Initialized;
  vtkMultiProcessController* Controller;

private:
  vtkPDataSetGhostGenerator(const vtkPDataSetGhostGenerator&) = delete;
  void operator=(const vtkPDataSetGhostGenerator&) = delete;
};

#endif /* vtkPDataSetGhostGenerator_h */

// VTK-HeaderTest-Exclude: vtkPDataSetGhostGenerator.h
