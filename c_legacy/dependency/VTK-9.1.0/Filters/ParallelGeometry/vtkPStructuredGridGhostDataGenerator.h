/*=========================================================================

 Program:   Visualization Toolkit
 Module:    vtkPStructuredGridGhostDataGenerator.h

 Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
 All rights reserved.
 See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

 This software is distributed WITHOUT ANY WARRANTY; without even
 the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the above copyright notice for more information.

 =========================================================================*/
/**
 * @class   vtkPStructuredGridGhostDataGenerator
 *  structured grids.
 *
 *
 *  A concrete implementation of vtkPDataSEtGhostGenerator for generating ghost
 *  data on a partitioned and distributed domain of structured grids.
 *
 * @warning
 *  <ol>
 *   <li>
 *    The input multi-block dataset must:
 *    <ul>
 *      <li> Have the whole-extent set </li>
 *      <li> Each block must be an instance of vtkStructuredGrid </li>
 *      <li> Each block must have its corresponding global extent set in the
 *           meta-data using the PIECE_EXTENT() key </li>
 *      <li> All blocks must have the same fields loaded </li>
 *      <li> The multi-block structure is consistent on all processes </li>
 *    </ul>
 *   </li>
 *   <li>
 *    The code currently does not handle the following cases:
 *    <ul>
 *      <li>Periodic boundaries</li>
 *      <li>Growing ghost layers beyond the extents of the neighboring grid</li>
 *    </ul>
 *   </li>
 *  </ol>
 *
 * @sa
 * vtkDataSetGhostGenerator, vtkStructuredGridGhostDataGenerator,
 * vtkPDataSetGhostGenerator, vtkPUniformGridGhostDataGenerator
 */

#ifndef vtkPStructuredGridGhostDataGenerator_h
#define vtkPStructuredGridGhostDataGenerator_h

#include "vtkDeprecation.h"                   // For VTK_DEPRECATED_IN_9_1_0
#include "vtkFiltersParallelGeometryModule.h" // For export macro
#include "vtkPDataSetGhostGenerator.h"

class vtkMultiBlockDataSet;
class vtkIndent;
class vtkPStructuredGridConnectivity;

class VTK_DEPRECATED_IN_9_1_0("Use vtkGhostCellsGenerator instead")
  VTKFILTERSPARALLELGEOMETRY_EXPORT vtkPStructuredGridGhostDataGenerator
  : public vtkPDataSetGhostGenerator
{
public:
  static vtkPStructuredGridGhostDataGenerator* New();
  vtkTypeMacro(vtkPStructuredGridGhostDataGenerator, vtkPDataSetGhostGenerator);
  void PrintSelf(ostream& os, vtkIndent indent) override;

protected:
  vtkPStructuredGridGhostDataGenerator();
  ~vtkPStructuredGridGhostDataGenerator() override;

  /**
   * Registers the grid associated with this instance of multi-block.
   */
  void RegisterGrids(vtkMultiBlockDataSet* in);

  /**
   * Creates the output.
   */
  void CreateGhostedDataSet(vtkMultiBlockDataSet* in, vtkMultiBlockDataSet* out);

  /**
   * Generates ghost layers.
   */
  void GenerateGhostLayers(vtkMultiBlockDataSet* in, vtkMultiBlockDataSet* out) override;

  vtkPStructuredGridConnectivity* GridConnectivity;

private:
  vtkPStructuredGridGhostDataGenerator(const vtkPStructuredGridGhostDataGenerator&) = delete;
  void operator=(const vtkPStructuredGridGhostDataGenerator&) = delete;
};

#endif /* vtkPStructuredGridGhostDataGenerator_h */

// VTK-HeaderTest-Exclude: vtkPStructuredGridGhostDataGenerator.h
