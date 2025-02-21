/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkBoostGraphAdapter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

/**
 * @class   vtkBoostBetweennessClustering
 * @brief   Implements graph clustering based on
 * edge betweenness centrality.
 *
 *
 *
 * This vtk class uses the Boost centrality clustering
 * generic algorithm to compute edge betweenness centrality on
 * the input graph (a vtkGraph).
 *
 * @sa
 * vtkGraph vtkBoostGraphAdapter
 */

#ifndef vtkBoostBetweennessClustering_h
#define vtkBoostBetweennessClustering_h

#include "vtkGraphAlgorithm.h"
#include "vtkInfovisBoostGraphAlgorithmsModule.h" // For export macro

class VTKINFOVISBOOSTGRAPHALGORITHMS_EXPORT vtkBoostBetweennessClustering : public vtkGraphAlgorithm
{
public:
  static vtkBoostBetweennessClustering* New();
  vtkTypeMacro(vtkBoostBetweennessClustering, vtkGraphAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkBoostBetweennessClustering();
  ~vtkBoostBetweennessClustering() override;

  ///@{
  /**
   * Get/Set the threshold value. Algorithm terminats when the maximum edge
   * centrality is below this threshold.
   */
  vtkSetMacro(Threshold, double);
  vtkGetMacro(Threshold, double);
  ///@}

  ///@{
  /**
   * Get/Set the flag that sets the rule whether or not to use the
   * edge weight array as set using \c SetEdgeWeightArrayName.
   */
  vtkSetMacro(UseEdgeWeightArray, bool);
  vtkBooleanMacro(UseEdgeWeightArray, bool);
  ///@}

  vtkSetMacro(InvertEdgeWeightArray, bool);
  vtkBooleanMacro(InvertEdgeWeightArray, bool);

  ///@{
  /**
   * Get/Set the name of the array that needs to be used as the edge weight.
   * The array should be a vtkDataArray.
   */
  vtkGetStringMacro(EdgeWeightArrayName);
  vtkSetStringMacro(EdgeWeightArrayName);
  ///@}

  ///@{
  /**
   * Set the edge centrality array name. If no output array name is
   * set then the name "edge_centrality" is used.
   */
  vtkSetStringMacro(EdgeCentralityArrayName);
  ///@}

protected:
  int RequestData(vtkInformation* request, vtkInformationVector** inputVector,
    vtkInformationVector* outputVector) override;

  int FillOutputPortInformation(int port, vtkInformation* info) override;

private:
  double Threshold;
  bool UseEdgeWeightArray;
  bool InvertEdgeWeightArray;
  char* EdgeWeightArrayName;
  char* EdgeCentralityArrayName;

  vtkBoostBetweennessClustering(const vtkBoostBetweennessClustering&) = delete;
  void operator=(const vtkBoostBetweennessClustering&) = delete;
};

#endif // vtkBoostBetweennessClustering_h
